#include "common.h"  // NOLINT

#include "core/SceneGrokker.h"

#include <mLibLodePNG.h>

#include "core/Classifier.h"
#include "core/Database.h"
#include "core/SkelPoseSampler.h"
#include "geo/geo.h"
#include "util/timer.h"
#include "vis/Heatmap.h"

using sg::vis::PoseHeatMap;

namespace sg {
namespace core {

SceneGrokker::SceneGrokker(const Database& _database, const Scan& _scan) : database(_database), scan(_scan) { }

double SceneGrokker::computeSceneIntensity(const ml::ColorImageR8G8B8A8& bmp, double cutoff) {
  const float scale = 1.0f / 255.0f;

  double result = 0.0;
  double sum = 0.0;

  ml::vec4uc bkgColor = bmp(0, 0);
  for (UINT y = 0; y < bmp.getHeight(); y++)
    for (UINT x = 0; x < bmp.getWidth(); x++)
      if (bmp(x, y) != bkgColor) {
        float testVal = bmp(y, x).r * scale;
        if (cutoff < 0.0) { result += testVal; }
        else if (testVal >= cutoff) { result++; }
        sum++;
      }
  return result / sum;
}

double SceneGrokker::computeSceneIntensity(const PoseHeatMap& heatMap, double cutoff) {
  double result = 0.0;
  double sum = 0.0;
  for (auto& h : heatMap.points()) {
    double testVal = h.likelihood();
    if (cutoff < 0.0) { result += testVal; }
    else if (testVal >= cutoff) { result++; }
    sum++;
  }
  return result / sum;
}

void SceneGrokker::computeSceneFeatureDescriptors(const string& dataDir) {
  cout << "computeSceneFeatureDescriptors" << endl;
  ml::Directory dir(dataDir + "evaluation/verbImagesTest/");
  set<string> verbSet;
  set<string> sceneSet;
  map<string, double> valuesCutoff[2];
  for (const string& s : dir.getFilesWithSuffix(".png")) {
    string sCut = ml::util::remove(s, ".png");
    auto parts = ml::util::split(sCut, "_");
    if (parts.size() == 2) {
      sceneSet.insert(parts[0]);
      verbSet.insert(parts[1]);
      const ml::ColorImageR8G8B8A8 img = ml::LodePNG::load(dir.getPath() + s);
      valuesCutoff[0][sCut] = computeSceneIntensity(img, -1.0);
      valuesCutoff[1][sCut] = computeSceneIntensity(img, 0.5);
    }
  }

  for (UINT cutoffIndex = 0; cutoffIndex < 2; cutoffIndex++) {
    ofstream file(dataDir + "evaluation/sceneDescriptors" + to_string(cutoffIndex) + ".csv");

    file << "scene";
    for (const string& verb : verbSet)
    { file << "," << verb; }
    file << endl;

    for (const string& scan : sceneSet) {
      file << scan;
      for (const string& verb : verbSet)
      { file << "," << valuesCutoff[cutoffIndex][scan + "_" + verb]; }
      file << endl;
    }
  }
}

void SceneGrokker::computeEvaluationDataset(const string& dataDir) {
  ml::Directory dir(dataDir + "evaluation/verbImagesTruth/");
  for (const string& s : dir.getFilesWithSuffix(".png")) {
    auto parts = ml::util::split(s, "_");
    if (parts.size() == 1) {
      //base scan name, ex.: gates450.png
      computeEvaluationDataset(dataDir, ml::util::remove(parts[0], ".png"));
    }
  }
}

void SceneGrokker::computeEvaluationDataset(const string& dataDir, const string& sceneName) {
  const string outDir = dataDir + "evaluation/verbImagesOut/";
  const string testDir = dataDir + "evaluation/verbImagesTest/";

  ml::util::makeDirectory(outDir + sceneName);
  ml::Directory truthDir(dataDir + "evaluation/verbImagesTruth/");

  ml::ColorImageR8G8B8A8 base = ml::LodePNG::load(truthDir.getPath() + sceneName + ".png");
  ml::ColorImageR8G8B8A8 maskOverlay = ml::LodePNG::load(truthDir.getPath() + sceneName + "_mask.png");

  ml::ColorImageR8G8B8A8 mask = makeMask(base, maskOverlay);

  for (const string& s : truthDir.getFilesWithSuffix(".png")) {
    if (ml::util::startsWith(s, sceneName) && !ml::util::endsWith(s, "_mask.png") && s != sceneName + ".png") {
      if (!ml::util::fileExists(testDir + s)) {
        ml::Console::log("file not found: " + testDir + s);
        continue;
      }
      ml::ColorImageR8G8B8A8 truth = ml::LodePNG::load(truthDir.getPath() + s);
      ml::ColorImageR8G8B8A8 test = ml::LodePNG::load(testDir + s);
      computeEvaluationDataset(base, mask, test, truth, outDir + ml::util::replace(s, ".png", ".csv"),
                               outDir + sceneName + "/" + ml::util::remove(s, ".png"));
    }
  }
}

ml::ColorImageR8G8B8A8 SceneGrokker::makeMask(const ml::ColorImageR8G8B8A8& base, const ml::ColorImageR8G8B8A8& maskOverlay) {
  ml::ColorImageR8G8B8A8 mask = base;
  mask.setPixels(ml::RGBColor::Black);
  for (UINT y = 0; y < base.getHeight(); y++)
    for (UINT x = 0; x < base.getWidth(); x++)
      if ((!maskOverlay.isValid(x,y) || base(x, y) != maskOverlay(x, y)) && 
          ml::RGBColor(base(x, y)) != ml::RGBColor::Magenta)
      { mask(x, y) = ml::RGBColor::White; }
  return mask;
}

using eval::BinaryConfusionMatrix;

vec<BinaryConfusionMatrix> SceneGrokker::computeConfusionMatrixSet(const string& dataDir, const string& sceneName,
  const string& verbID, const ml::ColorImageR8G8B8A8& test, double thresholdIncrement, const string& hybridImageSave) {
  const float scale = 1.0f / 255.0f;

  const string truthDir = dataDir + "evaluation/verbImagesTruth/";
  ml::ColorImageR8G8B8A8 base = ml::LodePNG::load(truthDir + sceneName + ".png");
  ml::ColorImageR8G8B8A8 maskOverlay = ml::LodePNG::load(truthDir + sceneName + "_mask.png");
  ml::ColorImageR8G8B8A8 mask = makeMask(base, maskOverlay);

  ml::ColorImageR8G8B8A8 truth = ml::LodePNG::load(truthDir + sceneName + "_" + verbID + ".png");

  vec<BinaryConfusionMatrix> result;

  if (truth.size() == 0)
  { return result; }

  if (hybridImageSave.size() > 0) {
    ml::ColorImageR8G8B8A8 hybrid = truth;
    for (UINT y = 0; y < mask.getHeight(); y++)
      for (UINT x = 0; x < mask.getWidth(); x++)
        if (ml::RGBColor(mask(x, y)) == ml::RGBColor::White) {
          hybrid(x, y) = test(x, y);
          bool truthVal = (truth(x, y) != base(x, y));
          if (truthVal)
          { hybrid(x, y).b = 200; }
          else
          { hybrid(x, y).b = 1; }
        } else
        { hybrid(x, y) = ml::RGBColor::Black; }
    ml::LodePNG::save(hybrid, hybridImageSave);
  }

  for (double threshold = 0.0; threshold < 1.0001; threshold += thresholdIncrement) {
    BinaryConfusionMatrix m(threshold);
    const UINT pixelIncrement = 1;
    for (UINT y = 0; y < mask.getHeight(); y += pixelIncrement)
      for (UINT x = 0; x < mask.getWidth(); x += pixelIncrement)
        if (ml::RGBColor(mask(x, y)) == ml::RGBColor::White) {
          bool testVal = (test(x, y).r * scale) >= threshold;
          bool truthVal = (truth(x, y) != base(x, y));
          m.add(truthVal, testVal);
        }
    result.push_back(m);
  }

  return result;
}

void SceneGrokker::computeEvaluationDataset(const ml::ColorImageR8G8B8A8& base, const ml::ColorImageR8G8B8A8& mask, const ml::ColorImageR8G8B8A8& test,
  const ml::ColorImageR8G8B8A8& truth, const string& csvFilename, const string& visFilenameBase) {
  const float scale = 1.0f / 255.0f;

  for (UINT pixelIncrement = 2; pixelIncrement <= 2; pixelIncrement++) {
    ofstream file("temp.csv");
    file << "test,ground truth" << endl;
    for (UINT y = 0; y < mask.getHeight(); y += pixelIncrement)
      for (UINT x = 0; x < mask.getWidth(); x += pixelIncrement)
        if (ml::RGBColor(mask(x, y)) == ml::RGBColor::White) {
          float testVal = test(x, y).r * scale;
          string testValStr = to_string(testVal);
          if (testValStr.size() > 5) { testValStr.resize(5); }

          string truthVal = "n";
          if (truth(x, y) != base(x, y))
          { truthVal = "y"; }

          file << testValStr << "," << truthVal << "\n";
        }

    if (pixelIncrement == 1)
    { ml::util::copyFile("temp.csv", csvFilename); }
    if (pixelIncrement == 2)
    { ml::util::copyFile("temp.csv", ml::util::replace(csvFilename, ".csv", "_small.csv")); }
  }

  const double thresholdIncrement = 0.05;
  for (double threshold = thresholdIncrement; threshold < 0.999; threshold += thresholdIncrement) {
    ml::ColorImageR8G8B8A8 result = base;
    //for (auto& color : result)
    //{ ml::RGBColor(color).value = color.value.grayscale(); }
    for (UINT y = 0; y < result.getHeight(); y++) {
      for (UINT x = 0; x < result.getWidth(); x++) {
        result(x, y) = ml::RGBColor(result(x, y)).grayscale();
      }
    }

    for (UINT y = 0; y < mask.getHeight(); y++)
      for (UINT x = 0; x < mask.getWidth(); x++)
        if (ml::RGBColor(mask(x, y)) == ml::RGBColor::White) {
          bool testVal = (test(x, y).r * scale) >= threshold;
          bool truthVal = (truth(x, y) != base(x, y));
          ml::RGBColor color = ml::RGBColor::Black;
          if (testVal && truthVal)
          { color = ml::RGBColor(89, 196, 120); }
          if (!testVal && truthVal)
          { color = ml::RGBColor(241, 84, 90); }
          if (testVal && !truthVal)
          { color = ml::RGBColor(255, 245, 63); }
          if (color != ml::RGBColor::Black)
          { result(x, y) = ml::RGBColor(ml::math::lerp(ml::vec3f(color), ml::vec3f(ml::RGBColor(base(x, y))), 0.2f)); }
        }

    ml::LodePNG::save(result, visFilenameBase + "_" + to_string(threshold) + ".png");
  }
}

void SceneGrokker::makeHeatMap(const ScenePoseClassifier& classifier, const vec<TransformedSkeleton>& skels,
                               PoseHeatMap* pHeatmap) const {
  util::Timer timer("makeHeatMap");
  SG_LOG_INFO << "[SceneGrokker] makeHeatMap using " << classifier.id << "...";
  pHeatmap->clear();
  pHeatmap->setSkeletons(skels);

  // Parameters
  const auto& p = *database.params;
  const sg::interaction::InteractionParams iparams(p);
  const int   numSamples            = p.get<int>("Evaluation.searchSampleCount");
  const float binSize               = p.get<float>("Evaluation.searchSampleBinSize");
  const float secondStageThresh     = p.get<float>("Evaluation.secondStageThresh");
  const int   numThetas             = p.get<int>("Evaluation.searchThetaDivisions");
  const int   numSkels              = static_cast<int>(skels.size());

  // Helper func to sample a given subdomain bbox
  const auto sampleBBox = [&](const geo::BBox2f& bbox, int numPoints, vec<PoseHeatMap::Point>* pPoints,
                              vec<float>* pThetas) {
    pPoints->resize(numPoints, PoseHeatMap::Point(numSkels, numThetas));
    SkelPoseSampler poser(bbox, skels, numPoints, numThetas);
    if (pThetas != nullptr) {  // Write out sampled theta values
      *pThetas = poser.getThetas();
    }
    const auto& samples = poser.getPoints();
    for (int iSample = 0; iSample < numPoints; ++iSample) {
      auto& entry = (*pPoints)[iSample];
      entry.pos = samples[iSample];
      std::fill(entry.likelihoods.data(), entry.likelihoods.data() + entry.likelihoods.num_elements(), 0);
    }

    // Accumulate classifier likelihood for all skeletons iterated by SkelPoseSampler
    Skeleton::SegmentsByJointPlusGaze activeSegs;  TransformedSkeleton tSkel;  SkelPoseSampler::PoseParams params;
    while (poser.getNext(&tSkel, &params)) {
      scan.getActiveSegments(tSkel.getSkeleton(), iparams.ignoreInferredJoints, iparams.kNearestSegsPerJoint,
                             iparams.maxDistToSegment, iparams.maxDistGaze, iparams.maxSegmentSizeRatio, &activeSegs);
      const SceneSkel sceneSkel = {&scan, &tSkel, &activeSegs};
      const double likelihood = classifier.classify(sceneSkel);
      (*pPoints)[params.iPoint].likelihoods[params.iPose][params.iTheta] += likelihood;
    }
  };

  // Get scan BBox and sample first pass
  geo::BBox2f sceneBBox;
  const auto maxPt = scan.bbox.getMax(), minPt = scan.bbox.getMin();
  sceneBBox.extend(geo::Vec2f(maxPt[0], maxPt[1]));
  sceneBBox.extend(geo::Vec2f(minPt[0], minPt[1]));
  vec<PoseHeatMap::Point> samplePoints;
  vec<float> sampledThetas;
  sampleBBox(sceneBBox, numSamples, &samplePoints, &sampledThetas);
  pHeatmap->addPoints(samplePoints, sampledThetas);

  // If 2nd stage sampling is enabled, find top subdomains and sample again
  if (secondStageThresh > 0) {
    util::Timer timer("makeHeatMap stage2");
    const vec<geo::BBox2f> bboxen = pHeatmap->topLikelihoodSampleBins(binSize, secondStageThresh);
    //pHeatmap->clear();  // Clear out first pass samples
    const int samplesPerBBox = numSamples / static_cast<int>(bboxen.size());
    for (const auto& bbox : bboxen) {
      sampleBBox(bbox, samplesPerBBox, &samplePoints, nullptr);
      pHeatmap->addPoints(samplePoints, sampledThetas);
    }
  }

  SG_LOG_INFO << "done. Used " << to_string(numSkels) << " skeletons";
}

}  // namespace core
}  // namespace sg
