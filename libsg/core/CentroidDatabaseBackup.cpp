#include "common.h"

#include "./CentroidDatabase.h"
#include "./Database.h"
#include "./SceneDatabase.h"
#include "./RecordingDatabase.h"
#include "./Skeleton.h"
#include "./Interaction.h"
#include "../util/Params.h"
#include "../stats/stats.h"
#include "../segmentation/MeshSegment.h"

#include <mLibCore.h>
#include <ext-boost/serialization.h>
#include <util/util.h>

#include <vector>
#include <string>

namespace sg {
namespace core {

void Centroid::recordVerb(const VerbCluster* v, float weight) {
  if (verbWeights.find(v->id) == verbWeights.end()) {
    verbWeights[v->id] = 0.0f;
  }
  verbWeights[v->id] += weight;
}

const std::vector< Centroid >& CentroidDatabase::filteredCentroids(const VerbCluster& v) const {
  if (_filteredCentroids.count(v.id) == 0) {
    MLIB_ASSERT_STR(false, "No centroids available for: " + v.id);
    return _centroids;
  } else {
    return _filteredCentroids.find(v.id)->second;
  }
}

double CentroidDatabase::dist(const std::vector<double> &transformedFeaturesA, const std::vector<double> &transformedFeaturesB)
{
    double sum = 0.0;
    for(UINT dimensionIndex = 0; dimensionIndex < transformedFeaturesA.size(); dimensionIndex++)
    {
        double diff = transformedFeaturesA[dimensionIndex] - transformedFeaturesB[dimensionIndex];
        sum += diff * diff;
    }
    return sqrt(sum);
}

void CentroidDatabase::computeSegmentFeatures(const sg::segmentation::MeshSegment& seg) const
{
    SegmentFeatureGeneratorBasic segGenerator;
    segGenerator.generate(seg);
}

double CentroidDatabase::dist(const Centroid& c, const sg::segmentation::MeshSegment& seg) const {
  computeSegmentFeatures(seg);
  return dist(c.transformedFeatures, seg.transformedFeatureCache);
}

static const unsigned int centroidJointCount = 14;
static const unsigned int centroidJoints[centroidJointCount] = {
    Skeleton::JointType_SpineBase,
    Skeleton::JointType_SpineMid,
    Skeleton::JointType_Neck,
    Skeleton::JointType_Head,
    Skeleton::JointType_ShoulderLeft,
    Skeleton::JointType_ElbowLeft,
    Skeleton::JointType_HandLeft,
    Skeleton::JointType_ShoulderRight,
    Skeleton::JointType_ElbowRight,
    Skeleton::JointType_HandRight,
    Skeleton::JointType_KneeLeft,
    Skeleton::JointType_FootLeft,
    Skeleton::JointType_KneeRight,
    Skeleton::JointType_FootRight
};

//void CentroidDatabase::computeTransforms(const SceneDatabase &scenes)
//{
//    SegmentFeatureGeneratorBasic segGenerator;
//
//    std::vector<const Scene*> allScenes;
//    const std::vector<std::string> sceneIds = scenes.sceneIds();
//    for (const std::string& sceneId : sceneIds) {
//      allScenes.push_back(&scenes.getScene(sceneId));
//    }
//
//    std::vector< ml::MathVector<double> > segments;
//    
//    /*const UINT segmentCount = 2000;
//    for(UINT iter = 0; iter < segmentCount; iter++)
//    {
//        const Scene* randScene = sg::util::DEF_RAND(allScenes);
//        const MeshSegment &seg = *sg::util::DEF_RAND(randScene->segments);*/
//
//    // the vast majority of clusters are too small, which really skews our median-based renormalization.
//    // to correct this, only for the purpose of computing the normalization transform, we are more aggressive
//    // in culling small segments.
//    const float clusteringDiagonalCutoff = 0.25f;
//
//    for(const Scene* scene : allScenes)
//        for(const MeshSegment *seg : scene->segments)
//        {
//            if(seg->diagonalLength() > clusteringDiagonalCutoff)
//            {
//                ml::MathVector<double> features;
//                for(double f : segGenerator.generate(*seg))
//                    features.push_back(f);
//                segments.push_back(features);
//            }
//        }
//
//    computeTransforms(segments);
//}

std::set<const MeshSegment*> CentroidDatabase::accumulateJointSegments(const Database& database, UINT jointIndex)
{
    std::map<const MeshSegment*, float> segmentUsage;
    const std::vector<std::string> recIds = database.recordings.recordingIds();
    for (const std::string& recId : recIds) {
      const Recording& rec = database.recordings.getRecording(recId, true, true);
      for (const Interaction* i : rec.interactions) {
        for (const auto& skelSegs : i->jointSegments)
          for (const auto& seg : skelSegs[jointIndex]) {
            if (segmentUsage.find(seg) == segmentUsage.end())
            { segmentUsage[seg] = 0.0f; }
            segmentUsage[seg] += 1.0f;
          }
      }
    }



    std::set<const MeshSegment*> result;

    const float segmentUsageCutoff = database.params->get<float>("Clustering.segmentUsageCutoff") / database.params->get<UINT>("Interaction.skeletonSkip");
    
    for(const auto &p : segmentUsage)
        if(p.second >= segmentUsageCutoff)
            result.insert(p.first);

    return result;
}

std::vector< ml::MathVector<double> > CentroidDatabase::makeTransformedFeatureVector(const Database &database, const std::set<const MeshSegment*> &segs)
{
    std::vector< ml::MathVector<double> > result;
    SegmentFeatureGeneratorBasic segGenerator;
    for(const MeshSegment *seg : segs)
    {
        ml::MathVector<double> features;
        for(double f : segGenerator.generate(*seg))
            features.push_back(f);
        database.transformSegFeaturesInPlace(features);
        result.push_back(features);
    }
    return result;
}

void CentroidDatabase::create(const Database &database)
{
    std::cout << "computing centroids" << std::endl;

    //_parent = &database;

    // TODO: parameterize
    const UINT maxIterations = 1000; // this doesn't really matter and I think we always converge in ~10 iterations

    const UINT maxClusterCount = database.params->get<UINT>("Clustering.maxClusterCount");
    const float clusterSizeFactor = database.params->get<float>("Clustering.clusterSizeFactor");
    const float verbFalloffA = database.params->get<float>("Clustering.verbFalloffA");
    const float verbFalloffB = database.params->get<float>("Clustering.verbFalloffB");
    const float centroidActivationSigma = database.params->get<float>("Clustering.centroidActivationSigma");
    const float verbFilterCutoff = database.params->get<float>("Clustering.verbFilterCutoff") / database.params->get<UINT>("Interaction.skeletonSkip");

    SegmentFeatureGeneratorBasic segGenerator;

    /*
    const bool investigateClusterCount = false;
    if(investigateClusterCount)
    {
        std::ofstream file("clusterCount.csv");
        
        const UINT testClusterMax = 20, testClusterSkip = 1;

        file << "joint name";
        for(UINT testClusterCount = 1; testClusterCount <= testClusterMax; testClusterCount += testClusterSkip)
            file << "," << testClusterCount;
        file << ",segment count";
        file << std::endl;

        for(UINT jointIndex : centroidJoints)
        {
            std::set<const MeshSegment*> activeSegments = accumulateJointSegments(jointIndex);
            std::vector< ml::MathVector<double> > segmentFeatures = makeTransformedFeatureVector(activeSegments);
            
            std::cout << "Joint: " << Skeleton::jointName(jointIndex) << ", " << segmentFeatures.size() << " segments" << std::endl;
            file << Skeleton::jointName(jointIndex);

            for(UINT testClusterCount = 1; testClusterCount <= testClusterMax; testClusterCount += testClusterSkip)
            {
                ml::KMeansClustering< ml::MathVector<double>, ml::MathVectorKMeansMetric<double> > clustering;
                clustering.cluster(segmentFeatures, testClusterCount, maxIterations, false, 0.0f);

                float quantizationError = clustering.quantizationError(segmentFeatures);
                //std::cout << testClusterCount << " clusters, error = " << quantizationError << std::endl;
                file << "," << quantizationError;
            }
            file << "," << segmentFeatures.size();
            file << std::endl;
        }
    }
    */

    for(UINT jointIndex : centroidJoints)
    {
        std::set<const MeshSegment*> activeSegments = accumulateJointSegments(database, jointIndex);
        std::vector< ml::MathVector<double> > segmentFeatures = makeTransformedFeatureVector(database, activeSegments);
        
        const UINT clusterCount = ml::math::clamp((int)ml::math::round(segmentFeatures.size() * clusterSizeFactor), 2, (int)maxClusterCount);

        std::cout << "clustering " << Skeleton::jointName(jointIndex) << ", " << segmentFeatures.size() << " segments, " << clusterCount << " clusters" << std::endl;

        ml::KMeansClustering< ml::MathVector<double>, ml::MathVectorKMeansMetric<double> > clustering;
        clustering.cluster(segmentFeatures, clusterCount, maxIterations, false, 0.0f);

        UINT centroidBaseIndex = (UINT)_centroids.size();
        for(UINT clusterIndex = 0; clusterIndex < clusterCount; clusterIndex++)
        {
            Centroid c;
            c.jointIndex = jointIndex;
            c.id = "c" + std::to_string(clusterIndex);
            c.transformedFeatures = clustering.clusterCenter(clusterIndex);
            _centroids.push_back(c);
        }

        const std::vector<std::string> recIds = database.recordings.recordingIds();
        for (const std::string& recId : recIds) {
          const Recording& rec = database.recordings.getRecording(recId, true, true);
          for (const Interaction* i : rec.interactions) {
            for (const auto& skelSegs : i->jointSegments) {
              for (const auto& seg : skelSegs[jointIndex]) {
                ml::MathVector<double> features;
                for (double f : segGenerator.generate(*seg))
                { features.push_back(f); }
                database.transformSegFeaturesInPlace(features);
                std::vector< std::pair<UINT, float> > distances = clustering.rankClusterDistances(features);

                _centroids[centroidBaseIndex + distances[0].first].recordVerb(i->verb, 1.0f);
                if (distances.size() >= 2) { _centroids[centroidBaseIndex + distances[1].first].recordVerb(i->verb, verbFalloffA); }
                if (distances.size() >= 3) { _centroids[centroidBaseIndex + distances[2].first].recordVerb(i->verb, verbFalloffB); }
              }
            }
          }
        }
    }

    // TODO: consider using a non-uniform radius, based on the falloff of inter-cluster distance
    for(auto &c : _centroids)
        c.activationSigma = centroidActivationSigma;

    std::ofstream file("centroidDescription.csv");
    for(const Centroid &c : _centroids)
    {
        file << c.id << "," << Skeleton::jointName(c.jointIndex) << "," << c.activationSigma;
        for(const auto &v : c.verbWeights)
            file << "," << v.first << "=" << v.second;
        file << std::endl;
    }

    //
    // fill _filteredCentroids
    //

    for(const Centroid &c : _centroids)
        for(const auto &v : c.verbWeights)
            if(v.second > verbFilterCutoff)
                _filteredCentroids[v.first].push_back(c);
}

bool CentroidDatabase::save(const std::string &filename) const {
  std::ofstream ofs(filename, std::ios::binary);
  { boost::archive::binary_oarchive(ofs) << *this; }
  ofs.close();
  std::cout << "Saved CentroidDatabase to " << filename << std::endl;
  return true;
}

bool CentroidDatabase::load(const Database& database, const std::string &filename) {
  if (!ml::util::fileExists(filename)) { return false; }
  std::ifstream ifs(filename, std::ios::binary);
  { boost::archive::binary_iarchive(ifs) >> *this; }
  ifs.close();
  std::cout << "Loaded CentroidDatabase from " << filename << std::endl;
  return true;
}

std::vector<std::vector<double>> CentroidDatabase::loadSegmentCentroids(const std::string& csvFile) {
  std::vector<std::vector<double>> centroids;
  const auto& lines = ml::util::getFileLines(csvFile);
  for (int i = 1; i < lines.size(); i++) {  // Skip header
    const std::string& l = lines[i];
    const auto& tokens = ml::util::split(l, ',');
    std::vector<double> feats;
    for (int j = 2; j < tokens.size(); j++) {  // Assumes first two columns are not numeric
      feats.push_back(std::stod(tokens[j]));
    }
    centroids.push_back(feats);
  }
  return centroids;
}

void CentroidDatabase::createCentroidSetWithWekaKmeans(const Database& database, const size_t numCentroids) {
  // Get all activated segments and construct segment features vector
  SegmentFeatureGeneratorSimplistic gen;
  const std::map<const MeshSegment*, std::vector<double>> segs = database.precomputeSegmentFeatures(gen);
  const size_t numFeats = gen.numFeatures();
  const size_t numSegs = segs.size();
  std::vector<double> segFeats(numSegs * numFeats);
  size_t featIdx = 0;
  for (const auto& segWithFeats : segs) {
    const std::vector<double>& feats = segWithFeats.second;
    std::copy(feats.begin(), feats.end(), &segFeats[featIdx]);
    featIdx += numFeats;
  }

  std::cout << "Extracting centroids...";
  const sg::stats::CentroidSet C = sg::stats::unsupervisedFeatureLearning(segFeats, numSegs, numCentroids);
  _centroidSet = C;
  std::cout << " Done." << std::endl;
}

std::vector<double> CentroidDatabase::computeCentroidActivations(const std::vector<double>& segFeats, const size_t numSegs) const {
  SegmentFeatureGeneratorSimplistic gen;
  assert(segFeats.size() == numSegs * gen.numFeatures());

  std::cout << "Computing activations...";
  const std::vector<double> activations = _centroidSet.computeActivation(segFeats, numSegs);
  std::cout << " Done." << std::endl;
  return activations;
}

}  // namespace core
}  // namespace sg
