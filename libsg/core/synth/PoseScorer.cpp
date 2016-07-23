#include "common.h"  // NOLINT

#include "core/Skeleton.h"
#include "core/SkelPoseSampler.h"
#include "core/synth/PoseScorer.h"
#include "vis/HeatMap.h"

using sg::core::SkelPoseSampler;
using sg::vis::PoseHeatMap;

namespace sg {
namespace core {
namespace synth {

PoseSearchParams::PoseSearchParams(const sg::util::Params& p) {
  numSamples            = p.get<int>("ISynth.searchSampleCount");
  binSize               = p.get<float>("ISynth.searchSampleBinSize");
  secondStageThresh     = p.get<float>("ISynth.secondStageThresh");
  numThetas             = p.get<int>("ISynth.searchThetaDivisions");
}

// Code extracted from SceneGrokker makeHeatMap
void makeHeatMap(const PoseScorer& scorer, 
                 const PoseSearchParams& poseSearchParams,
                 const vec<TransformedSkeleton>& skels,
                 const geo::BBox2f& searchBBox,
                 PoseHeatMap* pHeatmap) {
  SG_LOG_INFO << "[PoseScorer] makeHeatMap using " << scorer.name << "...";
  pHeatmap->setSkeletons(skels);
  pHeatmap->clear();

  // Parameters
  const int   numSamples            = poseSearchParams.numSamples;
  const float binSize               = poseSearchParams.binSize;
  const float secondStageThresh     = poseSearchParams.secondStageThresh;
  const int   numThetas             = poseSearchParams.numThetas;
  const int   numSkels              = static_cast<int>(skels.size());

  // Helper func to sample a given subdomain bbox
  const auto sampleBBox = [&](const geo::BBox2f& bbox, int numPoints,
                              vec<PoseHeatMap::Point>* pPoints,
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
      const double score = scorer.score(tSkel);
      (*pPoints)[params.iPoint].likelihoods[params.iPose][params.iTheta] += score;
    }
  };

  // Get scan BBox and sample first pass
  vec<PoseHeatMap::Point> samplePoints;
  vec<float> sampledThetas;
  sampleBBox(searchBBox, numSamples, &samplePoints, &sampledThetas);
  pHeatmap->addPoints(samplePoints, sampledThetas);

  // If 2nd stage sampling is enabled, find top subdomains and sample again
  if (secondStageThresh > 0) {
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

}  // namespace synth
}  // namespace core
}  // namespace sg
