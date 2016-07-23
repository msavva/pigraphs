#pragma once

#include "libsg.h"  // NOLINT

namespace sg {
namespace core {
namespace synth {

// Scores a pose
class PoseScorer {
public:
  string name;

  virtual ~PoseScorer() { }
  virtual double score(const TransformedSkeleton& skel) const = 0;
};

struct PoseSearchParams {
  explicit PoseSearchParams(const sg::util::Params& p);
  int numSamples;
  float binSize;
  float secondStageThresh;
  int numThetas;
};

void makeHeatMap(const PoseScorer& scorer, 
                 const PoseSearchParams& poseSearchParams,
                 const vec<TransformedSkeleton>& skels,
                 const geo::BBox2f& searchBBox,
                 sg::vis::PoseHeatMap* pHeatmap); 

}  // synth
}  // namespace core
}  // namespace sg
