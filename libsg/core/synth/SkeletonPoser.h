#pragma once

#include "libsg.h"  // NOLINT
#include "core/synth/InteractionScorer.h"
#include "core/synth/synth.h"

namespace sg {
namespace core {
namespace synth {

struct PlacementConstraints;

//! Encapsulates parameters for Skeleton Poser
struct SkeletonPoserParams {
  typedef std::function<int(const vis::PoseHeatMap& heatMap)> VisualizeHeatMapFn;
  //! Initializes skeleton poser params with some reasonable defaults
  SkeletonPoserParams() = default;
  explicit SkeletonPoserParams(const Scan* _pScan)
    : pScan(_pScan) { }
  void init(const util::Params& p);
  //! Use scan pose classifier or not (currently only support true)
  bool useScenePoseClassifier = true;
  //! Type of classifier to use
  string classifierType = "IGKNNSim";
  //! Scene for which we want to pose the skeleton in (please set this!!!)
  const Scan* pScan = nullptr;
  //! Composite action (corresponding to a set of verb noun) and their related interaction sets
  const interaction::VerbNounISetGroup* pISetGroup = nullptr;
  //! If specified, pose with these skeletons
  const vec<TransformedSkeleton>* pSkels = nullptr;
  SelectSkelStrategy selectSkelStrategy = SelectSkelStrategy::kFirst;
  //! How many skeletons to retrieve
  int topK = 1;
  //! How many skeletons to keep on beam
  int beamSize = 10;
  //! Whether to sample positions
  bool samplePositions = false;
  //! Whether to sample joint angles
  bool sampleJointAngles = false;
  //! Whether to optimize skeleton joint angles
  bool optimizeJointAngles = false;
  //! Parameters to use for interaction scoring
  InteractionScorerParams iscorerParams;
  //! Additional placement constraints (both input and output)
  PlacementConstraints* pPlacementConstraints = nullptr;
  //! Callback for visualizing heatmap of where the skeleton should be
  VisualizeHeatMapFn* pVisualizeHeatMapFn = nullptr; 
};


//! Class for positioning and posing a skeleton for interaction
class SkeletonPoser {
 public:
  void init(Database* pDatabase) {
    m_pDatabase = pDatabase;
  }

  //! Given a set of verb noun pairs,
  //! find some likely skeletons, positioned at the origin
  //! Returns number of posed skeletons
  int pose(const SkeletonPoserParams& params,
           vec<TransformedSkeleton>* pPosedSkels) const;

  //! Given a scan and a set of verb noun pairs,
  //! find some likely skeletons and their positions
  //! Returns number of posed skeletons
  int poseForScan(const SkeletonPoserParams& params,
                  vec<TransformedSkeleton>* pPosedSkels,
                  vis::PoseHeatMap* pHeatMap) const;

  //! Adjust position and pose of the input skeleton 
  //! for the given objects and returns improved 
  //! skeletons
  int adjust(const SkeletonPoserParams& params,
             const TransformedSkeleton& skel,
             const vec<ModelInstance>& objects,
             vec<TransformedSkeleton>* pPosedSkels) const;

 private:
  //! Returns some base skeletons for further positioning and posing
  int getBaseSkeletons(const SkeletonPoserParams& params,
                       vec<TransformedSkeleton>* pBaseSkels) const;

  size_t selectSkeletons(SelectSkelStrategy selectSkelStrategy,
                         const string& isetId,
                         size_t n,
                         vec<pair<Skeleton,double>>* pScoredSkeletons) const;

  //! Database from which we get interaction set and such
  Database* m_pDatabase = nullptr;
};

float unfloatSkel(const ModelInstance& mInst, Skeleton* pSkeleton, bool isStanding);
float ensureSkelAbove(const float groundHeight, Skeleton* pSkeleton);

}  // namespace synth
}  // namespace core
}  // namespace sg
