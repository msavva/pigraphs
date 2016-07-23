#include "common.h"  // NOLINT

#include "core/synth/SkeletonPoser.h"

#include "core/ClassifierDatabase.h"
#include "core/Database.h"
#include "core/PoseController.h"
#include "core/SkeletonDatabase.h"
#include "core/SceneGrokker.h"
#include "core/synth/InteractionScorer.h"
#include "core/synth/ModelPlacer.h"
#include "core/synth/PoseScorer.h"
#include "geo/voxels.h"

#include "ui/ErrorCode.h"
#include "vis/Heatmap.h"

//#include <boost/generator_iterator.hpp>

namespace sg {
namespace core {
namespace synth {

class InteractionPoseScorer : public PoseScorer {
public:
  InteractionPoseScorer(const InteractionScorer& iscorer,
                        const interaction::InteractionSet& iset,
                        const vec<ModelInstance>& mInsts)
    : m_iscorer(iscorer), m_iset(iset), m_mInsts(mInsts) {
    name = iset.id;
  }
  double score(const Skeleton& skel) const {
    return m_iscorer.scoreSkeleton(m_iset, m_mInsts, skel, false);
  }
  double score(const TransformedSkeleton& skel) const override {
    return m_iscorer.scoreSkeleton(m_iset, m_mInsts, skel.getSkeleton(), false);
  }
  double score(const TransformedSkeleton& skel, bool includeSkelScore,
               const boost::optional<double> baseSkelLP = boost::none) const {
    return m_iscorer.scoreSkeleton(m_iset, m_mInsts, skel.getSkeleton(), includeSkelScore);
  }
  const InteractionScorer& getInteractionScorer() const {
    return m_iscorer;
  }
  const interaction::InteractionSet& getInteractionSet() const {
    return m_iset;
  }

private:
  const InteractionScorer& m_iscorer;
  const interaction::InteractionSet& m_iset;
  const vec<ModelInstance>& m_mInsts;
};

void SkeletonPoserParams::init(const util::Params& p) {
  beamSize = p.get<int>("SkeletonPoser.beamSize");
  samplePositions = p.get<bool>("SkeletonPoser.samplePositions");
  sampleJointAngles = p.get<bool>("SkeletonPoser.sampleJointAngles");
  optimizeJointAngles = p.get<bool>("SkeletonPoser.optimizeJointAngles");
}

void extendBBox(const ModelInstance& mInst,
               float bboxPad, geo::BBox* pBBox) {
    geo::BBox mBBox = mInst.computeWorldBBox();
    geo::Vec3f v(bboxPad, bboxPad, bboxPad);
    mBBox.max() += v;
    mBBox.min() -= v;
    pBBox->extend(mBBox);
}

void extendBBox(const vec<ModelInstance>& mInsts,
                float bboxPad, geo::BBox* pBBox) {
  for (const ModelInstance& m : mInsts) {
    extendBBox(m, bboxPad, pBBox);
  }
}


//! Poses the skeleton using the passed in params
int SkeletonPoser::pose(const SkeletonPoserParams& params,
                        vec<TransformedSkeleton>* pPosedSkels) const {
  SG_LOG_SCOPE("SkelPoser");
  // Do some initial parameter checking
  if (pPosedSkels != nullptr) {
    // Clear the posed skeletons
    pPosedSkels->clear();
  } else {
    SG_LOG_ERROR << "Cannot pose: pPosedSkels is null";
    return ui::ErrorCodes::E_INVALID_ARGS;
  }

  // Do posing!!!
  int res = getBaseSkeletons(params, pPosedSkels);
  return res;
}

//! Poses the skeleton using the passed in params
//! Outputs the posed skeletons 
int SkeletonPoser::poseForScan(const SkeletonPoserParams& params,
                               vec<TransformedSkeleton>* pPosedSkels, vis::PoseHeatMap* pHeatMap) const {
  SG_LOG_SCOPE("SkelPoser");
  // Do some initial parameter checking
  if (pPosedSkels != nullptr) {
    // Clear the posed skeletons
    pPosedSkels->clear();
  } else {
    SG_LOG_ERROR << "Cannot pose: pPosedSkels is null";
    return ui::ErrorCodes::E_INVALID_ARGS;
  }
  if (pHeatMap == nullptr) {
    SG_LOG_ERROR << "Cannot pose: pHeatMap is null";
    return ui::ErrorCodes::E_INVALID_ARGS;
  }
  if (params.pISetGroup == nullptr) {
    SG_LOG_ERROR << "Cannot pose: no interaction set group";
    return ui::ErrorCodes::E_INVALID_ARGS;
  }
  if (params.pScan == nullptr) {
    SG_LOG_ERROR << "Cannot pose: no scene";
    return ui::ErrorCodes::E_INVALID_ARGS;
  }

  // Do posing!!!
  // What classifier to use?
  if (params.useScenePoseClassifier) {
    if (params.pISetGroup->pInteractionSet == nullptr) {
      SG_LOG_ERROR << "Cannot pose: no interaction set";
      return ui::ErrorCodes::E_INVALID_ARGS;
    }
    // Use SceneGrokker with the scan pose classifier
    SceneGrokker grokker(*m_pDatabase, *params.pScan);
    const interaction::InteractionSet& interactionSet = *params.pISetGroup->pInteractionSet;
    const ScenePoseClassifier* pClassifier = 
      m_pDatabase->classifiers->getScenePoseClassifier(params.classifierType, interactionSet);

    // Get base skeletons (poses) that we will use
    vec<TransformedSkeleton> baseSkels;
    getBaseSkeletons(params, &baseSkels);
    // Compute heapmap over positions in the scan for the skeletons
    grokker.makeHeatMap(*pClassifier, baseSkels, pHeatMap);
    if (params.pVisualizeHeatMapFn != nullptr) {
      (*params.pVisualizeHeatMapFn)(*pHeatMap);
    }

    // Get the topK most likely skeletons from the heapmap
    pHeatMap->getTopKSkels(pPosedSkels, params.topK);
    return static_cast<int>(pPosedSkels->size());
  } else {
    // TODO: Implement something here!!!
    SG_LOG_ERROR << "Cannot pose: useScenePoseClassifier=false is unsupported";
    return ui::ErrorCodes::E_UNSUPPORTED;
  }
}

typedef std::function<double(int)> scoreFun;
vec<size_t> greedyOptimizer(const vec<vec<int>>& vs, const scoreFun& eval) {
  vec<size_t> choices(vs.size());
  for (int iChoice = 0; iChoice < vs.size(); ++iChoice) {
    choices[iChoice] = util::argmax(vs[iChoice], eval);
  }
  return choices;
}

double greedyOptimizeJointAngles(const InteractionPoseScorer& scorer,
                                 const TransformedSkeleton& initialSkel,
                                 TransformedSkeleton* outSkel,
                                 vec<float>* pOutControlAngles = nullptr) {
  *outSkel = initialSkel;
  const auto& iset = scorer.getInteractionSet();
  const auto& iscorer = scorer.getInteractionScorer();
  // create set of angles choices for each control joint
  const float a = math::constants::PIf / 32;
  vec<vec<float>> angles(kNumControlJoints, {-4*a,-3*a,-2*a,-a, 0,
                                                a, 2*a, 3*a, 4*a});
  if (pOutControlAngles) {
    pOutControlAngles->resize(kNumControlJoints);
    std::fill(pOutControlAngles->begin(), pOutControlAngles->end(), 0.f);
  }

  // go over angles in order and pick argmax for each greedily
  for (int iDim = 6; iDim < angles.size(); ++iDim) {
    const vec<float>& iDimAngles = angles[iDim];  // angles for this control joint
    const int iJoint = kControlJoints[iDim];  // corresponding joint index
    const size_t iDimAngMaxIdx = util::argmax(iDimAngles, [&](float ang) {
      outSkel->setJointAngleDelta(iJoint, ang);
      const double score = scorer.score(*outSkel, true)
        + iscorer.scoreSkeletonJoint(iset, *outSkel, iJoint);
      outSkel->score = score;
      SG_LOG_DEBUG << "SCORE:" << score;
      return score;
    });
    outSkel->setJointAngleDelta(iJoint, iDimAngles[iDimAngMaxIdx]);
    if (pOutControlAngles) {
      (*pOutControlAngles)[iDim] = iDimAngles[iDimAngMaxIdx];
    }
  }

  return outSkel->score;
}

class JointAnglesGenerator {
public:
  typedef vec<float> result_type;
  JointAnglesGenerator()
    : angleValues(kNumControlJoints, {-4*a,-3*a,-2*a,-a, 0,
                                       a, 2*a, 3*a, 4*a})
    , angles(kNumControlJoints, 0) {}
  result_type operator()() {
    if (n > customValues.size()) {
      // Pick one angle and randomly tweak it
      int iDim = math::DEF_RAND.uniform_int(6, kNumControlJoints);
      const vec<float>& iDimAngles = angleValues[iDim];  // angles for this control joint
      angles[iDim] = *math::DEF_RAND.select(iDimAngles.begin(), iDimAngles.end());
      n++;
    } else {
      return customValues[n];
    }
    return angles; 
  }

  // create set of angles choices for each control joint
  const float a = math::constants::PIf / 32;
  size_t n = 0;
  vec<vec<float>> angleValues;
  result_type angles;
  vec<result_type> customValues;
};

int sampleGoodJointAngles(const InteractionPoseScorer& scorer,
                          const TransformedSkeleton& initialSkel,
                          int k,
                          vec<TransformedSkeleton>* pOut) {
  TransformedSkeleton tSkel = initialSkel;
  const auto& iset = scorer.getInteractionSet();
  const auto& iscorer = scorer.getInteractionScorer();

  // Start from current skeleton and maintain 
  // a list of K top transformed skeletons
  JointAnglesGenerator gen;
  //boost::generator_iterator_generator<JointAnglesGenerator>::type it 
  //  = boost::make_generator_iterator(gen);

  double baseSkelLP = iscorer.getSkeletonLogProb(iset, initialSkel.getSkeleton());

  // Scoring function
  const std::function<double(const vec<float>&)> scoreFn = 
    [&](const vec<float>& angs) {
      for (int iDim = 0; iDim < angs.size(); ++iDim) {
        const int iJoint = kControlJoints[iDim];  // corresponding joint index
        tSkel.setJointAngleDelta(iJoint, angs[iJoint]);
      }
      const double score = scorer.score(tSkel, true, baseSkelLP);
      tSkel.score = score;
      SG_LOG_DEBUG << "SCORE:" << score;
      return score;
  };

  // Make sure to include the current one and the greedy best too!
  vec<float> zeros(kNumControlJoints, 0.f);
  vec<float> greedyBestControlAngles;
  gen.customValues.push_back(zeros);
  TransformedSkeleton greedyBestSkel = initialSkel;
  double greedyScore = greedyOptimizeJointAngles(scorer, initialSkel, &greedyBestSkel, &greedyBestControlAngles);
  gen.customValues.push_back(greedyBestControlAngles);

  vec<pair<vec<float>, double>> topAngles;
  sg::util::getTopK(gen, scoreFn, k, math::constants::NEGINF, 100, &topAngles);

  // Go over top k and make them into transformed skeletons
  for (const auto&p : topAngles) {
    pOut->emplace_back(initialSkel);
    for (int iDim = 0; iDim < p.first.size(); ++iDim) {
      const int iJoint = kControlJoints[iDim];  // corresponding joint index
      pOut->back().setJointAngleDelta(iJoint, p.first[iJoint]);
    }
    pOut->back().score = p.second;
  }

  return static_cast<int>(topAngles.size());
}

int SkeletonPoser::adjust(const SkeletonPoserParams& params, 
                          const TransformedSkeleton& initialSkel, 
                          const vec<ModelInstance>& modelInstances, 
                          vec<TransformedSkeleton>* pPosedSkels) const {
  SG_LOG_SCOPE("SkelPoser");
  if (params.pPlacementConstraints == nullptr) {
    SG_LOG_ERROR << "Adjust not implemented when placement constraints is not provided!!";
    return 0;    
  }
  const SupportHierarchy& supportHierarchy = params.pPlacementConstraints->supportHierarchy;
  const ObjectJointInteractions& objJointInteractions = params.pPlacementConstraints->objectJointInteractions;
  Skeleton skel = initialSkel.getSkeleton();
  sg::vis::PoseHeatMap heatMap;

  InteractionScorer scorer;
  scorer.init(m_pDatabase, params.iscorerParams);

  vec<TransformedSkeleton> baseSkels;
  TransformedSkeleton unfloatedSkel = initialSkel;
  geo::BBox searchBBox; 
  float bboxPad = 0.1f; // pad bounding box by a bit
  vec<ModelInstance> scoredModelInstances;
  for (const ModelInstance& m : modelInstances) {
    if (m.placementScore) {
      scoredModelInstances.push_back(m);
    }
  }
  InteractionPoseScorer poseScorer(scorer, *params.pISetGroup->pInteractionSet, scoredModelInstances);

  if (objJointInteractions.iSupportObj >= 0) {
    SG_LOG_INFO << "Unfloat skeleton wrt to support object " << objJointInteractions.iSupportObj;
    const ModelInstance& supportModelInst = modelInstances.at(objJointInteractions.iSupportObj);
    float f = unfloatSkel(supportModelInst, &skel, objJointInteractions.isStanding);
    if (f != 0) {
      unfloatedSkel = TransformedSkeleton(std::make_shared<Skeleton>(skel),  ml::mat4f::identity(), initialSkel.score);
      baseSkels.push_back(unfloatedSkel.getNormalized());
      if (!params.samplePositions && !params.optimizeJointAngles) {
        pPosedSkels->push_back(unfloatedSkel);
      }
    }
    // Try to make sure skeleton stays in bound
    extendBBox(supportModelInst, -bboxPad, &searchBBox);
  } else {
    extendBBox(scoredModelInstances, bboxPad, &searchBBox);
  }
  baseSkels.push_back(unfloatedSkel.getNormalized());

  // search for joint angles optimizing pose score
  if (params.optimizeJointAngles) {
    SG_LOG_INFO << "OptimizeJointAngles";
    TransformedSkeleton tOpt;
    double score = greedyOptimizeJointAngles(poseScorer, unfloatedSkel, &tOpt);
    pPosedSkels->push_back(tOpt);
    for (int i = 0; i < kNumControlJoints; ++i) {
      const int iJ = kControlJoints[i];
      SG_LOG_INFO << Skeleton::kJointNames[iJ] << ":" << tOpt.getJointAngleDeltas()[iJ] << endl;
    }
    baseSkels.push_back(tOpt.getNormalized());
  } else if (params.sampleJointAngles) {
    SG_LOG_INFO << "SampleJointAngles";
    vec<TransformedSkeleton> okaySkels;
    int rv = sampleGoodJointAngles(poseScorer, unfloatedSkel, 10, &okaySkels);
    if (rv > 0) {
      for (size_t i = 0; i < okaySkels.size(); i++) {
        pPosedSkels->push_back(okaySkels[i]);
        baseSkels.push_back(okaySkels[i].getNormalized());
      }
    }
  }

  if (params.samplePositions) {
    SG_LOG_INFO << "SamplePositions";
    // Compute heapmap over positions in the scan for the skeletons
    geo::BBox2f searchBBox2;
    const auto maxPt = searchBBox.max(), minPt = searchBBox.min();
    searchBBox2.extend(geo::Vec2f(maxPt[0], maxPt[1]));
    searchBBox2.extend(geo::Vec2f(minPt[0], minPt[1]));
    SG_LOG_DEBUG << "searchBBox: " 
      << searchBBox.min().format(geo::kEigenJSONFormat) << ", "
      << searchBBox.max().format(geo::kEigenJSONFormat);
    SG_LOG_DEBUG << "searchBBox2: "
      << searchBBox2.min().format(geo::kEigenJSONFormat) << ", "
      << searchBBox2.max().format(geo::kEigenJSONFormat);
    PoseSearchParams poseSearchParams(*m_pDatabase->params);
    makeHeatMap(poseScorer, poseSearchParams, baseSkels, searchBBox2, &heatMap);
    if (params.pVisualizeHeatMapFn != nullptr) {
      (*params.pVisualizeHeatMapFn)(heatMap);
    }

    // Get the topK most likely skeletons from the heapmap
    vec<TransformedSkeleton> repositionedSkels;
    int rv = heatMap.getTopKSkels(&repositionedSkels, params.topK);
    // Combine and sort and get top K?
    if (rv > 0) {
      for (size_t i = 0; i < repositionedSkels.size(); i++) {
        pPosedSkels->push_back(repositionedSkels[i]);
      }
    }
  }

  // Sort and discard extras
  std::sort(pPosedSkels->begin(), pPosedSkels->end(), 
            [](const TransformedSkeleton& t1, const TransformedSkeleton& t2)
  {
    return t1.score > t2.score;
  });
  // Trim to limit
  if (pPosedSkels->size() > params.topK) {
    pPosedSkels->resize(params.topK);
  }
  return static_cast<int>(pPosedSkels->size());
}

int SkeletonPoser::getBaseSkeletons(const SkeletonPoserParams& params,
                                    vec<TransformedSkeleton>* pBaseSkels) const {
  // Get base skeletons (poses) that we will use
  if (params.pSkels != nullptr) {
    // We have skeletons that were passed in, the caller must really want us to use those
    int n = 0;
    for (const TransformedSkeleton& tSkel : *params.pSkels) {
      pBaseSkels->push_back(tSkel);
      n++;
    }
    return n;
  } else {
    // We don't have any skeletons, lets try to come up with some on our own      
    // TODO: Have a more intelligent way to make some skeletons!!!
    //for (const TransformedSkeleton& tSkel : params.pISetGroup->pInteractionSet->sampledSkeletons) {
    //  pBaseSkels->push_back(tSkel);
    //}

    string isetId = "";
    if (params.pISetGroup != nullptr) {
      if (params.pISetGroup->pInteractionSet != nullptr && m_pDatabase) {
        isetId = params.pISetGroup->pInteractionSet->id;
      }
    } 
    vec<pair<Skeleton,double>> scoredSkels;
    if (!isetId.empty()) {
      selectSkeletons(params.selectSkelStrategy, isetId, params.beamSize, &scoredSkels);
    }
    if (scoredSkels.empty()) {
      if (params.pISetGroup != nullptr) {
        SG_LOG_WARN << "Cannot find any poses for " << params.pISetGroup->id;
        // Sample pose basic pose
        const auto& isets = m_pDatabase->interactions.getInteractionSets(interaction::InteractionSetType::ISetType_Verb);
        const string& vsetId = interaction::InteractionSet::getVerbISetId(params.pISetGroup->basicPose);
        if (isets.count(vsetId) > 0) {
          const auto& basicPoseIset = isets.at(vsetId).get();
          selectSkeletons(params.selectSkelStrategy, vsetId, params.beamSize, &scoredSkels);
          if (!scoredSkels.empty()) {
            SG_LOG_INFO << "Using pose from basic pose " << basicPoseIset.id;
          } else {
            SG_LOG_WARN << "No poses to sample from for " << basicPoseIset.id;
          }
        } else {
          SG_LOG_WARN << "Cannot pose skeleton: No basic pose identified";
        }
      }
    }
    if (scoredSkels.empty()) {
      // Still empty
      SG_LOG_INFO << "Selecting skeletons from all isets";
      selectSkeletons(params.selectSkelStrategy, "", params.beamSize, &scoredSkels);
    }
    int nSkels = static_cast<int>(scoredSkels.size());
    for (auto& p : scoredSkels) {
      // Make sure skeletons are all above ground
      ensureSkelAbove(0.0, &p.first);
      ml::mat4f xform = p.first.getNormalizingTransform();
      std::shared_ptr<const Skeleton> pSkel = std::make_shared<const Skeleton>(p.first);
      pBaseSkels->emplace_back(pSkel, xform, p.second); 
    }
    return nSkels;
  }
}

size_t SkeletonPoser::selectSkeletons(SelectSkelStrategy selectSkelStrategy,
                                      const string& isetId,
                                      size_t n,
                                      vec<pair<Skeleton,double>>* pScoredSkeletons) const {
  const SkeletonDatabase& skels = *m_pDatabase->skeletons;
  SG_LOG_INFO << "Selecting " << n << " skeletons for iset='" 
    << isetId << "' using select strategy " 
    << SelectSkelStrategyNames[static_cast<size_t>(selectSkelStrategy)];
  switch (selectSkelStrategy) {
    case SelectSkelStrategy::kFirst:
      // Use first n skeletons
      return skels.getFirstKObservedSkeletons(isetId, n, pScoredSkeletons);
    case SelectSkelStrategy::kRandom:
      // Use random observed skeleton
      return skels.getRandomObservedSkeletons(isetId, n, pScoredSkeletons);
    case SelectSkelStrategy::kAverage: {
        // Use average skeleton
        Skeleton skel;
        double score;
        bool ok = skels.getAverageSkeleton(isetId, &skel, &score);
        if (ok) {
          pScoredSkeletons->emplace_back(skel, score);
          return 1;
        } else {
          return 0;
        }
      }
    case SelectSkelStrategy::kSample:
      // Sample skeletons
      return skels.sampleSkeletons(isetId, n, pScoredSkeletons);
    default:
      SG_LOG_WARN << "Unknown SelectSkelStrategy " << selectSkelStrategy;
      return 0;
  }
}

float minDistToSupport(const ModelInstance& mInst, 
                       const Skeleton& skel,
                       bool isStanding) {
  // Parameters for computing distance to support
  // TODO: These should be joint specific
  float supportRadius = 0.1f;    // radius of area of support in meters
  float supportVPadding = 0.05f; // padding away from support joint in meters
  float supportRadiusRatioThreshold = 0.6f;

  const ml::BinaryGrid3& solidVoxels = mInst.model.solidVoxels;
  const ml::BinaryGrid3& surfaceVoxels = mInst.model.surfaceVoxels;
  const auto& p = skel.jointPositions;
  const geo::Transform worldToGrid = 
    geo::from(mInst.model.modelToVoxel) * mInst.getParentToLocal();
  const geo::Vec3f up(0,0,1);

  float minDist = math::constants::POSINFf;
  const auto distFun = [&] (const Skeleton::JointType joint) {
    const geo::Vec3f jpos(p[joint]);
    const geo::Vec3f jpos_padded = jpos - geo::Vec3f(0,0,supportVPadding);
    boost::optional<geo::Vec3f> closest = 
      geo::findClosestSupportPoint(solidVoxels, surfaceVoxels,
        jpos_padded, worldToGrid, up, supportRadius, supportRadiusRatioThreshold);
    if (closest) {
      const geo::Vec3f d = jpos_padded - closest.get();
      float dZ = d.z();
      if (fabsf(dZ) < fabsf(minDist)) {
        minDist = dZ;
      }
    }
  };

  if (isStanding) {
    SG_LOG_INFO << "Unfloat using feet joints";
    for (const Skeleton::JointType joint : Skeleton::kFeetJoints) {
      distFun(joint);
    }
  } else {
    SG_LOG_INFO << "Unfloat using hip joints";
    supportVPadding = 0.1f;  //. Hips are large
    for (const Skeleton::JointType joint : Skeleton::kHipJoints) {
      distFun(joint);
    }
  }
  return minDist;
}

// Unfloat skeleton with respect to mInst
float unfloatSkel(const ModelInstance& mInst, Skeleton* pSkeleton, bool isStanding) {
  float zAdjustment = 0.0;
  float minDist = minDistToSupport(mInst, *pSkeleton, isStanding);
  if (isfinite(minDist)) {
    zAdjustment = -minDist;
    SG_LOG_INFO << "Unfloat dude with " << zAdjustment;
    pSkeleton->move(2, zAdjustment);
  }
  return zAdjustment;
}

// ensure skeleton is above ground
float ensureSkelAbove(const float groundHeight, Skeleton* pSkeleton) {
  float zAdjustment = 0.0;
  float minZ = math::constants::POSINFf;
  for (int iJoint = 0; iJoint < Skeleton::kNumJoints; ++iJoint) {
    const geo::Vec3f jpos(pSkeleton->jointPositions[iJoint]);
    minZ = std::min(minZ, jpos.z());
  }
  if (isfinite(minZ)) {
    zAdjustment = -minZ;
    SG_LOG_INFO << "Unfloat dude above ground with " << zAdjustment;
    pSkeleton->move(2, zAdjustment);
  }
  return zAdjustment;
}

}  // namespace synth
}  // namespace core
}  // namespace sg
