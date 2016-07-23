#include "common.h"  // NOLINT

#include "core/synth/InteractionScorer.h"

#include "core/Database.h"
#include "core/ModelInstance.h"
#include "core/Skeleton.h"
#include "core/SkeletonDatabase.h"
#include "core/synth/ModelPlacer.h"
#include "core/synth/ObjectLabeler.h"
#include "core/synth/synth.h"
#include "geo/geo.h"
#include "geo/OBB.h"
#include "geo/voxels.h"
#include "interaction/InteractionFrames.h"
#include "interaction/InteractionSet.h"
#include "interaction/ProtoInteractionGraph.h"

using namespace sg::interaction;
using namespace sg::geo;

namespace sg {
namespace core {
namespace synth {

void getLinks(const ObjectLabeler& labeler,
              const ProtoInteractionGraph& pig,
              const map<string, ProtoInteractionGraph::edge>& pigEdges,
              CategoryPIGLinkMap* pLinksByCategory) {
  for (const auto it : pigEdges) {
    const auto& pigEdge = it.second;
    const ProtoInteractionLink& pigLink = pig.getEdgeBundle(pigEdge);
    const ProtoInteractionNode& pigSegNode = pig.getVertexBundle(pigEdge.m_target);
    const string label = pigSegNode.label;
    const string category = labeler.getCategory(label);
    (*pLinksByCategory)[category].push_back(pigLink);
  }
}

void ensurePigJointLinks(const ObjectLabeler& labeler,
                         const WeightedPIG& wpig) {
  if (wpig.jointLinksByCategory.empty()) {
    wpig.populateJointLinks([&](string s) { return labeler.getCategory(s); });
  }
}

void InteractionScorerParams::init(const util::Params& p) {
  useInteractionVolume = p.get<bool>("ISynth.useInteractionVolume");
  usePigScore = p.get<bool>("ISynth.usePigScore");
  usePigSimScore = p.get<bool>("ISynth.usePigSimScore");
  modelCollisionWeight = p.get<double>("ISynth.modelCollisionWeight");;
  modelSkeletonCollisionWeight = p.get<double>("ISynth.modelSkeletonCollisionWeight");;
  interactionWeight = p.get<double>("ISynth.interactionWeight");;
  posePriorWeight = p.get<double>("ISynth.posePriorWeight");
  skelSelfCollisionWeight = p.get<double>("ISynth.skelSelfCollisionWeight");
  numSkelPtSamples = p.get<int>("ISynth.numSkelSamples");
  useLog = p.get<bool>("ISynth.useLogScore");
}

void InteractionScorer::init(Database* db, const InteractionScorerParams& params) {
  m_pDatabase = db;
  m_params = params;
  m_overlapScorer = std::make_shared<OverlapScorer>(db, params);
  m_pigScorer = std::make_shared<PigScorer>(db, params);
  m_pigSimScorer = std::make_shared<PigSimScorer>(db, params);
  m_iframesScorer = std::make_shared<IFramesScorer>(db, params);
}

// Main scoring functions

// Interaction score (without model skeleton collisions)
double InteractionScorer::scoreModelSkelInteraction(const InteractionSet& iset,
                                                    const ModelInstance& mInst,
                                                    const Skeleton& mySkel) const {
//  SG_LOG_SCOPE("IScorer");
  SG_LOG_TAG("Channel", "IScorer");
  double scorePig = 0.0;
  if (m_params.usePigScore) {
    scorePig = m_pigScorer->score(iset, mInst, mySkel);
  }
  double scorePigSim = 0.0;
  if (m_params.usePigSimScore) {
    scorePigSim = m_pigSimScorer->score(iset, mInst, mySkel);
  }
  double scoreIVolume = 0.0;
  if (m_params.useInteractionVolume) {
    scoreIVolume = m_iframesScorer->score(iset, mInst, mySkel);
  }
  double interactionScore = scorePig + scorePigSim + scoreIVolume;
  SG_LOG_DEBUG << " interactionScore=" << interactionScore 
    << " pigScore=" << scorePig 
    << " pigSimScore=" << scorePigSim 
    << " ivScore=" << scoreIVolume 
    << " for " << mInst.category;
  return interactionScore;
} // scoreModelSkelInteraction (ISet)

// Interaction score (without model skeleton collisions)
double InteractionScorer::scoreModelSkelInteraction(const InteractionSet& iset,
                                                    const vec<ModelInstance>& mInsts,
                                                    const Skeleton& mySkel) const {
//  SG_LOG_SCOPE("IScorer");
  SG_LOG_TAG("Channel", "IScorer");
  double scorePig = 0.0;
  if (m_params.usePigScore) {
    scorePig = m_pigScorer->score(iset, mInsts, mySkel);
  }
  double scorePigSim = 0.0;
  if (m_params.usePigSimScore) {
    scorePigSim = m_pigSimScorer->score(iset, mInsts, mySkel);
  }
  double scoreIVolume = 0.0;
  if (m_params.useInteractionVolume) {
    scoreIVolume = m_iframesScorer->score(iset, mInsts, mySkel);
  }
  double interactionScore = scorePig + scorePigSim + scoreIVolume;
  SG_LOG_DEBUG << " interactionScore=" << interactionScore 
    << " pigScore=" << scorePig 
    << " pigSimScore=" << scorePigSim 
    << " ivScore=" << scoreIVolume; 
  return interactionScore;
} // scoreModelSkelInteraction (ISet)

//! Scores mInst against other placed objects and skeleton
double InteractionScorer::scoreModelInstance(const PlacementState& state,
                                             const InteractionSet& iset,
                                             const ModelInstance& mInst,
                                             const Skeleton& mySkel) const {
//  SG_LOG_SCOPE("IScorer");
  SG_LOG_TAG("Channel", "IScorer");
  double interactionScore = 0.0;
  if (m_params.interactionWeight > 0) {
    interactionScore = m_params.interactionWeight * scoreModelSkelInteraction(iset, mInst, mySkel);
  }
  double modelCollisionScore = 0.0;
  if (m_params.modelCollisionWeight > 0) {
    modelCollisionScore = m_params.modelCollisionWeight * m_overlapScorer->scoreModelOverlap(state, mInst);
  }
  double skelModelCollisionScore = 0.0;
  if (m_params.modelSkeletonCollisionWeight > 0) {
    skelModelCollisionScore = m_params.modelSkeletonCollisionWeight * m_overlapScorer->score(iset, mInst, mySkel);
  }
  double totalScore = interactionScore + modelCollisionScore + skelModelCollisionScore;
  SG_LOG_DEBUG << " Model " << mInst.category << ", iset " << iset.id
    << " totalScore=" << totalScore
    << " interactionScore=" << interactionScore 
    << " modelCollisionScore=" << modelCollisionScore 
    << " skelModelCollisionScore=" << skelModelCollisionScore; 
  return totalScore;
} // scoreModelInstance

double InteractionScorer::scoreSkeleton(const interaction::InteractionSet& iset, 
                                        const vec<ModelInstance>& mInsts,
                                        const Skeleton& mySkel, 
                                        bool includeSkelScore,
                                        const boost::optional<double> baseSkelLP /* = boost::none */) const
{
//  SG_LOG_SCOPE("IScorer");
  SG_LOG_TAG("Channel", "IScorer");
  double interactionScore = 0.0;
  if (m_params.interactionWeight > 0) {
    interactionScore = m_params.interactionWeight * scoreModelSkelInteraction(iset, mInsts, mySkel);
  }
  double skelModelCollisionScore = 0.0;
  if (m_params.modelSkeletonCollisionWeight > 0) {
    skelModelCollisionScore = m_params.modelSkeletonCollisionWeight * m_overlapScorer->score(iset, mInsts, mySkel);
  }
  double skelScore = 0.0;
  if (includeSkelScore) {
    skelScore = scoreSkeleton(iset, mySkel, baseSkelLP);
  }
  double totalScore = interactionScore + skelModelCollisionScore + skelScore;
  SG_LOG_DEBUG << " Skeleton score for iset " << iset.id
    << " totalScore=" << totalScore
    << " interactionScore=" << interactionScore 
    << " skelModelCollisionScore=" << skelModelCollisionScore 
    << " skelScore=" << skelScore; 
  return totalScore;
} // scoreSkeleton

double InteractionScorer::getSkeletonLogProb(const interaction::InteractionSet& iset,
                                             const Skeleton& skel) const {
  ISetScoreFn scoreFn = [&] (const string& isetId) {
     return m_pDatabase->skeletons->predictInteractionSetLikelihood(isetId, skel);
  };
  double poseLp = scoreSkeletonWithISetBackoff(iset, scoreFn);
  return poseLp;
}

double InteractionScorer::scoreSkeleton(const InteractionSet& iset,
                                        const Skeleton& skel,
                                        const boost::optional<double> baseSkelLP /* = none */) const {
//  SG_LOG_SCOPE("IScorer");
  SG_LOG_TAG("Channel", "IScorer");
  double totalScore = 0;
  if (m_params.skelSelfCollisionWeight > 0) {
    double skelCollScore = 1.0 - skel.getSelfCollisionRatio();  // less collision -> better score
    if (m_params.useLog) {
      skelCollScore = log(skelCollScore);
    }
    totalScore += m_params.skelSelfCollisionWeight * skelCollScore;
  }
  if (baseSkelLP && m_params.posePriorWeight) {
    //Skeleton s;
    //double avgLp;
    //m_pDatabase->skeletons->getAverageSkeleton(iset.id, &s, &avgLp);
    double poseLp = getSkeletonLogProb(iset, skel);
    double poseScore = exp(poseLp - baseSkelLP.get());
    SG_LOG_INFO << "POSE:" << poseScore << endl;
    totalScore += m_params.posePriorWeight * poseScore;
  }
  return totalScore;
} // scoreSkeleton

double InteractionScorer::score(const InteractionSet& iset,
                                const vec<ModelInstance>& mInsts,
                                const Skeleton& skel) const {
//  SG_LOG_SCOPE("IScorer");
  SG_LOG_TAG("Channel", "IScorer");
  double interactionScore = scoreModelSkelInteraction(iset, mInsts, skel);
  double skelScore = scoreSkeleton(iset, skel);
  double modelOverlapScore = m_overlapScorer->scoreModelOverlap(mInsts);
  double totalScore = interactionScore + skelScore + modelOverlapScore;
  return totalScore;
}

double InteractionScorer::scoreSkeletonWithISetBackoff(const InteractionSet& iset, const ISetScoreFn& scoreFn) const {
  double score = 0.0;
  if (m_pDatabase->skeletons->hasSkelDistribution(iset.id)) {
    score = scoreFn(iset.id);
  } else {
    for (const VerbNoun& vn : iset.verbNouns) {
      const string vnISetId = InteractionSet::getVerbNounISetId(vn);
      if (m_pDatabase->skeletons->hasSkelDistribution(vnISetId)) {
        score += scoreFn(vnISetId);
      } else {
        const string vISetId = InteractionSet::getVerbISetId(vn.verb);
        score += scoreFn(vISetId);
      }
    }
    score /= iset.verbNouns.size();
  }
  return score;
}

double InteractionScorer::scoreSkeletonJoint(const InteractionSet& iset,
                                             const TransformedSkeleton& skel,
                                             int iJoint) const {
//  SG_LOG_SCOPE("IScorer");
  SG_LOG_TAG("Channel", "IScorer");
  // TODO: evaluate other scores (interaction etc.) per joint
  SkelState mySs;
  skel2state(skel.getSkeleton(), &mySs);
  mySs = makeHierarchical(mySs);
  ISetScoreFn scoreFn = [&] (const string& isetId) {
     const SkeletonDistribution& sDist = m_pDatabase->skeletons->getSkelDistribution(isetId);
     return sDist.joints[iJoint].prob(mySs.joints[iJoint]);
  };
  double score = scoreSkeletonWithISetBackoff(iset, scoreFn);
  SG_LOG_DEBUG << "JOINT " + Skeleton::kJointNames[iJoint] + ":" << score << endl;
  return m_params.posePriorWeight * score;
}; // scoreSkeleton


// Score support
double InteractionScorer::scoreModelSupport(const ModelInstance& mInst,
                                            const ModelInstance& parentInst) const {
//  SG_LOG_SCOPE("IScorer");
  SG_LOG_TAG("Channel", "IScorer");

  const auto& modelOBB = mInst.computeWorldOBB();
  const auto& parentOBB = mInst.computeWorldOBB();

  // Check how much is mInst supported by mInstSupport
  // Check number of overlap in voxels between 
  //  mInst bottom surface and mInstSupport top surface
  int nFacePoints = 30;
  vec<geo::Vec3f> obbBottomFacePoints(nFacePoints);
  // Sample bottom face points
  for (int i = 0; i < nFacePoints; i++) {
    float x = math::DEF_RAND.uniform_float(-1,1);
    float y = math::DEF_RAND.uniform_float(-1,1);
    obbBottomFacePoints[i] = modelOBB.localToWorld() * geo::Vec3f(x,y,-1.01f);
  } 
  geo::Vec3f obbBottomFaceCenter = modelOBB.localToWorld() * geo::Vec3f(0,0,-1.01f);
  const ml::BinaryGrid3& parentVoxels = parentInst.model.solidVoxels;
  const geo::Transform parentWorldToVoxel = geo::from(parentInst.model.modelToVoxel) 
    * parentInst.getParentToLocal();

  double supported = 0.0;
  for (const geo::Vec3f& p : obbBottomFacePoints) {
    // Check how many of this are supported by parent model
    bool inOBB = parentOBB.contains(p);
    geo::Vec3f parentGridCoord = parentWorldToVoxel * p;
    bool hasVoxel = geo::isValidAndSet(parentVoxels, parentGridCoord);
    supported += (inOBB)? 0.5:0.0 + (hasVoxel)? 0.5:0.0;
  }

  return supported/ nFacePoints;
} // scoreModelSupport

// ModelSkeletonScorer
vec<int> generateRange(int start, int end) {
  vec<int> v;
  v.reserve(end-start);
  for (int i = start; i < end; ++i) {
    v.push_back(i);
  }
  return v;
}
const vec<int> ModelSkeletonScorer::kAllJointGroupsLR = generateRange(0, Skeleton::kNumJointGroupsLR);

// Scoring with iframe
double IFramesScorer::score(const InteractionSet& iset,
                            const ModelInstance& mInst,
                            const Skeleton& mySkel) const {
  if (iset.protoInteractionFrame) {
    double score = iset.protoInteractionFrame->support(mySkel, mInst);
    if (m_params.useLog) {
      return log(score);
    } else {
      return score;
    }
  } else {
    return 0.0;
  }
}

// Scoring with PIG
// Score features using 
void scoreFeats(const vec<stats::Histogram<float>>& feats,
                const vec<float>& values, vec<float>* pScores) {
  pScores->resize(feats.size());
  for (size_t i = 0; i < feats.size(); ++i) {
    (*pScores)[i] = feats[i].weight(values[i]);
  }
}

inline void addPigFeatsScore(const string& tag,
                             const ProtoInteractionLink& link,
                             const vec<float>& values,
                             int iFeat, double w,
                             vec<double>* pScores,
                             vec<double>* pCounts,
                             vec<double>* pWeights) {
  double score = link.feats.at(iFeat).weight(values.at(iFeat));
  pScores->push_back(score);
  pCounts->push_back(static_cast<double>(link.count));
  pWeights->push_back(w); 
  // DEBUG message
  SG_LOG_TRACE << "Got feat score=" << score << ", ic=" << link.count
               << ", weight=" << w << " for value=" << values.at(iFeat) 
               << " " << tag;
}

double PigScorer::score(const InteractionSet& iset,
                        const ModelInstance& mInst,
                        const Skeleton& mySkel) const {
  return scoreJointGroups(iset, mInst, mySkel);
}

double PigScorer::scoreJointGroups(const InteractionSet& iset,
                                   const ModelInstance& mInst,
                                   const Skeleton& mySkel,
                                   const vec<int>& jointGroups /*= kAllJointGroupsLR*/,
                                   const double comWeight /*= 0.5*/) const {
  const std::shared_ptr<WeightedPIG> pPig = iset.getWeightedPig();
  if (pPig == nullptr) {
    return 0.0;
  }
  const WeightedPIG& wpig = *pPig;

  // Make sure wpig joint links by category are populated
  ensurePigJointLinks(m_pDatabase->getLabeler(), wpig);
  const PigFeatIndices& pfi = ProtoInteractionGraph::kPigFeatIndices;
  const auto& mInstOBB = mInst.computeWorldOBB();
//  const string& objectLabel = m_pDatabase->getLabeler().getAnnotationNoun(mInst.category);
  // Go through joint groups and find joint with highest weighted number of observances
  double jointScore = 0.0;
  vec<double> weights;
  vec<double> scores;
  vec<double> instanceCounts;
  for (int iJointGroup : jointGroups) {
    //const auto& joints = kSkelParamsJointGroupLR.kJointInvMap[iJointGroup];
    const auto& iBasicJointGroup = Skeleton::kJointGroupLRToJointGroup[iJointGroup];
    Joint joint;
    createCombinedJoint(kSkelParamsJointGroupLR, iJointGroup, mySkel, &joint);
    vecf feats;
    if (iJointGroup == Skeleton::JointGroupLR_Gaze) {
      gazeFeats(mySkel, &mInstOBB, &feats);
    } else {
      contactFeats(mySkel, joint, &mInstOBB, &feats);
    }
    string debugTag = mInst.category + " with joint " 
      + kSkelParamsJointGroupLR.kJointNames.at(iJointGroup);
    bool includeAngleScore = (iJointGroup == Skeleton::JointGroupLR_Gaze);
    const auto& catJointLinks = wpig.jointLinksByCategory[iJointGroup];
    if (catJointLinks.count(mInst.category) > 0) {
      for (const auto& it : catJointLinks.at(mInst.category)) {
        const ProtoInteractionLink& link = it;
        double jointWeight = wpig.jointGroupWeights.at(iBasicJointGroup);
        // Include distance score
        addPigFeatsScore(debugTag + "(rdist)", link, feats, pfi.iRelRDist, jointWeight,
                         &scores, &instanceCounts, &weights);

        // Include angle score
        if (includeAngleScore) {
          addPigFeatsScore(debugTag + "(angDxy)", link, feats, pfi.iRelAngDxy, jointWeight,
                           &scores, &instanceCounts, &weights);
        }
      }
    }
  }
  double totalInstanceCounts = 0.0;
  for (int i = 0; i < scores.size(); ++i) { 
    double ic = instanceCounts[i];
    double weight = weights[i];
    jointScore += weight*ic*scores[i];
    totalInstanceCounts += ic; 
  }
  if (totalInstanceCounts > 0) {
    jointScore /= totalInstanceCounts;
  }

  // Consider the center of mass distance and angle as well
  const int iR = pfi.iRelRDist;
  const int iAng = pfi.iRelAngDxy;
  vecf comLinkFeats;
  const Vec3f com = vec3f(mySkel.centerOfMass());
  const Vec3f gazeDir = vec3f(mySkel.gazeDirection);
  const Vec3f closestPt = mInstOBB.closestPoint(com);
  const Vec3f segNormal = mInstOBB.dominantNormal();
  relFeats(com, gazeDir, closestPt, segNormal, &comLinkFeats);
  double comScore = 0.0;
  double comAngleScore = 0.0;
    
  const ProtoInteractionGraph& pig = wpig.pig;
  const auto& catComLinks = wpig.jointLinksByCategory[PIG::kComIdx];
  if (comWeight > 0.0 && catComLinks.count(mInst.category) > 0) {
    size_t totalIC = 0;
    for (const auto& it : catComLinks.at(mInst.category)) {
      const ProtoInteractionLink& link = it;
      comScore = link.count*link.feats.at(iR).weight(comLinkFeats[iR]);
      comAngleScore = link.count*link.feats.at(iAng).weight(comLinkFeats[iAng]);
      totalIC += link.count;
    }
    comScore = comScore/totalIC;
    comAngleScore = comAngleScore/totalIC;
  }
  double totalScore = jointScore + comWeight*(comScore + comAngleScore);

  SG_LOG_DEBUG << "Got interactionScorePig=" << totalScore << ", jointScore=" << jointScore 
    << ", comScore=" << comScore << ", comAngleScore=" << comAngleScore << " for " << mInst.category;
  return totalScore;
} // score (PIG)

double PigScorer::scoreJoint(const interaction::InteractionSet& iset,
                             const vec<ModelInstance>& mInsts,
                             const TransformedSkeleton& mySkel,
                             int iJoint) const {
  double totalScore = 0.0;
  vec<int> jointGroups;
  jointGroups.push_back(Skeleton::kJointToJointGroupLR[iJoint]);
  for (const ModelInstance& mInst : mInsts) {
    double score = scoreJointGroups(iset, mInst, mySkel.getSkeleton(), jointGroups);
    totalScore += score;
  }
  return totalScore;
}


// Scoring using PIG sim
double PigSimScorer::score(const InteractionSet& iset,
                           const vec<ModelInstance>& mInsts,
                           const Skeleton& skel) const {
  SG_LOG_TRACE << "Creating interaction graph with models";
  // Make sure that models have segmentation (TODO: push into createInteractionGraph?)
  segmentation::SegmentationParams segParams(*m_pDatabase->params);
  for (const ModelInstance& mInst : mInsts) {
    m_pDatabase->models.ensureSegmentation(mInst.model.id, segParams, true);
  }
  const auto ig = 
    m_pDatabase->interactionFactory.createInteractionGraph(mInsts, skel, segmentation::kPartSegment,
                                                           m_pDatabase->getLabeler());
  double sim = iset.protoSimilarity(*ig, true);
  if (m_params.useLog) {
    double lsim = log(sim);
    SG_LOG_DEBUG <<  "Got pig similarity " << sim << " with log " << lsim;
    sim = lsim;
  } else {
    SG_LOG_DEBUG <<  "Got pig similarity " << sim;
  }
  return sim;
} // score (PigSim)

double PigSimScorer::score(const InteractionSet& iset,
                           const ModelInstance& mInst,
                           const Skeleton& skel) const {
  vec<ModelInstance> mInsts;
  mInsts.emplace_back(mInst);
  return score(iset, mInsts, skel);
}

// Overlap scorer

double OverlapScorer::scoreModelOverlap(const vec<ModelInstance>& mInsts) const {
  double totalOverlapScore = 0.0;
  for (size_t i = 0; i < mInsts.size(); ++i) {
    for (size_t j = i+1; j < mInsts.size(); ++j) {
      const auto& mInst = mInsts.at(i);
      const auto& mOther = mInsts.at(j);
      double overlap = voxelOverlapNormalized(mInst, mOther);
      double score = 1.0 - overlap;
      if (m_params.useLog) {
        score = log(score);
      }
      totalOverlapScore += score;
    }
  }
  return totalOverlapScore;
} // scoreModelOverlap

double OverlapScorer::scoreModelOverlap(const PlacementState& state,
                                        const ModelInstance& mInst) const {
  double totalOverlapScore = 0.0;
  for (const ModelInstance* pOther : state.placed) {
    // Make sure we are not checking against our self
    if (pOther != &mInst) {
      double overlap = voxelOverlapNormalized(mInst, *pOther);
      double score = 1.0 - overlap;
      if (m_params.useLog) {
        score = log(score);
      }
      totalOverlapScore += score;
    }
  }
  return totalOverlapScore;
} // scoreModelOverlap

double OverlapScorer::scoreModelSkeletonOverlap(const ModelInstance& mInst,
                                                const Skeleton& skel) const {
  SkelRange skelRange(skel);
  const Matrix3Xf skelPtsM = getSkelPoints(skelRange, m_params.numSkelPtSamples);

  const ml::BinaryGrid3& modelVoxels = mInst.model.solidVoxels;
  const Transform skelToVoxels = from(mInst.model.modelToVoxel) * mInst.getParentToLocal();

  // Overlap not good
  double overlap = ratioPointsInVoxels(skelToVoxels * skelPtsM, modelVoxels);
  double score = 1.0 - overlap;
  if (m_params.useLog) {
    score = log(score);
  }
  return score;
} // scoreModelSkeletonOverlap

double OverlapScorer::scoreModelSkeletonOverlap(const vec<ModelInstance>& mInsts,
                                                const Skeleton& skel) const {
  SkelRange skelRange(skel);
  const Matrix3Xf skelPtsM = getSkelPoints(skelRange, m_params.numSkelPtSamples);

  double totalOverlapScore = 0.0;
  for (const ModelInstance& mInst : mInsts) {
    const ml::BinaryGrid3& modelVoxels = mInst.model.solidVoxels;
    const Transform skelToVoxels = from(mInst.model.modelToVoxel) * mInst.getParentToLocal();
    // Overlap not good
    double overlap = ratioPointsInVoxels(skelToVoxels * skelPtsM, modelVoxels);
    double score = 1.0 - overlap;
    if (m_params.useLog) {
      score = log(score);
    }
    totalOverlapScore += score;
  }
  return totalOverlapScore;
} // scoreModelSkeletonOverlap

}  // namespace synth
}  // namespace core
}  // namespace sg
