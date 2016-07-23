#include "common.h"  // NOLINT

#include "core/synth/InteractionSynth.h"

#include "core/Database.h"
#include "core/Skeleton.h"
#include "core/synth/InteractionScorer.h"
#include "interaction/Interaction.h"
#include "interaction/ProtoInteractionGraph.h"
#include "ui/ErrorCode.h"
#include "vis/Heatmap.h"
#include "vis/vislog.h"

#include "../jace-proxy/nlputil.h"

using sg::core::Database;
using sg::core::TransformedSkeleton;
using sg::core::ModelInstance;
using sg::core::SkelRange;
using sg::interaction::VerbNoun; using sg::interaction::VerbNounISetGroup;
using sg::interaction::InteractionSet; using sg::interaction::InteractionSetType;

namespace sg {
namespace core {
namespace synth {

void InteractionSynth::init(Database* pDatabase) {
  m_pDatabase = pDatabase;
  m_skeletonPoser.init(pDatabase);
  m_modelPlacer.init(pDatabase);

  // Initialize our hard coded list of basic poses
  m_basicPoses.insert("sit");
  m_basicPoses.insert("stand");
  m_basicPoses.insert("lie");
}

int InteractionSynth::parseAndSynthesize(InteractionSynthParams& params,
                                         vec<TransformedSkeleton>* pPosedSkels,
                                         vec<ModelInstance>* pModelInstances) const {
  // Figure out set of verbs and nouns
  SG_LOG_SCOPE("ISynth");
  int rv = 0;
  if (params.parserType == "label") {
    params.verbNouns.clear();
    VerbNoun::getVerbNouns(params.text, &params.verbNouns);
  } else if (params.parserType == "nlp") {
    vec<nlputil::Action> actions;
    rv = parseActions(params.text, &actions);
    if (rv >= 0) {
      for (const nlputil::Action& action : actions) {
        params.verbNouns.insert({action.verb, action.object});
      }
    }
  } else {
    SG_LOG_ERROR << "Unsupported parserType=" << params.parserType;
    rv = ui::ErrorCodes::E_UNSUPPORTED;
  }
  if (rv >= 0) {
    rv = synthesize(params, pPosedSkels, pModelInstances);
  }
  return rv;
}

int InteractionSynth::synthesize(InteractionSynthParams& params,
                                 vec<TransformedSkeleton>* pPosedSkels,
                                 vec<ModelInstance>* pModelInstances) const {
  if (params.pScan != nullptr) {
    return synthesizeWithScan(params, pPosedSkels, pModelInstances);
  } else {
    return synthesizeNoScan(params, pPosedSkels, pModelInstances);
  }
}

int InteractionSynth::synthesizeNoScan(InteractionSynthParams& params,
                                       vec<TransformedSkeleton>* pPosedSkels,
                                       vec<ModelInstance>* pModelInstances) const {
  // Basic algorithm to synthesize interaction
  // 1. Take verb nouns and create a composite PIG
  VerbNounISetGroup vnISetGroup;
  populateVerbNounISetGroup(params, &vnISetGroup);
  // 2. Infer basic pose (standing vs not standing)
  inferBasicPose(params, &vnISetGroup);

  // 3. Select pose of person for interaction 
  // Put the skeleton at origin with identity transform
  // TODO: Check that our skeleton's basic pose matches our inferred basic pose
  SkeletonPoserParams poserParams(params.pScan);
  poserParams.init(*m_pDatabase->params);
  poserParams.iscorerParams = params.iscorerParams;
  poserParams.selectSkelStrategy = params.selectSkelStrategy;
  poserParams.pISetGroup = &vnISetGroup;
  poserParams.classifierType = params.classifierType;
  poserParams.pSkels = params.pSkels;
  poserParams.pVisualizeHeatMapFn = params.pVisualizeHeatMapFn;
  vec<TransformedSkeleton> posedSkels;
  int rv = m_skeletonPoser.pose(poserParams, &posedSkels);
  if (rv == 0) {
    SG_LOG_WARN << "Cannot find any poses for " << vnISetGroup.id;
    return rv;
  }
  SG_LOG_INFO << "Got " << posedSkels.size() << " posed skeletons";
  
  // Set out skeletons
  TransformedSkeleton skel(posedSkels[0], ml::mat4f::translation(geo::to<ml::vec3f>(params.targetPos)));
  pPosedSkels->push_back(skel);

  // 4. Retrieve and place models
  if (params.doModelPlacement) {
    if (params.placeWithPig && vnISetGroup.pInteractionSet == nullptr) {
      SG_LOG_ERROR << "Cannot find iset for " << vnISetGroup.id;
      return -1;
    }

    SkelRange skelRange(skel.getSkeleton());
    ModelPlacerParams placerParams;
    PlacementConstraints constraints;
    placerParams.predictAction = false;
    placerParams.predictSupportSurfaces = params.predictSupportSurfaces;
    placerParams.predictSegmentLabels = params.predictSegmentLabels;
    placerParams.useSkelRange = false;
    placerParams.pScan = nullptr;
    placerParams.pSkelRange = &skelRange;
    placerParams.pInteractionSet = vnISetGroup.pInteractionSet;
    placerParams.pVNISetGroup = &vnISetGroup;
    placerParams.basicPose = vnISetGroup.basicPose;
    placerParams.alignParams.pRenderFun = params.pRenderAlignmentStateFn;
    placerParams.iscorerParams = params.iscorerParams;
    // Special params
    placerParams.ensureNounPresent = m_pDatabase->params->getWithDefault<bool>(
      "ModelPlacer.ensureNounPresent", placerParams.ensureNounPresent);
    placerParams.useDistance = params.placeWithPig;
    placerParams.useAngle = params.placeWithPig;
    // Selecting model
    placerParams.selectModelStrategy = params.selectModelStrategy;
    // Visualization callback
    placerParams.pRenderFun = params.pRenderPlacementStateFn;
    // Placement interleaving
    placerParams.interleavePlacement = params.interleavePlacement;
    // Restrict models to whitelist
    placerParams.restrictModelsToWhitelist = m_pDatabase->params->get<bool>("ModelPlacer.restrictToWhitelist");
    // Placement constraints
    placerParams.pPlacementConstraints = &constraints;
    poserParams.pPlacementConstraints = &constraints;
    // Visualization log
    placerParams.pVislog = params.pVislog;
    int before = static_cast<int>(pModelInstances->size());
    int placed = m_modelPlacer.place(placerParams, pModelInstances);
    int iter = 0;
    while (iter < params.maxPoseIters) {
      iter++;
      SG_LOG_INFO << "Pose/Place iter " << iter << "/" << params.maxPoseIters;
      vec<TransformedSkeleton> adjustedSkels;
      SG_LOG_INFO << "Adjusting pose";
      rv = m_skeletonPoser.adjust(poserParams, skel, *pModelInstances, &adjustedSkels);
      bool isAllPlaced = true;
      for (const ModelInstance& mInst : *pModelInstances) {
        if (!mInst.placementScore) {
          isAllPlaced = false;
          break;
        }
      }
      if (rv > 0 || !isAllPlaced) {
        TransformedSkeleton& selectedSkel = (!adjustedSkels.empty())?
          adjustedSkels[0] : skel;
        if (rv > 0) {
          (*pPosedSkels)[pPosedSkels->size()-1] = selectedSkel;
        }
        skelRange = SkelRange(selectedSkel.getSkeleton());
        SG_LOG_INFO << "Adjusting objects";
        rv = m_modelPlacer.adjust(placerParams, pModelInstances);
        if (rv <= 0) break;
      }
    }

    int after = static_cast<int>(pModelInstances->size());
    if (params.pFinalScore) {
      if (vnISetGroup.pInteractionSet) {
        core::synth::InteractionScorer scorer;
        const InteractionSet& iset = *vnISetGroup.pInteractionSet;
        scorer.init(m_pDatabase, params.iscorerParams);
        const Skeleton skel = (*pPosedSkels).back().getSkeleton();
        *params.pFinalScore = scorer.score(iset, *pModelInstances, skel);
        SG_LOG_INFO << "Final score for scene " << *params.pFinalScore;
      } else {
        *params.pFinalScore = 0.0;
      }
    }

    return (after - before);
  }

  SG_LOG_INFO << "Skipping model placement";
  return 0;
}

int InteractionSynth::synthesizeWithScan(InteractionSynthParams& params,
                                         vec<TransformedSkeleton>* pPosedSkels,
                                         vec<ModelInstance>* pModelInstances) const {
  // 1. Take verb nouns and create a composite PIG
  VerbNounISetGroup vnISetGroup;
  populateVerbNounISetGroup(params, &vnISetGroup);

  if (vnISetGroup.pInteractionSet == nullptr) {
    SG_LOG_ERROR << "Cannot find iset for " << vnISetGroup.id;
    return -1;
  }

  // 2. Use SkeletonPoser to find poses for this action and position skeleton
  SkeletonPoserParams poserParams(params.pScan);
  poserParams.init(*m_pDatabase->params);
  poserParams.selectSkelStrategy = params.selectSkelStrategy;
  poserParams.pISetGroup = &vnISetGroup;
  poserParams.classifierType = params.classifierType;
  poserParams.pSkels = params.pSkels;
  poserParams.pVisualizeHeatMapFn = params.pVisualizeHeatMapFn;
  vec<TransformedSkeleton> posedSkels;
  vis::PoseHeatMap heatMap;
  int rv = m_skeletonPoser.poseForScan(poserParams, &posedSkels, &heatMap);

  if (rv == 0) {
    SG_LOG_WARN << "Cannot find any poses for " << vnISetGroup.id;
    return rv;
  }
  SG_LOG_INFO << "Got " << posedSkels.size() << " posed skeletons";

  // Set out skeletons
  pPosedSkels->push_back(posedSkels[0]);

  // 3. Retrieve and place models
  // For now pick the first posed skeleton
  if (params.doModelPlacement) {
    SkelRange skelRange(posedSkels[0].getSkeleton());
    ModelPlacerParams placerParams;
    placerParams.predictSupportSurfaces = params.predictSupportSurfaces;
    placerParams.predictAction = false;
    placerParams.predictSegmentLabels = params.predictSegmentLabels;
    placerParams.useSkelRange = false;
    placerParams.pScan = params.pScan;
    placerParams.pSkelRange = &skelRange;
    placerParams.pInteractionSet = vnISetGroup.pInteractionSet;
    placerParams.alignParams.pRenderFun = params.pRenderAlignmentStateFn;
    // Selecting model
    placerParams.selectModelStrategy = params.selectModelStrategy;
    // Restrict models to whitelist
    placerParams.restrictModelsToWhitelist = m_pDatabase->params->get<bool>("ModelPlacer.restrictToWhitelist");
    // Visualization callback
    placerParams.pRenderFun = params.pRenderPlacementStateFn;
    // Visualization log
    placerParams.pVislog = params.pVislog;
    int placed = m_modelPlacer.place(placerParams, pModelInstances);
    return placed;
  }

  SG_LOG_INFO << "Skipping model placement";
  return 0;
}

using interaction::ProtoInteractionGraph;
void InteractionSynth::populateVerbNounISetGroup(InteractionSynthParams& params,
                                                 VerbNounISetGroup* pISetGroup) const {
  const string partTypeStr = m_pDatabase->params->get<string>("Interaction.partType"); 
  segmentation::PartType partType = segmentation::getPartTypeFromString(partTypeStr); 
  auto& vnToRetargetedMap = pISetGroup->vnToRetargetedMap;
  pISetGroup->verbNouns = params.verbNouns;
  pISetGroup->verbs.clear();
  pISetGroup->nouns.clear();
  for (const VerbNoun& vn : pISetGroup->verbNouns) {
    pISetGroup->verbs.insert(vn.verb);
    pISetGroup->nouns.insert(vn.noun);
  }
  m_pDatabase->interactions.getRelatedInteractionSets(pISetGroup);
  if (params.parserType == "label") {
    // Get exact match
    const auto isetMap = m_pDatabase->interactions.getAllInteractionSets();
    SG_LOG_INFO << "Looking for exact iset " << params.text;
    if (isetMap.count(params.text) > 0) {
      SG_LOG_INFO << "Using exact iset " << params.text;
      pISetGroup->pInteractionSet = &isetMap.at(params.text).get();
    }
  }
  SG_LOG_INFO << "Initialized interaction set group for " << pISetGroup->id;

  // The different strategies for creating appropriate PIG to use are:
  // MATCH     (match only - doComposition, retarget, doBackoff all false)
  // RETARGET  (retarget a vn pair for a novel vn pair - retarget true)
  // COMPOSITE (do compositing - doComposition true)
  // BACKOFF   (retarget/composite if no match - doBackoff true)
  
  map<string, std::reference_wrapper<InteractionSet>> vnToISetMap;
  for (InteractionSet& vnISet : pISetGroup->vnISets) {
    SG_LOG_INFO << "Add iset " << vnISet.id;
    vnToISetMap.insert({vnISet.id, vnISet});
  }

  if (params.retarget) {
    // Retarget vn pairs that don't have a corresponding iset
    const interaction::InteractionSetMap& verbISets
      = m_pDatabase->interactions.getInteractionSets(InteractionSetType::ISetType_Verb);
    for (const auto& vn : pISetGroup->verbNouns) {
      const string isetId = InteractionSet::getVerbNounISetId(vn);
      if (vnToISetMap.count(isetId) == 0 && !vn.noun.empty()) {
        SG_LOG_INFO << "Need to retarget " << vn.verb << " for " << isetId;
        const InteractionSet& verbISet = verbISets.at(InteractionSet::getVerbISetId(vn.verb));
        const string& pigType = interaction::kAggregationStrategyIds[interaction::kPerVerbTargetProtoNodes];
        const std::shared_ptr<ProtoInteractionGraph> verbPig = 
          verbISet.pigs.at(partType).at(pigType);
        // TODO: Flesh out implementation
        std::shared_ptr<ProtoInteractionGraph> retargeted = verbPig->retarget(vn.verb, vn.noun);
        vnToRetargetedMap.insert({isetId, {isetId, &verbISet, retargeted}});
      }
    }
  }

  // First consider backoffs of isets
  if (params.doBackoff && pISetGroup->pInteractionSet == nullptr) {
    // No interaction set
    if (pISetGroup->verbNouns.size() == 1) {
      // Just one verbNoun, see if we have a corresponding ISet for that
      if (pISetGroup->vnISets.size() == 1) {
        pISetGroup->pInteractionSet = &pISetGroup->vnISets.at(0).get();
      } else {
        SG_LOG_INFO << "No appropriate vn found for " << pISetGroup->id;
      }
    }
    if (pISetGroup->pInteractionSet == nullptr) {
      if (pISetGroup->verbs.size() == 1) {
        // Just one verb, see if we have a corresponding ISet for that
        if (pISetGroup->vISets.size() == 1) {
          pISetGroup->pInteractionSet = &pISetGroup->vISets.at(0).get();
        } 
      }
    }

    // Indicate what interaction set we ended up with
    if (pISetGroup->pInteractionSet != nullptr) {
      SG_LOG_INFO << "No match found for " << pISetGroup->id;
      SG_LOG_INFO << "Backing off to interaction set " << pISetGroup->pInteractionSet->id;
    }
  }

  if (pISetGroup->pInteractionSet == nullptr && pISetGroup->verbNouns.size() == 1) {
    const auto vn = pISetGroup->verbNouns.begin();
    const string isetId = InteractionSet::getVerbNounISetId(*vn);
    if (vnToRetargetedMap.count(isetId) > 0 && !vn->noun.empty()) {
      // Let's create a composite PIG from our basic building blocks
      const interaction::RetargetedPIG& r = vnToRetargetedMap.at(isetId);
      pISetGroup->pInteractionSet = &pISetGroup->compositeInteractionSet;
      pISetGroup->compositeInteractionSet.id = pISetGroup->id;
      pISetGroup->compositeInteractionSet.type = InteractionSetType::ISetType_Composite;
      pISetGroup->compositeInteractionSet.verbNouns = pISetGroup->verbNouns;
      pISetGroup->compositeInteractionSet.verbs = pISetGroup->verbs;
      pISetGroup->compositeInteractionSet.nouns = pISetGroup->nouns;
      pISetGroup->compositeInteractionSet.protoInteractionGraph = r.retargetedPIG;
      pISetGroup->compositeInteractionSet.protoInteractionFrame = r.pOrigInteractionSet->protoInteractionFrame;
      pISetGroup->compositeInteractionSet.pigs = r.pOrigInteractionSet->pigs;
      SG_LOG_INFO << "Creating retargeted pig for " << isetId;
    }
  }

  // Create composite PIG
  if (params.doComposition && pISetGroup->verbNouns.size() > 1 && 
      (!params.doBackoff || pISetGroup->pInteractionSet == nullptr)) {
    SG_LOG_INFO << "Creating composite PIG for " << pISetGroup->id;
    // Let's create a composite PIG from our basic building blocks
    pISetGroup->pInteractionSet = &pISetGroup->compositeInteractionSet;
    pISetGroup->compositeInteractionSet.id = pISetGroup->id;
    pISetGroup->compositeInteractionSet.type = InteractionSetType::ISetType_Composite;
    pISetGroup->compositeInteractionSet.verbNouns = pISetGroup->verbNouns;
    pISetGroup->compositeInteractionSet.verbs = pISetGroup->verbs;
    pISetGroup->compositeInteractionSet.nouns = pISetGroup->nouns;
    // TODO: Populate
    // jointActivationProb
    // jointGroupActivationProb
    // jointGroupWeights
    pISetGroup->compositeInteractionSet.jointGroupWeights.fill(1.0);

    // Create a combined pig!!!
    vec<std::reference_wrapper<const ProtoInteractionGraph>> pigs;
    vec<vec<double>> weights;
    weights.reserve(pISetGroup->vnISets.size());
    bool useJointWeights = true;
    for (InteractionSet& vnISet : pISetGroup->vnISets) {
      pigs.emplace_back(*vnISet.protoInteractionGraph.get());

      if (useJointWeights) {
        weights.emplace_back();
        vec<double>& w = weights[weights.size()-1];
        w.resize(Skeleton::kNumJointGroupsLR);
        for (size_t wi = 0; wi < w.size(); ++wi) {
          const int wgi = Skeleton::kJointGroupLRToJointGroup.at(wi);
          w[wi] = vnISet.jointGroupWeights[wgi];
        }
      }
    }
    for (const auto rt  : vnToRetargetedMap) {
      pigs.emplace_back(*rt.second.retargetedPIG);

      if (useJointWeights) {
        weights.emplace_back();
        vec<double>& w = weights[weights.size()-1];
        w.resize(Skeleton::kNumJointGroupsLR);
        for (size_t wi = 0; wi < w.size(); ++wi) {
          const int wgi = Skeleton::kJointGroupLRToJointGroup.at(wi);
          w[wi] = rt.second.pOrigInteractionSet->jointGroupWeights[wgi];
        }
      }
    }
    if (pISetGroup->compositeInteractionSet.protoInteractionGraph == nullptr) {
      pISetGroup->compositeInteractionSet.protoInteractionGraph = 
        std::make_shared<ProtoInteractionGraph>(pISetGroup->id);
    }
    // TODO: Pass in jointGroupWeights and get better jointGroupWeights...
    ProtoInteractionGraph* pCombined = pISetGroup->compositeInteractionSet.protoInteractionGraph.get();
    ProtoInteractionGraph::combine(
      pigs, weights, interaction::kPIGCombineJointsMax,
      pCombined);
    if (params.pVislog) {
      pCombined->dumpFeats(params.pVislog->getFilename("combined.pig.csv"));
      pCombined->dumpFeatsSummary(params.pVislog->getFilename("combined.pig.summary.csv"));
      const ObjectLabeler& labeler = m_pDatabase->getLabeler();
      labeler.dumpJointCategoryProbs(*pCombined, params.pVislog->getFilename("combined.pigjc.csv"));
      labeler.dumpJointLabelProbs(*pCombined, params.pVislog->getFilename("combined.pigjl.csv"));
    }
  }
  SG_LOG_INFO << "Created interaction set group for " << pISetGroup->id;
}

void InteractionSynth::inferBasicPose(InteractionSynthParams& params,
                                      VerbNounISetGroup* pISetGroup) const {
  for (const string& verb : pISetGroup->verbs) {
    if (m_basicPoses.count(verb) > 0) {
      pISetGroup->basicPose = verb;
      break;
    } 
  }

  if (pISetGroup->basicPose.empty()) {
    // Haven't figured out our basic pose yet
    // Can assume there is no basic pose
    // Let's see what we can guess...
    // P(basicPose|vns) = sum_vn P(basicPose|vn)P
    const auto& verbStats = m_pDatabase->interactions.getVerbStats();
    stats::Counter<string, double> basicPoseProbs;
    for (const string& p : m_basicPoses) {
      // TODO: Replace with vn's instead of v's
      //for (const VerbNoun& vn : pISetGroup->verbNouns) {
      for (const string& v : pISetGroup->verbs) {
        double prob = verbStats.getCondProb(p, v);
        basicPoseProbs.inc(p, prob);
      }
    }
    pISetGroup->basicPose = basicPoseProbs.argMax();
  }

  SG_LOG_INFO << "Inferred basic pose: " << pISetGroup->basicPose;
}

}  // namespace synth
}  // namespace core
}  // namespace sg
