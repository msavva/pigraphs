#pragma once

#include "libsg.h"  // NOLINT

#include "core/synth/ModelAligner.h"
#include "core/synth/ModelPlacer.h"
#include "core/synth/SkeletonPoser.h"
#include "core/synth/InteractionScorer.h"

namespace sg {
namespace core {
namespace synth {

//! Encapsulates parameters for Interaction Synthesis
struct InteractionSynthParams {  
  typedef std::function<int(const vis::PoseHeatMap& heatMap)> VisualizeHeatMapFn;
  InteractionSynthParams() = default;
  InteractionSynthParams(const Scan* _pScan, const set<interaction::VerbNoun>& _vns)
    : pScan(_pScan)
    , verbNouns(_vns) { }
  InteractionSynthParams(const Scan* _pScan, const string& _text)
    : pScan(_pScan)
    , text(_text) { }
  //! Scene against which to synthesize the interaction
  //! Specify nullptr to synthesize a interaction without a base scene
  const Scan* pScan = nullptr;
  //! Input text
  string text;
  //! Type of parser to use ("label", "nlp")
  string parserType = "label";
  //! What are the set of verbnouns?
  set<interaction::VerbNoun> verbNouns;
  //! Should we try to predict the labels
  bool predictSegmentLabels = false;
  //! Should we try to predict the support surfaces
  bool predictSupportSurfaces = false;
  //! Should we try to retarget a existing verb noun PIG to a novel verb noun pair?
  bool retarget = false;
  //! Should we try to form a composite PIG from the atomic actions?
  bool doComposition = false;
  //! Should we do backoff and use existing PIGs when available?
  bool doBackoff = false;
  //! Should we do model placement or not?
  bool doModelPlacement = true;
  //! Should we use pig for placement?
  bool placeWithPig = true;
  //! Parameters to use for interaction scoring
  InteractionScorerParams iscorerParams;
  //! How many times to iterate between model placement and skeleton placement
  int maxPoseIters = 1;
  //! Should we try to interleave placement of models and skeleton
  bool interleavePlacement = true;
  //! Classifier to use for determing where the skeleton should be
  string classifierType = "IGKNNSim";
  //! If specified, pose with these skeletons
  const vec<TransformedSkeleton>* pSkels = nullptr;
  //! Use this as the position for the center of the arrangement (used if pScan is nullptr)
  geo::Vec3f targetPos;
  //! Callback for visualizing heatmap of where the skeleton should be
  VisualizeHeatMapFn* pVisualizeHeatMapFn = nullptr; 
  //! Callback for visualizing placement
  ModelPlacerParams::RenderPlacementStateFn* pRenderPlacementStateFn = nullptr;
  //! Callback for visualizing alignment
  ModelAlignerParams::RenderAlignmentStateFn* pRenderAlignmentStateFn = nullptr;
  //! Select model parameter
  SelectModelStrategy selectModelStrategy = SelectModelStrategy::kFirst;
  //! Select skeleton parameter
  SelectSkelStrategy selectSkelStrategy = SelectSkelStrategy::kFirst;
  //! Visual logging file
  vis::VisLog* pVislog = nullptr;
  //! Final score
  double* pFinalScore = nullptr;
};

//! Interaction Synthesizer
//! Note: InteractionSynthParams are not const because we may need to populate the verbNouns
class InteractionSynth {
 public:
  void init(Database* pDatabase);

  //! Given text and a scene, parse the text and synthesize the interaction corresponding to the text
  int parseAndSynthesize(InteractionSynthParams& params,
                         vec<TransformedSkeleton>* pPosedSkels,
                         vec<ModelInstance>* pModelInstances) const;

  //! Given a scan and a set of verb noun pairs, synthesize the interaction
  //! The output is a vector of skeletons and a vector of placed models
  //! For now, we will output just one skeleton for a static scene
  int synthesize(InteractionSynthParams& params,
                 vec<TransformedSkeleton>* pPosedSkels,
                 vec<ModelInstance>* pModelInstances) const;

  //! Return the skeleton poser
  const SkeletonPoser& getSkeletonPoser() const {
    return m_skeletonPoser;
  }

  //! Return the model placer
  const ModelPlacer& getModelPlacer() const {
    return m_modelPlacer;
  }

 private:
  //! Synthesize from an existing scan
  int synthesizeWithScan(InteractionSynthParams& params,
                         vec<TransformedSkeleton>* pPosedSkels,
                         vec<ModelInstance>* pModelInstances) const;
  
  //! Synthesize from scratch (no scan available)
  int synthesizeNoScan(InteractionSynthParams& params,
                       vec<TransformedSkeleton>* pPosedSkels,
                       vec<ModelInstance>* pModelInstances) const;

  //! Create a interaction set group for the given set of verb and noun 
  void populateVerbNounISetGroup(InteractionSynthParams& params,
                                 interaction::VerbNounISetGroup* pISetGroup) const;
  //! Infer the basic pose (currently, whether the skeleton should be standing or not
  void inferBasicPose(InteractionSynthParams& params,
                      interaction::VerbNounISetGroup* pISetGroup) const;

  //! Database from which we search for models and and such
  Database* m_pDatabase = nullptr;

  SkeletonPoser m_skeletonPoser;
  ModelPlacer m_modelPlacer;
  set<string> m_basicPoses;
};

}  // synth
}  // namespace core
}  // namespace sg


