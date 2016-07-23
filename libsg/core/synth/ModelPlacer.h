#pragma once

#include "libsg.h"  // NOLINT
#include "core/Skeleton.h"
#include "core/synth/InteractionScorer.h"
#include "core/synth/ModelAligner.h"
#include "core/synth/ModelRetriever.h"
#include "core/synth/synth.h"

namespace sg {
namespace core {
namespace synth {

struct PlacementConstraints;

//! Encapsulates placement state for debugging and visualization
struct PlacementState {
  //! Description of the placement state (used in the visual logging)
  string description;
  //! Model instances to consider
  vec<ModelInstance>* pModelInstances = nullptr;
  //! Already placed models
  vec<ModelInstance*> placed;
  //! Placed model indices
  set<size_t> placedIndices;
  //! Aligment state of model currently being aligned
  AlignmentState* pAlignmentState = nullptr;
  //! Scores of placements
  vec<pair<geo::Vec3f, float>>  placementScores;
  //! Visual logging file
  vis::VisLog* pVislog = nullptr;
  //! Placed skeleton
  const Skeleton* pSkel = nullptr;
  //! Placed skeleton OBB
  const geo::OBB* pSkelOBB = nullptr;

  void init(vec<ModelInstance>* pInsts);
  void reset() {
    placedIndices.clear();
  }
  bool isAllPlaced() {
    return pModelInstances && pModelInstances->size() == placedIndices.size();
  }
  bool isPlaced(size_t i) {
    return placedIndices.count(i) > 0;
  }
  void markPlaced(size_t i) {
    if (pModelInstances && i < pModelInstances->size() && placedIndices.count(i) == 0) {
      placedIndices.insert(i);
      placed.push_back(&pModelInstances->at(i));
    }
  }
};

//! Encapsulates parameters for ModelPlacer
struct ModelPlacerParams {
  bool predictAction = false;
  bool predictSegmentLabels = false;
  bool predictSupportSurfaces = false;
  bool useSkelRange = false;
  bool debugViz = false;  // whether to do in-the-loop debug rendering
  //! The scanned scan to place the models over (if nullptr, placement is done on an empty room)
  const Scan* pScan = nullptr;
  //! A sequence of skeletons to use for placement (should not be null)
  const SkelRange* pSkelRange = nullptr;
  //! Interaction set to use when placing models
  //! (must not be null when predictingSegmentLabels and not predictingAction)
  const interaction::InteractionSet* pInteractionSet = nullptr;
  const interaction::VerbNounISetGroup* pVNISetGroup = nullptr;
  //! Basic pose to use ("stand", "sit", "lie")
  string basicPose; 
  //! Filter on the category of objects to place (leave empty for no filtering)
  set<string> categoriesToPlace;
  //! Parameters to use for alignment
  ModelAlignerParams alignParams;
  //! Parameters to use for interaction scoring
  InteractionScorerParams iscorerParams;
  //! Parameter for making sure there is the requested object
  bool ensureNounPresent = false;
  //! Additional parameters for positioning
  bool useDistance = false;
  bool useAngle = false;
  //! Select model parameter
  SelectModelStrategy selectModelStrategy = SelectModelStrategy::kFirst;
  //! Render placement for debugging
  typedef std::function<void(const PlacementState& state)> RenderPlacementStateFn;
  RenderPlacementStateFn* pRenderFun = nullptr;
  //! Should we try to interleave placement of models and skeleton
  bool interleavePlacement = true;
  //! Should we restrict the models to the whitelist?
  bool restrictModelsToWhitelist = false;
  //! Additional placement constraints (both input and output)
  PlacementConstraints* pPlacementConstraints = nullptr;
  //! Visual logging file
  vis::VisLog* pVislog = nullptr;
};

typedef vec<std::shared_ptr<interaction::InteractionGraph>> VecIGPtr;

struct SupportHierarchy {
  set<int> roots;
  vec<int> modelToParentIndex;
  vec<vec<int>> modelToChildIndices;
  // These shouldn't be supported by the room, but no support was found
  vec<int> notSupported;

  void clear() {
    roots.clear();
    modelToChildIndices.clear();
    modelToParentIndex.clear();
    notSupported.clear();
  }

  //! Dumps debug summary of support hierarchy to ostream
  void dump(ostream& os, vec<ModelInstance>& modelInstances) const;
};

struct InteractingObject {
  string label;
  string category;
  double score = 0.0;
  bool aboveThreshold = false;
  vec<int> objectIndices;
};

struct ObjectJointInteractions {
  string sceneType;
  vec<vec<Skeleton::JointGroupLR>> interactingJointGroups;
  vec<vec<Skeleton::JointGroupLR>> nonGazeInteractingJointGroups;
  vec<vec<int>> objIndicesByJointGroup;
  vec<InteractingObject> objsByJointGroup;
  int iSupportObj;
  bool isStanding;
  float maxContactDist;
  float maxGazeDist;
};

//! Encapsulates constraints used for placement
//! Relationships that should holds between objects and object and skeleton
struct PlacementConstraints {
  SupportHierarchy supportHierarchy; 
  ObjectJointInteractions objectJointInteractions;
};

//! Retrieves and places models in a scene
class ModelPlacer {
 public:
  //! Initializes this model placer
  void init(Database* db);

  //! Retrieves and places models for a skeleton range.
  //! Returns the number of model instances placed, with placed objected pushed onto out.
  int place(ModelPlacerParams& params, vec<ModelInstance>* out) const;

  //! Adjust placment of models for a skeleton range.
  //! Returns the number of model instances placed, with placed objected pushed onto out.
  int adjust(ModelPlacerParams& params, vec<ModelInstance>* pModelInstances) const;

 private:
  //! Retrieves and places models in a scan for a skeleton range.
  //! Returns the number of model instances placed, with placed objected pushed onto out.
  int placeInScan(ModelPlacerParams& params, 
                  vec<ModelInstance>* out) const;

  //! Retrieves and places models in a scan for a skeleton with a given interaction
  //! Returns the number of model instances placed, with placed objected pushed onto out.
  int placeInScan(RetrievalStrategy retrievalStrategy, ModelPlacerParams& params, // NOLINT
                  const set<string>& filterCategories,
                  vec<ModelInstance>* out) const;

  //! Retrieves and places models in a new room for a skeleton with a given interaction
  //! Returns the number of model instances placed, with placed objected pushed onto out.
  int placeInRoom(const ModelPlacerParams& params, 
                  vec<ModelInstance>* out) const;

  //! Retrieves and places models in a scan for a skeleton with a given interaction
  int placeForInteraction(const interaction::Interaction& interaction,
                          RetrievalStrategy retrievalStrategy,
                          ModelPlacerParams& params, // NOLINT
                          const set<string>& filterCategories,
                          vec<ModelInstance>* out) const;

  //! Retrieves and places models in a scan for a given skeleton range using a given IG
  int placeWithIG(const interaction::InteractionGraph& ig,
                  ModelPlacerParams& params,  // NOLINT
                  const set<string>& filterCategories,
                  vec<ModelInstance>* out) const;

  //! Retrieves and places models in a scan for a given skeleton range by predicting labels using a given PIG
  int placeWithPIG(const interaction::ProtoInteractionGraph& pig,
                   ModelPlacerParams& params,  // NOLINT
                   const set<string>& filterCategories,
                   vec<ModelInstance>* out) const;

  //! Retrieves and places models in a scan for a set of active segments
  int placeWithAnnotation(const segmentation::VecConstSegPtr& segs, 
                          ModelPlacerParams& params, // NOLINT
                          const set<string>& filterCategories,
                          vec<ModelInstance>* out) const;

  //! Retrieves and places models in a scan for a set of segments belonging to one category
  int placeWithCategorizedSegs(const segmentation::VecConstSegPtr& segs, const string& category,
                               ModelPlacerParams& params,  // NOLINT
                               vec<ModelInstance>* out) const;

  //! Retrieves and places models in a scan for a set of categorized segments
  int placeWithCategorizedSegs(const map<string, segmentation::VecConstSegPtr>& segsByCategory,
                               ModelPlacerParams& params,  // NOLINT
                               vec<ModelInstance>* out) const;

  //! Retrieves relevant models and joint interaction
  int retrieve(const ModelPlacerParams& params,
               vec<ModelInstance>* pModels,
               ObjectJointInteractions* pObjJointInteractions) const;

  //! Create support hierarchy
  int buildSupportHierarchy(const ModelPlacerParams& params,
                            const string& sceneType,
                            vec<ModelInstance>& selectedModels,
                            SupportHierarchy* pSupport) const;

  //! Places models using basic support hierarchy
  void placeUsingSupport(const ModelPlacerParams& params,
                         const Skeleton& skel,
                         const SupportHierarchy& supportHierarchy,
                         vec<ModelInstance>& selectedModels,
                         PlacementState& state) const;

  //! Places models using PIG and support hierarchy
  void placeUsingPIGSupport(const ModelPlacerParams& params,
                            const Skeleton& skel,
                            const SupportHierarchy& supportHierarchy,
                            const ObjectJointInteractions& objJointInteractions,
                            vec<ModelInstance>& selectedModels,
                            PlacementState& state) const;

  //! Prepares models by ensuring segmentation and voxels
  void prepareModels(const ModelPlacerParams& params,
                     const vec<ModelInstance>& models) const;

  //! Database from which we search for models and and such
  Database* m_pDatabase = nullptr;
  ModelRetriever m_retriever;
};

}  // namespace synth
}  // namespace core
}  // namespace sg


