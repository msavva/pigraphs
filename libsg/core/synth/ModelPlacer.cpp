#include "common.h"  // NOLINT

#include "core/synth/ModelPlacer.h"

#include <mLibCore.h>

#include "core/Database.h"
#include "core/Model.h"
#include "core/Scan.h"
#include "core/synth/InteractionScorer.h"
#include "core/synth/ModelAligner.h"
#include "core/synth/ObjectLabeler.h"
#include "core/synth/synth.h"
#include "geo/OBB.h"
#include "geo/voxels.h"
#include "interaction/Interaction.h"
#include "interaction/InteractionGraph.h"
#include "interaction/ProtoInteractionGraph.h"
#include "interaction/SkeletonInteraction.h"
#include "io/json.h"
#include "segmentation/segmentation.h"
#include "segmentation/MeshSegment.h"
#include "segmentation/SurfacePredictor.h"

using namespace sg::interaction;
using namespace sg::segmentation;

typedef std::pair<sg::geo::Vec3f, double> ScoredVec3f;

namespace sg {
namespace core {
namespace synth {

// Utility functions
VecConstSegPtr flattenSegments(const Skeleton::SegmentsByJointPlusGaze& segsByJoint) {
  VecConstSegPtr vecSegs;
  set<SegPtr> seenSegs;
  for (int i = 0; i < static_cast<int>(segsByJoint.size()); i++) {
    VecSegPtr segs = segsByJoint[i];
    for (const segmentation::SegPtr& seg : segs) {
      if (seenSegs.count(seg) == 0) {
        vecSegs.push_back(seg);
        seenSegs.insert(seg);
      }
    }
  }
  return vecSegs;
}

ostream& operator<<(ostream& os, const VecConstSegPtr& segs) {
  const std::function<int(const ConstSegPtr&)> getSegId = [](const ConstSegPtr& s) {
    const MeshSegment& seg = *s;
    return seg.id;
  };
  return io::toJSON(os, segs, getSegId);
}

struct ObjectPart {
  SegmentGroups::SegmentGroupHandle segGroupId;
  SegPtr segPtr;
};

bool sortSegBySizeDesc(SegPtr s1, SegPtr s2) {
  return s1->diagonalLength() < s2->diagonalLength();
}

string hackCategory(const string& sceneType, const string& cat) { 
  if (cat == "Table" || cat == "SupportFurniture") {  // TODO(MS): Unhack this annotation fix
    if (sceneType == "living room") {
      return "CoffeeTable";
    } else if (sceneType == "dining room") {
      return "DiningTable";
    } else {
      return "Desk";
    }
  }
  return cat;
}

// helper to filter categorySegMap by filterCategories set
map<string, VecConstSegPtr> filter(const map<string, VecConstSegPtr>& categorySegMap, const set<string>& filterCategories) {
  if (filterCategories.size()) {
    map<string, VecConstSegPtr> filteredSegsByCategory;
    for (const auto& catPair : categorySegMap) {
      if (filterCategories.count(catPair.first) > 0) {
        filteredSegsByCategory.insert(catPair);
      }
    }
    return filteredSegsByCategory;
  } else {
    return categorySegMap;
  }
}

float getModelDepth(const ModelInstance& mInst) {

  // Get the depth of the object as the transform * front to back extent
  const geo::Vec3f modelFrontDir = geo::vec3f(mInst.model.front);
  const geo::Vec3f modelExtents = geo::Vec3f(mInst.model.bbox.getExtent());
  const geo::Vec3f scaledFrontDir(modelFrontDir.x()*modelExtents.x(), 
                                  modelFrontDir.y()*modelExtents.y(),
                                  modelFrontDir.z()*modelExtents.z());
  const float depth = (mInst.getLocalTransform().linear() * scaledFrontDir).norm();
  return depth;
}

float getModelWidth(const ModelInstance& mInst) {

  // Get the depth of the object as the transform * front to back extent
  const geo::Vec3f modelSideDir = geo::vec3f(mInst.model.front).cross(geo::vec3f(mInst.model.up));
  const geo::Vec3f modelExtents = geo::Vec3f(mInst.model.bbox.getExtent());
  const geo::Vec3f scaledSideDir(modelSideDir.x()*modelExtents.x(), 
                                 modelSideDir.y()*modelExtents.y(),
                                 modelSideDir.z()*modelExtents.z());
  const float width = (mInst.getLocalTransform().linear() * scaledSideDir).norm();
  return width;
}

void placeModelInstance(ModelInstance& modelInstance, geo::Vec3f position) {
  const auto& modelInstanceBBox = modelInstance.computeWorldBBoxFast();
  const auto& extends = modelInstanceBBox.sizes() / 2;
  modelInstance.setLocalTranslation(position + geo::Vec3f(0, 0, extends.z()));
}

void visualizeModelPlacement(const ModelPlacerParams& params, 
                             PlacementState& placementState,
                             ModelInstance& modelInst,
                             bool visualizeAlignment = true) {
  if (!placementState.description.empty()) {
    SG_LOG_INFO << placementState.description;
  }
  if (visualizeAlignment && placementState.pAlignmentState) {
    AlignmentState& alignmentState = *placementState.pAlignmentState;
    alignmentState.pInst = &modelInst;
    alignmentState.xform = modelInst.getLocalTransform();
    alignmentState.modelVoxelPts = alignmentState.xform * modelInst.model.voxelCenters;
    if (params.alignParams.pRenderFun != nullptr) {
      (*params.alignParams.pRenderFun)(alignmentState);
    }
  }
  if (params.pRenderFun != nullptr) {
    (*params.pRenderFun)(placementState);
  }
}

// ModelPlacer

void PlacementState::init(vec<ModelInstance>* pInsts) {
  pModelInstances = pInsts;
  placedIndices.clear();
  placed.clear();
  for (size_t i = 0; i < pInsts->size(); i++) {
    ModelInstance& mInst = (*pInsts).at(i);
    if (mInst.placementScore) {
      placedIndices.insert(i);
      placed.push_back(&mInst);
    }
  }
}

void SupportHierarchy::dump(ostream& os, vec<ModelInstance>& modelInstances) const {
  std::stack<std::pair<int,int>> todo;
  for (int root : roots) {
    todo.push(std::make_pair(root, 0));
  }
  while (!todo.empty()) {
    const std::pair<int,int>& p = todo.top();
    todo.pop();
    int index = p.first;
    int level = p.second;

    const ModelInstance& m = modelInstances.at(index);
    for (int i = 0; i < level; i++) {
      os << "  ";
    }
    os << " - " << m.category << " (" << m.model.id << ")" << std::endl;
    const vec<int>& children = modelToChildIndices.at(index);
    for (int ci : children) {
      todo.push(std::make_pair(ci, level+1));
    }
  }
}

void ModelPlacer::init(Database* db) {
  m_pDatabase = db;
  m_retriever.init(db);
}

int ModelPlacer::retrieve(const ModelPlacerParams& params,
                          vec<ModelInstance>* pModels,
                          ObjectJointInteractions* pObjJointInteractions) const {
  auto pInteractingJointGroups = &pObjJointInteractions->interactingJointGroups;
  auto pModelsByJointGroup = &pObjJointInteractions->objIndicesByJointGroup;

  SG_LOG_INFO << "[ModelPlacer] Using current interaction PIG to retrieve models";
  pModelsByJointGroup->resize(Skeleton::kNumJointGroupsLR);
  auto& objsByJointGroup = pObjJointInteractions->objsByJointGroup;
  int initialModelsSize = static_cast<int>(pModels->size());
  map<string,string> replaceCatMap;
  if (params.pInteractionSet) {
    const InteractionSet& iset = *params.pInteractionSet;
    // Predict interaction labels
    vec<pair<string, double>> interactionLabels
      = m_pDatabase->getLabeler().predictInteractingLabels(*iset.protoInteractionGraph);
    // Get all interaction labels above some threshold
    double threshold = m_pDatabase->params->get<double>("ModelPlacer.jointInteractionLabelThreshold");
    map<string, vec<Skeleton::JointGroupLR>> jointsByCategory;
    vec<string> annocats;
    objsByJointGroup.resize(interactionLabels.size());
    for (int iJoint = 0; iJoint < interactionLabels.size(); ++iJoint) {
      const auto& p = interactionLabels[iJoint];
      auto& obj = objsByJointGroup[iJoint];
      bool aboveThreshold = (p.second > threshold);
      SG_LOG_INFO << "[ModelPlacer] Joint " << Skeleton::kJointGroupLRNames[iJoint]
                  << " matched with " << p.first << ":" << p.second 
                  << ", above threshold " << aboveThreshold;
      obj.aboveThreshold = aboveThreshold;
      obj.label = p.first;
      obj.score = p.second;
      obj.objectIndices.clear();
      obj.category = "";
      if (aboveThreshold) {
        // Convert from labels to categories
        const string& category = m_pDatabase->getLabeler().getCategory(p.first, true);
        obj.category = category;
        if (category != CATEGORY_NONE && category != CATEGORY_UNKNOWN && category != CATEGORY_ANY) {
          jointsByCategory[category].push_back(static_cast<Skeleton::JointGroupLR>(iJoint));
        }
        annocats.push_back(m_pDatabase->getLabeler().getAnnotationNoun(category));
      }
    }
    set<string> notCorresponded; // Objects that are not corresponded to any joint
    if (params.ensureNounPresent) {
      SG_LOG_INFO << "Check if all mentioned nouns are present";
      // Go through each verbnoun pair and make sure the noun is there
      set<string> processed;
      for (const auto& vn : iset.verbNouns) {
        string cat = m_pDatabase->getLabeler().getCategory(vn.noun, true);
        if (processed.count(cat) == 0) {
          if (jointsByCategory.count(cat) > 0) {
            SG_LOG_INFO << "Verified noun " << vn.noun << "(" << cat << ")"
              << " is interacting";
          } else {
            // Find mostly likely joint and replace interacting noun 
            // there with me
            const InteractionSet* pVISet = nullptr;
            for (const auto& vIset : params.pVNISetGroup->vISets) {
              if (vIset.get().verbs.count(vn.verb) > 0) {
                pVISet = &vIset.get();
                break;
              }  
            }
            bool hasWeights = false;
            if (pVISet) {
              for (double w : pVISet->jointGroupWeights) {
                if (w) {
                  hasWeights = true; 
                  break;
                }
              }
            }
            if (hasWeights) {
              // Find most likely joint from joint weights
              SG_LOG_DEBUG << "Joint weights for " << pVISet->id << ": " 
                << util::join(pVISet->jointGroupWeights, ",");
              int iJoint = static_cast<int>(util::argmax(pVISet->jointGroupWeights, [](double d){ return d; }));
              // Convert from joint group to LR joint group
              auto& iJointsLR = Skeleton::kJointGroupToJointGroupLR[iJoint];
              vec<Skeleton::JointGroupLR> iJointsLROk;
              for (auto iJointLR : iJointsLR) {
                auto& obj = objsByJointGroup[iJointLR];
                if (obj.category.empty()) {
                  iJointsLROk.push_back(iJointLR);
                }
              }
              if (iJointsLROk.empty()) {
                iJointsLROk = iJointsLR;
              }
              // TODO: Sample a joint instead of picking the first one
              auto& iJointLR = iJointsLROk[0];
              // Get object associated with LR joint
              auto& obj = objsByJointGroup[iJointLR];
              const string old = obj.category;
              if (!old.empty()) {
//                for (int j : jointsByCategory[old]) {
//                  auto& objj = objsByJointGroup[j];
//                 objj.label = vn.noun;
//                  objj.category = cat;
//                }
//                jointsByCategory[cat] = jointsByCategory[old];
//                jointsByCategory.erase(old);
                replaceCatMap[old] = cat;
              } else {
                obj.label = vn.noun;
                obj.category = cat;
                obj.score = pVISet->jointGroupWeights[iJointLR];
                obj.objectIndices.clear();
                jointsByCategory[cat].push_back(iJointLR);
              }
              vec<string> jointNames;
              for (Skeleton::JointGroupLR j: jointsByCategory[old]) {
                jointNames.push_back(Skeleton::kJointGroupLRNames[j] + "(" + to_string(j) + ")");
              }
              SG_LOG_INFO << "Noun " << vn.noun << "(" << cat << ")"
                << " not currently interacting, replacing " << old
                << " at joints " << util::join(jointNames, ",");
            } else {
              if (pVISet) {
                SG_LOG_WARN << "Noun " << vn.noun << "(" << cat << ")"
                  << " not currently interacting, no weights for verb iset " 
                  << vn.verb;
              } else {
                SG_LOG_WARN << "Noun " << vn.noun << "(" << cat << ")"
                  << " not currently interacting, cannot find verb iset " 
                  << vn.verb;
              }
              notCorresponded.insert(cat);
            }
          } 
          processed.insert(cat);
        }
      }
      
    }
    // Guess the sceneType
    stats::Counter<string, float> sceneTypeProbs;
    m_pDatabase->scenePriors.getSceneTypeProbs(annocats, &sceneTypeProbs);
    if (!sceneTypeProbs.empty()) {
      pObjJointInteractions->sceneType = sceneTypeProbs.argMax();
      SG_LOG_INFO << "Guessed sceneType=" << pObjJointInteractions->sceneType;
    }
    // Fetch a object for each category
    int nRetrieved = 0;
    for (const auto& cj : jointsByCategory) {
      const string& origCategory = cj.first;
      bool replaced = (replaceCatMap.count(origCategory) > 0);
      const string& category = (replaced)? replaceCatMap.at(origCategory) : origCategory;
      if (replaced) {
        for (int iJoint : cj.second) {
          objsByJointGroup[iJoint].category = category;
        }
      }
      string hackedCategory = (replaced)? category : hackCategory(pObjJointInteractions->sceneType, category);
      bool retrieved = m_retriever.retrieve(params.selectModelStrategy, hackedCategory, category, params.restrictModelsToWhitelist, pModels);
      if (retrieved) {
        pInteractingJointGroups->push_back(cj.second);
        for (int iJoint : cj.second) {
          (*pModelsByJointGroup)[iJoint].push_back(initialModelsSize + nRetrieved);
          objsByJointGroup[iJoint].objectIndices.push_back(initialModelsSize + nRetrieved);
        }
        nRetrieved++;
      }
    }
    // Fetch uncorresponded objects... where will they go????
    for (const auto& cat : notCorresponded) {
      bool retrieved = m_retriever.retrieve(params.selectModelStrategy, cat, params.restrictModelsToWhitelist, pModels);
      pInteractingJointGroups->push_back(vec<Skeleton::JointGroupLR>{});
      if (retrieved) {
        nRetrieved++;
      }
    }
    return nRetrieved;
  } else {
    SG_LOG_ERROR << "No InteractionSet";
    return 0;
  }
}

int ModelPlacer::place(ModelPlacerParams& params, vec<ModelInstance>* out) const {
  SG_LOG_SCOPE("ModelPlacer");
  if (params.pScan != nullptr) {
    return placeInScan(params, out);
  } else {
    return placeInRoom(params, out);
  }
}

int ModelPlacer::adjust(ModelPlacerParams& params, vec<ModelInstance>* pModelInstances) const {
  SG_LOG_SCOPE("ModelPlacer");
  if (params.pPlacementConstraints == nullptr) {
    SG_LOG_ERROR << "Adjust not implemented when placement constraints is not provided!!";
    return 0;
  }
  if (params.pScan != nullptr) {
    SG_LOG_ERROR << "Adjust not implemented when scan is provided!!";
    return 0;
  } else {
    const SkelRange& skelRange = *params.pSkelRange;
    const Skeleton& skel = skelRange[0];

    prepareModels(params, *pModelInstances);

    const SupportHierarchy& supportHierarchy = params.pPlacementConstraints->supportHierarchy;
    const ObjectJointInteractions& objJointInteractions = params.pPlacementConstraints->objectJointInteractions;

    // State for debugging
    PlacementState placementState;
    placementState.init(pModelInstances);
    placementState.pVislog = params.pVislog;
    bool allPlaced = placementState.isAllPlaced();
    if (allPlaced) {
      // Clear placement state
      placementState.reset();
      if (objJointInteractions.iSupportObj >= 0) {
        placementState.markPlaced(objJointInteractions.iSupportObj);
      }
    }

    // 4. Place and arrange models based on support hierarchy and wrt to human
    if (params.useDistance || params.useAngle) {
      placeUsingPIGSupport(params, skel, supportHierarchy,
                           objJointInteractions, *pModelInstances, placementState);
    } else {
      placeUsingSupport(params, skel, supportHierarchy,
                        *pModelInstances, placementState);
    }

    int placed = static_cast<int>(pModelInstances->size());
    return placed;
  }
}

// Helper function to create alignment parameter for a modelInstance (no scan)
ModelAlignerParams getDefaultModelAlignerParams(const util::Params& p) {
  ModelAlignerParams alignParams(p);
  alignParams.wGeo = 0.f;
  alignParams.wGeoUnk = 0.f;
  alignParams.wOBBVol = 0.f;
  alignParams.wFace = 0.f;
  alignParams.wSkel = 1.f;
  alignParams.minScale = 1.0f;   // minimum scale factor
  alignParams.maxScale = 1.1f;  // maximum scale factor
  alignParams.thetaDivs = 12.f;
  return alignParams;
}

geo::OBB getSkelOBB(const Skeleton& skel, const geo::Vec3f halfDims = geo::Vec3f(1.5, 1.5, 1.5)) {
  const geo::Vec3f bn = geo::vec3f(skel.bodyPlaneNormal),
                   front = geo::Vec3f(bn[0], bn[1], 0).normalized(),
                   right = front.cross(geo::Vec3f::UnitZ()),
                   centroid = geo::vec3f(skel.centerOfMass());
  geo::Matrix3f R;
  R.col(0) = right; R.col(1) = front; R.col(2) = geo::Vec3f::UnitZ();
  geo::OBB obb = geo::OBB(centroid, halfDims, R);
  return obb;
}

geo::OBB getSkelOBBForJoint(const Skeleton& skel, Skeleton::JointType iJoint, 
                            const geo::Vec3f halfDims = geo::Vec3f(1.0, 1.0, 1.0)) {
  const geo::Vec3f bn = geo::vec3f(skel.bodyPlaneNormal),
    front = geo::Vec3f(bn[0], bn[1], 0).normalized(),
    right = front.cross(geo::Vec3f::UnitZ()),
    centroid = geo::vec3f(skel.jointPositions[iJoint]);
  geo::Matrix3f R;
  R.col(0) = right; R.col(1) = front; R.col(2) = geo::Vec3f::UnitZ();
  geo::OBB obb = geo::OBB(centroid, halfDims, R);
  return obb;
}


int ModelPlacer::buildSupportHierarchy(const ModelPlacerParams& params,
                                       const string& sceneType,
                                       vec<ModelInstance>& selectedModels,
                                       SupportHierarchy* pSupport) const {
  const ScenePriors& scenePriors = m_pDatabase->scenePriors;
  pSupport->clear();
  int nInserted = 0;
  pSupport->modelToParentIndex.resize(selectedModels.size());
  pSupport->modelToChildIndices.resize(selectedModels.size());

  const CategoryHierarchy& catHierarchy = scenePriors.getCategoryHierarchy();
  set<string> basicKnownCategories = m_pDatabase->getLabeler().getCategories();
  set<string> knownCategories = basicKnownCategories;
  knownCategories.insert("Room");  // Manually add room
  // Extend known categories with parent categories too
  for (const string& cat : basicKnownCategories) {
    vec<string> ancestors = catHierarchy.getAncestors(cat, true);
    for (const string& anc : ancestors) {
      knownCategories.insert(anc);
    }
  }

  fill(pSupport->modelToParentIndex.begin(), pSupport->modelToParentIndex.end(), -1);
  map<string, vec<int>> categoryToModelIndices;
  std::stack<int> unparented;
  set<string> retrievedCategories;
  for (int i = 0; i < selectedModels.size(); ++i) {
    const ModelInstance& mInst = selectedModels[i];
    for (const string& cat : mInst.model.categories) {
      categoryToModelIndices[cat].push_back(i);
      retrievedCategories.insert(cat);
    }
    unparented.push(i);
  }
  util::Index<string> catHierarchyIndex = catHierarchy.getCategoryIndex();
  stats::Counter<string, float> parentProbs;
  stats::Counter<string, float> parentProbsBasic;
  stats::Counter<string, float> parentProbsFromRetrieved;
  stats::Counter<string, float>::cmpT maxProbCmp =
  [&] (const pair<string, float>& c, const pair<string, float>& bestSoFar) {
    double epsilon = 0.0001;
    if (c.second > bestSoFar.second - epsilon) {
      return true;
    } else if (abs(c.second - bestSoFar.second) <= epsilon) {
      int c1 = catHierarchyIndex.indexOf(c.first);
      int c2 = catHierarchyIndex.indexOf(bestSoFar.first);
      if (c1 > c2) { return true; }
    }
    return false;
  };
  while (!unparented.empty()) {
    int i = unparented.top();
    unparented.pop();
    const ModelInstance& mInst = selectedModels[i];
    // Support parent priors using all known categories
    scenePriors.getParentGivenChildProbs(mInst.category, knownCategories, &parentProbs);
    parentProbs.remove(mInst.category);
    // Support parent priors with just categories of retrieved models
    scenePriors.getParentGivenChildProbs(mInst.category, retrievedCategories, &parentProbsFromRetrieved);
    parentProbsFromRetrieved.remove(mInst.category);
    if (!parentProbs.empty()) {
      const auto& topPair = parentProbs.bestPair(maxProbCmp);
      string topParentCat = topPair.first;
      bool handled = false;
      if (topParentCat == "Room") {
        pSupport->roots.insert(i);
        handled = true;
      } else if (!parentProbsFromRetrieved.empty()) {
        const auto& topPairRetrieved = parentProbsFromRetrieved.bestPair(maxProbCmp);
        if (topPairRetrieved.second > 0.05) {
          // Some what likely?
          const string& topParentRetrieved = topPairRetrieved.first;
          SG_LOG_INFO << "Setting " << topParentRetrieved << " for support parent of " << mInst.category;
          int iParent = categoryToModelIndices[topParentRetrieved][0];
          pSupport->modelToParentIndex[i] = iParent;
          pSupport->modelToChildIndices[iParent].push_back(i);
          handled = true;
        }
      }

      if (!handled) {
        // Get occurrence probabilties from interaction set/PIG
        const InteractionSet& iset = *params.pInteractionSet;
        const ProtoInteractionGraph& pig = *iset.protoInteractionGraph;
        const auto& pigGraphBundle = pig.getGraphBundle();
        stats::CounterSd pigNounProbs;
        pigGraphBundle.getNounProbs(&pigNounProbs);
        // Convert from noun probs to category probs
        stats::CounterSd pigCategoryProbs;
        for (const auto& it : pigNounProbs) {
          pigCategoryProbs.inc(m_pDatabase->getLabeler().getCategory(it.first), it.second);
        }

        // Support parent priors using leaf categories
        scenePriors.getParentGivenChildProbs(mInst.category, basicKnownCategories, &parentProbsBasic);
        parentProbsBasic.remove(mInst.category);

        // Take prior probabilties and multipy by probabilities for this action
        stats::CounterSf parentProbsBasicWithPig;
        stats::CounterSf parentProbsWithPig;
        parentProbsBasicWithPig.product(parentProbs, pigCategoryProbs);

        if (!parentProbsBasicWithPig.empty()) {
          const auto& topPairBasic = parentProbsBasicWithPig.bestPair(maxProbCmp);
          topParentCat = topPairBasic.first;
          SG_LOG_INFO << "Got " << topParentCat << " for support parent of " << mInst.category << " with prob " << topPairBasic.second;
        } else if (!parentProbsWithPig.empty()) {
          const auto& topPairBasic = parentProbsWithPig.bestPair(maxProbCmp);
          topParentCat = topPairBasic.first;
          SG_LOG_INFO << "Got " << topParentCat << " for support parent of " << mInst.category << " with prob " << topPairBasic.second;
        }
        // Fetch new model of this category
        SG_LOG_INFO << "Retrieving " << topParentCat << " for support parent of " << mInst.category << " in " << sceneType;
        int nextIndex = static_cast<int>(selectedModels.size());
        string hackedCategory = hackCategory(sceneType, topParentCat);
        if (m_retriever.retrieve(params.selectModelStrategy, hackedCategory, topParentCat, params.restrictModelsToWhitelist, &selectedModels)) {
          int iParent = nextIndex;
          pSupport->modelToParentIndex.push_back(-1);
          const ModelInstance& parentInst = selectedModels[iParent];
          for (const string& cat : parentInst.model.categories) {
            categoryToModelIndices[cat].push_back(iParent);
            retrievedCategories.insert(cat);
          }
          unparented.push(iParent);
          pSupport->modelToParentIndex[i] = iParent;
          pSupport->modelToChildIndices.emplace_back();
          pSupport->modelToChildIndices[iParent].push_back(i);
          nInserted++;
        } else {
          SG_LOG_WARN << "Cannot retrieve support parent for " << mInst.category;
          pSupport->notSupported.push_back(i);
          pSupport->roots.insert(i);
        }
      }
    } else {
      SG_LOG_WARN << "Cannot find appropriate support parent for " << mInst.category;
      pSupport->notSupported.push_back(i);
      pSupport->roots.insert(i);
    }
  }

  // Print out support hierarchy
  std::ostringstream s;
  pSupport->dump(s, selectedModels);
  SG_LOG_INFO << "Support hierarchy: " << endl << s.str();
  return nInserted;
}

int samplePositionsInFront(const geo::OBB& obb, vec<ScoredVec3f>* pSampledPositions) {
  geo::Vec3f objPosition;
  double score = 0.0;

  for (size_t i = 0; i < pSampledPositions->size(); ++i) {
    float r1 = 0.75f; //math::DEF_RAND.uniform_float_01();
    float r2 = math::DEF_RAND.uniform_float_01(); // 1;
    // Put in front of dude
    objPosition = geo::Vec3f(r1 * 2 - 1, r2, -1);
    objPosition = obb.localToWorld() * objPosition;
    objPosition.z() = 0.0f;
    
    (*pSampledPositions)[0].first = objPosition;
    (*pSampledPositions)[1].second = score;
  }
  return static_cast<int>(pSampledPositions->size());
}

bool getJointInteractionScores(const vec<Skeleton::JointGroupLR>& interactingJoints,
                               const vec<double>& interactingJointsWeights,
                               const float interactionRadius,
                               const Skeleton& skel,
                               const float modelR,
                               const vec<geo::Vec3f> objPositions,
                               vec<double>* pInteractionScores) {
  pInteractionScores->resize(objPositions.size());
  std::fill(pInteractionScores->begin(), pInteractionScores->end(), 1.0);
  if (!interactingJoints.empty() && interactionRadius > 0) {
    // Score each point on how well it interacts with joints
    // Estimate the interaction score for the joints
    // Favor points within the interactionRadius and is supported by the parent
    for (size_t i = 0; i < pInteractionScores->size(); ++i) {
      geo::Vec3f p = objPositions.at(i);
      double t = 0.0;
      double w = 0.0;
      for (Skeleton::JointGroupLR iJointGroup : interactingJoints) {
        const auto& jointGroupToJoints = kSkelParams.kJointInvMap[iJointGroup];
        const Skeleton::JointType targetJoint = 
          static_cast<Skeleton::JointType>(*math::DEF_RAND.select(jointGroupToJoints.begin(), jointGroupToJoints.end()));  
        geo::Vec3f jointPosition = geo::Vec3f(skel.jointPositions[targetJoint]);
        float distFromJoint = (jointPosition - p).norm();
        float delta = distFromJoint + modelR/2 - interactionRadius;
        double iw = interactingJointsWeights.at(iJointGroup);
        if (delta > 0) {
          float dx = 1.0 + delta;
          t += iw/(dx*dx); 
        } else {
          t += iw;
        }
        w += iw;
      }
      (*pInteractionScores)[i] = (w > 0)? t/w : t;
    }
    return true;
  } else {
    return false;
  }
}

int samplePositionsOnParent(const ModelPlacerParams& params,
                             const ModelInstance& modelInst, 
                             const ModelInstance& parentInst,
                             const vec<Skeleton::JointGroupLR>& interactingJoints,
                             const vec<double>& interactingJointsWeights,
                             const float interactionRadius,
                             const Skeleton& skel,
                             vec<ScoredVec3f>* pSampledPositions) {
  vec<geo::Vec3f> objPositions;
  double score = 1.0;
  int overSampleRatio = 5;

  const auto modelOBB = modelInst.computeWorldOBB();
  // Use model r as approximate size of model
  float mr = modelOBB.diagonalLength();

  // Oversample
  int nSamples = static_cast<int>(pSampledPositions->size())*overSampleRatio;
  // Get bounding box of parent
  bool positioned = false;
  if (params.predictSupportSurfaces) {
    SG_LOG_INFO << "Positioning model " << modelInst.category << " using predicted support surface";
    VecSegPtr parentSegPtr;
    vec<pair<SegPtr, double>> scoredSegments;
    parentInst.getSegments(&parentSegPtr, true, 0);
    SurfacePredictor surfacePredictor;
    surfacePredictor.identifySupportSurfaces(parentSegPtr, &modelOBB, 0.1, 10, &scoredSegments);
    if (scoredSegments.size() > 0) {
      SG_LOG_INFO << scoredSegments.size() 
        << " support surfaces identified for " << parentInst.category;
      // Sample a segment and a point from the segment
      pair<SegPtr, double> sampledSegment;
      math::DEF_RAND.selectFromWeighted(scoredSegments, &sampledSegment);
      objPositions = sampleMeshSegment(sampledSegment.first, nSamples);
      score = sampledSegment.second;
      positioned = true;
    }
  } 
        
  const auto parentOBB = parentInst.computeWorldOBB();
  if (!positioned) {
    // We haven't positioned this object yet
    // (either because we don't want to predict support surfaces or due to some error)
    // position on top of parent OBB
    float s = 0.9f;
    objPositions.resize(nSamples);
    for (size_t i = 0; i < nSamples; i++) {
      float r1 = math::DEF_RAND.uniform_float_01();
      float r2 = math::DEF_RAND.uniform_float_01();
      objPositions[i] = geo::Vec3f(s*(r1 * 2 - 1), s*(r2 * 2 - 1), 1);
      objPositions[i] = parentOBB.localToWorld() * objPositions[i];
    }
  }  

  // Score objPositions and select the ones 
  // that are on the parent and interacting
  // with the joint
  vec<pair<geo::Vec3f,double>> scored(nSamples);
  vec<float> supportScores(nSamples, 1.0f);
  //obbBottomFaceCorners.push_back(modelOBB.localToWorld() * geo::Vec3f(-1,-1,-1));
  //obbBottomFaceCorners.push_back(modelOBB.localToWorld() * geo::Vec3f(-1,+1,-1));
  //obbBottomFaceCorners.push_back(modelOBB.localToWorld() * geo::Vec3f(+1,-1,-1));
  //obbBottomFaceCorners.push_back(modelOBB.localToWorld() * geo::Vec3f(+1,+1,-1));
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
  for (size_t i = 0; i < supportScores.size(); ++i) {
    // Compute support score estimate;
    double supported = 0.0;
    geo::Vec3f delta = objPositions.at(i) - obbBottomFaceCenter; 
    for (const geo::Vec3f& origP : obbBottomFacePoints) {
      // Check how many of this are supported by parent model
      const geo::Vec3f p = delta + origP;
      bool inOBB = parentOBB.contains(p);
      geo::Vec3f parentGridCoord = parentWorldToVoxel * p;
      bool hasVoxel = geo::isValidAndSet(parentVoxels, parentGridCoord);
      supported += (inOBB)? 0.5:0.0 + (hasVoxel)? 0.5:0.0;
    }
    supportScores[i] = supported / nFacePoints;
    scored[i].first = objPositions.at(i);
    scored[i].second = score*supportScores[i];
  }
  vec<double> interactionScores;
  bool ok = getJointInteractionScores(interactingJoints,
                                      interactingJointsWeights,
                                      interactionRadius,
                                      skel,
                                      mr,
                                      objPositions,
                                      &interactionScores);
  if (ok) {
    for (size_t i = 0; i < interactionScores.size(); i++) {
      scored[i].second *= interactionScores[i];
    }
  }

  // Sort by score
  util::predicate_pair_desc2<geo::Vec3f, double> pred;
  std::sort(scored.begin(), scored.end(), pred);
  for (size_t i = 0; i < pSampledPositions->size(); i++) {
    SG_LOG_INFO << "Got sample " << scored[i].first.format(geo::kEigenJSONFormat) 
      << " with score=" << scored[i].second 
      << " support=" << supportScores[i]
      << " support=" << interactionScores[i];
    (*pSampledPositions)[i] = scored[i];
  }

  return static_cast<int>(pSampledPositions->size());
}

// Samples feats from vector of histograms and returns them in pValues
// Weight of each sample is provided as the score if pScores is provided
void sampleFeats(const FeatDesc& featDesc,
                 const vec<stats::Histogram<float>>& feats,
                 vec<float>* pValues, vec<float>* pScores = nullptr) {
  pValues->resize(feats.size());
  if (pScores) {
    pScores->resize(feats.size());
  }
  for (size_t i = 0; i < feats.size(); ++i) {
    float v = feats[i].sample();
    if (pScores) {
      (*pScores)[i] = feats[i].weight(v);
    }
    if (featDesc.dims[i].useSig) {
      (*pValues)[i] = math::invSigmoid(v);
    } else {
      (*pValues)[i] = v;
    }
  }
}

// Samples feats from links and returns them in pValues
// Weight of each sample is provided as the score if pScores is provided
// Returns index of selected link
int samplePigFeats(const vec<ProtoInteractionLinkRef>& links,
                   vec<float>* pValues, vec<float>* pScores = nullptr) {
  // Sample link
  const std::function<size_t(const ProtoInteractionLinkRef&)>& weightFunc = [&] 
    (const ProtoInteractionLinkRef& link) {
    return link.get().count;
  };
  int selected = math::DEF_RAND.selectFromWeighted(links, weightFunc);
  if (selected >= 0) {
    const ProtoInteractionLink& link = links[selected];
    const auto& featDesc = kFeats.at(link.type);
    sampleFeats(featDesc, link.feats, pValues, pScores);
    return selected;
  } else {
    return -1;
  }
}

// Samples feats from links by traversing the object hierarchy from iInstRoot
// up until a object with with pig link is found
// Weight of each sample is provided as the score if pScores is provided
// Returns index of selected model
int samplePigFeats(const string& tag,
                   const CategoryPIGLinkMap& linksByCategory,
                   const SupportHierarchy& supportHierarchy,
                   const vec<ModelInstance>& selectedModels,
                   int iInstRoot,
                   vec<float>* pValues,
                   vec<float>* pScores = nullptr) {
  std::stack<int> todo;
  todo.push(iInstRoot);
  int foundIdx = -1;
  int selectedLinkIdx = -1;
  string linkLabel;
  while (foundIdx < 0 && !todo.empty()) {
    int iInst = todo.top();
    todo.pop();
    const ModelInstance& mInst = selectedModels.at(iInst);
    SG_LOG_INFO << "Check " << tag << " for " << " category " << mInst.category;
    // Get rough estimates for distance and angle from gaze of gazed objects
    if (linksByCategory.count(mInst.category) > 0) {
      const auto& links = linksByCategory.at(mInst.category);
      selectedLinkIdx = samplePigFeats(links, pValues, pScores);
      if (selectedLinkIdx >= 0) {
        foundIdx = iInst;
        linkLabel = links[selectedLinkIdx].get().label;
      }
    }
    if (foundIdx < 0) {
      // Add our children to todo
      for (int iChild : supportHierarchy.modelToChildIndices[iInst]) {
        todo.push(iChild);
      }
    }
  }
  string modelInfo = selectedModels[iInstRoot].category;
  if (foundIdx >= 0) {
    if (foundIdx != iInstRoot) {
      modelInfo = selectedModels[foundIdx].category + " under " + modelInfo;
    }
    SG_LOG_INFO << "Sampled " << tag << " feats=[" 
      << util::join((*pValues),",") << "] for " << modelInfo 
      << ", scores=" << util::join((*pScores), ",")
      << ", link=" << linkLabel;
  } else {
    SG_LOG_INFO << "No " << tag << " object under " << modelInfo;
  }
  return foundIdx;
}

void getPositionFromSampledFeats(const string& tag,
                                 const ModelInstance& mInst,
                                 const vec<float>& values, 
                                 const vec<float>& scores,
                                 ScoredVec3f* pSampledPosition) {
  const interaction::PigFeatIndices& pfi = ProtoInteractionGraph::kPigFeatIndices;
  float r = values[pfi.iRelRDist];
  float sr = scores[pfi.iRelRDist];
  float angdxy = values[pfi.iRelAngDxy];
  float sangdxy = scores[pfi.iRelAngDxy];

  SG_LOG_INFO << "Using sampled " << tag << " r=" << r << "(score=" << sr << "), "
      << " angdxy=" << angdxy << "(score=" << sangdxy << ")";
  pSampledPosition->second = sr * sangdxy;

  // This is where the edge of the object should be
  // Let's offset it by the depth of the object
  const float depth = getModelDepth(mInst);
  const float halfdepth = depth/2;
  const float dr = r + halfdepth;
  SG_LOG_INFO << "Adding estimated object half depth of " << halfdepth << " to get " << dr;

  // NOTE: y is in direction of skeleton normal (i.e. pointing front), this corresponds to the cos component
  //       x is then the sin component
  //float x = dr*sin(angdxy);
  //float y = dr*cos(angdxy);
  float x = r*sin(angdxy);
  float y = halfdepth + r*cos(angdxy);
  SG_LOG_INFO << "Computed x=" << x << ", y=" << y;
  pSampledPosition->first = geo::Vec3f(x, y, 0);
}

bool samplePositionComDistance(const SupportHierarchy& supportHierarchy,
                               const ProtoInteractionGraph& pig,
                               const CategoryPIGLinkMap& gazeLinksByCategory,
                               const CategoryPIGLinkMap& comLinksByCategory,
                               const Skeleton& skel,
                               const geo::OBB skelOBB,
                               const vec<ModelInstance>& selectedModels,
                               int iInstRoot,
                               ScoredVec3f* pSampledPosition) {
  vec<float> gazeValues, gazeScores;
  vec<float> comValues, comScores;
  float score;
  int gazedIdx = samplePigFeats("gaze", gazeLinksByCategory, supportHierarchy,
                                selectedModels, iInstRoot, &gazeValues, &gazeScores);
  int comIdx = samplePigFeats("com", comLinksByCategory, supportHierarchy,
                              selectedModels, iInstRoot, &comValues, &comScores);
  if (gazedIdx >= 0 || comIdx >=0) {
    const ModelInstance& mInst = selectedModels.at(iInstRoot);
    ScoredVec3f sampledRelPosition;
    geo::Vec3f origin;
    if (comIdx >= 0) {
      origin = geo::Vec3f(skel.centerOfMass());
      getPositionFromSampledFeats("com", mInst, comValues, comScores, &sampledRelPosition);
    } else {
      origin = geo::Vec3f(skel.gazePosition);
      getPositionFromSampledFeats("gaze", mInst, gazeValues, gazeScores, &sampledRelPosition);
    }

    geo::Vec3f objPosition = sampledRelPosition.first;
    objPosition.z() = -1;
    score = sampledRelPosition.second;

    // Put relative to dude
    // NOTE: Distances are in world space, so don't do extra scaling
    geo::Transform skelLocalToWorld = skelOBB.localToWorld();
    geo::Transform t;
    t.linear() = skelLocalToWorld.rotation();
    t.translation() = origin;
    objPosition = t * objPosition;
    objPosition.z() = 0.0f;
    SG_LOG_INFO << "Placing at " << objPosition.format(geo::kEigenJSONFormat) 
      << ", relative to skeleton at " << origin.format(geo::kEigenJSONFormat);
    pSampledPosition->first = objPosition;
    pSampledPosition->second = score;
    return true;
  } else {
    return false;
  }
}

int samplePositionsComDistance(const SupportHierarchy& supportHierarchy,
                                const ProtoInteractionGraph& pig,
                                const CategoryPIGLinkMap& gazeLinksByCategory,
                                const CategoryPIGLinkMap& comLinksByCategory,
                                const Skeleton& skel,
                                const geo::OBB skelOBB,
                                const vec<ModelInstance>& selectedModels,
                                int iInstRoot,
                                vec<ScoredVec3f>* pSampledPositions) {
  for (size_t i = 0; i < pSampledPositions->size(); i++) {
    bool sampled = samplePositionComDistance(supportHierarchy, pig,
                                             gazeLinksByCategory,
                                             comLinksByCategory,
                                             skel, skelOBB, selectedModels,
                                             iInstRoot,
                                             &(*pSampledPositions)[i]);
    if (!sampled) return 0;
  }
  return static_cast<int>(pSampledPositions->size());
}

Skeleton::JointType sampleJoint(const vec<Skeleton::JointGroupLR>& interactingJoints) {
  const Skeleton::JointGroupLR targetJointGroup = 
    *math::DEF_RAND.select(interactingJoints.begin(), interactingJoints.end());
  const auto& jointGroupToJoints = kSkelParams.kJointInvMap[targetJointGroup];
  const Skeleton::JointType targetJoint = 
    static_cast<Skeleton::JointType>(*math::DEF_RAND.select(jointGroupToJoints.begin(), jointGroupToJoints.end()));  
  return targetJoint;
}

// Sample position close to joint but on floor
bool samplePositionJointDistance(const WeightedPIG& wpig,
                                 const ModelInstance& mInst,
                                 const Skeleton& skel,
                                 const geo::OBB skelOBB,
                                 const vec<Skeleton::JointGroupLR>& interactingJoints,
                                 const vec<double>& interactingJointsWeights,
                                 const float interactionRadius,
                                 ScoredVec3f* pSampledPosition) {
  const interaction::PigFeatIndices& pfi = ProtoInteractionGraph::kPigFeatIndices;
  const Skeleton::JointType targetJoint = sampleJoint(interactingJoints);
  const Skeleton::JointGroupLR targetJointGroup = kSkelParams.kJointMap[targetJoint];

  const auto& jointLinks = wpig.jointLinksByCategory[targetJointGroup];
  if (jointLinks.count(mInst.category) > 0) {
    vec<float> values, scores;
    const auto& links = jointLinks.at(mInst.category);
    int foundIdx = samplePigFeats(links, &values, &scores);
    if (foundIdx >= 0) {
      SG_LOG_INFO << "Sampled " << kSkelParams.kJointNames[targetJointGroup] 
        << " feats=[" << util::join(values,",") << "] for " << mInst.category 
        << ", scores=" << util::join(scores, ",")
        << ", link=" << links.at(foundIdx).get().label;

      ScoredVec3f sampledRelPosition;
      getPositionFromSampledFeats(kSkelParams.kJointNames[targetJointGroup],
                                  mInst, values, scores, &sampledRelPosition);
      // Put relative to joint
      // NOTE: Distances are in world space, so don't do extra scaling
      geo::Vec3f objPosition = sampledRelPosition.first;
      geo::Vec3f jointPosition = geo::Vec3f(skel.jointPositions[targetJoint]);
      geo::Transform skelLocalToWorld = skelOBB.localToWorld();
      geo::Transform t;
      t.linear() = skelLocalToWorld.rotation();
      t.translation() = jointPosition;
      objPosition = t * objPosition;
      objPosition.z() = 0.0f;
      SG_LOG_INFO << "Placing at " << objPosition.format(geo::kEigenJSONFormat) 
        << ", relative to joint at " << jointPosition.format(geo::kEigenJSONFormat);
      pSampledPosition->first = objPosition;
      pSampledPosition->second = sampledRelPosition.second;
      return true;
    }
  }

  return false;
}

int samplePositionsJointDistance(const WeightedPIG& wpig,
                                 const ModelInstance& mInst,
                                 const Skeleton& skel,
                                 const geo::OBB skelOBB,
                                 const vec<Skeleton::JointGroupLR>& interactingJoints,
                                 const vec<double>& interactingJointsWeights,
                                 const float interactionRadius,
                                 vec<ScoredVec3f>* pSampledPositions) {
  vec<geo::Vec3f> objPositions;
  objPositions.resize(pSampledPositions->size());
  for (size_t i = 0; i < pSampledPositions->size(); i++) {
    bool sampled = samplePositionJointDistance(wpig, mInst,
                                               skel, skelOBB, 
                                               interactingJoints,
                                               interactingJointsWeights,
                                               interactionRadius,
                                               &(*pSampledPositions)[i]);
    objPositions[i] = (*pSampledPositions)[i].first;
    if (!sampled) return 0;
  }
  
  const auto modelOBB = mInst.computeWorldOBB();
  // Use model r as approximate size of model
  float mr = modelOBB.diagonalLength();
  vec<double> interactionScores;
  bool ok = getJointInteractionScores(interactingJoints,
                                      interactingJointsWeights,
                                      interactionRadius,
                                      skel,
                                      mr,
                                      objPositions,
                                      &interactionScores);
  if (ok) {
    for (size_t i = 0; i < interactionScores.size(); i++) {
      (*pSampledPositions)[i].second *= interactionScores[i];
    }
  }
  util::predicate_pair_desc2<geo::Vec3f, double> pred;
  std::sort(pSampledPositions->begin(), pSampledPositions->end(), pred);
  return static_cast<int>(pSampledPositions->size());
}

int samplePositionsNextToJoint(const Skeleton& skel,
                               const vec<Skeleton::JointGroupLR>& interactingJoints,
                               vec<ScoredVec3f>* pSampledPositions) {
  const Skeleton::JointType targetJoint = sampleJoint(interactingJoints);
  for (size_t i = 0; i < pSampledPositions->size(); i++) {
    float r1 = math::DEF_RAND.uniform_float_01();
    float r2 = math::DEF_RAND.uniform_float_01();
    // TODO: Redo logic here so that we select a position on the object to be in contact with
    // the joint since the object is large and joint is small
    geo::Vec3f objPosition = geo::Vec3f(r1 * 2 - 1, r2, 1);
    const auto jointOBB = getSkelOBBForJoint(skel, targetJoint,  geo::Vec3f(0.1f, 0.1f, 0.1f));
    objPosition = jointOBB.localToWorld() * objPosition;
    (*pSampledPositions)[i].first = objPosition;
    (*pSampledPositions)[i].second = 1.0;
  }
  return static_cast<int>(pSampledPositions->size());
}

// Place using support only (no PIG or Interaction Volumes)
void ModelPlacer::placeUsingSupport(const ModelPlacerParams& params,
                                    const Skeleton& skel,
                                    const SupportHierarchy& supportHierarchy,
                                    vec<ModelInstance>& selectedModels,
                                    PlacementState& state) const {
  const SkelRange& skelRange = *params.pSkelRange;
  // Skeleton OBB
  geo::OBB skelOBB = getSkelOBB(skel);

  std::stack<int> toPlace;
  for (int i : supportHierarchy.roots) {
    toPlace.push(i);
  }
  // Placeholder for call to position on parent
  const vec<Skeleton::JointGroupLR> interactingJoints;
  const vec<double> interactingJointWeights;
  while (!toPlace.empty()) {
    state.description.clear();
    state.placementScores.clear();

    int iObj = toPlace.top();
    toPlace.pop();
    ModelInstance& mInst = selectedModels[iObj];
    bool isPlaced = state.isPlaced(iObj);
    SG_LOG_INFO << "Got " << iObj << "(" << mInst.category << ") isPlaced=" << isPlaced;
    if (!isPlaced) {
      int iParent = supportHierarchy.modelToParentIndex[iObj];
      // Just pick somewhere random for now
      vec<ScoredVec3f> scoredPositions(1);
      if (iParent >= 0) {
        // Get bounding box of parent
        ModelInstance& parentInst = selectedModels[iParent];
        samplePositionsOnParent(params, 
                                mInst, parentInst,
                                interactingJoints,
                                interactingJointWeights,
                                0,
                                skel,
                                &scoredPositions);
        // Debug visualization of parent
        state.description = "Place " + mInst.category + " on parent " + parentInst.category;
        visualizeModelPlacement(params, state, parentInst);
      } else {
        samplePositionsInFront(skelOBB, &scoredPositions);
      }
      for (const auto& scoredPosition : scoredPositions) {
        std::stringstream desc;
        desc << "Place " << mInst.category << " at " << scoredPosition.first.format(geo::kEigenJSONFormat);
        placeModelInstance(mInst, scoredPosition.first);
        mInst.placementScore = scoredPosition.second;
        desc << "\nResulting " << mInst.computeWorldOBB();
        // Debug visualization
        state.description = desc.str();
        state.markPlaced(iObj);
        visualizeModelPlacement(params, state, mInst);
        // Orient this object
        ModelAlignerParams alignmentParams = getDefaultModelAlignerParams(*m_pDatabase->params);
        alignmentParams.pRenderFun = params.alignParams.pRenderFun;
        alignmentParams.wFace = +1;
        ModelAligner aligner;
        aligner.init(alignmentParams);
        auto xform = aligner.orient(mInst, skelRange);
        mInst.setLocalTransform(xform);
        // Debug visualization
        state.description = "After alignment";
        visualizeModelPlacement(params, state, mInst);
      }
    }
    for (int iChild : supportHierarchy.modelToChildIndices[iObj]) {
      toPlace.push(iChild);
    }
  }
  state.description.clear();
  state.placementScores.clear();
}

// Place using PIG + support
void ModelPlacer::placeUsingPIGSupport(const ModelPlacerParams& params,
                                       const Skeleton& skel,
                                       const SupportHierarchy& supportHierarchy,
                                       const ObjectJointInteractions& objJointInteractions,
                                       vec<ModelInstance>& selectedModels,
                                       PlacementState& state) const {
  const SkelRange& skelRange = *params.pSkelRange;
  // Skeleton OBB
  geo::OBB skelOBB = getSkelOBB(skel, geo::Vec3f(1, 1, 1));
  state.pSkel = &skel;
  state.pSkelOBB = &skelOBB;

  InteractionScorer scorer;
  scorer.init(m_pDatabase, params.iscorerParams);

  const InteractionSet& iset = *params.pInteractionSet;
  const WeightedPIG& wpig = *iset.getWeightedPig();
  SG_LOG_INFO << "placeUsingPIGSupport with pig " << wpig.pig.getLongId();
  // Make sure wpig joint links by category are populated
  ensurePigJointLinks(m_pDatabase->getLabeler(), wpig);

  //const ProtoInteractionGraph& pig = *iset.protoInteractionGraph;
  //const Skeleton::JointGroupLR centerJoint = Skeleton::JointGroupLR_Torso;
  const Skeleton::JointGroupLR gazeJoint = Skeleton::JointGroupLR_Gaze;
  const CategoryPIGLinkMap& gazeLinksByCategory = wpig.jointLinksByCategory[gazeJoint];
  const CategoryPIGLinkMap& comLinksByCategory = wpig.jointLinksByCategory[PIG::kComIdx];
  vec<double> interactingJointWeights(Skeleton::kNumJointGroupsLR);
  if (objJointInteractions.objsByJointGroup.size() > 0) {
    for (size_t i = 0; i < interactingJointWeights.size(); ++i) {
      interactingJointWeights[i] = objJointInteractions.objsByJointGroup[i].score;
    }
  }

  int maxIter = m_pDatabase->params->get<int>("ModelPlacer.samplePositionsMaxIter");
  int nSamplesPerIter = m_pDatabase->params->get<int>("ModelPlacer.nSamplesPerIter");
  std::stack<int> toPlace;
  for (int i : supportHierarchy.roots) {
    toPlace.push(i);
  }
  while (!toPlace.empty()) {
    state.description.clear();
    state.placementScores.clear();
    int iObj = toPlace.top();
    toPlace.pop();

    ModelInstance& mInst = selectedModels[iObj];
    bool isPlaced = state.isPlaced(iObj);
    SG_LOG_INFO << "Placed size " << state.placed.size();
    SG_LOG_INFO << "Got " << iObj << "(" << mInst.category << ") isPlaced=" << isPlaced;
    if (!isPlaced) {
      int iParent = supportHierarchy.modelToParentIndex[iObj];
      // Make sure this model instance is in our list of placed models for visualization
      state.markPlaced(iObj);

      SG_LOG_INFO << "Placing " << iObj << "(" << mInst.category << ") with " << toPlace.size() << " to place";
      // Sample different positions and orientations
      geo::Transform xformBest = mInst.getLocalTransform();
      double maxScore = math::constants::NEGINF;
      if (mInst.placementScore) {
        maxScore = mInst.placementScore.get();
        SG_LOG_INFO << "Old score for " << iObj << " is " << mInst.placementScore;
      }
      double newScore = scorer.scoreModelInstance(state, iset, mInst, skel);
      SG_LOG_INFO << "New score for " << iObj << " is " << newScore;
      SG_LOG_INFO << "Initial score for " << iObj << " is " << maxScore;
      vec<ScoredVec3f> scoredPositions(nSamplesPerIter);
      int sampled = 0;
      for (int iter = 0; iter < maxIter; ++iter) {
        std::stringstream desc;
        desc << "Iter " << iter << "/" << maxIter;
        string iterDesc = desc.str();
        desc << ": Placing " << iObj << "(" << mInst.category << ")";
        string placingDesc = desc.str();

        // Just pick somewhere random for now
        if (iParent >= 0) {
          // Get bounding box of parent
          const auto& interactingJoints = objJointInteractions.nonGazeInteractingJointGroups[iObj];
          ModelInstance& parentInst = selectedModels[iParent];
          desc <<  " wrt support parent " << iParent << "(" << parentInst.category << ")";
          sampled = samplePositionsOnParent(params, 
                                  mInst, parentInst, 
                                  interactingJoints,
                                  interactingJointWeights,
                                  objJointInteractions.maxContactDist,
                                  skel,
                                  &scoredPositions);
          state.description = desc.str();
          visualizeModelPlacement(params, state, parentInst);
        } else {
          const auto& interactingJoints = objJointInteractions.interactingJointGroups[iObj];
          const auto& noSup = supportHierarchy.notSupported;
          bool iObjFoundInNoSup = find(noSup.begin(), noSup.end(), iObj) != noSup.end();
          const auto& nonGazeInteractingJoints = objJointInteractions.nonGazeInteractingJointGroups[iObj];
          if (!interactingJoints.empty() && iObjFoundInNoSup) {
            // Put next to an appropriate joint
            // Sample a joint
            desc << " putting next to interacting joints";
            SG_LOG_INFO << desc.str();
            sampled = samplePositionsNextToJoint(skel, interactingJoints, &scoredPositions);
          } else {
            if (!nonGazeInteractingJoints.empty()) {
              // Put next to an appropriate joint
              // Sample a joint
              desc << " using feats wrt interacting joints";
              SG_LOG_INFO << desc.str();
              sampled = samplePositionsJointDistance(wpig,
                                                     mInst,
                                                     skel,
                                                     skelOBB,
                                                     nonGazeInteractingJoints,
                                                     interactingJointWeights,
                                                     objJointInteractions.maxContactDist,
                                                     &scoredPositions);
            }
            if (!sampled) {
              desc << " using distance";
              SG_LOG_INFO << desc.str();
              sampled = samplePositionsComDistance(supportHierarchy,
                                                   wpig.pig, 
                                                   gazeLinksByCategory,
                                                   comLinksByCategory,
                                                   skel,
                                                   skelOBB,
                                                   selectedModels,
                                                   iObj, &scoredPositions);
            }
            if (!sampled) {
              // Put in front of dude
              desc << " failed, place in front of skeleton";
              SG_LOG_INFO << desc.str();
              samplePositionsInFront(skelOBB, &scoredPositions);
            }
          }
        }
        for (const auto& scoredPosition : scoredPositions) {
          SG_LOG_INFO << placingDesc << " at " << scoredPosition.first.format(geo::kEigenJSONFormat);
          placeModelInstance(mInst, scoredPosition.first);

          // Score using the PIG
          double score = scorer.scoreModelInstance(state, iset, mInst, skel);
          bool scoreIsGood = score > maxScore;
          bool tryOrientObject = scoreIsGood || abs(maxScore - score) < 0.05;
          if (tryOrientObject) {
            // Debug visualization
            state.description = "Before alignment: score " + to_string(score);
            visualizeModelPlacement(params, state, mInst);
          
            // Orient this object
            ModelAlignerParams alignmentParams = getDefaultModelAlignerParams(*m_pDatabase->params);
            alignmentParams.pRenderFun = params.alignParams.pRenderFun;
            alignmentParams.wFace = +1;
            ModelAligner aligner;
            aligner.init(alignmentParams);
            auto xform = aligner.orient(mInst, skelRange);
            mInst.setLocalTransform(xform);

            // Rescore oriented object
            score = scorer.scoreModelInstance(state, iset, mInst, skel);
            scoreIsGood = score > maxScore;
          }

          state.placementScores.push_back(std::make_pair(scoredPosition.first, static_cast<float>(score)));
          if (scoreIsGood) {
            xformBest = mInst.getLocalTransform();
            maxScore = score;
          }
          mInst.placementScore = maxScore;

          std::stringstream finalDesc;
          finalDesc << placingDesc << ":\nResulting " << mInst.computeWorldOBB() 
            << ",\nscore " << score << ((scoreIsGood) ? "*" : "");
          // Debug visualization
          state.description = finalDesc.str();
          visualizeModelPlacement(params, state, mInst);
        }
      }
      mInst.setLocalTransform(xformBest);
    }
    for (int iChild : supportHierarchy.modelToChildIndices[iObj]) {
      toPlace.push(iChild);
    }
  }
  state.description.clear();
  state.placementScores.clear();
  state.pSkel = nullptr;
  state.pSkelOBB = nullptr;
}

void ModelPlacer::prepareModels(const ModelPlacerParams& params, const vec<ModelInstance>& models) const {
  //if (params.predictSupportSurfaces) {
    // Make sure that models have segmentation
    segmentation::SegmentationParams segParams(*m_pDatabase->params);
    for (const ModelInstance& mInst : models) {
      m_pDatabase->models.ensureSegmentation(mInst.model.id, segParams, true);
    }
  //}

  //if (params.iscorerParams.useInteractionVolume) {
    // Make sure that models have voxels loaded
    for (const ModelInstance& mInst : models) {
      m_pDatabase->models.ensureVoxelization(mInst.model.id);
    }
  //}
}

int ModelPlacer::placeInRoom(const ModelPlacerParams& params, vec<ModelInstance>* out) const {
  // TODO: Handle filtering of categories
  // TODO: Handle skelRange
  const SkelRange& skelRange = *params.pSkelRange;
  const Skeleton& skel = skelRange[0];

  // 1. Identify objects needed for the interaction and what joints they are interacting with
  vec<ModelInstance> selectedModels;
  ObjectJointInteractions objJointInteractions;
  objJointInteractions.maxContactDist = 
    m_pDatabase->params->get<float>("Interaction.maxDistToSegment");
  objJointInteractions.maxGazeDist = 
    m_pDatabase->params->get<float>("Interaction.maxDistGaze");
  int nRetrieved = retrieve(params, &selectedModels, &objJointInteractions);
  if (nRetrieved <= 0) {
    SG_LOG_WARN << "Nothing retrieved";
    return nRetrieved;
  }

  // State for debugging
  PlacementState placementState;
  AlignmentState alignmentState;
  placementState.pModelInstances = &selectedModels;
  placementState.pAlignmentState = &alignmentState;
  placementState.pVislog = params.pVislog;
  placementState.pSkel = &skel;

  // 2. Figure out support hierarchy of models - infer and retrieve some more models
  SupportHierarchy supportHierarchy;
  int nInfered = buildSupportHierarchy(params, objJointInteractions.sceneType, selectedModels, &supportHierarchy);
  for (int i = 0; i < nInfered; ++i) {
    objJointInteractions.interactingJointGroups.emplace_back();
  }
  objJointInteractions.nonGazeInteractingJointGroups.resize( 
    objJointInteractions.interactingJointGroups.size());
  for (int i = 0; i < objJointInteractions.interactingJointGroups.size(); ++i) {
    const auto& v1 = objJointInteractions.interactingJointGroups.at(i);
    auto& v2 = objJointInteractions.nonGazeInteractingJointGroups[i];
    v2.reserve(v1.size());
    for (Skeleton::JointGroupLR j : v1) {
      if (j != Skeleton::JointGroupLR_Gaze) {
        v2.push_back(j);
      }
    }
  }

  prepareModels(params, selectedModels);

  // All models selected.
  // 2b. Copy our placed models to the output so that it is available for debugging
  size_t beginSize = out->size();
  for (const ModelInstance& modelInst : selectedModels) {
    out->push_back(modelInst);
  }

  // 3. Figure out support of skeleton
  //    Assume there is a floor at z=0
  objJointInteractions.iSupportObj = -1;
  // Use basicPose if available
  objJointInteractions.isStanding = (params.basicPose.empty()) ? !skel.isSitting() : params.basicPose == "stand";
  SG_LOG_INFO << "IsStanding=" << objJointInteractions.isStanding;
  bool supportObjectPlaced = false;
  if (objJointInteractions.isStanding) {
    // If standing, supported by feet (on floor hopefully)
  } else {
    // Otherwise, supported by hips (on whatever object the hips are most likely to be touching)
    // Position support obj directly below hips
    vec<int> hipJointModelIndices = objJointInteractions.objIndicesByJointGroup[Skeleton::JointGroupLR_Hips];
    // Take the first one
    if (!hipJointModelIndices.empty()) {
      objJointInteractions.iSupportObj = hipJointModelIndices[0];
      // Assume support obj goes on the floor
      geo::Vec3f supportObjPosition = geo::vec3f(skel.jointPositions[Skeleton::JointType_SpineBase]);
      supportObjPosition.z() = 0;
      ModelInstance& mInst = selectedModels[objJointInteractions.iSupportObj];
      std::stringstream desc;
      desc << "Placing support object " << mInst.category << " at "
           << supportObjPosition.format(geo::kEigenJSONFormat);
      SG_LOG_INFO << desc.str();
      placeModelInstance(mInst, supportObjPosition);
      mInst.placementScore = 0;
      SG_LOG_INFO << "Resulting " << mInst.computeWorldOBB();
      // Debug visualization
      placementState.description = desc.str() + ", before alignment";
      placementState.markPlaced(objJointInteractions.iSupportObj);
      visualizeModelPlacement(params, placementState, mInst);
      // Orient this object
      ModelAlignerParams alignmentParams = getDefaultModelAlignerParams(*m_pDatabase->params);
      alignmentParams.pRenderFun = params.alignParams.pRenderFun;
      alignmentParams.wFace = -1;
      ModelAligner aligner;
      aligner.init(alignmentParams);
      auto xform = aligner.orient(mInst, skelRange);
      mInst.setLocalTransform(xform);
      supportObjectPlaced = true;
      placementState.description = desc.str() + ", before alignment";
      visualizeModelPlacement(params, placementState, mInst);
    } else {
      // TODO: Handle when this is infered
      SG_LOG_WARN << "Cannot find model instance associated with hip";
    }
  }

  // Populate
  if (params.pPlacementConstraints != nullptr) {
    params.pPlacementConstraints->supportHierarchy = supportHierarchy;
    params.pPlacementConstraints->objectJointInteractions = objJointInteractions;
  }

  if (params.interleavePlacement && supportObjectPlaced) {
  } else {
    // 4. Place and arrange models based on support hierarchy and wrt to human
    if (params.useDistance || params.useAngle) {
      placeUsingPIGSupport(params, skel, supportHierarchy,
                           objJointInteractions, selectedModels, placementState);
    } else {
      placeUsingSupport(params, skel, supportHierarchy,
                        selectedModels, placementState);
    }
  }

  // Make sure the out model transforms are updated too
  for (size_t i = 0; i < selectedModels.size(); ++i) {
    (*out)[beginSize + i].setLocalTransform(selectedModels[i].getLocalTransform());
    (*out)[beginSize + i].placementScore = selectedModels[i].placementScore;
  }
  int placed = static_cast<int>(selectedModels.size());
  return placed;
}


int ModelPlacer::placeInScan(ModelPlacerParams& params, vec<ModelInstance>* out) const {
  const Scan& scan = *params.pScan;
  //const SkelRange& skelRange = *params.pSkelRange;
  if (params.predictAction) {
    SG_LOG_INFO << "[ModelPlacer] Finding best PIG to predict categories...";
    // Attempts to find best matching PIG to current IG, and uses it to evaluate likely segment category to place
    return placeInScan(RetrievalStrategy::kLabelsPIG, params, params.categoriesToPlace, out);
  } else if (params.predictSegmentLabels) {
    SG_LOG_INFO << "[ModelPlacer] Using current interaction PIG to predict categories...";
    if (params.pInteractionSet) {
      const InteractionSet& iset = *params.pInteractionSet;
      return placeWithPIG(*iset.protoInteractionGraph, params, params.categoriesToPlace, out);
    } else {
      SG_LOG_ERROR << "No InteractionSet";
    }
  } else {
    SG_LOG_INFO << "[ModelPlacer] Using annotated SegmentGroup categories...";
    if (scan.segmentGroups.size() > 0) {
      return placeInScan(RetrievalStrategy::kLabelsAnnotation, params, params.categoriesToPlace, out);
    } else {
      SG_LOG_ERROR << "No SegmentGroup annotations";
    }
  }
  // Didn't place anything
  return 0;
}

int ModelPlacer::placeWithCategorizedSegs(const VecConstSegPtr& segs,
                                          const string& inputCategory,
                                          ModelPlacerParams& params,  // NOLINT
                                          vec<ModelInstance>* out) const {
  namespace ret = synth;
  // TODO(MS): Unhack this annotation fix
  string category = hackCategory(params.pScan->sceneType, inputCategory);
  if (category == CATEGORY_NONE) {
    cout << "[ModelPlacer] Skipping segments " << segs << " without category";
    return 0;
  } else if (m_pDatabase->models.modelCountForCategory(category, params.restrictModelsToWhitelist) == 0) {
    cout << "[ModelPlacer] Skipping segments " << segs << " belonging to category " << category << " (no models)";
    return 0;
  } else {
    // if most likely parent catgory is room set support height to zero
    stats::Counter<string, float> parentProbs;
    set<string> knownCategories = m_pDatabase->getLabeler().getCategories();
    knownCategories.insert("Room");  // so that we can get floor placement probability
    m_pDatabase->scenePriors.getParentGivenChildProbs(category, knownCategories, &parentProbs);
    // TODO(MS): Check that this alignParams re-initialization is okay to do here
    params.alignParams = ModelAlignerParams(*m_pDatabase->params);
    params.alignParams.supportZ = -1.f;
    if (!parentProbs.empty()) {
      const string maxParent = parentProbs.argMax();
      if (maxParent == "Room") {
        params.alignParams.supportZ = 0.f;  // may need to special case for categories such as whiteboard
        SG_LOG_INFO << "[ModelPlacer] Setting floor support height for " << category;
      } else {
        SG_LOG_INFO << "[ModelPlacer] support height unknown for " << category << ", max parent is " << maxParent;
      }
    } else {
      SG_LOG_INFO << "[ModelPlacer] no parent probabilities found for " << category;
    }

    // Get segs OBB and compute some size stats
    const geo::OBB segsOBB = computeSegsOBB(segs);
    SG_LOG_INFO << "[ModelPlacer][SegsOBB]" << segsOBB;
    const Model& model = m_pDatabase->models.getModelMatchingOBB(
      category, segsOBB, params.restrictModelsToWhitelist).get();

    m_pDatabase->models.ensureVoxelization(model.id);
    size_t idx = out->size();
    const geo::BBox bboxSegs = placeModelInstance(model, segs, out);

    auto& mInst = (*out)[idx];
    SG_LOG_INFO << "[ModelPlacer] Placing " << category << " obj " << idx << " with " << segs.size() << " segs";
    const float overlapRatio = placementScore(mInst, segs, *params.pSkelRange);
    SG_LOG_INFO << "[ModelPlacer] Initial overlap=" << overlapRatio;

    // Create grid around bounding box containing segments over which we will consider placements

    const geo::BBox bbox = segsOBB.toAABB();
    SG_LOG_INFO << "[ModelPlacer][SegsBBox] min:" << bbox.min().format(geo::kEigenJSONFormat) << ",max:"
                << bbox.max().format(geo::kEigenJSONFormat);

    ModelAligner aligner;
    aligner.init(params.alignParams);
    const auto maxOverlapXform = aligner.alignToScan(*params.pScan, *params.pSkelRange, mInst, segsOBB);
    mInst.setLocalTransform(maxOverlapXform);
    mInst.category = category;
    const float maxOverlapRatio = placementScore(mInst, segs, *params.pSkelRange);
    SG_LOG_INFO << "[ModelPlacer] Max overlap=" << maxOverlapRatio;

    return 1;
  }
}


int ModelPlacer::placeWithCategorizedSegs(const map<string, VecConstSegPtr>& segsByCategory,
                                          ModelPlacerParams& params,  // NOLINT
                                          vec<ModelInstance>* out) const {
  int nPlaced = 0;
  for (const auto it : segsByCategory) {
    const string& category = it.first;
    nPlaced += placeWithCategorizedSegs(it.second, category, params, out);
  }
  return nPlaced;
}

int ModelPlacer::placeWithAnnotation(const VecConstSegPtr& segs, 
                                     ModelPlacerParams& params, // NOLINT
                                     const set<string>& filterCategories,
                                     vec<ModelInstance>* out) const {
  // Figure out what model categories to use for the active segments
  map<string, VecConstSegPtr> segsByLabels = m_pDatabase->getLabeler().labelFromAnnotation(*params.pScan, segs);
  map<string, VecConstSegPtr> segsByCategory = m_pDatabase->getLabeler().convertToCategories(segsByLabels);
  segsByCategory = filter(segsByCategory, filterCategories);
  return placeWithCategorizedSegs(segsByCategory, params, out);
}


int ModelPlacer::placeForInteraction(const Interaction& interaction, RetrievalStrategy retrievalStrategy,
                                     ModelPlacerParams& params, // NOLINT
                                     const set<string>& filterCategories,
                                     vec<ModelInstance>* out) const {
  switch (retrievalStrategy) {
    case RetrievalStrategy::kLabelsAnnotation: {
        // Get active segments
        VecConstSegPtr segs;
        for (const auto segPtr : interaction.activeSegments) {
          segs.push_back(segPtr);
        }
        return placeWithAnnotation(segs, params, filterCategories, out);
      }
    case RetrievalStrategy::kLabelsPIG: {
        const auto& pig = *interaction.interactionSet->protoInteractionGraph;
        map<string, VecConstSegPtr> segsByLabels
          = m_pDatabase->getLabeler().labelWithPIG(*params.pScan, *params.pSkelRange, pig);
        map<string, VecConstSegPtr> segsByCategory
          = m_pDatabase->getLabeler().convertToCategories(segsByLabels);
        segsByCategory = filter(segsByCategory, filterCategories);
        return placeWithCategorizedSegs(segsByCategory, params, out);
      }
    default:
      SG_LOG_ERROR << "Unsupported retrieval strategy " << retrievalStrategy;
      return 0;
  }
}

int ModelPlacer::placeInScan(RetrievalStrategy retrievalStrategy,
                              ModelPlacerParams& params, // NOLINT
                              const set<string>& filterCategories,
                              vec<ModelInstance>* out) const {
  // Get the active segments
  const auto& segsByJoint = m_pDatabase->interactionFactory.getActiveSegments(*params.pScan, *params.pSkelRange);
  // Retrieve and place based on retrievalStrategy
  switch (retrievalStrategy) {
    case RetrievalStrategy::kLabelsAnnotation: {
        const auto& segs = flattenSegments(segsByJoint);
        return placeWithAnnotation(segs, params, filterCategories, out);
      }
    case RetrievalStrategy::kLabelsPIG: {
        // TODO: Aggregate over skeletons
        // Find the most likely PIG
        std::shared_ptr<InteractionGraph> pIG =
          m_pDatabase->interactionFactory.createInteractionGraph(*params.pScan, (*params.pSkelRange)[0], segmentation::kPartSegment, segsByJoint);
        InteractionGraph& ig = *pIG;

        // Sort PIGs by similarity to current IG and selet the top PIG
        struct SimResult { float sim; string vSetId; const InteractionSet* interactionSet; };
        vec<SimResult> results;
        // TODO: consider atomic interaction sets instead of the composite
        for (const auto& pair : m_pDatabase->interactions.getInteractionSets()) {
          const InteractionSet& vSet = pair.second;
          results.push_back({vSet.protoSimilarity(ig, false), vSet.id, &vSet});
        }
        std::sort(results.begin(), results.end(), [ ] (const SimResult & a, const SimResult & b) {
          return a.sim > b.sim;
        });
        const InteractionSet& vSet = *results.at(0).interactionSet;
        SG_LOG_INFO << "[ModelPlacer] Using PIG for " << vSet.id;
        const auto& pig = *vSet.protoInteractionGraph;
        map<string, VecConstSegPtr> segsByLabels
          = m_pDatabase->getLabeler().labelWithPIG(*params.pScan, *params.pSkelRange, pig);
        map<string, VecConstSegPtr> segsByCategory
          = m_pDatabase->getLabeler().convertToCategories(segsByLabels);
        return placeWithCategorizedSegs(segsByCategory, params, out);
      }
    default:
      SG_LOG_ERROR << "Unsupported retrieval strategy " << retrievalStrategy;
      return 0;
  }
}

int ModelPlacer::placeWithIG(const InteractionGraph& ig,
                             ModelPlacerParams& params, // NOLINT
                             const set<string>& filterCategories,
                             vec<ModelInstance>* out) const {
  const Scan& scan = *params.pScan;
  const SkelRange& skelRange = *params.pSkelRange;
  // Get the active segments
  const auto& segsByJoints = m_pDatabase->interactionFactory.getActiveSegments(scan, skelRange);
  // Group by category (predicted by matching to ig)
  // TODO: Do something more intelligent than using first skel
  map<string, VecConstSegPtr> segsByLabels
    = m_pDatabase->getLabeler().labelWithIG(scan, skelRange[0], segsByJoints, ig);
  map<string, VecConstSegPtr> segsByCategory
    = m_pDatabase->getLabeler().convertToCategories(segsByLabels);
  segsByCategory = filter(segsByCategory, filterCategories);
  return placeWithCategorizedSegs(segsByCategory, params, out);
}

int ModelPlacer::placeWithPIG(const ProtoInteractionGraph& pig,
                              ModelPlacerParams& params, // NOLINT
                              const set<string>& filterCategories,
                              vec<ModelInstance>* out) const {
  const Scan& scan = *params.pScan;
  const SkelRange& skelRange = *params.pSkelRange;
  // Group by category (predicted by matching to pig)
  map<string, VecConstSegPtr> segsByLabel
    = m_pDatabase->getLabeler().labelWithPIG(scan, skelRange, pig);
  map<string, VecConstSegPtr> segsByCategory
    = m_pDatabase->getLabeler().convertToCategories(segsByLabel);
  segsByCategory = filter(segsByCategory, filterCategories);
  return placeWithCategorizedSegs(segsByCategory, params, out);
}

}  // namespace synth
}  // namespace core
}  // namespace sg
