#pragma once

#include "libsg.h"  // NOLINT
#include "core/Classifier.h"
#include "core/Skeleton.h"
#include "interaction/Interaction.h"
#include "interaction/InteractionFrames.h"
#include "interaction/ProtoInteractionGraph.h"
#include "segmentation/Part.h"

#include <boost/serialization/version.hpp>

namespace sg {
namespace interaction {

//! Representation of an action as a set of concurrently active verbs. Extracted from recorded observations and
//! containing useful entities specific to each action: representative skeletons, pointers to all interactions,
//! joint interaction probabilities, trained classifiers, and interaction graph entities

enum InteractionSetType {
  ISetType_Composite = 0,
  ISetType_VerbNoun  = 1,
  ISetType_Verb  = 2,
  ISetType_All  = 3,
  ISetType_Count = 4
};

struct InteractionSet : public io::Serializable {
  static const int kNumInteractionSetTypes = ISetType_Count;
  static const arr<string, kNumInteractionSetTypes> kInteractionSetNames;
  static const map<string, InteractionSetType> kInteractionSetTypeLookupByName;

  BOOST_BINARY_SERIALIZABLE_FUNCS
  InteractionSet();

  static string getComponentISetId(const set<VerbNoun>& vns) {
    return "c/" + VerbNoun::getVerbNounSetId(vns);
  }
  static string getVerbNounISetId(const VerbNoun& vn) {
    return "vn/" + vn.verb + "_" + vn.noun;
  }
  static string getVerbISetId(const string& vb) { return "v/" + vb; }

  //! Sample skeletons from interactions belonging to this InteractionSet
  void sampleSkeletons(UINT sampleCount);

  //! Cluster skeletons from interactions belonging to this InteractionSet and retain clusterCount as representatives
  void clusterSkeletons(UINT clusterCount);

  //! Compute the activation probabilities for joints and joint groups
  void computeJointActivationProbabilities();

  //! Return the average [0,1] similarity of ig to the PIG for this InteractionSet
  float protoSimilarity(const InteractionGraph& ig, bool useJointWeights) const;

  //! Return the average [0,1] similarity of ig to the IGs this InteractionSet contains
  float similarity(const InteractionGraph& ig) const;

  //! Return the average [0,1] similarity of ig to the top nearestNeighborPercent of IGs in this InteractionSet
  float similarity(const InteractionGraph& ig, float nearestNeighborPercent, int maxNumNeighbors) const;

  std::shared_ptr<WeightedPIG> getWeightedPig() const;
  std::shared_ptr<WeightedPIG> getWeightedPig(const PartType& pt, const string& pigType) const;

  //! Interaction set type ("composite", "verb", or "verbNoun")
  InteractionSetType type;
  //! Aggregated ID: ordered action verbs (correspond to Interactions aggregated into this InteractionSet)
  string id;
  //! Set of verb-noun pairs for this interaction set
  set<VerbNoun> verbNouns;
  //! Set of verbs describing this set of interactions
  set<string> verbs;
  //! Set of nouns associated with this set of interactions
  set<string> nouns;
  //! All annotated interactions
  vec<Interaction*> interactions;
  //! Sampled skeletons for this interaction set
  vec<core::TransformedSkeleton> sampledSkeletons;
  //! Probability for each joint to be activated by action
  arr<double, Skeleton::kNumJoints+1> jointActivationProb;
  //! Probability for each joint group to be activated by action
  arr<double, Skeleton::kNumJointGroups> jointGroupActivationProb;
  //! Joint group weights
  arr<double, Skeleton::kNumJointGroups> jointGroupWeights;

  // These classifiers may be null
  core::SegmentPoseClassifier* segmentPoseClassifier;
  core::SegmentPerJointGroupClassifier* segPerJointGroupClassifier[Skeleton::kNumJointGroups];
  core::SegmentSkelPerJointGroupClassifier* segJointPerJointGroupClassifier[Skeleton::kNumJointGroups];
  core::SegmentJointsAggregatedClassifier* segmentJointsAggregatedClassifier;
  core::SegmentCentroidActivationAggregatedClassifier* segmentCentroidActivationAggregatedClassifier;
  core::SegmentJointsLinearWeightedAggregatedClassifier* segmentJointsLinearWeightedAggregatedClassifier;
  std::shared_ptr<core::InteractionGraphKNNSimilarity> igKNNSimilarityClassifier;

  //! InteractionGraphs extracted from observed interactions that belong to this InteractionSet
  //!   indexed by partType
  vec<vec<std::shared_ptr<InteractionGraph>>> interactionGraphs;
  //! Prototypical interaction graph representing interactions of this InteractionSet
  std::shared_ptr<ProtoInteractionGraph> protoInteractionGraph;
  //! Prototypical (i.e, aggregated) InteractionFrame for all observed interactions in this InteractionSet
  std::shared_ptr<InteractionFrames> protoInteractionFrame;

  //! Prototypical interaction graphs by partType and then aggregation type
  vec<map<string, std::shared_ptr<ProtoInteractionGraph>>> pigs;
private:
  //! cached pig with weights and stuff
  mutable std::shared_ptr<WeightedPIG> m_weightedPig;
  mutable vec<map<string, std::shared_ptr<WeightedPIG>>> m_weightedPigs;
  //! boost serialization function
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {  // NOLINT
    boost::serialization::split_member(ar, *this, version);
  }
  template <typename Archive>
  void save(Archive& ar, const unsigned int version) const {  // NOLINT
    ar & type;
    ar & id;
    ar & jointActivationProb;
    ar & jointGroupActivationProb;
    ar & jointGroupWeights;
    size_t nProtoInteractionGraphs = 0;
    for (const auto& pigsPerPart : pigs) {
      nProtoInteractionGraphs += pigsPerPart.size();
    }
    ar & nProtoInteractionGraphs;
    for (const auto& pigsPerPart : pigs) {
      for (const auto& pigPair : pigsPerPart) {
        ar & *pigPair.second;
      }
    }
    bool hasProtoInteractionGraph = protoInteractionGraph != nullptr; 
    ar & hasProtoInteractionGraph;
    if (hasProtoInteractionGraph) {
      ar & protoInteractionGraph->getPartType();
      ar & protoInteractionGraph->getPigType();
    }

    bool hasProtoInteractionFrame = protoInteractionFrame != nullptr;
    ar & hasProtoInteractionFrame;
    if (hasProtoInteractionFrame) {
      ar & *protoInteractionFrame;
    }
  }
  template <typename Archive>
  void load(Archive& ar, const unsigned int version) {  // NOLINT
    ar & type;
    ar & id;
    // Populate verbNouns, verbs, nouns from id
    populateVerbNounsFromId();
    ar & jointActivationProb;
    ar & jointGroupActivationProb;
    ar & jointGroupWeights;

    // Load PIGs
    bool hasProtoInteractionGraph;
    pigs.clear();
    pigs.resize(segmentation::kPartTypeCount);
    size_t nProtoInteractionGraphs;
    ar & nProtoInteractionGraphs;
    for (size_t i = 0; i < nProtoInteractionGraphs; ++i) {
      std::shared_ptr<ProtoInteractionGraph> pPIG =
        std::make_shared<ProtoInteractionGraph>(id);
      ar & *pPIG;
      pigs[pPIG->getPartType()][pPIG->getPigType()] = pPIG;
    }
    ar & hasProtoInteractionGraph;
    if (hasProtoInteractionGraph) {
      string pigType;
      PartType partType = segmentation::kPartSegment;
      ar & partType;
      ar & pigType;
      protoInteractionGraph = pigs.at(partType).at(pigType);
    } else {
      protoInteractionGraph = nullptr;
    }

    // Load interaction frames
    bool hasProtoInteractionFrame;
    ar & hasProtoInteractionFrame;
    if (hasProtoInteractionFrame) {
      protoInteractionFrame = std::make_shared<InteractionFrames>();
      ar & *protoInteractionFrame;
    } else {
      protoInteractionGraph = nullptr;
    }
  }
  void populateVerbNounsFromId();
};

struct RetargetedPIG {
  string id;
  const InteractionSet* pOrigInteractionSet;
  std::shared_ptr<ProtoInteractionGraph> retargetedPIG;
};

//! Related interaction sets for a set of verbNouns 
struct VerbNounISetGroup {
  string id;
  set<VerbNoun> verbNouns;
  set<string> verbs;
  set<string> nouns;
  const InteractionSet* pInteractionSet;
  InteractionSet compositeInteractionSet; // Composited interaction set
  vec<std::reference_wrapper<InteractionSet>> cISets;
  vec<std::reference_wrapper<InteractionSet>> vISets;
  vec<std::reference_wrapper<InteractionSet>> vnISets;
  map<string, RetargetedPIG> vnToRetargetedMap;
  string basicPose; // "stand", "lie", "sit"

  bool isStanding() const { return basicPose == "stand"; };

  void init(const InteractionSet* pISet) {
    clear();
    if (pISet) {
      pInteractionSet = pISet;
      id = pInteractionSet->id;
      verbNouns = pInteractionSet->verbNouns;
      for (const VerbNoun& vn : pInteractionSet->verbNouns) {
        verbs.insert(vn.verb);
        nouns.insert(vn.noun);
      }    
    }
  }

  void clear() {
    verbNouns.clear();
    verbs.clear();
    nouns.clear();
    basicPose = "";
    clearISets();
  }

  void clearISets() {
    pInteractionSet = nullptr;
    cISets.clear();
    vISets.clear();
    vnISets.clear();
    vnToRetargetedMap.clear();
  }
};

}  // namespace interaction
}  // namespace sg

BOOST_CLASS_VERSION(sg::interaction::InteractionSet, 3)


