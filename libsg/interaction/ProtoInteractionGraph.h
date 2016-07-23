#pragma once

#include "libsg.h"  // NOLINT
#include "graph/Graph.h"
#include "interaction/InteractionGraph.h"
#include "interaction/SkeletonInteraction.h"
#include "segmentation/Part.h"
#include "stats/histogram.h"

#include <boost/serialization/version.hpp>

namespace sg {
namespace interaction {

//! Node representing distribution of features over a skeleton joint or prototypical segment
struct ProtoInteractionNode {
  FeatType type;                        //! feature type (indicates what type of vertex this is)
  string id;                            //! readable identifier
  string label;                         //! categorical label for this node
  size_t interactionCount;              //! count of number of interactions
  size_t count;                         //! count of aggregated nodes for feats
  vec<stats::Histogram<float>> feats;   //! vector of histograms over feature dimensions
  int jointId;                          //! id of the joint (-1 if not a joint)

  void clear();

 private:
  friend class boost::serialization::access;
  template<class Archive> void serialize(Archive & ar, const unsigned int version) {  // NOLINT
    ar & type & id & label & interactionCount & count & feats & jointId;
  }
};

//! Edge representing distribution of features over skeleton bone, contact event, or gaze event
struct ProtoInteractionLink {
  FeatType type;                        //! feature type (indicates what type of link this is)
  string id;                            //! readable identifier
  string label;                         //! categorical label for this link
  size_t interactionCount;              //! count of number of interactions
  size_t count;                         //! count of aggregated nodes for feats
  vec<stats::Histogram<float>> feats;   //! vector of histograms over feature dimensions

  void clear();

private:
  friend class boost::serialization::access;
  template<class Archive> void serialize(Archive & ar, const unsigned int version) {  // NOLINT
    ar & type & id & label & interactionCount & count & feats;
  }
};

//! Set of annotations applying to an interaction graph prototype
struct ProtoInteractionAnnotation {
  PartType                            partType;     //! Part type used for constructing this PIG
  string                              pigType;      //! Aggregation strategy used for this PIG
  string                              id;           //! identifier for interaction prototype
  stats::Counter<string>              verbs;        //! counter of verbs mentioned in instances for this proto
  stats::Counter<string>              nouns;        //! counter of nouns mentioned in instances for this proto
  map<string, stats::Counter<size_t>> nounsCount;   //! counter of how many times each noun is mentioned in instances for this proto
  size_t                              count;        //! count of aggregated nodes
  stats::Histogram<float>             zCOMHist;     //! histogram of absolute height of center of mass of Skeletons
  map<string, stats::Histogram<float>> rDistHist;   //! label -> histogram of horizontal distances from COM
  map<string, stats::Histogram<float>> vOffsetHist; //! label -> histogram of vertical displacement from COM
  map<string, stats::Histogram<float>> angDxyHist;  //! label -> theta angle wrt body orientation direction

  void getNounProbs(stats::CounterSd* pNounsProbs) const;
  void clear();

 private:
  friend class boost::serialization::access;
  template<class Archive> void serialize(Archive & ar, const unsigned int version) {  // NOLINT
    if (version > 1) {
      ar & partType;
    } else if (Archive::is_loading::value) {
      partType = segmentation::kPartSegment;
    }
    
    if (version > 0) {
      ar & pigType;
    }
    ar & id;
    if (version > 0) {
      ar & verbs;
    }
    ar & nouns & nounsCount & count & zCOMHist & rDistHist & vOffsetHist & angDxyHist;
  }
};

//! Identifers for different PIG combination algorithms {PIG} -> composite PIG
enum PIGCombineStrategy {
  kPIGCombineJointsAverage = 0, //! for each joint, take weighted average of contributing PIGs
  kPIGCombineJointsMax = 1,     //! for each joint, take PIG with max weight
  kPIGCombineStrategyCount = 2
};

// Helper data structure of indices for PIG features
struct PigFeatIndices {
  int iRelHeight;
  int iRelRDist;
  int iRelVOffset;
  int iRelAngDxy;
  int iRelCosDS;
};

//! Encapsulates a prototypical InteractionGraph: an aggregated representation of InteractionGraph instances belonging
//! to the same action (identified by a verb)
class ProtoInteractionGraph
  : public graph::Graph<ProtoInteractionNode, ProtoInteractionLink, ProtoInteractionAnnotation>
  , public io::Serializable {
 public:
  static const PigFeatIndices kPigFeatIndices;
  //! Used to index into the m_vert and m_edges for the pig
  static const int kComIdx;

  BOOST_BINARY_SERIALIZABLE_FUNCS
  //! Create a ProtoInteractionGraph with the given verb identifier
  explicit ProtoInteractionGraph(const string& id);
  //! Write this ProtoInteractionGraph out into ostream os as GraphML
  void writeGraphML(ostream& os) override;  // NOLINT
  void writeGraphML(const string& filename);
  //! VIP: delete copy and assignment since vertex and edge descriptors in underlying graph are invalidated
  ProtoInteractionGraph(const ProtoInteractionGraph&) = delete;
  ProtoInteractionGraph& operator=(const ProtoInteractionGraph&) = delete;

  //! Convenience pair of pointers to link and node members
  struct ProtoInteractionPair { ProtoInteractionLink* link; ProtoInteractionNode* node; };

  //! Return the Part type
  const PartType& getPartType() const {
    return getGraphBundle().partType;
  }

  //! Return the PIG type
  const string& getPigType() const {
    return getGraphBundle().pigType;
  }

  //! Return the id
  const string& getId() const {
    return getGraphBundle().id;
  }

  //! Return the id along with the part type and pig type
  string getLongId() const {
    const auto g = getGraphBundle();
    return segmentation::kPartTypeNames[g.partType] + "." + g.pigType + "." + g.id;
  }

  //! Returns a ProtoInteractionLinkNode pair for the com and label combination,
  //! initializing new members if necessary
  ProtoInteractionPair getProtoPairForCom(const string& label);
  //! Returns a ProtoInteractionLinkNode pair for the given joint and label combination,
  //! If iJoint == kComIdx, the ProtoInteractionLinkNode pair for the com and label combination is returned
  //! initializing new members if necessary
  ProtoInteractionPair getProtoPair(const int iJoint, const string& label);

  //! Returns ProtoInteractionNode for the COM initializing a new member if necessary
  ProtoInteractionNode& getProtoCom();
  const ProtoInteractionNode& getProtoCom() const;

  //! Returns ProtoInteractionNode for the given joint initializing a new member if necessary
  ProtoInteractionNode& getProtoJoint(const int iJoint);
  const ProtoInteractionNode& getProtoJoint(const int iJoint) const;

  //! Returns ProtoInteractionLink for the given bone initializing a new member if necessary
  ProtoInteractionLink& getProtoBone(const int iBone);
  const ProtoInteractionLink& getProtoBone(const int iBone) const;

  //! Return all contact edges for given joint index
  const map<string, edge>& getContactEdges(int iJoint) const {
    return m_edges[iJoint];
  }

  //! Return all gaze edges
  const map<string, edge>& getGazeEdges() const {
    return m_edges[kSkelParams.kGazeJoint];  // NOTE: Assumes gaze stored at kGazeJoint
  }

  //! Return all com edges
  const map<string, edge>& getComEdges() const {
    return m_edges[kComIdx];  // NOTE: Assumes com edges stored at kComIdx
  }

  //! Dumps features for this ProtoInteractionGraph into the given file
  bool dumpFeats(const string& file) const;
  bool dumpFeatsSummary(const string& file) const;

  //! Clears the pig
  void clear();

  //! Retargets a pig by taking nodes with from label and convert it to nodes with to label
  std::shared_ptr<ProtoInteractionGraph> retarget(const string& fromLabel, const string& toLabel);

  //! Combines a vector of proto interaction pigs into one
  static void combine(const vec<std::reference_wrapper<const ProtoInteractionGraph>> pigs,
                      const vec<vec<double>>& jointGroupWeights,
                      const PIGCombineStrategy pigCombineStrategy,
                      ProtoInteractionGraph* pCombined);

private:
  friend class ProtoInteractionGraphAggregator;
  friend struct WeightedPIG;

  // boost serialization support
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {  // NOLINT
    boost::serialization::split_member(ar, *this, version);
  }
  template <typename Archive>
  void save(Archive& ar, const unsigned int version) const {  // NOLINT
    typedef Graph<ProtoInteractionNode, ProtoInteractionLink, ProtoInteractionAnnotation> MyGraphT;
    ar & boost::serialization::base_object<const MyGraphT>(*this);
    ar & m_skel;
    ar & m_verts;
    // Convert m_edges to vertex pairs and store
    vec<map<string, vertexpair>> vertPairs(m_edges.size());
    for (int i = 0; i < vertPairs.size(); ++i) {
      auto& vpMap = vertPairs[i];
      for (const auto& idEdge : m_edges[i]) {
        vpMap.insert(make_pair(idEdge.first, edgeToVertPair(idEdge.second)));
      }
    }
    ar & vertPairs;
  }
  template <typename Archive>
  void load(Archive& ar, const unsigned int version) {  // NOLINT
    typedef Graph<ProtoInteractionNode, ProtoInteractionLink, ProtoInteractionAnnotation> MyGraphT;
    ar & boost::serialization::base_object<MyGraphT>(*this);
    ar & m_skel;
    ar & m_verts;
    // Convert m_edges to vertex pairs and store
    vec<map<string, vertexpair>> vertPairs;
    ar & vertPairs;
    m_edges.resize(vertPairs.size());
    for (int i = 0; i < m_edges.size(); ++i) {
      auto& eMap = m_edges[i];
      for (const auto& idVertPair : vertPairs[i]) {
        eMap.insert(make_pair(idVertPair.first, vertPairToEdge(idVertPair.second)));
      }
    }
    // Make sure our m_verts and m_edges array are large enough
    if (m_verts.size() < kVertsSize) {
      m_verts.resize(kVertsSize);
    }
    if (m_edges.size() < kVertsSize) {
      m_edges.resize(kVertsSize);
    }
  }

  // VIP NOTE: vertex and edge descriptors are invalidated when the inherited Graph member is copied
  //! Storage for skeleton nodes and edges (of type kJoint and kBone)
  GraphMembers<GraphT>   m_skel;
  //! Storage for vertices not in the skeleton (of type kSeg)
  //! Contact and gaze seg prototype vertices (indexed by iJoint) per label
  //! Com seg prototype vertices (indexed by kComIdx)
  vec<map<string, vertex>>            m_verts;
  //! Storage for edges not in the skeleton (of type kContact, kGaze, kComLink)
  //! Prototype contact (and gaze after joints) edges (indexed by iJoint) per label
  //! Prototype comLink edges (indexed by kComIdx)
  //! Edges of kSpatial are not yet included
  vec<map<string, edge>>              m_edges;

  //! Size of m_verts and m_edges vectors (update if more verts/edges added)
  static const size_t kVertsSize;
};

//! Convenience typedef
typedef ProtoInteractionGraph PIG;

//! Identifers for different aggregation algorithms {IG} -> PIG
enum AggregationStrategy {
  kSingleProtoNodes = 0,          //! each joint gets a single ProtoInteractionNode for all interacting geometry
  kPerPartProtoNodes = 1,         //! link joint to a ProtoInteractionNode for each object part label seen interacting with it
  kPerCategoryProtoNodes = 2,     //! link joint to a ProtoInteractionNode for each category seen interacting with it
  kPerVerbTargetProtoNodes = 3,   //! link joint to a ProtoInteractionNode for each verb which has a target geo node
  kAggregationStrategyCount = 4
};

//! Names for aggregation algorithms
static const arr<string, kAggregationStrategyCount> kAggregationStrategyIds = {
  "SingleProtoNodes",
  "PerPartProtoNodes",
  "PerCategoryProtoNodes",
  "PerVerbTargetProtoNodes"
};

//! Aggregates IGs into PIG
class ProtoInteractionGraphAggregator {
 public:
  explicit ProtoInteractionGraphAggregator(const AggregationStrategy aggStrat = kSingleProtoNodes)
    : m_aggStrat(aggStrat) { }
  explicit ProtoInteractionGraphAggregator(const string& aggStratId);

  //! Aggregate InteractionGraph ig into ProtoInteractionGraph pig
  void addInteractionGraph(const InteractionGraph& ig, ProtoInteractionGraph* pig) const;

  AggregationStrategy getAggregationStrategy() const {
    return m_aggStrat;
  }

  const string& getAggregationStrategyName() const {
    return kAggregationStrategyIds[m_aggStrat];
  }

 private:
  //! Helper functions for aggregation
  void aggregateByTargetVerb(const IG& ig, int iJoint,
                              const vec<PIG::edge>& igEdges,
                              PIG* pig) const;
  void aggregateByLabel(const IG& ig, int iJoint,
                        const vec<PIG::edge>& igEdges,
                        PIG* pig) const;

  const string kAnyLabelId = "any";  //! identifier used as common label in SingleProtoNodes aggregation strategy
  AggregationStrategy m_aggStrat;    //! aggregation strategy used by this aggregator
};

//! Helper types
typedef std::reference_wrapper<const ProtoInteractionLink> ProtoInteractionLinkRef;
typedef map<string, vec<ProtoInteractionLinkRef>> StringPIGLinkMap;
typedef interaction::StringPIGLinkMap CategoryPIGLinkMap;
//! Pig with weighted joints and helper function to get joint links by category
struct WeightedPIG {
  WeightedPIG(const interaction::ProtoInteractionGraph& _pig,
              const arr<double, Skeleton::kNumJointGroups>& w) 
              : pig(_pig), jointGroupWeights(w) {}
  //! Prototypical interaction graph
  const interaction::ProtoInteractionGraph& pig;
  //! Joint group weights
  arr<double, Skeleton::kNumJointGroups> jointGroupWeights;
  //! joint links by category
  mutable vec<CategoryPIGLinkMap> jointLinksByCategory;
  
  void populateJointLinks(const std::function<string(const string&)> getCat) const;
};

}  // namespace interaction
}  // namespace sg

BOOST_CLASS_VERSION(sg::interaction::ProtoInteractionAnnotation, 2)


