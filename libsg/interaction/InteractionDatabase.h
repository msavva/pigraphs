#pragma once

#include "libsg.h"  // NOLINT

#include "core/Skeleton.h"
#include "geo/ScoredTransform.h"
#include "interaction/InteractionSet.h"
#include "stats/stats.h"
#include "util/index.h"

#include <boost/serialization/version.hpp>

#define INTERACTION_DB_VERSION  3

namespace sg {
namespace interaction {

typedef map<string, std::reference_wrapper<InteractionSet>> InteractionSetMap;
typedef arr<InteractionSetMap, InteractionSet::kNumInteractionSetTypes> InteractionSetTypeToMap;

//! Holds statistics across interactions and interaction sets
class InteractionDatabase : public io::Serializable {
 public:
  InteractionDatabase(const util::Params& params, const InteractionFactory& factory) 
    : m_version(INTERACTION_DB_VERSION)
    , m_params(params)
    , m_interactionFactory(factory)
    , m_pDb(nullptr) { }
  void init(const core::Database& db, bool computeIGs);

  //! Makes sure the given interaction is populated with the iSet information
  //! Also add the interaction to our list of known interactions if not already present
  void InteractionDatabase::registerAndPopulateISets(Interaction* pInteraction);

  //! Creates InteractionSets given the interactions in all currently loaded recordings
  void generateInteractionSets(const core::RecordingDatabase& recordings);

  //! Returns all interaction sets of the given type
  const InteractionSetMap& getInteractionSets(const string& type) const {
    return m_interactionSetsByType.at(InteractionSet::kInteractionSetTypeLookupByName.at(type));
  }
  const InteractionSetMap& getInteractionSets(const InteractionSetType type = ISetType_Composite) const {
    return m_interactionSetsByType.at(type);
  }

  InteractionSetMap& getAllInteractionSets() {
    return m_interactionSetsByType[ISetType_All];
  }
  const InteractionSetMap& getAllInteractionSets() const {
    return m_interactionSetsByType[ISetType_All];
  }
  //! Get interactionSet by id 
  const InteractionSet* getInteractionSet(const string& id) const {
    const auto& all = getAllInteractionSets();
    if (all.count(id) > 0) {
      return &all.at(id).get();
    } else {
      return nullptr;
    }
  }
  //! Convenience function that populates a vector with pointers to our interaction sets
  void getAllInteractionSets(vec<const InteractionSet*>* out) const {
    for (auto& interactionSet : m_allInteractionSets) {
      out->push_back(&interactionSet.second);
    }
  }
  //! Convenience function for getting related interaction sets for a set of VerbNouns
  void getRelatedInteractionSets(VerbNounISetGroup* pISetGroup) const;
  //! Make sure pigs for part type and aggrType are set
  void selectPigs(const PartType& partType, const string& aggrType);

  //! Dumps summary of the annotated interactions to a CSV file
  void saveInteractionRecordSummary(const string& csvFile) const;
  void saveInteractionSummary(const string& csvFile) const;
  string getInteractionSummaryFile() const;

  //! Get statistics
  const stats::GridStats2D<string>& getVerbStats() const { 
    return m_verbStats;
  }
  const stats::GridStats2DT<string, string>& getNounSceneTypeStats() const {
    return m_nounSceneTypeStats;
  }

  const map<string, vec<string>>& getNounToParts() const {
    return m_nounToParts;
  }

  //! Get indices
  const util::Index<string>& getPartIndex() const {
    return m_partIndex;
  }

  const util::Index<string>& getNounIndex() const {
    return m_nounIndex;
  }

  const util::Index<string>& getSceneTypeIndex() const {
    return m_sceneTypeIndex;
  }

  //! Compute interaction statistics
  void computeStats();

  //! Save statistics for interactions
  void saveStats(const string& dirname) const;
  void loadStats(const string& dirname);

  //! Returns smoothing parameter
  size_t getLaplaceSmoothingParam() const { return m_laplaceSmoothing; }

  //! Loads this InteractionDatabase from binary format file
  bool loadBinary(const string& file) override;

  //! Save this InteractionDatabase to binary format file
  bool saveBinary(const string& file) const override;

 private:
  void createIndices(const core::ScanDatabase* pScans);
  void computeVerbStats();
  void computeVerbNounStats();
  void computeVerbJointStats();
  void computeNounJointStats();
  void computeVerbJointMatchNounStats();
  void computeVerbNounJointMatchNounStats();
  void computeSceneTypeStats();
  void populateInteractionSetWeights();

  //! Create interaction graphs for all interaction sets
  void computeInteractionGraphs();
  //! Creates interaction graphs for all interactions in vSet
  void generateInteractionGraphs(const InteractionSet& vSet, const PartType partType,
                                 vec<std::shared_ptr<InteractionGraph>>* igs) const;
  //! Create interaction frames for all interaction sets
  void computeInteractionFrames();
  //! Prepare skeletons and transforms for serialization
  void prepareSerializableSkeletons(vec<Skeleton>* pSkels, vec<geo::ScoredTransform>* pTransforms) const;
  //! Populate skeletons and using serialized skels and transforms
  void populateInteractionSetSkeletons(vec<Skeleton>& skels, const vec<geo::ScoredTransform>& transforms);
  //! Populate noun to parts map
  void populateNounToParts();

  unsigned int m_version;
  const util::Params& m_params;
  util::Index<string> m_partIndex;
  util::Index<string> m_nounIndex;
  util::Index<string> m_verbIndex;
  util::Index<string> m_verbNounIndex;
  util::Index<string> m_jointGroupIndex;
  util::Index<string> m_sceneTypeIndex;
  // Verb-verb stats
  stats::GridStats2D<string> m_verbStats;
  // Cross stats
  stats::GridStats2DT<string, string> m_verbJointGroupStats;
  // Only count if the verb noun are part of the same verbNoun group
  stats::GridStats2DT<string, string> m_verbNounStats;
  // Only count if the joint is activating the noun
  stats::GridStats2DT<string, string> m_nounJointGroupStats;
  stats::GridStats2DT<string, string> m_verbJointGroupMatchNounStats;
  stats::GridStats2DT<string, string> m_verbNounJointGroupMatchNounStats;
  // Scene object stats
  stats::GridStats2DT<string, string> m_nounSceneTypeStats;

  // TODO: Serialize away
  map<string, std::shared_ptr<Interaction>> m_interactions;
  map<string, InteractionSet> m_allInteractionSets;
  InteractionSetTypeToMap m_interactionSetsByType;
  
  // Skeletons for use in loaded interactions
  vec<Skeleton> m_skeletons;

  // For creating IGs
  const InteractionFactory& m_interactionFactory;

  const core::Database* m_pDb;

  const size_t m_laplaceSmoothing = 0;
  //! Map of parts from noun (computed from m_partIndex)
  map<string, vec<string>> m_nounToParts;

  friend class boost::serialization::access;
  //! boost serialization function
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {  // NOLINT
    boost::serialization::split_member(ar, *this, version);
  }
  template <typename Archive>
  void save(Archive& ar, const unsigned int version) const {  // NOLINT
    // Interaction sets
    ar & m_allInteractionSets;
    // Reconstruct m_interactionSetsByType
    // Indices
    ar & m_partIndex;
    ar & m_nounIndex;
    ar & m_sceneTypeIndex;
    // Reconstruct m_verbIndex, m_verbNounIndex, m_jointGroupIndex from interaction sets
    // Stats
    ar & m_verbStats;
    ar & m_verbJointGroupStats;
    ar & m_verbNounStats;
    ar & m_nounJointGroupStats;
    ar & m_verbJointGroupMatchNounStats;
    ar & m_verbNounJointGroupMatchNounStats;
    ar & m_nounSceneTypeStats;

    // Convert transformed skeletons into 
    // a vector of skeletons and a vector of ScoredTransforms
    // for serialization
    vec<Skeleton> skels;
    vec<geo::ScoredTransform> transforms;
    prepareSerializableSkeletons(&skels, &transforms);
    ar & skels;
    ar & transforms;
  }
  template <typename Archive>
  void load(Archive& ar, const unsigned int version) {  // NOLINT
    m_version = version;
    if (m_version != INTERACTION_DB_VERSION) {
      // Exit early (things are not going to load)
      return;
    }

    // Interaction sets
    ar & m_allInteractionSets;
    // Reconstruct m_interactionSetsByType
    for (int i = 0; i < ISetType_Count; ++i) {
      m_interactionSetsByType[i].clear();
    }
    for (auto& it : m_allInteractionSets) {
      InteractionSet& vSet = it.second;
      m_interactionSetsByType[it.second.type].insert({it.first, vSet});
      m_interactionSetsByType[ISetType_All].insert({it.first, vSet});
    }

    // Indices
    ar & m_partIndex;
    populateNounToParts();
    ar & m_nounIndex;
    ar & m_sceneTypeIndex;
    // Reconstruct m_verbIndex, m_verbNounIndex, m_jointGroupIndex from interaction sets
    createIndices(nullptr);
    // Stats
    m_verbStats.init(&m_verbIndex);
    ar & m_verbStats;

    m_verbJointGroupStats.init(&m_verbIndex, &m_jointGroupIndex);
    ar & m_verbJointGroupStats;

    m_verbNounStats.init(&m_verbIndex, &m_nounIndex);
    ar & m_verbNounStats;

    m_nounJointGroupStats.init(&m_nounIndex, &m_jointGroupIndex);
    ar & m_nounJointGroupStats;

    m_verbJointGroupMatchNounStats.init(&m_verbIndex, &m_jointGroupIndex);
    ar & m_verbJointGroupMatchNounStats;

    m_verbNounJointGroupMatchNounStats.init(&m_verbNounIndex, &m_jointGroupIndex);
    ar & m_verbNounJointGroupMatchNounStats;

    m_nounSceneTypeStats.init(&m_nounIndex, &m_sceneTypeIndex);
    ar & m_nounSceneTypeStats;

    // Interaction set weights
    populateInteractionSetWeights();

    // Transformed skeletons
    vec<geo::ScoredTransform> transforms;
    ar & m_skeletons;
    ar & transforms;
    populateInteractionSetSkeletons(m_skeletons, transforms);
  }

};

}  // namespace interaction
}  // namespace sg

BOOST_CLASS_VERSION(sg::interaction::InteractionDatabase, INTERACTION_DB_VERSION)


