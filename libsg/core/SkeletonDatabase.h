#pragma once

#include "libsg.h"  // NOLINT

#include "core/SkeletonDistribution.h"
#include "io/io.h"

namespace sg {
namespace core {

class SkeletonDatabase : public io::Serializable {
 public:
  BOOST_BINARY_SERIALIZABLE_FUNCS
  explicit SkeletonDatabase(const util::Params& params);

  void init(const RecordingDatabase& recs, interaction::InteractionDatabase& interactionDb);

  void init(const MotionDatabase& motions, const vec<string>& keywords = vec<string>());

  void finalizeLoad();

  //! Save Skeleton features for each Skeleton in this database to csv format files in dir
  void saveSkelFeats(const string& dir) const;

  bool getRandomObservedSkeleton(const string& interactionSetId, 
                                 Skeleton* pSkel, double* pScore = nullptr) const;

  bool getAverageSkeleton(const string& interactionSetId, 
                          Skeleton* pSkel, double* pScore = nullptr) const;

  bool sampleSkeleton(const string& interactionSetId,
                      Skeleton* pSkel, double* pScore = nullptr) const;

  //! Samples k skeletons
  size_t sampleSkeletons(const string& interactionSetId,
                         size_t k,
                         vec<pair<Skeleton,double>>* pScoredSkeletons) const;

  //! Returns the first k observed skeletons
  size_t getFirstKObservedSkeletons(const string& interactionSetId,
                                    size_t k,
                                    vec<pair<Skeleton,double>>* pScoredSkeletons) const;

  //! Returns k observed skeletons selected in random
  size_t getRandomObservedSkeletons(const string& interactionSetId,
                                    size_t k,
                                    vec<pair<Skeleton,double>>* pScoredSkeletons) const;

  const vec<string>& getInteractionIds() const { return m_interactionIds; }

  vec<string> getInteractionSetIds() const;

  //! Returns sorted pair of <iSetId,logLikelihood> for given Skeleton
  vec<pair<string, double>> predictInteractionSetLikelihoods(const Skeleton& s) const;

  //! Returns log likelihood of skeleton for the given iSet Id
  double predictInteractionSetLikelihood(const string& isetId, const Skeleton& s) const;

  //! Returns log likelihood of skelstate for the given iSet Id
  double predictInteractionSetLikelihood(const string& isetId, const SkelState& ss) const;

  //! logistic regression weights per joint (and intercept is last)
  typedef arr<double, Skeleton::kNumJoints + 1> JointLogitWeights;
  const JointLogitWeights& getJointWeights(const string& verb) const {
    return m_logitJointWeightsPerVerb.at(verb);
  }

  arr<float, Skeleton::kNumJoints> getNormalizedJointWeights(const string& verb) const;

  //! Returns given SkeletonDistribution
  //! Optionally consider only a subset of joints
  const SkeletonDistribution& getSkelDistribution(const string& isetId) const {
    if (m_isetIdToSkelDists.count(isetId) > 0) {
      return m_isetIdToSkelDists.at(isetId);
    } else {
      throw SG_EXCEPTION("Unknown isetId for skeleton distribution: " + isetId);
    }
  }

  bool hasSkelDistribution(const string& isetId) const {
    return m_isetIdToSkelDists.count(isetId) > 0;
  }

private:
  //! boost serialization function
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive& ar, const unsigned int version) {  // NOLINT
    boost::serialization::split_member(ar, *this, version);
  }
  template <typename Archive>
  void save(Archive& ar, const unsigned int) const {  // NOLINT
    ar & m_skeletons;
    ar & m_interIdToSkelIndices;
    ar & m_isetIdToSkelIndices;
    ar & m_interactionIds;
    ar & m_isetIdToInteractionIndex;
  }
  template <typename Archive>
  void load(Archive& ar, const unsigned int) {  // NOLINT
    ar & m_skeletons;
    ar & m_interIdToSkelIndices;
    ar & m_isetIdToSkelIndices;
    ar & m_interactionIds;
    ar & m_isetIdToInteractionIndex;
  }

  void loadFromRecordingsAndInteractions(const RecordingDatabase& recs,
                                         interaction::InteractionDatabase& interactionDb);


  // Serialized state
  vec<Skeleton> m_skeletons;
  vec<string> m_interactionIds;
  map<string, vec<size_t>> m_interIdToSkelIndices;
  map<string, vec<size_t>> m_isetIdToSkelIndices;
  map<string, vec<size_t>> m_isetIdToInteractionIndex;

  // Working state -- not serialized
  map<string, SkeletonDistribution> m_isetIdToSkelDists;
  map<string, JointLogitWeights> m_logitJointWeightsPerVerb;
  const util::Params& m_params;
};

}  // namespace core
}  // namespace sg
