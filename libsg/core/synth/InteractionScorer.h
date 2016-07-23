#pragma once

#include "libsg.h"

namespace sg {
namespace core {
namespace synth {

struct PlacementState;

struct InteractionScorerParams {
  void init (const util::Params& p);
  bool usePigSimScore = false;
  bool usePigScore = false;
  bool useInteractionVolume= false;
  double modelSkeletonCollisionWeight = 1.0;
  double modelCollisionWeight = 1.0;
  double interactionWeight = 1.0;
  double posePriorWeight = 1.0;
  double skelSelfCollisionWeight = 1.0;
  int numSkelPtSamples = 100;
  bool useLog = false;
};

// Forward declare
class ModelSkeletonScorer;
class OverlapScorer;

// Provides functions for scoring interactions
class InteractionScorer {
public:
  //! Initializes this interaction scorer
  void init(Database* db, const InteractionScorerParams& params);
  // Score support for model
  double scoreModelSupport(const ModelInstance& mInst,
                           const ModelInstance& mInstSupport) const;
  // Score interaction of one model and skeleton
  double scoreModelSkelInteraction(const interaction::InteractionSet& iset,
                                   const ModelInstance& mInst,
                                   const Skeleton& mySkel) const;
  // Score for skeleton interaction with several models
  double scoreModelSkelInteraction(const interaction::InteractionSet& iset,
                                   const vec<ModelInstance>& mInsts,
                                   const Skeleton& mySkel) const;
  // Score placement of model instance wrt to skeleton and other models
  double scoreModelInstance(const PlacementState& state,
                            const interaction::InteractionSet& iset,
                            const ModelInstance& mInst,
                            const Skeleton& mySkel) const;

  // Returns the log probability of skeleton for iset
  double getSkeletonLogProb(const interaction::InteractionSet& iset,
                            const Skeleton& mySkel) const;
  // Score skeleton by itself
  double scoreSkeleton(const interaction::InteractionSet& iset,
                       const Skeleton& mySkel,
                       const boost::optional<double> baseSkelLP = boost::none) const;
  // Score skeleton with flag to include skeleton score
  double scoreSkeleton(const interaction::InteractionSet& iset,
                       const vec<ModelInstance>& mInsts,
                       const Skeleton& mySkel,
                       bool includeSkelScore,
                       const boost::optional<double> baseSkelLP = boost::none) const;

  // Score change in joint
  double scoreSkeletonJoint(const interaction::InteractionSet& iset,
                            const TransformedSkeleton& mySkel,
                            int iJoint) const;

  // Score everything!
  double score(const interaction::InteractionSet& iset,
               const vec<ModelInstance>& mInsts,
               const Skeleton& skel) const;

private:
  //! Interaction scorer params
  InteractionScorerParams m_params;

  //! Database from which we get information and the object labeller
  Database* m_pDatabase = nullptr;

  // Helper scorers
  std::shared_ptr<OverlapScorer> m_overlapScorer;
  std::shared_ptr<ModelSkeletonScorer> m_pigScorer; 
  std::shared_ptr<ModelSkeletonScorer> m_pigSimScorer; 
  std::shared_ptr<ModelSkeletonScorer> m_iframesScorer; 

  // Does backoff scoring with skeleton distributions
  typedef std::function<double(const string& isetId)> ISetScoreFn;
  double InteractionScorer::scoreSkeletonWithISetBackoff(
    const sg::interaction::InteractionSet& iset,
    const ISetScoreFn& scoreFn) const;
};


// Scores interaction between models and a skeleton
class ModelSkeletonScorer {
public:
  ModelSkeletonScorer(Database* db, const InteractionScorerParams& params) {
    init(db, params);
  }
  virtual ~ModelSkeletonScorer() {};

  static const vec<int> kAllJointGroupsLR;

  //! Initializes this model skeleton scorer
  void init(Database* db, const InteractionScorerParams& params) {
    m_pDatabase = db;
    m_params = params;
  }

  // Score skeleton against one model
  virtual double score(const interaction::InteractionSet& iset,
                       const ModelInstance& mInst,
                       const Skeleton& skel) const {
    return 0.0;
  }

  // Score skeleton against the models
  virtual double score(const interaction::InteractionSet& iset,
                       const vec<ModelInstance>& mInsts,
                       const Skeleton& skel) const {
    double totalScore = 0.0;
    for (const ModelInstance& mInst : mInsts) {
      totalScore += score(iset, mInst, skel);
    }
    return totalScore;
    //size_t n = mInsts.size();
    //return (n > 1)? totalScore/n : totalScore;
  }

  // Score joint interaction against models
  virtual double scoreJoint(const interaction::InteractionSet& iset,
                            const vec<ModelInstance>& mInsts,
                            const TransformedSkeleton& mySkel,
                            int iJoint) const {
    return 0.0;
  }

protected:
  //! Interaction scorer params
  InteractionScorerParams m_params;

  //! Database from which we get information and the object labeller
  Database* m_pDatabase = nullptr;
};

class PigScorer : public ModelSkeletonScorer {
public:
  PigScorer(Database* db, const InteractionScorerParams& params) 
    : ModelSkeletonScorer(db, params) {
  }
  double score(const interaction::InteractionSet& iset,
               const ModelInstance& mInst,
               const Skeleton& skel) const override;

  // Score joint interaction against models
  double scoreJoint(const interaction::InteractionSet& iset,
                    const vec<ModelInstance>& mInsts,
                    const TransformedSkeleton& mySkel,
                    int iJoint) const override;

  // Score joint interaction against models
  double scoreJointGroups(const interaction::InteractionSet& iset,
                          const ModelInstance& mInst,
                          const Skeleton& mySkel,
                          const vec<int>& jointGroups = kAllJointGroupsLR,
                          const double comWeight = 0.5) const;
};

class PigSimScorer : public ModelSkeletonScorer {
public:
  PigSimScorer(Database* db, const InteractionScorerParams& params) 
    : ModelSkeletonScorer(db, params) {
  }
  // Score skeleton against one model
  double score(const interaction::InteractionSet& iset,
               const ModelInstance& mInst,
               const Skeleton& skel) const override;

  // Score skeleton against the models
  double score(const interaction::InteractionSet& iset,
               const vec<ModelInstance>& mInsts,
               const Skeleton& skel) const override;
};

class IFramesScorer : public ModelSkeletonScorer {
public:
  IFramesScorer(Database* db, const InteractionScorerParams& params) 
    : ModelSkeletonScorer(db, params) {
  }
  // Score skeleton against one model
  double score(const interaction::InteractionSet& iset,
               const ModelInstance& mInst,
               const Skeleton& skel) const override;
};

class OverlapScorer : public ModelSkeletonScorer {
public:
  OverlapScorer(Database* db, const InteractionScorerParams& params) 
    : ModelSkeletonScorer(db, params) {
  }
  // Score skeleton against one model
  double score(const interaction::InteractionSet& iset,
               const ModelInstance& mInst,
               const Skeleton& skel) const override {
    return scoreModelSkeletonOverlap(mInst, skel);
  }

  // Score skeleton against the models
  double score(const interaction::InteractionSet& iset,
               const vec<ModelInstance>& mInsts,
               const Skeleton& skel) const override {
    return scoreModelSkeletonOverlap(mInsts, skel);
  }

  // Score overlap of model against other models
  double scoreModelOverlap(const vec<ModelInstance>& mInsts) const;
  double scoreModelOverlap(const PlacementState& state, 
                           const ModelInstance& mInst) const;
  // Score overlap of model and skeleton
  double scoreModelSkeletonOverlap(const ModelInstance& mInst,
                                   const Skeleton& skel) const;
  double scoreModelSkeletonOverlap(const vec<ModelInstance>& mInsts,
                                   const Skeleton& skel) const;
};

//! Helper function for populating pig joint links
void ensurePigJointLinks(const ObjectLabeler& labeler,
                         const interaction::WeightedPIG& wpig);

}  // synth
}  // namespace core
}  // namespace sg
