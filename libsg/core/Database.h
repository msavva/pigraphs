#pragma once

#include "libsg.h"  // NOLINT
#include "core/CentroidDatabase.h"
#include "core/ModelDatabase.h"
#include "core/RecordingDatabase.h"
#include "core/ScanDatabase.h"
#include "core/SceneDatabase.h"
#include "core/ScenePriors.h"
#include "interaction/InteractionDatabase.h"
#include "interaction/InteractionFactory.h"
#include "interaction/ModelAnnotation.h"

namespace sg {
namespace core {

//! Point-in-time snippet of single skeleton interacting
struct InteractionRecord {
  InteractionRecord() = default;
  const interaction::Interaction* pInteraction;   //! Active Interaction range
  const size_t skelIndex;                         //! Index of Skeleton within above Interaction
};

typedef map<const Skeleton*, vec<const InteractionRecord>> SkeletonToInteractionsMap;

//! A global data storage manager/encapsulator, not a Database per se
struct Database {
  explicit Database(util::Params& _params);
  ~Database();

  void init(util::Params& _params);
  void load(bool computeAllActiveSegments, bool computeInteractionGraphs);
  void computeCentroids(bool forceRecompute);
  
  //! Loads all predefined classifiers for each InteractionSet in the database
  void loadClassifiers(bool forceRetrain);

  void describeDataset() const;

  //! Retrieve feature generators of appropriate type
  const SegmentFeatureGenerator* getSegmentFeatureGenerator(const SegmentFeatureGenerator::FeatureType type) const;
  
  //! Aggregates all segments activated by interactions in all recordings and dumps their features in csvFile
  void saveAllActiveSegments(const string& csvFile) const;

  // TODO: Is this function still needed?
  //! Retrieve map from skeleton to all interactions skeleton is involved in, for all skeletons in all recordings of this Database
  SkeletonToInteractionsMap getSkeletonInteractionRecords() const;

  //! Precompute all segment features for current database under geoGenerator
  map<const MeshSegment*, vecd> precomputeSegmentFeatures(const SegmentFeatureGenerator& geoGenerator) const;

  const synth::ObjectLabeler& getLabeler() const {
    return m_labeler;
  }

  /*const*/ util::Params* params;
  ScanDatabase scans;
  ScanDatabase scansCornellPLY;
  ModelDatabase models;
  interaction::ModelAnnotationDatabase modelAnnotations;
  RecordingDatabase recordings;
  ClassifierDatabase* classifiers;
  CentroidDatabase centroids;
  interaction::InteractionFactory  interactionFactory;
  interaction::InteractionDatabase interactions;
  ScenePriors scenePriors;
  SceneDatabase reconScenes;
  SkeletonDatabase* skeletons;
  MotionDatabase* motions;

 private:
  mutable arr<const SegmentFeatureGenerator*, SegmentFeatureGenerator::kNumFeatureTypes> featureGenerators;
  
  bool m_isLoaded;
  synth::ObjectLabeler m_labeler;  //! Labeler for categorizing our segments
};

}  // namespace core
}  // namespace sg


