#pragma once

#include "libsg.h"  // NOLINT
//#include "util/Params.h"
#include "core/features.h"
#include "eval/ConfusionMatrix.h"

// This forward declaration is CRAZYEEE! But it hides jace proxy headers from everyone else
namespace jace { namespace proxy { namespace edu { namespace stanford { namespace graphics { namespace wekautils { class Classer; }}}}}};
namespace wekautil { typedef jace::proxy::edu::stanford::graphics::wekautils::Classer Classifier; }

namespace sg {
namespace core {

//! Converts csvFile to Weka ARFF format file (assuming class atttribute is 0th column and should be nominal). Saves to "basename.arrf" and returns that filename
string CSVtoARFF(const string& csvFile);


// Container base type for Classifiers
class ClassifierBase {
 public:
  ClassifierBase();
  virtual ~ClassifierBase();
  bool acceptsHeader(const string& header) const;  //! Returns whether classifier accepts feature vector of signature header (comma separated column names)
  bool isValid() const { return pWekaClasser != nullptr; }  //! Whether this Classifier has a trained Weka classifier object
  virtual double classify(const vecd& feats) const;  //! Classify given feature vector and return likelihood of positive instance
  
  //! Test classifier on given instances file and sets confusion matrix. Returns true if success, false otherwise
  bool test(const string& testInstances, const string& reportFile, eval::BinaryConfusionMatrix* confMatrix) const;

  //! Get classifier weights
  bool getClassifierWeights(vec<double>* pWeights) const; //! Outputs classifier weights (true if successful, false otherwise)

  wekautil::Classifier* pWekaClasser;  //! Pointer to Weka classifier implementation
  string wekaClassifierType;           //! What kind of Weka classifier to train
  string wekaClassifierOptions;        //! What options to give the Weka classifier
  string type; //! Classifier type
  string id;   //! Classifier id (derived from type and simpleId)
  string simpleId;      //! Simplified classifier id
  string classifierDir; //! Directory in which the classifier was located
  string datasetDir;    //! Directory in which the classifier dataset was located
};

// Abstract interface for Classifier classes: can classify a ClassifiablyType and is initialized under specific Params and ConditionType
template <typename ClassifiableType, typename ConditionT>
class Classifier : public ClassifierBase {
 public:
  //! Initialize this Classifier given database and condition represented by ConditionType
  virtual void init(const Database& database, const ConditionT& condition) = 0;
  //! Returns the confidence score of ClassifiableType x under this Classifier
  virtual double classify(const ClassifiableType& x) const = 0;
};

//! Scene+Skeleton -> confidence score for particular VerbSet (how good Skeleton+Verb is in that Scene)
class ScenePoseClassifier : public Classifier<SceneSkel, interaction::InteractionSet> { };

//! Skeleton -> confidence score for particular InteractionSet
class PoseClassifier : public Classifier<Skeleton, interaction::InteractionSet> { };

//! MeshSegment -> confidence for causing VerbSet action
class SegmentClassifier : public Classifier<segmentation::MeshSegment, interaction::InteractionSet> { };

//! Scores a MeshSegment for evidence causing VerbSet actions
class SegmentPoseClassifier : public Classifier<SegmentSkeletonPair, interaction::InteractionSet> {
 public:
  void init(const Database& database, const interaction::InteractionSet& verbSet) override { }
  double classify(const SegmentSkeletonPair& x) const override;
};

//! InteractionGraph average kNN similarity ScenePoseClassifier
class InteractionGraphKNNSimilarity : public ScenePoseClassifier {
public:
  //! Type of similarity to use
  enum SimType {
    SimType_IFSUP  = 0,  // InteractionFrame support in scan voxels
    SimType_IGKNN  = 1,  // k nearest neighbor IG-IG similarity
    SimType_PIGIG  = 2,  // PIG - IG similarity
    SimType_PIGIGW = 3,  // PIG - IG similarity with weighted joints
    SimType_Manual = 4,  // Manual rule-based test
    SimType_Count  = 5
  };
  void init(const Database& database, const interaction::InteractionSet& verbSet) override;
  double classify(const SceneSkel& x) const override;
private:
  const Database* m_database = nullptr;
  const interaction::InteractionSet* m_pInteractionSet = nullptr;
  float m_NNpercent = 0.1f;
  int m_maxNumNNs = 5;
  SimType m_simType = SimType_IGKNN;
};


//! Aggregated kNN MeshSegment classifications version of ScenePoseClassifier
class ScenePoseClassifierAggregatedSegments : public ScenePoseClassifier {
 public:
  void init(const Database& database, const interaction::InteractionSet& iSet) override { m_pInteractionSet = &iSet; }
  double classify(const SceneSkel& x) const override;
 private:
  const interaction::InteractionSet* m_pInteractionSet = nullptr;
};

//! Bag of words classification version of ScenePoseClassifier
class ScenePoseClassifierBoW : public ScenePoseClassifier {
 public:
  ~ScenePoseClassifierBoW();
  void init(const Database& database, const interaction::InteractionSet& iSet) override;
  double classify(const SceneSkel& x) const override;
 private:
  const Database* m_database = nullptr;
  const interaction::InteractionSet* m_interactionSet = nullptr;
  SceneSkelCentroidActivationFeatGen* m_gen = nullptr;
};

//! MeshSegment features -> confidence score for VerbSet + specific joint group
class SegmentPerJointGroupClassifier : public Classifier<segmentation::MeshSegment,
                                                         pair<const interaction::InteractionSet&, size_t>> {
 public:
  void init(const Database& database, const pair<const interaction::InteractionSet&, size_t>& verbSetJointGroup) override;
  double classify(const segmentation::MeshSegment& x) const override;

 private:
  const interaction::InteractionSet* m_pInteractionSet = nullptr;
  size_t m_jointGroupIndex = 0;
  SegmentFeatureGenerator::FeatureType m_featureType = SegmentFeatureGenerator::FeatureType::FeatureType_MeshSimplistic;
  const SegmentFeatureGenerator* pFeatGenerator = nullptr;
};

//! MeshSegment + Joint features -> confidence score for VerbSet + specific joint group
class SegmentSkelPerJointGroupClassifier : public Classifier<SegmentSkeletonJoint,
                                                             pair<const interaction::InteractionSet&, size_t>> {
public:
  void init(const Database& database, const pair<const interaction::InteractionSet&, size_t>& verbSetJointGroup) override;
  double classify(const SegmentSkeletonJoint& x) const override;

private:
  const interaction::InteractionSet* m_pInteractionSet = nullptr;
  size_t m_jointGroupIndex = 0;
  SegmentFeatureGenerator::FeatureType m_segFeatureType = SegmentFeatureGenerator::FeatureType::FeatureType_MeshSimplistic;
  const SegmentFeatureGenerator* pSegFeatGenerator = nullptr;
  SegmentJointFeatureGeneratorBasic segJointGen;
};

//! ScenePoseClassifier that aggregates over each joint segment interaction
class SegmentJointsAggregatedClassifier : public ScenePoseClassifier {
 public:
  void init(const Database& database, const interaction::InteractionSet& verbSet) override;
  double classify(const SceneSkel& x) const override;
  ~SegmentJointsAggregatedClassifier();

 protected:
  const interaction::InteractionSet* m_pInteractionSet = nullptr;
  bool m_ignoreInferredJoints = false;
  unsigned int m_kNearestSegsPerJoint = 1;
  double m_maxDistToSegment = 0.1;
  SceneSkelJointAggrFeatureGeneratorBasic* m_gen = nullptr;
};

//! ScenePoseClassifier that aggregates over each joint segment interaction
class SegmentJointsLinearWeightedAggregatedClassifier : public SegmentJointsAggregatedClassifier {
public:
  void init(const Database& database, const interaction::InteractionSet& verbSet) override;
  double classify(const SceneSkel& x) const override;
protected:
  double m_jointGroupWeights[Skeleton::kNumJointGroups];
};

//! ScenePoseClassifier that aggregates over each joint segment interaction
class SegmentCentroidActivationAggregatedClassifier : public ScenePoseClassifier {
 public:
  void init(const Database& database, const interaction::InteractionSet& verbSet) override;
  double classify(const SceneSkel& x) const override;
  ~SegmentCentroidActivationAggregatedClassifier();

 protected:
  const interaction::InteractionSet* m_pInteractionSet = nullptr;
  bool m_ignoreInferredJoints = false;
  unsigned int m_kNearestSegsPerJoint = 1;
  double m_maxDistToSegment = 0.1;
  SceneSkelCentroidActivationFeatGen* m_gen = nullptr;
};

}  // namespace core
}  // namespace sg


