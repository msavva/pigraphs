#pragma once

#include "libsg.h"  // NOLINT
#include "core/Skeleton.h"

namespace sg {
namespace core {

typedef segmentation::MeshSegment MeshSegment;

struct SegmentJoint {
  const segmentation::MeshSegment* segment;
  size_t jointIndex;
};

struct SegmentSkeletonJoint {
  const segmentation::MeshSegment* segment;
  TransformedSkeleton skeleton;
  size_t jointIndex;
};

typedef pair<const MeshSegment*, const TransformedSkeleton*> SegmentSkeletonPair;

//! Encapsulates a skeleton in a scan that has activated a set of segments in the scene
//! NOTE: some of the member pointers may be null depending on use case
struct SceneSkel {
  const Scan* scene;
  const TransformedSkeleton* tSkel;
  const Skeleton::SegmentsByJointPlusGaze* activatedSegsByJoint;
};

//! Generic abstract class defining feature generation interface
template <typename T>
class FeatureGenerator {
 public:
  virtual ~FeatureGenerator() { }

  //! Returns names of feature fields
  virtual vec<string> fieldNames() const = 0;
  //! Generates values for features given an entity x
  virtual vecd generate(const T& x) const = 0;
  //! Return number of features this generator produces for each entity of type T
  size_t numFeatures() const { return fieldNames().size(); }

  //! Append feature names to a existing vector
  void appendFieldNames(vec<string>* v) const {
    const vec<string> names = fieldNames();
    v->insert(v->end(), names.begin(), names.end());
  }
  //! Append features to a existing vector
  void append(const T& x, vecd* v) const {
    const vecd feats = generate(x);
    v->insert(v->end(), feats.begin(), feats.end());
  }
};

class SegmentFeatureGenerator : public FeatureGenerator<MeshSegment> {
 public:
  enum FeatureType {
    FeatureType_MeshSimplistic,
    FeatureType_MeshCentroid,
    FeatureType_Wolf,
    FeatureType_Count
  };
  static const int kNumFeatureTypes = FeatureType_Count;
  static const arr<const string, kNumFeatureTypes> kFeatureTypes;
  static FeatureType getFeatureType(const string& featureTypeName);
};

class SegmentSkelFeatureGenerator : public FeatureGenerator<SegmentSkeletonPair> { };
class SegmentJointFeatureGenerator : public FeatureGenerator<SegmentJoint> { };
class SegmentSkelJointFeatureGenerator : public FeatureGenerator<SegmentSkeletonJoint> { };
class SceneSkelFeatureGenerator : public FeatureGenerator<SceneSkel> { };

class SegmentFeatureGeneratorSimplistic : public SegmentFeatureGenerator {
 public:
  vec<string> fieldNames() const override;
  vecd generate(const MeshSegment& seg) const override;
};

//! Implements segment features from "Fast Semantic Segmentation of
// 3D Point Clouds using a Dense CRF with Learned Parameters" [Wolf et al. ICRA 2015]
class SegmentFeatureGeneratorWolf : public SegmentFeatureGenerator {
 public:
  vec<string> fieldNames() const override;
  vecd generate(const MeshSegment& seg) const override;
};

//! Implements segment features from "Enhancing Semantic Segmentation
// for Robitics: The Power of 3-D Entangled Forests" [Wolf et al. RAL 2016]
class SegmentFeatureGeneratorWolf2016 : public SegmentFeatureGenerator {
 public:
  vec<string> fieldNames() const override;
  vecd generate(const MeshSegment& seg) const override;
};

class SegmentCentroidActivationFeatGen : public SegmentFeatureGenerator {
 public:
  SegmentCentroidActivationFeatGen(const CentroidDatabase& centroidDb, const bool splitCentroidsPerJointGroup)
    : m_splitCentroidsPerJoint(splitCentroidsPerJointGroup)
    , m_centroidDb(centroidDb)
    , m_segFeatGen() { }
  vec<string> fieldNames() const override;
  vecd generate(const MeshSegment& segJoint) const override;
 private:
  const bool m_splitCentroidsPerJoint;
  const CentroidDatabase& m_centroidDb;
  const SegmentFeatureGeneratorSimplistic m_segFeatGen;
};

class SegmentSkelFeatureGeneratorBasic : public SegmentSkelFeatureGenerator {
 public:
  vec<string> fieldNames() const override;
  vecd generate(const SegmentSkeletonPair& pair) const override;
};

class SegmentJointFeatureGeneratorBasic : public SegmentSkelJointFeatureGenerator {
 public:
  vec<string> fieldNames() const override;
  vecd generate(const SegmentSkeletonJoint& segJoint) const override;
};

class SceneSkelCentroidActivationFeatGen : public SceneSkelFeatureGenerator {
 public:
  SceneSkelCentroidActivationFeatGen(const interaction::InteractionSet& interactionSet,
                                     const CentroidDatabase& centroidDb,
                                     bool splitCentroidsPerJointGroup,
                                     bool useActivation)
    : m_interactionSet(interactionSet)
    , m_centroidDb(centroidDb)
    , m_splitCentroidsPerJointGroup(splitCentroidsPerJointGroup)
    , m_useActivation(useActivation)
    , m_segFeatGen() { }
  vec<string> fieldNames() const override;
  vecd generate(const SceneSkel& sceneSkel) const override;

 private:
  const interaction::InteractionSet& m_interactionSet;
  const CentroidDatabase& m_centroidDb;
  const bool m_splitCentroidsPerJointGroup;
  const bool m_useActivation;
  const SegmentFeatureGeneratorSimplistic m_segFeatGen;
};

class SceneSkelJointAggrFeatureGeneratorBasic : public SceneSkelFeatureGenerator {
 public:
  SceneSkelJointAggrFeatureGeneratorBasic(const interaction::InteractionSet& interactionSet,
                                          bool useActivation,
                                          bool useJointSegScores,
                                          bool useJointSegFeatures)
    : m_useActivation(useActivation)
    , m_useJointSegScores(useJointSegScores)
    , m_useJointSegFeatures(useJointSegFeatures)
    , m_interactionSet(&interactionSet)
  {}
  vec<string> fieldNames() const override;
  vecd generate(const SceneSkel& sceneSkel) const override;
  pair<double, int> jointInteractionScore(const SceneSkel& sceneSkel, size_t iJointGroup) const;
 private:
  const bool m_useActivation;
  const bool m_useJointSegScores;
  const bool m_useJointSegFeatures;
  const interaction::InteractionSet* m_interactionSet;
};

}  // namespace core
}  // namespace sg


