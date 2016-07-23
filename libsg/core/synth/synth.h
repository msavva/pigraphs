#pragma once

#include "libsg.h"  // NOLINT
#include "geo/geo.h"
#include "util/smartenum.h"

namespace sg {
namespace core {

//! Namespace for algorithms related to posing, retrieval and alignment for interaction synth
namespace synth {

//! Identifers for different retrieval strategies
enum class RetrievalStrategy {
  kLabelsAnnotation       = 0,  //! Retrieve models using the annotated segment group labels
  kLabelsPIG              = 1,  //! Retrieve models using a matching PIG for the labels
  kCount = 2
};
OSTREAMABLE(RetrievalStrategy)

enum class SelectModelStrategy {
  kFirst = 0,   //! Select first matching model
  kRandom = 1,  //! Select random matching model
  kCount = 2
};
OSTREAMABLE(SelectModelStrategy)

//! Strategy for selecting skeletons
//! kFirst = select first k skeletons
//! kRandom = select random k skeletons from observed
//! kSample = sample k skeletons
//! kAverage = select average skeleton
#define SELECT_SKEL_STRATEGY_TYPES(m,t)  \
        m(t, First)                      \
        m(t, Random)                     \
        m(t, Sample)                     \
        m(t, Average)

SMARTENUMCLASS_DEFINE_ENUM(SelectSkelStrategy, SELECT_SKEL_STRATEGY_TYPES)
static SMARTENUMCLASS_DEFINE_NAMES(SelectSkelStrategy, SELECT_SKEL_STRATEGY_TYPES)
inline SMARTENUMCLASS_DEFINE_GET_VALUE_FROM_STRING(SelectSkelStrategy, SelectSkelStrategy::kCount)
OSTREAMABLE(SelectSkelStrategy)

//! Create a ModelInstance of model, transform to random position on floor of scan and push to out
void placeModelInstance(const Model& model, const Scan& scene, vec<ModelInstance>* out);

//! Create a ModelInstance of model, transform to centroid of segs, push instance to out and return AABB of segs
geo::BBox placeModelInstance(const Model& model, const segmentation::VecConstSegPtr& segs, vec<ModelInstance>* out);

//! Compute overlap ratio between the current placement of mInst and all segments in seg - overlap of skelPoints with
//! mInst at same position
//! [-1,1] where -1 is all skelRange skel points within mInst, and no mInst points within OBB of seg,
//! and 1 is all mInst sample points contained by seg's OBB and none of skel points within mInst
float placementScore(const ModelInstance& mInst, const segmentation::VecConstSegPtr& segs, const SkelRange& skelRange,
                     size_t numSamples = 1000);

//! Helper function for sampling from a TriMesh
vec<geo::Vec3f> sampleMesh(const ml::TriMeshf& mesh, size_t numSamples);

//! Helper function for sampling from a MeshSegment
vec<geo::Vec3f> sampleMeshSegment(const segmentation::SegPtr segPtr, size_t numSamples);

//! Helper to return a set of numPoints points randomly sampled from the volumes of the skeletons in skelRange
geo::Matrix3Xf getSkelPoints(const SkelRange& skelRange, int numPoints);

}  // namespace synth
}  // namespace mesh
}  // namespace sg


