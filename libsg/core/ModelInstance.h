#pragma once

#include "libsg.h"  // NOLINT

#include <mLibCore.h>

#include "core/SceneNode.h"
#include <boost/optional.hpp>

namespace sg {
namespace core {

typedef segmentation::SegPtr SegPtr;
typedef segmentation::VecSegPtr VecSegPtr;

//! An instance of a model of the given id with a particular transform and color
class ModelInstance : public SceneNode {
 public:
  explicit ModelInstance(const Model& _model,
                         const geo::Transform& _xform = geo::Transform::Identity(),
                         const ml::vec4f& _color = ml::ColorUtils::colorById<ml::vec4f>(0),
                         const string& _category = "")
    : SceneNode(_xform), model(_model), color(_color), category(_category) { }

  //! Computes world AABB of ModelInstance by transforming model-space BBox
  //! by current transform and fitting a new AABB
  geo::BBox computeWorldBBoxFast() const;
  //! Computes world AABB of ModelInstance by transforming each model point
  //! by current transform and fitting a new AABB
  geo::BBox computeWorldBBox() const;
  //! Computes world OBB of ModelInstance by transforming model voxel centers
  //! by current transform and fitting an OBB
  geo::OBB computeWorldOBB() const;

  //! Retrieves transformed segments associated with this model instance
  bool getSegments(VecSegPtr* pSegs, bool constrainZup, float shrinkPercentile = 0) const;

  const Model& model;
  ml::vec4f color;
  string category; // Category used to retrieve this modelInstance
  boost::optional<double> placementScore;    // Placement score associated with the model
};

//! Compute raw (unnormalized) volumetric overlap between two ModelInstances
//! Optionally, subsample computation by using only kSamples points
float voxelOverlapRaw(const ModelInstance& mInstA, const ModelInstance& mInstB,
                      int kSamples = std::numeric_limits<int>::max());

//! Compute normalized [0,1] volumetric overlap between two ModelInstances
//! Optionally, subsample computation by using only kSamples points
float voxelOverlapNormalized(const ModelInstance& mInstA, const ModelInstance& mInstB,
                             int kSamples = std::numeric_limits<int>::max());

//! Compute raw (unnormalized) volumetric overlap between all pairs of ModelInstances
//! Optionally, subsample computation by using only kSamples points
float voxelOverlapRaw(const vec<ModelInstance>& modelInstances, 
                      int kSamples = std::numeric_limits<int>::max());


}  // namespace core
}  // namespace sg


