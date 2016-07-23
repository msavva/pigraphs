#pragma once

#include <mLibCore.h>

#include "libsg.h"  // NOLINT

#include "geo/geo.h"
#include "segmentation/Part.h"

namespace sg {
namespace segmentation {

struct MeshSegment : public Part {
  MeshSegment(const ml::TriMeshf& _mesh, const vec<size_t>& _elements, int _id,
              bool constrainZup, float shrinkPercentile = 0, 
              const geo::Transform& _xform = geo::Transform::Identity(),
              // Mesh tri indices (when appropriate)
              const vec<size_t>& _triIndices = vec<size_t>(),
              // Number of points to sample for creating obbs
              const size_t nPointsToSampleForObb = 0);
  ~MeshSegment();

  //! Mesh from which this segment came from
  const ml::TriMeshf* mesh;
  //! Transform on the mesh used to create the points
  const geo::Transform transform;  
  //! Indicies into vertices of the mesh
  vec<size_t> elements;
  //! Transformed points of the mesh segment (+ any sampled points)
  vec<ml::vec3f> points;
  mutable vecd rawFeatureCache;
  ml::vec3f shrunkOBBaxes;
  //! Area weighted normal over the transformed mesh for the mesh segment
  ml::vec3f areaWeightedNormal;
  ml::vec3f segNormal;
  //! Indicies into triangles of the mesh
  vec<size_t> triIndices;

  //! Returns a copy of this mesh segment with the given transform
  std::shared_ptr<MeshSegment> copy(bool constrainZup, 
                                    float shrinkPercentile = 0, 
                                    const geo::Transform& xform = geo::Transform::Identity()) {
    return std::make_shared<MeshSegment>(*mesh,elements,id,constrainZup,shrinkPercentile,xform,triIndices);
  }

  void samplePoints(size_t numSamples, vec<geo::Vec3f>* pPoints) const;

 private:
  void init(bool constrainZup, float shrinkPercentile, size_t nPointsToSampleForObb);
  ml::vec3f computeAreaWeightedNormal();
};

}  // namespace segmentation
}  // namespace sg


