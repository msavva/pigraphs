#pragma once

#include <functional>

#include <mLibCore.h>

#include "libsg.h"  // NOLINT

namespace sg {
namespace segmentation {

// Convenience typedefs
typedef sg::geo::OBB OBB;
typedef vec<pair<UINT, UINT>> MeshEdges;
typedef vec<int> SegIndices;
typedef vec<size_t> Segment;
typedef std::function<bool(const MeshSegment&)> SegmentFilter;
typedef std::shared_ptr<MeshSegment> SegmentPtr;
typedef vec<SegmentPtr> VecSegPtr;

//! Encapsulation struct for segmentation parameters
struct SegmentationParams {
  explicit SegmentationParams(const sg::util::Params& p);
  float kthresh;          //! Felzenswalb local merging threshold (higher values lead to larger segments)
  unsigned minSegVerts;   //! Segments with fewer than this number of vertices are merged into a larger segments
  float minSegDiag;       //! Minimum diagonal size below which segments are rejected
  float segAspectCutoff;  //! Cutoff aspect ratio above which segments are rejected
  float colorWeight;      //! Weight of HSL color difference in edge weight formula (0 = no color, 1 = only color)
  bool constrainZup;      //! Whether to constrain OBB fits to have Z-axis be upwards
};

//! Return edges between vertices in TriMeshf
MeshEdges getEdges(const ml::TriMeshf& mesh);

//! Create MeshSegments corresponding to segment ids in SegIndices. Segments passing filter are retained in acceptedSegs,
//! and the rest are in rejectedSegs. Parameter constrainZup enforces one OBB axis to be upwards in the Z direction
void createSegments(const ml::TriMeshf& mesh, const SegIndices& ids, const SegmentFilter& filter, bool constrainZup,
                    VecSegPtr* acceptedSegs, VecSegPtr* rejectedSegs);

//! Create MeshSegments corresponding to segment ids in SegIndices. 
//! The segment ids are retained as they are and are assumed to come from a valid segmentation 
//! with ids >=0 indicated accepted (retained in acceptedSegs), and the rest are in rejectedSegs.
//! Parameter constrainZup enforces one OBB axis to be upwards in the Z direction
void createSegments(const ml::TriMeshf& mesh, const SegIndices& ids, bool constrainZup,
                    VecSegPtr* acceptedSegs, VecSegPtr* rejectedSegs);

//! Returns a vector re-mapping vertices to weld identical vertices to same index
vec<int> weldVertices(const ml::TriMeshf& mesh);

// Generic segmentation interface
class Segmentator {
 public:
  //! Given a mesh (bunch of triangles), segments the mesh and returns 
  //! the segment indices
  virtual SegIndices segment(const ml::TriMeshf& mesh) = 0;

  //! Given a vector of meshes, returns the segment indices for each mesh
  vec<SegIndices> segment(const vec<const ml::TriMeshf>& meshes);
};

// Based on graph-cut approach of "Efficient Graph-Based Image Segmentation" Felzenswalb et al. 2007
// http://cs.brown.edu/~pff/segment/
class SegmentatorFelzenswalb : public Segmentator {
 public:
  //! Creates a new Felzenswalb segmentator 
  SegmentatorFelzenswalb(const SegmentationParams& p)
    : m_kthr(p.kthresh)
    , m_minSegVerts(p.minSegVerts)
    , m_colorWeight(p.colorWeight) { }
  virtual SegIndices segment(const ml::TriMeshf& mesh);
 private:
  const float m_kthr;
  const size_t m_minSegVerts;
  const float m_colorWeight;
};

}  // namespace segmentation
}  // namespace sg


