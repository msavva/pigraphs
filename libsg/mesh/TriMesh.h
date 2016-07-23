#pragma once

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include "libsg.h"  // NOLINT

namespace sg {
namespace mesh {

// TriMesh traits
struct TriTraits : public OpenMesh::DefaultTraits {
  typedef OpenMesh::Vec3d Point;
  typedef OpenMesh::Vec3d Normal;
  typedef OpenMesh::Vec4f Color;
};

// Convenience typedef for OpenMesh TriMesh
typedef OpenMesh::TriMesh_ArrayKernelT<TriTraits>  TriMesh;

}  // namespace mesh
}  // namespace sg


