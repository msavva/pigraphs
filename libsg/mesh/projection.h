#pragma once

#include <mLibCore.h>

#include "libsg.h"  // NOLINT
#include "util/grid.h"

namespace sg {
namespace mesh {
namespace projection {

//! Vertices ordered in descending order by one of their coordinates (typically z)
struct VertexStack { std::list<ml::TriMeshf::Vertex> verts; };

//! A BinnedGrid containing VertexStacks at each bin
typedef util::BinnedGrid<VertexStack> VertexStackGrid;

//! Projects mesh vertices into VertexStackGrid grid with bins of given size.  Grid is initialized and VertexStacks
//! are ordered by descending vertex height.
void projectMeshXY(const ml::TriMeshf& mesh, float binSize, VertexStackGrid* grid);

//! Create TriMesh visualization of VertexStackGrid
ml::TriMeshf toTriMesh(const VertexStackGrid& vStackGrid, float thickness = 0.01f);

}  // namespace projection
}  // namespace mesh
}  // namespace sg


