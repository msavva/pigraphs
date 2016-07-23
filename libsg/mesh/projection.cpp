#include "common.h"  // NOLINT

#include "mesh/projection.h"

#include "geo/geo.h"

void sg::mesh::projection::projectMeshXY(const ml::TriMeshf& mesh, float binSize, VertexStackGrid* grid) {
  const auto& V = mesh.getVertices();
  const auto bbox = mesh.computeBoundingBox();
  // Initialize grid
  const auto dims = grid->dimensions();
  if (dims[0] != bbox.getExtentX() || dims[1] != bbox.getExtentY()) {
    grid->init(bbox.getMinX(), bbox.getMinY(), bbox.getMaxX(), bbox.getMaxY(), binSize);
  }
  // Add vertices to appropriate grid bin
  for (const auto& vert : V) {
    VertexStack& vStack = grid->get(vert.position.x, vert.position.y);
    vStack.verts.push_back(vert);
  }
  // Sort vertices by height
  for (VertexStack& vStack : *grid) {
    if (vStack.verts.empty()) { continue; }
    typedef ml::TriMeshf::Vertex VertT;
    vStack.verts.sort([ ] (const VertT & a, const VertT & b) { return a.position.z > b.position.z; });
  }
}

ml::TriMeshf sg::mesh::projection::toTriMesh(const VertexStackGrid& vStackGrid, float thickness /*= 0.01f*/) {
  vec<ml::TriMeshf> meshes;
  const geo::Vec2f
    min  = vStackGrid.min(),
    max  = vStackGrid.max();
  const float step = vStackGrid.binSize();
  for (float x = min.x(); x < max.x(); x += step) {
    for (float y = min.y(); y < max.y(); y += step) {
      const VertexStack& vStack = vStackGrid.get(x, y);
      if (vStack.verts.empty()) { continue; }
      const auto& topVert = vStack.verts.front();
      const ml::vec3f pos(x, y, topVert.position.z);
      const auto bottom = ml::vec3f(x, y, 0);
      meshes.push_back(ml::Shapesf::cylinder(bottom, pos, thickness * 0.5f, 2, 4, topVert.color));
    }
  }

  return ml::meshutil::createUnifiedMesh(meshes);
}
