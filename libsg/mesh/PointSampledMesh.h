#pragma once

#include <unordered_map>

#include "libsg.h"  // NOLINT
#include "geo/geo.h"
#include "mesh/MeshSamples.h"

namespace sg {
namespace mesh {

// Point sampled representation of mesh (uses KD Tree for sampled points)
// A wrapper class for obtaining a set of point samples from a source triangle
// Mesh object. Parameter add_centroids forces the centroids of all triangles
// to be added in as samples (similarly for add_vertices)
template <typename Mesh, typename KdTree>
class PointSampledMesh {
 public:
  typedef geo::Vec3f Vec3f;
  typedef geo::KNNWorkspace KNNWorkspace;
  typedef std::unordered_map<size_t, vec<size_t>> Tri2PointMap;
  explicit PointSampledMesh(Mesh* mesh, size_t n_points,
                            bool add_centroids = true,
                            bool add_vertices = true) {
    vec<typename Mesh::FaceHandle> faces;
    std::copy(mesh->faces_begin(), mesh->faces_end(),
              std::back_inserter(faces));
    samples_ = new MeshSamples(mesh, faces.begin(), faces.end(), n_points,
                               add_centroids, add_vertices);
    tri2pointMap_ = new Tri2PointMap(samples_->size());
    for (size_t i = 0; i < samples_->size(); i++) {
      const size_t triIdx = samples_->faces()[i].idx();
      (*tri2pointMap_)[triIdx].push_back(i);
    }
    kd_tree_ = new KdTree(samples_->points(), 10 /* leaf_max_size */);
  }

  ~PointSampledMesh() {
    if (samples_) { delete samples_; }
    if (kd_tree_) { delete kd_tree_; }
    if (tri2pointMap_) { delete tri2pointMap_; }
  }

  // Find k nearest neighbors to query point within KD tree and return in ws
  void findKNN(const Vec3f& query, size_t k, KNNWorkspace* ws) const {
    kd_tree_->queryKNN(query, k, &(ws->out_indices), &(ws->out_dists_sq));
  }

  // Find points in KD tree within radius r of query and return in ws
  size_t findRadius(const Vec3f& query, float r, KNNWorkspace* ws) const {
    size_t k = kd_tree_->queryRadius(query, r * r, &(ws->out_pairs));
    return k;
  }

  const MeshSamples& samples() const { return *samples_; }

 private:
  PointSampledMesh(const PointSampledMesh&);
  PointSampledMesh& operator=(const PointSampledMesh&);

  MeshSamples* samples_;
  KdTree* kd_tree_;
  Tri2PointMap* tri2pointMap_;
};

}  // namespace mesh
}  // namespace sg


