#include "common.h"  // NOLINT

#include "mesh/ClusterMesh.h"

#include "math/math.h"

namespace sg {
namespace mesh {

ClusterMesh::ClusterMesh(Mesh* _mesh)
  : mesh_(_mesh)
  , clusters_(0)
  , n_clustered_(0)
  , n_faces_(_mesh->n_faces())
  , rand_elems_(n_faces_) { reset(); }

ClusterMesh::~ClusterMesh() { for (auto c : clusters_) { if (c) { delete c; } } }

void ClusterMesh::reset() {
  clusters_.clear();
  n_clustered_ = 0;

  // Reset properties
  // Cluster indices
  if (mesh_->get_property_handle(fprop_cluster_, "fprop_cluster")) {
    mesh_->remove_property(fprop_cluster_);
  }
  mesh_->add_property(fprop_cluster_, "fprop_cluster");
  mesh_->property(fprop_cluster_).set_persistent(true);
  for (auto f_it : mesh_->faces()) {
    mesh_->property(fprop_cluster_, f_it) = Cluster::NONE;
  }

  // Edge transition weights
  if (mesh_->get_property_handle(eprop_w_, "eprop_w")) {
    mesh_->remove_property(eprop_w_);
  }
  mesh_->add_property(eprop_w_, "eprop_w");
  mesh_->property(eprop_w_).set_persistent(true);
  for (auto e_it : mesh_->edges()) {
    mesh_->property(eprop_w_, e_it) = -1.0f;
  }

  // Face degrees (edge weight sums)
  if (mesh_->get_property_handle(fprop_d_, "fprop_d")) {
    mesh_->remove_property(fprop_d_);
  }
  mesh_->add_property(fprop_d_, "fprop_d");
  mesh_->property(fprop_d_).set_persistent(true);
  for (auto f_it : mesh_->faces()) {
    mesh_->property(fprop_d_, f_it) = -1.0f;
  }

  // Copy and randomize element order
  rand_elems_.clear();
  rand_elems_.resize(n_faces_);
  std::copy(mesh_->faces_begin(), mesh_->faces_end(), begin(rand_elems_));
  math::DEF_RAND.shuffle(begin(rand_elems_), end(rand_elems_));
}

ClusterMesh::ClusterHandle ClusterMesh::newCluster(const elT& x) {
  clusters_.push_back(new Cluster(x));
  n_clustered_++;
  ClusterHandle cid = static_cast<ClusterHandle>(clusters_.size() - 1);
  mesh_->property(fprop_cluster_, x) = cid;
  return cid;
}

bool ClusterMesh::addToCluster(const ClusterHandle& cid, const elT& x) {
  if (isUnclustered(x)) {
    if (getCluster(x) == cid) {  // Sanity check to prevent re-adding
      __debugbreak();
      return false;
    }
    setCluster(x, cid);
    n_clustered_++;
    return true;
  }
  return false;
}

float ClusterMesh::normalDistance(const ClusterHandle& C, const elT& f) {
  const vec<elT>& Cels = clusters_[C]->elements();
  const size_t nPoints = Cels.size();
  geo::MeshBackedTri<typename Mesh::FaceHandle> tri;
  geo::Vec3f Cnormal;
  const Mesh& m = *mesh_;

  if (nPoints < 3) {  // Just use seed normal
    tri.makeTriFromOpenMesh(m, clusters_[C]->seed());
    Cnormal = tri.normal();
  } else {  // Fit plane and get its normal
    VecVec3f points(nPoints);
    for (size_t i = 0; i < nPoints; i++) {
      tri.makeTriFromOpenMesh(m, Cels[i]);
      points[i] = tri.centroid();
    }
    Cnormal = geo::CoordSystem(points.begin(), points.end()).R().col(2);
  }

  const Vec3f b = geo::vec3f(m.normal(f));
  const float dist = 1.0f - abs(Cnormal.dot(b));
  return dist;
}

void ClusterMesh::getUnclusteredNeighborsFaceTraversal(const elT& x,
                                                       vec<elT>& neighbors) {
  for (auto ff_it = mesh_->cff_iter(x); ff_it.is_valid(); ++ff_it) {
    auto neighbor = *ff_it;
    if (isUnclustered(neighbor)) { neighbors.push_back(neighbor); }
  }
}

//void ClusterMesh::getUnclusteredNeighborsKNN
//(const PointSampledMesh<Mesh>& pmesh, const size_t k, const elT& x,
// vec<elT>& neighbors) {
//  geo::Tri<Mesh> t(mesh_, x);
//  Mesh::ConstFaceVertexIter fv_it = mesh_->cfv_iter(x);
//  Vec3f c = t.centroid();
//  set<elT> seenNeighbors;
//  geo::KNNWorkspace ws;
//  pmesh.findKNN(c, k, &ws);
//  const auto faces = pmesh.samples().faces();
//  for (size_t nId : ws.out_indices) {
//    const auto n = faces[nId];
//    if (isUnclustered(n) && seenNeighbors.count(n) == 0) {
//      neighbors.push_back(n);
//      seenNeighbors.insert(n);
//    }
//  }
//}

void ClusterMesh::getUnclusteredNeighborsKNN
(const PointSampledMesh<Mesh, KdTree>& pmesh, size_t k, const elT& x,
 vec<elT>& neighbors) {
  geo::MeshBackedTri<Mesh::FaceHandle> t;
  t.makeTriFromOpenMesh(*mesh_, x);
  set<elT> seenNeighbors;
  geo::KNNWorkspace ws;
  vec<size_t> indices;
  pmesh.findKNN(t.v0(), k, &ws);
  move(begin(ws.out_indices), end(ws.out_indices),
            back_inserter(indices));
  pmesh.findKNN(t.v1(), k, &ws);
  move(begin(ws.out_indices), end(ws.out_indices),
            back_inserter(indices));
  pmesh.findKNN(t.v2(), k, &ws);
  move(begin(ws.out_indices), end(ws.out_indices),
            back_inserter(indices));
  const auto faces = pmesh.samples().faces();
  for (size_t nId : indices) {
    const auto n = faces[nId];
    if (isUnclustered(n) && seenNeighbors.count(n) == 0) {
      neighbors.push_back(n);
      seenNeighbors.insert(n);
    }
  }
  cout << neighbors.size() << endl;
}

void ClusterMesh::colorFacesByClusterId() {
  for (auto f_it : mesh_->faces()) {
    mesh_->set_color(f_it, ml::ColorUtils::colorById<Color>(getCluster(f_it)));
  }
}

vec<ClusterMesh::elT>
ClusterMesh::getCoarseSeedFaces(size_t n_seeds) {
  // Initializations
  const size_t n_faces = mesh_->n_faces();
  vec<elT> seeds, faces(std::begin(mesh_->faces()), std::end(mesh_->faces()));
  MeshSamples samples(*mesh_, begin(faces), end(faces), 0, true);
  set<elT> unselected;
  unselected.insert(begin(faces), end(faces));

  // Select face farthest from model centroid and add to seeds
  Vec3f centroid(0, 0, 0);
  for (auto p : samples.points()) { centroid += p; }
  centroid /= static_cast<float>(n_faces);
  geo::KNNWorkspace ws;
  KdTree kdtree(samples.points());
  kdtree.queryKNN(centroid, n_faces, &ws.out_indices, &ws.out_dists_sq);
  elT farthest_face = faces[ws.out_indices.back()];
  seeds.push_back(farthest_face);
  unselected.erase(farthest_face);

  // Loop until n_faces have been selected
  while (seeds.size() < n_seeds) {
    // Loop over all unselected faces
    elT farthest_unselected;
    float farthest_unselected_dist = std::numeric_limits<float>::min();
    for (auto f : unselected) {
      const Vec3f& c_i = samples.points()[f.idx()];

      // Find minimum distance of face to a seed
      float min_dist = std::numeric_limits<float>::max();
      for (auto seed : seeds) {
        const size_t f_j = seed.idx();
        const Vec3f& c_j = samples.points()[f_j];
        float dist = (c_i - c_j).squaredNorm();
        if (dist < min_dist) { min_dist = dist; }
      }
      // Select unselected face with max of min dist (farthest from seeds)
      if (min_dist > farthest_unselected_dist) {
        farthest_unselected_dist = min_dist;
        farthest_unselected = f;
      }
    }
    seeds.push_back(farthest_unselected);
    unselected.erase(farthest_unselected);
  }
  return seeds;
}

vec<ClusterMesh::Part*>
ClusterMesh::partsFromClusters(size_t nSample) {
  vec<Part*> parts;
  const float totalArea = geo::totalSurfaceArea(*mesh_,
                                                std::begin(mesh_->faces()),
                                                std::end(mesh_->faces()));
  const float invArea = 1.0f / totalArea;
  for (size_t ci = 0; ci < clusters_.size(); ci++) {
    const Cluster* c = clusters_[ci];
    const float a = geo::totalSurfaceArea(*mesh_, c->elements());
    const size_t nSamples = static_cast<size_t>(std::max(a * invArea * nSample, 100.0f));
    Part* p = new Part(c, mesh_, nSamples, geo::OBB::MIN_PCA | geo::OBB::AABB_FALLBACK);
    cout << "Part " << ci << " nSamples=" << nSamples << endl;
    parts.push_back(p);
  }
  return parts;
}

vec<ClusterMesh::elT> ClusterMesh::getClusterRandomFaces() {
  vec<elT> samples;
  for (Cluster* c : clusters_) { samples.push_back(c->seed()); }
  return samples;
}

}  // namespace mesh
}  // namespace sg
