#pragma once

#include "libsg.h"  // NOLINT
#include "ann/KdTreeNanoflann.h"
#include "geo/geo.h"
#include "geo/OBB.h"
#include "mesh/PointSampledMesh.h"
#include "mesh/TriMesh.h"

namespace sg {
namespace mesh {

// Class representing a clustered mesh
class ClusterMesh {
 public:
  typedef geo::Vec3f Vec3f;
  typedef geo::VecVec3f VecVec3f;
  typedef TriMesh Mesh;
  typedef Mesh::FaceHandle elT;
  typedef Mesh::VertexHandle VertexHandle;
  typedef Mesh::Point Point;
  typedef Mesh::Normal Normal;
  typedef Mesh::Color Color;
  typedef ann::ANN<VecVec3f, float> KdTree;

  // Represents a segmentation cluster containing mesh elements
  struct Cluster {
    explicit Cluster(const elT& x) : seed_(x) { *this += seed_; }
    size_t size() const { return elements_.size(); }
    void operator += (const elT& x) { elements_.push_back(x); }
    elT seed() const { return seed_; }
    const vec<elT>& elements() const { return elements_; }
    typedef int Handle;
    static const Handle NONE = -1;

   private:
    // Disable copy and copy-assignment constructors
    Cluster(const Cluster&);
    Cluster& operator=(const Cluster&);
    const elT seed_;
    vec<elT> elements_;
  };
  typedef Cluster::Handle ClusterHandle;

  // A "part" of the mesh created from a cluster by sampling and OBB fitting
  struct Part {
    Part(const Cluster* c, Mesh* m, size_t nSamples, const geo::OBB::FitOptFlags opts)
      : cluster(c)
      , samples(*m, begin(c->elements()), end(c->elements()), nSamples)
      , obb(samples.pBegin(), samples.pEnd(), opts) { }
    const Cluster* cluster;     // Mesh Cluster this Part represents
    const MeshSamples samples;  // MeshSamples from Cluster faces
    const geo::OBB obb;         // OBB bounding samples
  };

  // Constructor
  explicit ClusterMesh(Mesh* _mesh);

  // Destructor
  ~ClusterMesh();

  Mesh* mesh() const { return mesh_; }

  // Reset state to unclustered
  void reset();

  // Create new cluster starting with seed face x. Returns ClusterHandle of
  // newly created Cluster
  ClusterHandle newCluster(const elT& x);

  // Add element x to the Cluster with ClusterHandle cid.  If element x is
  // unclustered, then it is added to the Cluster and true is returned,
  // otherwise false is returned
  bool addToCluster(const ClusterHandle& cid, const elT& x);

  // Normal distance between the normal of Cluster with handle C and normal
  // of element f
  float normalDistance(const ClusterHandle& C, const elT& f);

  // Retrieve unclustered neighbors of f and return in neighbors
  void getUnclusteredNeighborsFaceTraversal(const elT& x,
                                            vec<elT>& neighbors);

  // Retrieve k nearest unclustered neighbors of x using point sample based
  // nearest neighbor search on pmesh
  void getUnclusteredNeighborsKNN(const PointSampledMesh<Mesh, KdTree>& pmesh,
                                  size_t k, const elT& x, vec<elT>& neighbors);

  // Color all mesh faces by the id of the cluster to which they belong
  void colorFacesByClusterId();

  // Simple accessors
  size_t numClusters() const    { return clusters_.size(); }
  size_t numUnclustered() const { return numFaces() - n_clustered_; }
  size_t numClustered() const   { return n_clustered_; }
  size_t numFaces() const       { return n_faces_; }
  bool allClustered() const     { return numFaces() == numClustered(); }

  // Get the ClusterHandle of the cluster to which element x belongs
  ClusterHandle getCluster(const elT x) {
    return mesh_->property(fprop_cluster_, x);
  }

  // Return a vector of pointers to all clusters
  vec<Cluster*> getClusters() const { return clusters_; }

  // Return whether element x belongs to a cluster
  bool isUnclustered(const elT x) { return getCluster(x) == Cluster::NONE; }

  // Set the cluster of element x to be the one with ClusterHandle c
  void setCluster(const elT x, const ClusterHandle c) {
    *clusters_[c] += x;
    mesh_->property(fprop_cluster_, x) = c;
  }

  // Return a random element out of all elements that are yet to be clustered
  elT randomUnclustered() {
    auto it = std::find_if(begin(rand_elems_), end(rand_elems_),
    [this](const elT & x) { return isUnclustered(x); });
    return *it;
  }

  // Return a vector of one randomly chosen face per Cluster
  vec<elT> getClusterRandomFaces();

  // Return a vector of n_seeds seed elements. Elements are chosen iteratively
  // at random by maximizing distance from already chosen elements
  vec<elT> getCoarseSeedFaces(size_t n_seeds);

  // Create and return a set of parts corresponding to this ClusterMesh's
  // clusters. Use nSample total samples distributed by area
  // TODO(ms): Clean this up so that part storage is managed within ClusterMesh
  vec<Part*> partsFromClusters(size_t nSample);

 private:
  Mesh* mesh_;
  vec<Cluster*> clusters_;                       // indexed clusters
  size_t n_clustered_;                                     // # clustered
  size_t n_faces_;                                         // # total els
  vec<elT> rand_elems_;                          // Randomized els
  OpenMesh::FPropHandleT<ClusterHandle> fprop_cluster_;  // Cluster assignment
  OpenMesh::EPropHandleT<double> eprop_w_;               // Transition probab
  OpenMesh::FPropHandleT<double> fprop_d_;               // Degree (sum of w)

  // Disallow copy and copy assignment for now by making them private
  ClusterMesh(const ClusterMesh&);
  ClusterMesh& operator=(const ClusterMesh&);
};

}  // namespace mesh
}  // namespace sg


