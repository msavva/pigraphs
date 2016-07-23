#pragma once

#include <functional>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "libsg.h"  // NOLINT
#include <mesh/ClusterMesh.h>

namespace sg {
namespace segmentation {

// Wrapper class for a mesh of type T that has elements of type elT clustered into clusters
class SegmentatorClusterMesh {
 public:
  // Convenience typedefs
  typedef sg::mesh::ClusterMesh ClusterMesh;
  typedef ClusterMesh::elT elT;
  typedef ClusterMesh::Mesh Mesh;
  typedef ClusterMesh::Color Color;

  // Enum for different algorithm types
  static enum Algorithm {
    REGION_GROW_TOPOLOGY = 0,
    REGION_GROW_KNN,
    RANDOM_WALK
  };

  // Simple region growing segmentation. Start at random seed locations in
  // ClusterMesh and find neighbors by calling neighbor_finder. The parameter
  // normal_dist_eps controls thresholding of cluster growth at the given
  // normal distance value (i.e. faces will not be merged into a cluster if
  // their normal distance to the cluster is higher than the threshold).
  // A negative value indicates no thresholding.
  static void regionGrowingSegmentation(ClusterMesh* cmesh,
                                        std::function<void(const elT&, vec<elT>&)> neighbor_finder,
                                        float normal_dist_eps = -1);

  // Perform random walk based segmentation as described by [Lai et al. 2009]
  static void randomWalkSegmentationFromSeeds(ClusterMesh* cmesh,
                                              const vec<elT>& seeds,
                                              const string& debug_file = "");

 private:
  // Tri-Tri "graphical model" distance function from [Lai et al. 2009]
  // Returns vector indexed by edge id with value of p_i,k for that edge.
  // Corresponds to transition across edge from one face to another
  // NOTE: p_i,k values should be normalized over each face when used as
  // transition probabilities
  static vecf compute_d_1s(Mesh* mesh);

  // Forms Laplacian matrix of ClusterMesh cmesh with seed faces given in seeds.
  // Returns the matrix and also a re-ordering of face handles in L_idx vector.
  // Seed faces come first and then all other face handles, index in that vector
  // corresponds to matrix index.  Laplacian has form of d_i (1.0 in our case)
  // if i=j (diagonal), -w_i_j if v_i and v_j are adjacent, and 0 otherwise
  Eigen::SparseMatrix<float>
  static formLaplacian(ClusterMesh* cmesh,
                       const vec<elT>& seeds,
                       vec<elT>* L_idx_to_face_handle);

  // Takes Laplacian formulation of random walks problem, splits up matrix
  // into relevant components depending on number of unlabeled faces and seeds
  // to solve system and return potential matrix X [n_unlabeled x n_seeds]
  static Eigen::SparseMatrix<float>
  solveSystem(const Eigen::SparseMatrix<float>& L, const size_t n_unlabeled,
              const size_t n_seeds);

  // Assign faces in ClusterMesh to clusters by labeling faces with the same
  // cluster of the seed with maximum potential at each face
  static void assignClustersByMaxPotential(ClusterMesh* cmesh,
                                           const vec<elT>& seeds,
                                           const Eigen::SparseMatrix<float>& X,
                                           const vec<elT>& L_i_face);

  // Helper visualization function for debugging purposes
  // Color faces by "normalized" values of solved potentials in X
  static void colorFacePotentials(ClusterMesh* cmesh,
                                  const vec<elT>& seeds,
                                  const Eigen::SparseMatrix<float>& X,
                                  const vec<elT>& L_i_to_face);
};

}  // namespace segmentation
}  // namespace sg


