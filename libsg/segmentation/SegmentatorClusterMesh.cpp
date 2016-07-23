#include "common.h"  // NOLINT

#include "segmentation/SegmentatorClusterMesh.h"

#include <deque>
#include <functional>

#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include "math/math.h"

namespace sg {
namespace segmentation {

void SegmentatorClusterMesh::regionGrowingSegmentation(ClusterMesh* cmesh,
                                                       std::function<void(const elT&, vec<elT>&)> neighbor_finder,
                                                       float normal_dist_eps /* = -1 */) {
  std::deque<elT> Q;
  std::modulus<size_t> mod;
  vec<elT> neighbors;

  // Loop until all elements clustered
  while (!cmesh->allClustered()) {
    // Choose seed element, insert into Q, create new cluster C from seed
    auto seed = cmesh->randomUnclustered();
    Q.push_back(seed);
    auto C = cmesh->newCluster(seed);

    // While there are more elements in Q
    while (!Q.empty()) {
      // Get next element s from Q
      elT s = Q.back();
      Q.pop_back();

      // If we don't threshold by normal, or s can be clustered into C
      if (normal_dist_eps < 0 || cmesh->normalDistance(C, s)
          < normal_dist_eps) {
        // Cluster s into C
        cmesh->addToCluster(C, s);

        // Insert neighbors of s into Q
        neighbors.clear();
        neighbor_finder(s, neighbors);

        for (auto n : neighbors) { Q.push_back(n); }
      }

      // Report remaining
      size_t remaining = cmesh->numUnclustered();
      if (mod(remaining, static_cast<size_t>(1000)) == 0) { cout << remaining << endl;}
    }
  }
}

void SegmentatorClusterMesh::randomWalkSegmentationFromSeeds(ClusterMesh* cmesh,
                                                             const vec<elT>& seeds,
                                                             const string& debug_file /* = "" */) {
  // Constants
  const size_t n_faces = cmesh->mesh()->n_faces();
  const size_t n_seeds = seeds.size();
  const size_t n_unlabeled = n_faces - n_seeds;

  // Will store mapping from Laplacian indices to faces
  vec<elT> L_idx_to_face;

  const Eigen::SparseMatrix<float> L =
    formLaplacian(cmesh, seeds, &L_idx_to_face);

  const Eigen::SparseMatrix<float> X = solveSystem(L, n_unlabeled, n_seeds);

  assignClustersByMaxPotential(cmesh, seeds, X, L_idx_to_face);

  // Write out to file for debugging
  if (debug_file.size() > 0) {
    ofstream file;
    file.open(debug_file);
    file << "L=" << endl << L.toDense() << endl;
    file << "X=" << endl << X.toDense() << endl;
    file.close();
  }
}

vecf SegmentatorClusterMesh::compute_d_1s(Mesh* mesh) {
  // Constants
  const size_t n_edges = mesh->n_edges();
  const float eta_convex = 0.2f;
  const float eta_concave = 1.0f;
  const float sigma = 1.0f;
  const float eps = 1e-6f;  // Clamping to prevent corner case 0 or 1 probs

  // Storage for edges
  vecf d_1s(n_edges);
  vecf edge_lengths(n_edges);

  // Compute edge lengths and d_1 values
  float dihedral_angle, eta, d_1;
  Mesh::EdgeHandle e_handle;
  Mesh::EdgeIter e_it;
  int e_idx;
  for (e_it = mesh->edges_sbegin(); e_it != mesh->edges_end();
       ++e_it) {
    e_handle = *e_it;
    e_idx = e_handle.idx();
    dihedral_angle = static_cast<float>(mesh->calc_dihedral_angle_fast(e_handle));
    eta = (dihedral_angle >= 0) ? eta_convex : eta_concave;
    edge_lengths[e_idx] = static_cast<float>(mesh->calc_edge_length(e_handle));
    d_1 = eta * (1.0f - abs(cosf(dihedral_angle)));
    d_1 = math::clamp(d_1, eps, 1.0f - eps);
    d_1s[e_idx] = d_1;
  }

  // Compute mean d_1
  float d_mean = 0.0f;
  for (auto d_1 : d_1s) { d_mean += d_1; }
  d_mean /= n_edges;
  assert(d_mean > 0.0f);
  float neg_mean_inv = - 1.0f / d_mean;

  // Compute max edge length and normalize edge lengths
  // NOTE: This is not mentioned in Lai et al.
  float edge_max = 0.0f;
  for (auto l : edge_lengths) if (l > edge_max) { edge_max = l; }
  edge_max += eps;  // Prevents normalization to exactly 1
  float edge_max_inv = 1.0f / edge_max;
  for (size_t i = 0; i < n_edges; i++) { edge_lengths[i] *= edge_max_inv; }

  // Normalize d_1's and compute probabilities p_i,k
  vecf probs(n_edges);
  for (size_t i = 0; i < n_edges; i++) {
    const float p = edge_lengths[i] * expf(d_1s[i] * neg_mean_inv);
    const float p2 = math::clamp(p, eps, 1.0f - eps);
    probs[i] = p2;
    assert(p2 < 1.0f && p2 > 0.0f);
  }

  return probs;
}

Eigen::SparseMatrix<float>
SegmentatorClusterMesh::formLaplacian(ClusterMesh* cmesh,
                                      const vec<elT>& seeds,
                                      vec<elT>* L_idx_to_face_handle) {
  // Constants
  const size_t n_faces = cmesh->mesh()->n_faces();
  const size_t n_seeds = seeds.size();
  const size_t n_unlabeled = n_faces - n_seeds;

  // Compute tri-tri edge weights
  vecf e_weights = compute_d_1s(cmesh->mesh());

  // We'll store re-ordered faces here: seeds first, then unlabeled faces
  L_idx_to_face_handle->resize(n_faces);
  for (size_t i = 0; i < n_seeds; i++) { (*L_idx_to_face_handle)[i] = seeds[i]; }
  size_t idx = n_seeds;
  Mesh::FaceIter f_it = cmesh->mesh()->faces_sbegin();
  for (; idx < n_faces && f_it != cmesh->mesh()->faces_end(); ++f_it) {
    // Add if not found in seeds (brute search for now since few seeds)
    if (std::find(begin(seeds), end(seeds), *f_it) == end(seeds)) {
      (*L_idx_to_face_handle)[idx] = *f_it;
      idx++;
    }
  }

  // Vector mapping faces to their new indices (for ordering elements in L)
  map<size_t, size_t> face_idx_to_L_idx;
  size_t face_idx;
  for (size_t L_i = 0; L_i < n_faces; L_i++) {
    face_idx = (*L_idx_to_face_handle)[L_i].idx();
    face_idx_to_L_idx[face_idx] = L_i;
  }

  // Create sparse Laplacian matrix
  Eigen::SparseMatrix<float> L(static_cast<int>(n_faces), static_cast<int>(n_faces));
  L.reserve(Eigen::VectorXi::Constant(n_faces, 4));

  // Now loop over face handles, retrieve appropriate w_ij and put in L
  Mesh::FaceEdgeIter fe_it;
  Mesh::FaceHandle f_i, f_j1, f_j2, f_j;
  Mesh::HalfedgeHandle h_handle;
  size_t e_idx, f_j_idx, j;  // NOTE: f_j_idx = original, j = re-ordered
  float weight;
  for (size_t i = 0; i < n_faces; i++) {
    f_i = (*L_idx_to_face_handle)[i];
    fe_it = cmesh->mesh()->fe_iter(f_i);
    float sum_weight = 0.0f;
    for (; fe_it.is_valid(); ++fe_it) {
      e_idx = fe_it->idx();
      weight = e_weights[e_idx];
      sum_weight += weight;
      // Grab half-edge handle to find opposite face
      h_handle = cmesh->mesh()->halfedge_handle(*fe_it, 0);
      f_j1 = cmesh->mesh()->face_handle(h_handle);
      f_j2 = cmesh->mesh()->opposite_face_handle(h_handle);
      f_j = (f_j1 == f_i) ? f_j2 : f_j1;  // Pick opposite face
      if (cmesh->mesh()->is_valid_handle(f_j)) {  // Handles boundary cases
        f_j_idx = f_j.idx();
        j = face_idx_to_L_idx[f_j_idx];
        L.insert(static_cast<int>(i), static_cast<int>(j)) = -weight;
      }
    }
    assert(sum_weight > 0.0f);
    L.insert(static_cast<int>(i), static_cast<int>(i)) = sum_weight;
  }

  return L;
}

Eigen::SparseMatrix<float>
SegmentatorClusterMesh::solveSystem(const Eigen::SparseMatrix<float>& L,
                                    size_t n_unlabeled,
                                    size_t n_seeds) {
  // Sanity check size of L
  const size_t n_all = n_unlabeled + n_seeds;
  assert(L.rows() == n_all);
  assert(L.cols() == n_all);

  // Now break L up into components L = [L_M B ; B^T L_U] and solve system
  // L_U * X = -B^T * M to obtain potentials X (see definition of M below)

  // L_U is the [n_unlabeled x n_unlabeled] bottom right block which forms
  // the left hand-side matrix of the system
  const Eigen::SparseMatrix<float> L_U =
    L.bottomRightCorner(static_cast<int>(n_unlabeled), static_cast<int>(n_unlabeled));

  // B_T is the [n_unlabeled x n_seeds] bottom left block which forms part
  // of the right hand-side
  const Eigen::SparseMatrix<float> B_T =
    L.bottomLeftCorner(static_cast<int>(n_unlabeled), static_cast<int>(n_seeds));

  // M [n_seeds x n_seeds] has columns m^s for each seed s
  // row i of m^s equal to 1 if i=s, 0 otherwise
  // Since we only have one seed element at i=s and we re-ordered faces so
  // that seeds span from 0 to n_seeds-1, this is just the identity matrix I
  // Eigen::SparseMatrix<float> M(n_seeds, n_seeds);
  // M.setIdentity();

  // Factor the left hand side matrix L_U using sparse Cholesky LDLT
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<float> > solver(L_U);
  if (solver.info() != Eigen::Success) {
    cout << "Factoring L_U matrix failed with code=" << solver.info()
              << endl;
  }

  // Form right hand side. Note: we skip idempotent multiplication by identity
  const Eigen::SparseMatrix<float> B = -1.0f * B_T;  // * M;

  // Solve for X [n_unlabeled x n_seeds] potentials matrix using CG
  Eigen::SparseMatrix<float> X = solver.solve(B);

  return X;
}

void SegmentatorClusterMesh::assignClustersByMaxPotential(ClusterMesh* cmesh,
                                                          const vec<elT>& seeds,
                                                          const Eigen::SparseMatrix<float>& X,
                                                          const vec<elT>& L_i_face) {
  // Constants and initialization
  const float eps = 0.001f;
  const size_t n_seeds = seeds.size();
  const size_t n_unlabeled = X.innerSize();

  // Create cluster for each seed
  vec<ClusterMesh::ClusterHandle> clusters(n_seeds);
  for (size_t i = 0; i < n_seeds; i++) {
    clusters[i] = cmesh->newCluster(seeds[i]);
  }

  // Assign each face to cluster with max potential
  vec<Eigen::MatrixXf::Index> max_idx(n_unlabeled);
  const Eigen::MatrixXf X_dense = X.toDense();
  for (size_t f_i = 0; f_i < n_unlabeled; f_i++) {
    X_dense.row(f_i).maxCoeff(&max_idx[f_i]);
    const elT face = L_i_face[f_i + n_seeds];  // note offset
    cmesh->addToCluster(clusters[max_idx[f_i]], face);
  }
}

void SegmentatorClusterMesh::colorFacePotentials(ClusterMesh* cmesh,
                                                 const vec<elT>& seeds,
                                                 const Eigen::SparseMatrix<float>& X,
                                                 const vec<elT>& L_i_to_face) {
  // Constants and initialization
  const float eps = 0.001f;
  const size_t n_seeds = seeds.size();
  assert(X.outerSize() == n_seeds);
  const size_t n_unlabeled = X.innerSize();

  // Subtract min values and divide by max to ensure values range in [0,1]
  // This makes it saner to mix colors below
  Eigen::MatrixXf X_norm = X;
  Eigen::VectorXf mins = X_norm.rowwise().minCoeff();
  X_norm = (X_norm.colwise() - mins);
  Eigen::VectorXf maxs = X_norm.rowwise().maxCoeff();
  for (size_t i = 0; i < n_unlabeled; i++) {
    for (size_t j = 0; j < n_seeds; j++) {
      float max = maxs[i];
      if (max > eps) { X_norm(i, j) /= max; }
    }
  }

  // Weight color of each seed by potential for face and add in
  for (size_t s_i = 0; s_i < n_seeds; s_i++) {
    Color c_seed = cmesh->mesh()->color(seeds[s_i]);
    Color c(1, 1, 1, 1);
    for (size_t f_i = 0; f_i < n_unlabeled; f_i++) {
      float potential = X_norm.coeff(f_i, s_i);
      auto face_handle = L_i_to_face[f_i + n_seeds];  // note n_seeds offset
      if (s_i == 0) {
        c = c_seed * potential; c[3] = 1;
      } else {
        c = cmesh->mesh()->color(face_handle);
        c += c_seed * potential;
        c[3] = 1;
      }
      cmesh->mesh()->set_color(face_handle, c);
    }
  }
}

}  // namespace segmentation
}  // namespace sg
