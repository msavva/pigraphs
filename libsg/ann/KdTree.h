#pragma once

#include "libsg.h"  // NOLINT

namespace sg {
namespace ann {

//! Templated interface for all Kd-tree implementations.
//! vec_t - type of query element MUST have .data() member function and operator[]
//! num_t - type of scalar element
//! index_t - type of index into dataset
template <typename vec_t = vecf, typename num_t = float, typename index_t = size_t>
class KdTree {
 public:
  //! Query for k closest points to given query point. Indices to points stored
  //! in out_indices and distances to points in out_dists.
  //! Returns number of neighbor points retrieved.
  virtual int queryKNN(const vec_t& query, size_t k,
                       vec<index_t>* out_indices,
                       vec<num_t>* out_dists) const = 0;

  //! Query for k closest points to given query points. Indices to points stored
  //! in out_indices and distances to points in out_dists.
  //! Returns number of neighbor points retrieved.
  virtual int queryKNN(size_t numQueries, const num_t* queryDataPtr, size_t k,
                       vec<index_t>* out_indices,
                       vec<num_t>* out_dists) const = 0;

  //! Search for neighbors within radius of distance r.
  //! Memory is preallocated so that up to max_neighbors points will be returned
  //! with indices in out_indices and distances in out_dists.
  //! If less than max_neighbors, then first invalid out_indices is set to -1
  //! Returns number of neighbor points retrieved.
  virtual int queryRadius(const vec_t& query, num_t r,
                          vec<index_t>* out_indices,
                          vec<num_t>* out_dists, int max_neighbors) const = 0;

  //! Search for neighbors within radius of distance r.
  //! Memory is preallocated so that up to max_neighbors points will be returned
  //! with indices in out_indices and distances in out_dists.
  //! If less than max_neighbors, then first invalid out_indices is set to -1
  //! Returns number of neighbor points retrieved.
  virtual int queryRadius(size_t numQueries, const num_t* queryDataPtr, num_t r,
                          vec<index_t>* out_indices,
                          vec<num_t>* out_dists, int max_neighbors) const = 0;

  //! Search for neighbors within radius of distance r.
  //! Radius search with automatic memory allocation, as pairs of (index,dist)
  //! in out_pairs.
  //! Returns number of neighbor points retrieved.
  virtual int queryRadius(const vec_t& query, num_t r,
                          vec<pair<index_t, num_t>>* out_pairs) const = 0;

  virtual ~KdTree() { }
};

// Common setting convenience typedefs
//! Kd-tree on vecf elements
typedef KdTree<vecf, float, size_t> KdTree3f;

}  // namespace ann
}  // namespace sg


