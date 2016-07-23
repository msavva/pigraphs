#pragma once

#include <nanoflann.hpp>

#include "libsg.h"  // NOLINT

namespace sg {
namespace ann {

/** Vector-of-vectors adaptor for nanoflann, without duplicating the storage.
*  The i'th vector represents a point in the state space.
*
*  \tparam DIM If set to >0, it specifies a compile-time fixed dimensionality
*          for the points in the data set, allowing more compiler optimizations.
*  \tparam num_t The type of the point coordinates (typically, double or float).
*  \tparam Distance The distance metric to use: nanoflann::metric_L1,
*          nanoflann::metric_L2, nanoflann::metric_L2_Simple, etc.
*  \tparam IndexType The type for indices in the KD-tree index (typically,
*          size_t or int)
*/
// TODO(MS): Make template same as FLANN
template < typename VecOfVecT, typename num_t = double, int DIM = -1,
           typename Dist = nanoflann::metric_L2, typename IndexType = size_t >
class ANN {
 public:
  typedef typename VecOfVecT::value_type VecT;
  typedef ANN<VecOfVecT, num_t, DIM, Dist> self_t;
  typedef typename Dist::template traits<num_t, self_t>::distance_t metric_t;
  typedef nanoflann::KDTreeSingleIndexAdaptor<metric_t, self_t, DIM, IndexType>
  index_t;

  /// Constructor: takes const ref to vector of vectors object with data points
  ANN(const VecOfVecT& _m, int leaf_max_size = 10)
    : m_(_m) {
    assert(_m.size() != 0 && _m[0].size() != 0);
    const int dims = static_cast<int>(_m[0].size());
    if (DIM > 0 && dims != DIM) {
      throw std::runtime_error("Data set dimensionality differs from 'DIM'");
    }
    index_ = new index_t(dims, *this,
                         nanoflann::KDTreeSingleIndexAdaptorParams(leaf_max_size, dims));
    index_->buildIndex();
  }

  ~ANN() { delete index_; }

  // Query for k closest points to given query_point.  Return indices to points
  // in in out_indices and squared distances to points in out_dists.  Indices
  // are with respect to data container on which constructor was called.
  inline void queryKNN(const VecT& query, size_t k,
                       vec<IndexType>* out_indices,
                       vec<num_t>* out_dists) const {
    out_indices->clear();
    out_indices->resize(k);
    out_dists->clear();
    out_dists->resize(k);
    nanoflann::KNNResultSet<num_t, IndexType> resultSet(k);
    resultSet.init(&(out_indices->front()), &(out_dists->front()));
    index_->findNeighbors(resultSet, &query[0], params_);
  }

  // Query for points within radius r of given query_point.
  // Returns number of neighbors found.
  inline size_t queryRadius(const VecT& query, num_t r,
                            vec<pair<IndexType, num_t>>* out_pairs) const {
    out_pairs->clear();
    return index_->radiusSearch(&query[0], r, *out_pairs, params_);
  }

  // Interface expected by KDTreeSingleIndexAdaptor
  const self_t& derived() const { return *this; }
  self_t& derived() { return *this; }

  // Must return the number of data points
  inline size_t kdtree_get_point_count() const { return m_.size(); }

  // Distance between vector "p1[0:size-1]" and point with index "idx_p2"
  inline num_t kdtree_distance(const num_t* p1, size_t idx_p2,
                               size_t size) const {
    num_t s = 0;
    for (size_t i = 0; i < size; i++) {
      const num_t d = p1[i] - m_[idx_p2][i];
      s += d * d;
    }
    return s;
  }

  // Returns the dim'th component of the idx'th point in the class:
  inline num_t kdtree_get_pt(size_t idx, size_t dim) const {
    return m_[idx][dim];
  }

  // Optional Bbox: return false to default to standard bbox computation
  //   Return true if BBOX already computed by class and in "bb".
  //   Look at bb.size() to find out expected dimensionality (e.g. 2 or 3)
  template <class BBox>
  bool kdtree_get_bbox(const BBox& bb) const { return false; }

 private:
  const VecOfVecT& m_;              // Reference to data
  index_t* index_;                  // Kd-tree index
  nanoflann::SearchParams params_;  // Default params

  // Disallow copy and assignment
  ANN(const ANN&);
  void operator=(const ANN&);
};  // ANN

}  // namespace ann
}  // namespace sg


