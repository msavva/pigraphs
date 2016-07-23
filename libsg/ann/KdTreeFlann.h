#pragma once

#pragma warning(push, 0)
//#define FLANN_USE_CUDA  // TODO(MS): Compile and link CUDA support
#include <flann/flann.hpp>
#pragma warning(pop)

#include "libsg.h"  // NOLINT
#include "ann/KdTree.h"

namespace sg {
namespace ann {

template <typename ElementIter_t, typename num_t = float, int DIM = 3, typename index_t = size_t>
class KdTreeFlann : public KdTree<typename ElementIter_t::value_type, num_t, index_t> {
 public:
  //! Type of elements
  typedef typename ElementIter_t::value_type vec_t;

  //! Constructor from nElements and index-based element start pointer function.
  //! NOTE: Assumes contiguous element memory storage
  KdTreeFlann(size_t nElements, const std::function<const num_t*(size_t)> getElemPtrFun)
    : index_(flann::KDTreeSingleIndexParams()) {
    data_.resize(nElements * DIM);
    const num_t* currElemStartPtr = nullptr;
    for (size_t i = 0, outBase = 0; i < nElements; i++, outBase += DIM) {
      currElemStartPtr = getElemPtrFun(i);
      std::copy(currElemStartPtr, currElemStartPtr + DIM, &data_[outBase]);
    }
    buildIndex(nElements);
  }

  //! Constructor from start and end iterators to data point range
  KdTreeFlann(ElementIter_t start, ElementIter_t end)
    : index_(flann::KDTreeSingleIndexParams()) {
    const size_t nElements = std::distance(start, end);
    const vec_t& first = *start;
    assert(nElements != 0 && first.size() != 0);
    const size_t dims = first.size();
    if (DIM > 0 && static_cast<int>(dims) != DIM) {
      throw std::runtime_error("Data set dimensionality differs from 'DIM'");
    }

    // Makes a copy of the data
    data_.resize(nElements * DIM);
    size_t base = 0;
    for (ElementIter_t it = start; it != end; it++, base += DIM) {
      for (size_t i = 0; i < DIM; i++) {
        data_[base + i] = (*it)[i];
      }
    }
    buildIndex(nElements);
  }

  int queryKNN(const vec_t& queries, size_t k,
               vec<index_t>* out_indices,
               vec<num_t>* out_dists) const {
    out_indices->resize(k);
    out_dists->resize(k);

    flann::Matrix<num_t> fq(const_cast<num_t*>(queries.data()), 1, DIM);
    flann::Matrix<index_t> indices(out_indices->data(), 1, k);
    flann::Matrix<num_t> dists(out_dists->data(), 1, k);
    const int kActual = index_.knnSearch(fq, indices, dists, k, params_);

    return kActual;
  }

  int queryKNN(size_t numQueries, const num_t* queriesDataPtr, size_t k,
               vec<index_t>* out_indices,
               vec<num_t>* out_dists) const {
    out_indices->resize(numQueries * k);
    out_dists->resize(numQueries * k);

    flann::Matrix<num_t> fq(const_cast<num_t*>(queriesDataPtr), numQueries, DIM);
    flann::Matrix<index_t> indices(out_indices->data(), numQueries, k);
    flann::Matrix<num_t> dists(out_dists->data(), numQueries, k);
    const int kActual = index_.knnSearch(fq, indices, dists, k, params_);

    return kActual;
  }

  int queryRadius(const vec_t& queries, num_t r,
                  vec<index_t>* out_indices,
                  vec<num_t>* out_dists, int max_neighbors) const {
    // Some sanity checks for sizes and storage
    const size_t sizeIndices = out_indices->size(), sizeDists = out_dists->size();
    assert(sizeIndices == sizeDists);
    const size_t querySize = queries.size();
    const size_t numQueries = querySize / DIM;
    assert(numQueries > 0 && (querySize % DIM == 0));
    assert(static_cast<size_t>(max_neighbors * numQueries) <= sizeIndices);

    flann::Matrix<num_t> fq(const_cast<num_t*>(queries.data()), numQueries, DIM);
    flann::Matrix<index_t> indices(out_indices->data(), numQueries, max_neighbors);
    flann::Matrix<num_t> dists(out_dists->data(), numQueries, max_neighbors);
    flann::SearchParams params(params_);
    params.max_neighbors = max_neighbors;

    const int k =
      index_.radiusSearch(fq, indices, dists, static_cast<num_t>(r), params);

    return k;
  }

  int queryRadius(size_t numQueries, const num_t* queriesDataPtr, num_t r,
                  vec<index_t>* out_indices,
                  vec<num_t>* out_dists, int max_neighbors) const {
    // Some sanity checks for sizes and storage
    const size_t sizeIndices = out_indices->size(), sizeDists = out_dists->size();
    assert(sizeIndices == sizeDists);
    assert(static_cast<size_t>(max_neighbors * numQueries) <= sizeIndices);

    flann::Matrix<num_t> fq(const_cast<num_t*>(queriesDataPtr), numQueries, DIM);
    flann::Matrix<index_t> indices(out_indices->data(), numQueries, max_neighbors);
    flann::Matrix<num_t> dists(out_dists->data(), numQueries, max_neighbors);
    flann::SearchParams params(params_);
    params.max_neighbors = max_neighbors;

    const int k =
      index_.radiusSearch(fq, indices, dists, static_cast<num_t>(r), params);

    return k;
  }

  int queryRadius(const vec_t& query, num_t r,
                  vec<pair<index_t, num_t>>* out_pairs) const {
    flann::Matrix<num_t> fq(const_cast<num_t*>(query.data()), 1, DIM);
    vec<vec<index_t>> indices(1);
    vec<vec<num_t>> dists(1);

    const int k =
      index_.radiusSearch(fq, indices, dists, static_cast<float>(r), params_);

    if (k > 0) {
      out_pairs->resize(k);
      for (size_t i = 0; i < k; i++) {
        (*out_pairs)[i] = std::make_pair(indices[0][i], dists[0][i]);
      }
    }
    return k;
  }

 private:
  vec<num_t> data_;                           // Raw data storage
  flann::Index<flann::L2_Simple<num_t>> index_;       // FLANN index
  flann::SearchParams params_;                        // Default params

  // Disallow copy and assignment
  KdTreeFlann(const KdTreeFlann&) = delete;
  KdTreeFlann& operator=(const KdTreeFlann&) = delete;

  void buildIndex(size_t nElements) {
    const flann::Matrix<num_t> dataset(data_.data(), nElements, DIM);
    index_.buildIndex(dataset);

    params_.checks = -1;        // unlimited (exact search)    
//    params_.cores = omp_get_num_procs();   // get number of processor available to this process
    params_.cores = omp_get_max_threads();   // get max number of threads
    params_.max_neighbors = -1; // unlimited
  }
};

// Common setting convenience typedefs
//! FLANN-based Kd-tree on vec<vecf> container
typedef KdTreeFlann<vec<vecf>::iterator, float, 3, size_t> KdTree3f_FLANN;
typedef KdTreeFlann<vec<vecf>::iterator, float, 2, size_t> KdTree2f_FLANN;

}  // namespace ann
}  // namespace sg


