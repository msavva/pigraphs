#pragma once

#include "libsg.h"  // NOLINT

namespace sg {
namespace stats {

//! Encapsulation for centroid set retrieved from unsupervised feature learning and relevant convenience functions
struct CentroidSet {
  typedef vecd Vec;

  CentroidSet() : k(0), d(0), C(), P(), M(), Sinv() { }
  CentroidSet(size_t _k, size_t _d, const Vec& _C, const Vec& _P, const Vec& _M, const Vec& _Sinv)
    : k(_k), d(_d), C(_C), P(_P), M(_M), Sinv(_Sinv) { }

  //! Given a row-major vector of samples v with numSamples, normalize and whiten them and return transformed vector
  Vec whiten(const Vec& v, size_t numSamples) const;

  //! Given a row-major vector of samples v with numSamples that were normalized and whitened by this CentroidSet,
  //! invert whitening and normalization and return untransformed vector
  Vec invertWhitening(const Vec& v, size_t numSamples) const;

  //! Return [1 x d] row vector representation of i-th centroid in this CentroidSet
  Vec getCentroid(size_t i) const;

  //! Given a row-major vector of samples v with numSamples, compute and return activations against CentroidSet
  //! as vector of length numSamples (i-th entry corresponds to i-th sample in v)
  //! NOTE: Input is assumed to be untransformed and thus normalization and whitening is applied
  Vec computeActivation(const Vec& v, size_t numSamples) const;

  //! Same as above plus summation at the end
  Vec computeActivationSummed(const Vec& v, size_t numSamples) const;

  //! boost serialization function
  template<class Archive>
  inline void serialize(Archive& ar, const unsigned int version) {  // NOLINT
    ar & k & d & C & P & M & Sinv;
  }

  size_t k;  //! Number of centroids
  size_t d;  //! Number of feature dimensions (sample columns)
  Vec C;     //! Flattened row-major values of whitened centroids matrix: [k x d]; k= numCentroids, d= feature dimension
  Vec P;     //! Flattened row-major [d x d] whitening transform matrix: for a [n x d] sample matrix X, X_w = X * P
  Vec M;     //! Sample mean vector used in (de-)normalization
  Vec Sinv;  //! Sample standard deviation used in (de-)normalization
};

//! Retrieve centroids to be used for classifier learning using Coates and Ng's unsupervised feature learning method
//! samples is row-major storage of matrix of dimensions [n x d] where n=numSamples and d is the feature dimension
//! epsZCA is regularization constant to avoid near-zero eigenvalues (0.01-0.1 range is good according to Coates & Ng)
CentroidSet unsupervisedFeatureLearning(const vecd& samples, size_t numSamples, size_t numCentroids, double epsZCA);

}  // namespace stats
}  // namespace sg


