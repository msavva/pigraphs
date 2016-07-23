#include "common.h"  // NOLINT

#include "stats/centroidset.h"

#include <Eigen/Eigenvalues>

#include "util/eigenutil.h"
#include "../jace-proxy/wekautil.h"

namespace sg {
namespace stats {

using namespace sg::eigenutil;  // NOLINT

//TODO(MS): Refactor this massive function into smaller reusable pieces
CentroidSet unsupervisedFeatureLearning(const vecd& samples, size_t numSamples,
                                        size_t numCentroids, double epsZCA) {

  const size_t numDims = samples.size() / numSamples;
  const MatrixXd X = vec2matd(samples, numSamples, numDims);

  /// STEP 1: Normalize

  // Get mean, variance and standard deviation
  const RowVectorXd Xmean = X.colwise().mean();
  //cout << "Xmean:" << endl << Xmean << endl;
  const MatrixXd Xcentered = X.rowwise() - Xmean;
  const MatrixXd Xvar = Xcentered.array().square().colwise().sum() / static_cast<double>(X.rows() - 1);
  //cout << "Xvar:" << endl << Xvar << endl;
  const MatrixXd Xstd = Xvar.cwiseSqrt();
  //cout << "Xstd:" << endl << Xstd << endl;

  // Normalize sample matrix
  const RowVectorXd XstdInv = Xstd.array().inverse();
  const MatrixXd XstdInvDiag = XstdInv.asDiagonal();
  MatrixXd Xnormed = Xcentered * XstdInvDiag;
  //cout << "Xnormed:" << endl << Xnormed << endl;

  /// STEP 2: Whiten

  // NOTE: samples are already centered so we don't need to subtract mean
  //const Eigen::RowVectorXd XnormedMean = Xnormed.colwise().mean();
  //cout << "XnormedMean:" << endl << XnormedMean << endl;
  //const MatrixXd XnormedCentered = Xnormed.rowwise() - XnormedMean;
  //cout << "XnormedCentered:" << endl << XnormedCentered << endl;

  // Form covariance matrix of normalized samples and compute eigen decomposition
  const MatrixXd Xcov = (Xnormed.adjoint() * Xnormed) / static_cast<double>(Xnormed.rows());
  //cout << "Xcov:" << endl << Xcov << endl;
  Eigen::SelfAdjointEigenSolver<MatrixXd> eig(Xcov);
  //cout << "EigenSolveInfo:" << eig.info() << endl;
  MatrixXd D = eig.eigenvalues().asDiagonal();
  MatrixXd V = eig.eigenvectors();
  cout << "D=" << D << endl;
  //cout << "V=" << V << endl;

  // Construct pseudo-inverse whitening matrix P = V * (D + I*epsZCA)^(-1/2) * V'
  VectorXd Ieps = VectorXd::Constant(D.cols(), epsZCA);
  //cout << "Ieps=" << endl << Ieps << endl;
  const VectorXd diag = eig.eigenvalues() + Ieps;
  const MatrixXd Dinv = diag.cwiseSqrt().cwiseInverse().asDiagonal();
  //cout << "Dinv=" << endl << Dinv << endl;
  MatrixXd P = V * Dinv * V.adjoint();
  //cout << "P=" << endl << P << endl;

  const MatrixXd Xwhitened = Xnormed * P;
  //cout << "Xwhitened DIMS=" << Xwhitened.rows() << "," << Xwhitened.cols() << endl;

  const MatrixXd Pinv = P.inverse();
  const MatrixXd Xnormed2 = Xwhitened * Pinv;
  const MatrixXd original = (Xnormed2 * XstdInvDiag.inverse()).rowwise() + Xmean;
  //cout << "Xwhitened=" << endl << Xwhitened << endl;
  //cout << "Pinv=" << endl << Pinv << endl;
  //cout << "I=" << endl << P * Pinv << endl;

  //for (int i = 0; i < 3; i++) {
  //  cout << i << ":" << Xnormed.row(i) << endl;
  //  cout << i << ":" << Xwhitened.row(i) << endl;
  //  cout << i << ":" << Xnormed2.row(i) << endl;
  //  cout << i << ":" << original.row(i) << endl;
  //  cout << i << ":" << X.row(i) << endl;
  //}

  // Compute centroids
  vecd segFeatsWhitened(numDims * numSamples);
  std::copy(Xwhitened.data(), Xwhitened.data() + numDims * numSamples, segFeatsWhitened.begin());
  vecd centroidFeats = wekautil::getKmeansCentroids(segFeatsWhitened, numDims, numSamples, numCentroids);
  //const Eigen::Map<MatrixXdRowMajor> C(centroidFeats.data(), numSamples, numDims);
  //for (size_t i = 0; i < numCentroids; i++) {
  //  cout << C.row(i) << endl;
  //}

  const CentroidSet s(numCentroids, numDims, centroidFeats, mat2vecd(P), mat2vecd(Xmean), mat2vecd(XstdInv));
  return s;
}

CentroidSet::Vec CentroidSet::whiten(const Vec& v, size_t numSamples) const {
  assert(v.size() == numSamples * d);
  const MatrixXd X = vec2matd(v, numSamples, d);
  const RowVectorXd Xmean = vec2rowvecd(M, d);
  const MatrixXd XstdInv = vec2rowvecd(Sinv, d).asDiagonal();
  const MatrixXd Xcentered = X.rowwise() - Xmean;
  const MatrixXd Xnormed = Xcentered * XstdInv;
  const MatrixXd Xwhitened = Xnormed * vec2matd(P, d, d);

  return mat2vecd(Xwhitened);
}

CentroidSet::Vec CentroidSet::invertWhitening(const Vec& v, size_t numSamples) const {
  assert(v.size() == numSamples * d);
  const MatrixXd Xwhitened = vec2matd(v, numSamples, d);
  //TODO(MS): Cache the inverses
  const MatrixXd Pinv = vec2matd(P, d, d).inverse();
  const MatrixXd Xstd = vec2rowvecd(Sinv, d).asDiagonal().inverse();
  const RowVectorXd Xmean = vec2rowvecd(M, d);
  const MatrixXd Xnormed = Xwhitened * Pinv;
  const MatrixXd original = (Xnormed * Xstd).rowwise() + Xmean;

  return mat2vecd(original);
}

CentroidSet::Vec CentroidSet::computeActivation(const Vec& v, size_t numSamples) const {
  assert(v.size() == numSamples * d);

  //TODO(MS): Cache repeated computations
  const MatrixXd CMat = vec2matd(C, k, d);                          // [k x d] matrix
  const Vec vWhitened = whiten(v, numSamples);
  const MatrixXd X = vec2matd(vWhitened, numSamples, d);            // [n x d] matrix

  // Compute 'triangle' activation function
  const RowVectorXd cc = CMat.array().pow(2).rowwise().sum();  // [1 x k] vector
  //cout << "cc=" << cc << endl;
  const VectorXd xx = X.array().pow(2).rowwise().sum();        // [n x 1] vector
  //cout << "xx=" << xx << endl;
  const MatrixXd xc = X * CMat.adjoint();                      // [n x k] response matrix
  //cout << "xc=" << xc.row(0) << endl;
  const MatrixXd z = (((-2 * xc).colwise() + xx).rowwise() + cc).cwiseSqrt();  // [n x k] dist each sample to centroids
  //cout << "z=" << z.row(0) << endl;
  const VectorXd mu = z.rowwise().mean();                      // [n x 1] mean distance for each sample
  //cout << "mu=" << mu << endl;
  const MatrixXd a = ((z * -1.0).colwise() + mu).cwiseMax(0);    // [n x k] thresholded activation
  //cout << "a=" << a.row(0) << endl;

  return mat2vecd(a);
}

// TODO(MS): Reduce code duplication with above
CentroidSet::Vec CentroidSet::computeActivationSummed(const Vec& v, size_t numSamples) const {
  assert(v.size() == numSamples * d);

  //TODO(MS): Cache repeated computations
  const MatrixXd CMat = vec2matd(C, k, d);                          // [k x d] matrix
  const Vec vWhitened = whiten(v, numSamples);
  const MatrixXd X = vec2matd(vWhitened, numSamples, d);            // [n x d] matrix

  // Compute 'triangle' activation function
  const RowVectorXd cc = CMat.array().pow(2).rowwise().sum();  // [1 x k] vector
  //cout << "cc=" << cc << endl;
  const VectorXd xx = X.array().pow(2).rowwise().sum();        // [n x 1] vector
  //cout << "xx=" << xx << endl;
  const MatrixXd xc = X * CMat.adjoint();                      // [n x k] response matrix
  //cout << "xc=" << xc.row(0) << endl;
  const MatrixXd z = (((-2 * xc).colwise() + xx).rowwise() + cc).cwiseSqrt();  // [n x k] dist each sample to centroids
  //cout << "z=" << z.row(0) << endl;
  const VectorXd mu = z.rowwise().mean();                      // [n x 1] mean distance to centroids for each sample
  //cout << "mu=" << mu << endl;
  const MatrixXd a = ((z * -1.0).colwise() + mu).cwiseMax(0);    // [n x k] thresholded activation
  //cout << "a=" << a.row(0) << endl;
  const RowVectorXd aSum = a.colwise().sum();                  // [1 x k] summed activations
  //cout << "aSum=" << aSum << endl;

  return mat2vecd(aSum);
}

sg::stats::CentroidSet::Vec CentroidSet::getCentroid(size_t i) const {
  MatrixXd Cmat = vec2matd(C, k, d);
  assert(i >= 0 && i < static_cast<size_t>(Cmat.rows()));
  const RowVectorXd Ci = Cmat.row(i);
  Vec result;
  for (int iDim = 0; iDim < Ci.size(); iDim++) {
    result.push_back(Ci[iDim]);
  }
  return result;
}

}  // namespace stats
}  // namespace sg
