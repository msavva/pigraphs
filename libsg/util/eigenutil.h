#pragma once

#include <Eigen/Core>

#include "libsg.h"  // NOLINT

namespace sg {
namespace eigenutil {

using Eigen::MatrixXd; using Eigen::VectorXd; using Eigen::RowVectorXd;
using Eigen::MatrixXf; using Eigen::VectorXf; using Eigen::RowVectorXf;

//! Map used for wrapping row-major std::vector with Eigen matrices
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXdRowMajor;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXfRowMajor;

//! Copy Eigen matrix to std::vector in row-major order
inline vecd mat2vecd(const Eigen::MatrixXd& M) {
  vecd v(M.rows() * M.cols());
  Eigen::Map<MatrixXdRowMajor>(v.data(), M.rows(), M.cols()) = M;
  return v;
}

//! Copy Eigen matrix to std::vector in row-major order
inline vecf mat2vecf(const Eigen::MatrixXf& M) {
  vecf v(M.rows() * M.cols());
  Eigen::Map<MatrixXfRowMajor>(v.data(), M.rows(), M.cols()) = M;
  return v;
}

//! Wrap row-major order std::vector using Eigen matrix
inline MatrixXd vec2matd(const vecd& v, size_t nRows, size_t nCols) {
  return Eigen::Map<const MatrixXdRowMajor>(v.data(), nRows, nCols);
}

//! Wrap row-major order std::vector using Eigen matrix
inline MatrixXf vec2matf(const vecf& v, size_t nRows, size_t nCols) {
  return Eigen::Map<const MatrixXfRowMajor>(v.data(), nRows, nCols);
}

//! Wrap row-major order std::vector using Eigen matrix
inline RowVectorXd vec2rowvecd(const vecd& v, size_t nCols) {
  return Eigen::Map<const RowVectorXd>(v.data(), 1, nCols);
}

//! Wrap row-major order std::vector using Eigen matrix
inline RowVectorXf vec2rowvecf(const vecf& v, size_t nCols) {
  return Eigen::Map<const RowVectorXf>(v.data(), 1, nCols);
}

}  // namespace eigenutil
}  // namespace sg


