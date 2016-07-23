#pragma once

#include "libsg.h"  // NOLINT

namespace sg {
namespace eval {

// discounted cumulative gain
template <typename VecT>
double DCG(const VecT& x, int k = -1) {
  if (k < 0) { k = static_cast<int>(x.size()); }
  double sum = 0.0;
  for (int i = 0; i < k; ++i) {
    double w = 1.0 / log2(i + 1);
    sum += w * x[i];
  }
  return sum;
}

// ideal discounted cumulative gain
template <typename VecT>
double IDCG(const VecT& x, int k = -1) {
  VecT y(x);
  typedef typename VecT::value_type T;
  std::sort(y.begin(), y.end(), [] (T a, T b) { return a > b; });
  return DCG(y, k);
}

// normalized discounted cumulative gain
template <typename VecT>
double NDCG(const VecT& x, int k = -1) {
  return DCG(x, k) / IDCG(x, k);
}

// precision at k=i
template <typename VecT>
vec<double> PrecisionVec(const VecT& x) {
  double sum = 0.0;
  vec<double> P(x.size());
  for (int i = 0; i < x.size(); ++i) {
    if (x[i] > 0) { sum++; }
    P[i] = sum / (i + 1);
  }
  return P;
}

// precision at k
template <typename VecT>
vec<double> Precision(const VecT& x, int k = -1) {
  if (k < 0) { k = static_cast<int>(x.size() - 1); }
  return PrecisionVec(x)[k];
}

// recall at k=i
template <typename VecT>
vec<double> RecallVec(const VecT& x, double cutoff) {
  double sum = 0.0;
  vec<double> R(x.size());
  for (int i = 0; i < x.size(); ++i) {
    if (x[i] > 0) { sum++; }
    R[i] = sum / cutoff;
  }
  return R;
}

// recall at k
template <typename VecT>
vec<double> Recall(const VecT& x, double cutoff, int k = -1) {
  if (k < 0) { k = static_cast<int>(x.size() - 1); }
  return RecallVec(x)[k];
}

// F1 score at k=i
template <typename VecT>
vec<double> F1Vec(const vec<double>& Pv, const vec<double>& Rv) {
  vec<double> F1(Pv.size());
  for (int i = 0; i < Pv.size(); ++i) {
    F1[i] = 2.0 * Rv[i] * Pv[i] / (Rv[i] + Pv[i]);
  }
  return F1;
}

// F1 score at k=i
template <typename VecT>
double F1(const VecT& x, double cutoff, int k = -1) {
  if (k < 0) { k = static_cast<int>(x.size() - 1); }
  return F1Vec(PrecisionVec(x), RecallVec(x, cutoff))[k];
}

// average precision
template <typename VecT>
double AveragePrecision(const VecT& x) {
  double sum = 0.0;
  vec<double> precisions;
  for (int i = 0; i < x.size(); ++i) {
    if (x[i] > 0) {
      sum++;
      precisions.push_back(sum / (i + 1));
    }
  }
  double sumPrecisions = 0.0;
  if (precisions.size() == 0) { return 0.0; }
  for (int i = 0; i < precisions.size(); ++i) {
    sumPrecisions += precisions[i];
  }
  return sumPrecisions / precisions.size();
}


// vector add
inline vec<double> operator+(const vec<double>& x, const vec<double>& y) {
  vec<double> z(x);
  for (int i = 0; i < y.size(); ++i) {
    z[i] += y[i];
  }
  return z;
}

// vector mul
inline vec<double> operator*(double m, const vec<double>& x) {
  vec<double> y(x);
  for (int i = 0; i < y.size(); ++i) {
    y[i] *= m;
  }
  return y;
}

// Convenience struct for passing around averaged eval metrics
struct AverageMetrics {
  vec<double> P;
  vec<double> R;
  vec<double> F1;
  double PatN;
  double RatN;
  double F1atN;
  double mAP;
  double ndcg;
  size_t num;
};

struct MetricsSet;
// Collection of summed metrics over multiple retrieval observations
struct SummedMetrics {
  vec<double> Psum;
  vec<double> Rsum;
  vec<double> F1sum;
  double apSum;
  double ndcgSum;
  size_t numSum;
  void operator+=(const SummedMetrics& o);
  void operator+=(const MetricsSet& o);
  AverageMetrics getAverages() const;
};

// Set of useful retrieval metrics
struct MetricsSet {
  vec<double> P;   // Precision at k
  vec<double> R;   // Recall at k
  vec<double> F1;  // F1 score at k
  double ap;       // Average Precision
  double ndcg;     // Normalized discounted cumulative gain
  size_t num;      // Number of observations included

  // Computes metrics from raw observations
  MetricsSet(const vec<int>& x, double cutoff)
    : P(PrecisionVec(x))
    , R(RecallVec(x, cutoff))
    , F1(F1Vec<const vec<int>>(P, R))
    , ap(AveragePrecision(x))
    , ndcg(NDCG(x))
    , num(1) { }

  // Computes average metrics from SummedMetrics
  explicit MetricsSet(const SummedMetrics& s) {
    double norm = 1.0 / s.numSum;
    P = norm * s.Psum;
    R = norm * s.Rsum;
    F1 = norm * s.F1sum;
    ap = norm * s.apSum;
    ndcg = norm * s.ndcgSum;
    num = s.numSum;
  }
};

}  // namespace eval
}  // namespace sg
