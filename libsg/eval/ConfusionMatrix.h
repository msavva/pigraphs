#pragma once

#include "libsg.h"  // NOLINT

namespace sg {
namespace eval {

//! Row-major confusion matrix (for 0=Negative,1=Positive class, order is TN,FP,FN,TP)
struct BinaryConfusionMatrix {
  BinaryConfusionMatrix() : _m(), _cutoff(-1.0) { _m.fill(0.0); }
  explicit BinaryConfusionMatrix(double cutoff) : _cutoff(cutoff) { _m.fill(0.0); }
  explicit BinaryConfusionMatrix(const arr<double, 4>& m) : _m(m), _cutoff(-1.0) { }
  double& operator[](size_t i) { return _m[i]; }
  double cutoff() const { return _cutoff; }
  double tp() const { return _m[3]; }
  double tn() const { return _m[0]; }
  double fp() const { return _m[1]; }
  double fn() const { return _m[2]; }
  double goldPos() const { return tp() + fn(); }
  double goldNeg() const { return tn() + fp(); }
  double labeledPos() const { return tp() + fp(); }
  double labeledNeg() const { return tn() + fn(); }
  double total() const { return tp() + tn() + fp() + fn(); }
  double accuracy() const {
    const double t = total();
    return (t == 0) ? 0 : (tp() + tn()) / t;
  }
  double precision() const {
    const double t = labeledPos();
    return (t == 0) ? 0 : tp() / t;
  }
  double recall() const {
    const double t = goldPos();
    return (t == 0) ? 0 : tp() / t;
  }
  double f1() const {
    const double t = (precision() + recall());
    return (t == 0) ? 0 : 2 * precision() * recall() / t;
  }
  BinaryConfusionMatrix& operator+=(const BinaryConfusionMatrix& rhs) {
    for (int i = 0; i < 4; i++) { _m[i] += rhs._m[i]; }
    return *this;
  }
  void operator*=(double s) {
    _m[0] *= s;
    _m[1] *= s;
    _m[2] *= s;
    _m[3] *= s;
  }
  void add(bool gold, bool label) {
      if( gold &&  label) _m[3]++; //tp
      if(!gold && !label) _m[0]++; //tn
      if(!gold &&  label) _m[1]++; //fp
      if( gold && !label) _m[2]++; //fn
  }
  void addTN(int n = 1) { _m[0] += n; }
  void addFP(int n = 1) { _m[1] += n; }
  void addFN(int n = 1) { _m[2] += n; }
  void addTP(int n = 1) { _m[3] += n; }
  static string csvHeader() { return "TP,TN,FP,FN,P,R,ACC,F1,cutoff"; }
  friend ostream& operator<<(ostream& os, const BinaryConfusionMatrix& M);

 private:
  arr<double, 4> _m;
  double _cutoff;
};

inline ostream& operator<<(ostream& os, const BinaryConfusionMatrix& M) {
  // "ACC,P,R,F1"
  os << M.accuracy() << "," << M.precision() << "," << M.recall() << ","  << M.f1();
  return os;
}

inline BinaryConfusionMatrix operator+(BinaryConfusionMatrix lhs, const BinaryConfusionMatrix& rhs) {
  return lhs += rhs;
}

typedef map<string, BinaryConfusionMatrix> BinaryConfusionMatrixSet;

inline BinaryConfusionMatrix microAverage(const BinaryConfusionMatrixSet& set) {
  BinaryConfusionMatrix all;
  for (const auto& pair : set) { all += pair.second; }
  // note: normalization not needed as we are taking all the raw counts together
  return all;
}

}  // namespace eval
}  // namespace sg
