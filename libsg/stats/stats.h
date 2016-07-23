#pragma once

#include "libsg.h"  // NOLINT

#include "math/math.h"
#include "util/grid.h"
#include "util/index.h"

namespace sg {
namespace stats {

//! Represents a continuous interval [min,max)
template <typename T>
struct Interval {
  Interval(T _min, T _max) : min(_min), max(_max) { }
  Interval() : min(0), max(0) { }

  T min, max;

 private:
  friend class boost::serialization::access;
  template<class Archive> void serialize(Archive & ar, const unsigned int version) {  // NOLINT
    ar & min & max;
  }
};

template <typename T>
ostream& operator<<(ostream& os, const Interval<T>& i) {  // NOLINT
  os << "[" << i.min << "," << i.max << "]";
  return os;
}

//! Represents a continuous 2D interval [minX,maxX) X [minY,maxY)
template <typename T>
struct Interval2D {
  Interval2D(T _minX, T _maxX, T _minY, T _maxY) : minX(_minX), maxX(_maxX), minY(_minY), maxY(_maxY) { }
  Interval2D() : minX(0), maxX(0), minY(0), maxY(0) { }

  T minX, maxX, minY, maxY;

 private:
  friend class boost::serialization::access;
  template<class Archive> void serialize(Archive & ar, const unsigned int version) {  // NOLINT
    ar & minX & maxX & minY & maxY;
  }
};

template <typename T>
ostream& operator<<(ostream& os, const Interval2D<T>& i) {  // NOLINT
  os << "[[" << i.minX << "," << i.maxX << "]x[" << i.minY << "," << i.maxY << "]]";
  return os;
}

template <typename countT>
void computeMarginals(const vec<countT>& rawCounts, const countT totalRawCount, 
                      vec<double>* pMargProbs, vec<double>* pMargLogProbs) {
  pMargProbs->resize(rawCounts.size());
  pMargLogProbs->resize(rawCounts.size());
  const double logTotal = log(totalRawCount);
  const double total = static_cast<double>(totalRawCount);
  for (int i = 0; i < rawCounts.size(); i++) {
    (*pMargLogProbs)[i] = log(rawCounts[i]) - logTotal;
    (*pMargProbs)[i] = rawCounts[i] / total;
  }
}

template <typename countT>
void computeNPMI(const sg::util::NumericGrid<countT>& rawCooccurrences, 
                 const countT totalRawCount, 
                 const vec<double>& margLogProbs1,
                 const vec<double>& margLogProbs2,
                 sg::util::NumericGrid<double>* pNPMI) {
  int nRows = static_cast<int>(margLogProbs1.size());
  int nCols = static_cast<int>(margLogProbs2.size());
  pNPMI->init(nRows, nCols);
  pNPMI->clear(0.0);
  const double logTotal = log(totalRawCount);
  for (int i = 0; i < nRows; i++) {
    for (int j = 0; j < nCols; j++) {
      if (rawCooccurrences(i, j) > 0) {
        double logpxy = log(rawCooccurrences(i, j)) - logTotal;
        double logpx = margLogProbs1[i];
        double logpy = margLogProbs2[j];
        double pmi = logpxy - logpx - logpy;
        (*pNPMI)(i, j) = -pmi / logpxy;
      }
    }
  }
}

template <typename countT>
void computeCondProb(const sg::util::NumericGrid<countT>& rawCooccurrences,
                     bool conditionOnCol,
                     const vec<countT>& rawCounts1,
                     const vec<countT>& rawCounts2,
                     sg::util::NumericGrid<double>* pCondProb) {
  int nRows = static_cast<int>(rawCounts1.size());
  int nCols = static_cast<int>(rawCounts2.size());
  pCondProb->init(nRows, nCols);
  pCondProb->clear(0.0);
  for (int i = 0; i < nRows; i++) {
    const double rowTotal = static_cast<double>(rawCounts1[i]);
    for (int j = 0; j < nCols; j++) {
      if (rawCooccurrences(i, j) > 0) {
        const double total = (conditionOnCol)? static_cast<double>(rawCounts2[j]) : rowTotal;
        (*pCondProb)(i, j) = rawCooccurrences(i, j) / total;
      }
    }
  }
}

template <typename T, typename countT = size_t>
struct GridStats2D {
  void clear() {
    countT zero = static_cast<countT>(0);
    std::fill(rawCounts.begin(), rawCounts.end(), zero);
    totalRawCounts = zero;
    rawCooccurrenceCounts.clear(zero);
    npmi.clear(0.0);
    condProb.clear(0.0);
    std::fill(marginalProb.begin(), marginalProb.end(), 0.0);
    std::fill(marginalLogProb.begin(), marginalLogProb.end(), math::constants::NEGINF);
  }
  void init(const sg::util::Index<T>* pIndex) {
    m_pIndex = pIndex;
    init(pIndex->size());
  }
  void init(size_t n) {
    m_n = n;
    rawCounts.resize(n);
    marginalProb.resize(n);
    marginalLogProb.resize(n);
    int nRows = static_cast<int>(n);
    int nCols = static_cast<int>(n);
    rawCooccurrenceCounts.init(nRows, nCols);
    npmi.init(nRows, nCols);
    condProb.init(nRows, nCols);
    clear();
  }
  void laplaceSmoothing(countT c) {
    for (int i = 0; i < m_n; i++) {
      rawCounts[i] += c;
      totalRawCounts += c;
      for (int j = i + 1; j < m_n; j++) {
        rawCooccurrenceCounts(i, j) += c;
      }
    }
  }
  void smoothAndComputeStats(countT c) {
    // Do add one smoothing...
    if (c != 0) {
      laplaceSmoothing(c);
    }
    computeStats();
  }
  void computeStats() {
    computeMarginals(rawCounts, totalRawCounts, &marginalProb, &marginalLogProb);
    computeNPMI(rawCooccurrenceCounts, totalRawCounts, marginalLogProb, marginalLogProb, &npmi);
    computeCondProb(rawCooccurrenceCounts, false, rawCounts, rawCounts, &condProb);
  }
  double getCondProb(const T& t1, const T& t2) const {
    int i1 = m_pIndex->indexOf(t1);
    int i2 = m_pIndex->indexOf(t2);
    if (i1 >= 0 && i2 >= 0) {
      return condProb(i1,i2);
    } else {
      return 0.0;
    }
  }

  const sg::util::Index<T>& index() const { return *m_pIndex; }
  size_t size() const { return m_n; }

  // Raw counts
  countT totalRawCounts;
  vec<countT> rawCounts;
  sg::util::NumericGrid<countT> rawCooccurrenceCounts;

  // Probabilities and stats
  vec<double> marginalProb;
  vec<double> marginalLogProb;
  sg::util::NumericGrid<double> npmi;
  sg::util::NumericGrid<double> condProb;

 private:
  const sg::util::Index<T>* m_pIndex;
  size_t m_n;

  // boost serialization support
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {  // NOLINT
    boost::serialization::split_member(ar, *this, version);
  }
  template <typename Archive>
  void save(Archive& ar, const unsigned int version) const {  // NOLINT
    ar << totalRawCounts;
    ar << rawCounts;
    ar << rawCooccurrenceCounts;
  }
  template <typename Archive>
  void load(Archive& ar, const unsigned int version) {  // NOLINT
    ar >> totalRawCounts;
    ar >> rawCounts;
    ar >> rawCooccurrenceCounts;
    // Recompute stats
    m_n = rawCounts.size();
    computeStats();
  }
};

template <typename T1, typename T2, typename countT = size_t>
struct GridStats2DT {
  void clear() {
    countT zero = static_cast<countT>(0);
    std::fill(rawCounts1.begin(), rawCounts1.end(), zero);
    std::fill(rawCounts2.begin(), rawCounts2.end(), zero);
    totalRawCounts = zero;
    rawCooccurrenceCounts.clear(zero);
    npmi.clear(0.0);
    condProb12.clear(0.0);
    condProb21.clear(0.0);
    std::fill(marginalProb1.begin(), marginalProb1.end(), 0.0);
    std::fill(marginalLogProb1.begin(), marginalLogProb1.end(), math::constants::NEGINF);
    std::fill(marginalProb2.begin(), marginalProb2.end(), 0.0);
    std::fill(marginalLogProb2.begin(), marginalLogProb2.end(), math::constants::NEGINF);
  }
  void init(const sg::util::Index<T1>* pIndex1, const sg::util::Index<T2>* pIndex2) {
    m_pIndex1 = pIndex1;
    m_pIndex2 = pIndex2;
    init(m_pIndex1->size(), m_pIndex2->size());
  }
  void init(size_t n1, size_t n2) {
    m_n1 = n1;
    m_n2 = n2;
    rawCounts1.resize(n1);
    rawCounts2.resize(n2);
    marginalProb1.resize(n1);
    marginalLogProb1.resize(n1);
    marginalProb2.resize(n2);
    marginalLogProb2.resize(n2);
    int nRows = static_cast<int>(n1);
    int nCols = static_cast<int>(n2);
    rawCooccurrenceCounts.init(nRows, nCols);
    npmi.init(nRows, nCols);
    condProb12.init(nRows, nCols);
    condProb21.init(nRows, nCols);
    clear();
  }
  void inc(int i, int j, countT c) {
    rawCooccurrenceCounts(i, j) += c;
    rawCounts1[i] += c;
    rawCounts2[j] += c;
    totalRawCounts += c;
  }
  void laplaceSmoothing(countT c) {
    for (int i = 0; i < m_n1; i++) {
      for (int j = 0; j < m_n2; j++) {
        inc(i,j,c);
      }
    }
  }
  void smoothAndComputeStats(countT c) {
    // Do add one smoothing...
    if (c != 0) {
      laplaceSmoothing(c);
    }
    computeStats();
  }
  void computeStats() {
    computeMarginals(rawCounts1, totalRawCounts, &marginalProb1, &marginalLogProb1);
    computeMarginals(rawCounts2, totalRawCounts, &marginalProb2, &marginalLogProb2);
    computeNPMI(rawCooccurrenceCounts, totalRawCounts, marginalLogProb1, marginalLogProb2, &npmi);
    computeCondProb(rawCooccurrenceCounts, false, rawCounts1, rawCounts2, &condProb12);
    computeCondProb(rawCooccurrenceCounts, true, rawCounts1, rawCounts2, &condProb21);
  }

  const sg::util::Index<T1>& index1() const { return *m_pIndex1; }
  const sg::util::Index<T2>& index2() const { return *m_pIndex2; }
  size_t size1() const { return m_n1; }
  size_t size2() const { return m_n2; }

  // Raw counts
  countT totalRawCounts;
  vec<countT> rawCounts1;
  vec<countT> rawCounts2;
  sg::util::NumericGrid<countT> rawCooccurrenceCounts;

  // Probabilities and stats (computed from rawCounts)
  vec<double> marginalProb1;
  vec<double> marginalProb2;
  vec<double> marginalLogProb1;
  vec<double> marginalLogProb2;
  sg::util::NumericGrid<double> npmi;
  sg::util::NumericGrid<double> condProb12;
  sg::util::NumericGrid<double> condProb21;
private:
  const sg::util::Index<T1>* m_pIndex1;
  const sg::util::Index<T2>* m_pIndex2;
  size_t m_n1;
  size_t m_n2;

  // boost serialization support
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {  // NOLINT
    boost::serialization::split_member(ar, *this, version);
  }
  template <typename Archive>
  void save(Archive& ar, const unsigned int version) const {  // NOLINT
    ar << totalRawCounts;
    ar << rawCounts1;
    ar << rawCounts2;
    ar << rawCooccurrenceCounts;
  }
  template <typename Archive>
  void load(Archive& ar, const unsigned int version) {  // NOLINT
    ar >> totalRawCounts;
    ar >> rawCounts1;
    ar >> rawCounts2;
    ar >> rawCooccurrenceCounts;
    // Recompute stats
    m_n1 = rawCounts1.size();
    m_n2 = rawCounts2.size();
    computeStats();
  }

};

}  // namespace stats
}  // namespace sg


