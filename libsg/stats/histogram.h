#pragma once

#include "libsg.h"  // NOLINT
#include "stats/Counter.h"
#include "util/util.h"
#include "util/log.h"

namespace sg {
namespace stats {

//! Histogram over a given closed interval [min,max] with equal sized numBins
template <typename T, typename countT = size_t>
class Histogram {
 public:
  //! Histogram with prespecified range and numBins manually
  Histogram(const Interval<T>& _range, int _numBins, string _id, bool _isAngle = false) {
    init(_range, _numBins, _id, _isAngle);
  }
  //! Default [0, 1] / 10 bins histogram NOTE: These default values are tested in the test suite!
  Histogram() : Histogram(Interval<T>(0, 1), 10, "") { }

  //! Add value x to the histogram.
  //! Values outside the histogram range are ignored and warning is issued
  void add(T x, countT count = 1) {
    _add(x, count);
    if (m_isAngle) { 
      T s = (T) sin(x);
      T c = (T) cos(x);
      m_moments[0]+=count*c; 
      m_moments[1]+=count*s; 
      m_moments[2]+=count*c*c; 
      m_moments[3]+=count*s*s; 
    } else {
      m_moments[0]+=count*x; 
      m_moments[1]+=count*x*x;
    }
  }

  //! Add value from another histogram to this histogram
  //! Values outside the histogram range are ignored and warning is issued
  void addAll(Histogram<T,countT> h, countT count = 1) {
    for (int i = 0; i < h.m_numBins; i++) {
      // Sample from bin
      for (int j = 0; j < h.count(i); j++) {
        auto bInt = h.binInterval(i);
        float selected = math::DEF_RAND.uniform_float(bInt.min, bInt.max);
        _add(selected, count);
      }
    }
    //! Add up moments
    assert(m_moments.size() == h.m_moments.size());
    for (size_t i = 0; i < m_moments.size(); ++i) {
      m_moments[i] += count*h.m_moments[i];
    }
  }

  //! Clears the current histogram, dropping all counts
  void clear() {
    std::fill(m_acc.begin(), m_acc.end(), 0);
    m_numSamples = 0;
    for (size_t i = 0; i < m_moments.size(); ++i) {
      m_moments[i] = 0;
    }
  }

  T mean() const {
    if (m_isAngle) {
      T mc = m_moments[0]/m_numSamples;
      T ms = m_moments[1]/m_numSamples;
      T ang = (T) atan2(ms, mc);
      if (ang < 0) {
        ang += (T) math::constants::PI2;
      }
      return ang;
     } else {
      return m_moments[0]/m_numSamples; 
    }
  }

  //! Returns the count of the bin with index iBin
  countT count(int iBin) const { return m_acc[iBin]; }

  //! Returns the total number of samples aggregated
  countT totalCount() const { return m_numSamples; }

  //! Returns the normalized weight of the bin 
  T binWeight(int iBin) const { 
    if (iBin >= 0 && iBin < m_numBins) {
      return static_cast<T>(m_acc[iBin])/m_numSamples; 
    } else {
      return 0;
    }
  }

  //! Returns the binWeight of bin that this value belongs to
  //! This weight will be dependent on the bin size
  T weight(T x) const {
    int idx = binIndex(x);
    return binWeight(idx);
  }

  //! Returns the estimated density of this value
  //! This should be fairly independent of the bin size
  T density(T x) const {
    // Divide by binsize so we have a good pdf that sums up to 1 when integrated
    return weight(x)/getBinSize();
  }

  //! Returns the estimated density of this value - assumes width is much smaller than the binSize
  T density(T x, T width) const {
    return density(x)*width;
  }

  //! Returns interval of bin with index iBin
  Interval<T> binInterval(int iBin) const {
    const T start = iBin * m_binSize + m_range.min;
    return Interval<T>(start, start + m_binSize);
  }

  //! Returns vector of un-normalized count for each histogram bin
  const vec<countT>& counts() const { return m_acc; }

  //! Returns the range of this histogram
  Interval<T> getRange() const { return m_range; }

  //! Returns the number of bins of this histogram
  int getNumBins() const { return m_numBins; }

  //! Returns the bin size of this histogram
  T getBinSize() const { return m_binSize; }

  T sample() const {
    // Select a bin
    countT r = static_cast<countT>(math::DEF_RAND.uniform_float_01() * m_numSamples);
    countT t = 0;
    int iBin = 0;
    for (iBin = 0; iBin < m_numBins-1; ++iBin) {
      t += m_acc[iBin];
      if (t > r) {
        break;
      }
    }
    // Return a value from the bin
    Interval<T> bInt = binInterval(iBin);
    float selected = math::DEF_RAND.uniform_float(bInt.min, bInt.max);
    return selected;
  }

  //! Copies un-normalized counts for each histogram bin to passed vector
  template <typename VecT>
  void counts(VecT* out) const {
    out->resize(m_acc.size());
    std::copy(m_acc.begin(), m_acc.end(), out->begin());
  }

  //! Copies normalized counts for each histogram bin to passed vector
  template <typename VecT>
  void normalizedCounts(VecT* out) const {
    out->resize(m_acc.size());
    const VecT::value_type invNumSamples = static_cast<VecT::value_type>(1) / m_numSamples;
    std::transform(m_acc.begin(), m_acc.end(), out->begin(), [invNumSamples] (const countT x) {
      return invNumSamples * x;
    });
  }

  const string& id() const { return m_id; }

  template <typename T, typename countT> 
  friend ostream& operator<<(ostream& os, const Histogram<T,countT>& hist);  // NOLINT

 protected:
  //! Initialize this Histogram with the given range, numBins and id
  void init(const Interval<T>& _range, int _numBins, string _id, bool _isAngle = false) {
    m_range = _range;
    m_numBins = _numBins;
    m_binSize = (m_range.max - m_range.min) / m_numBins;
    m_invBinSize = 1 / m_binSize;
    m_numSamples = 0;
    m_acc.resize(0);
    m_acc.resize(m_numBins, 0);
    m_id = _id;
    m_isAngle = _isAngle;
    int dim = (_isAngle)? 2 : 1;
    m_moments.resize(dim*2);
  }

  //! Returns the index of the bin to which x is assigned, or -1 if x is outside the range
  int binIndex(T x) const {
    // Check x within range (will also not allow x that is nan)
    if (x >= m_range.min && x <= m_range.max) {
      int idx = static_cast<int>(std::floor((x - m_range.min) * m_invBinSize));
      if (idx == m_numBins) { idx--; }  // Include upper boundary since we are a closed range on both ends
      assert(idx >= 0 && idx < m_numBins);
      return idx;
    } else {
      return -1; // Underflow or overflow
    }
  }

  void _add(T x, countT count = 1) {
    int idx = binIndex(x);
    if (idx < 0 || idx >= m_numBins) {
      SG_LOG_WARN << "Warning: x=" << x << " outside range [" << m_range.min << "," << m_range.max << "] of histogram " << m_id;
      return;
    }
    m_acc[idx]+=count;
    m_numSamples+=count;
  }

  int m_numBins;        // number of bins of this histogram
  Interval<T> m_range;  // Range of this histogram
  T m_binSize;          // Size of histogram bins
  T m_invBinSize;       // Cached constant for determining bin index
  countT m_numSamples;  // Total number of samples added
  vec<countT> m_acc;    // Stored histogram bin counts
  string m_id;          // Identifier for this histogram

  // Also keep track of moments (we track 2 moments for now)
  // Can be used to estimate distributions
  // for angles, we keep two fields per moment
  bool m_isAngle = false;
  vec<T> m_moments;

 private:
  // boost serialization support
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {  // NOLINT
    ar & m_numBins;
    ar & m_range;
    ar & m_binSize;
    ar & m_invBinSize;
    ar & m_numSamples;
    ar & m_acc;
    ar & m_id;
    ar & m_isAngle;
    ar & m_moments;
  }
};

//! Overloaded ostream operator<< for writing histograms out
template <typename T, typename countT>
ostream& operator<<(ostream& os, const Histogram<T,countT>& hist) {
  const countT total = hist.totalCount();
  os << "{id: " << hist.m_id << ", range: " << hist.m_range << ", numBins: " << hist.m_numBins
     << ", count: " << total << ", bins: [" << endl;
  for (int i = 0; i < hist.m_numBins; ++i) {
    const auto& bin = hist.binInterval(i);
    const countT count = hist.count(i);
    const double ratio = total > 0 ? static_cast<double>(count) / total : 0;
    os << "\t{range: " << bin << ", count: " << count << ", ratio: " << ratio << "}" << endl;
  }
  os << "]}" << endl;
  return os;
}

////! Vector of histograms for data with numDims dimensions
//template <typename T, size_t numDims>
//class HistogramVec {
//  //! Create vector of histogram each with specified range and numBins
//  HistogramVec(const Interval<T>& _range, int _numBins) : m_hists(Histogram<T>(_range, _numBins)) { }
//  //! Default [0, 1] / 10 bins histograms
//  HistogramVec() : m_hists(Histogram<T>(Interval<T>(0, 1), 10)) { }
//
//  //! Accumulates each element of data into corresponding dimension
//  template <typename VecT>
//  void addVec(const VecT& data) {
//    assert(data.size() == m_hists.size());
//    for (int i = 0; i < data.size(); ++i) {
//      m_hists[i].add(data[i]);
//    }
//  }
//
//  //! Clear all histogram dimensions
//  void clear() {
//    for (auto& h : m_hists) { h.clear(); }
//  }
//
//  //! Return by-reference histogram at i-th dimension
//  Histogram& operator[](int i) { return m_hists[i]; }
//  //! Return by-const-reference histogram at i-th dimension
//  const Histogram& operator[](int i) const { return m_hists[i]; }
//
// private:
//  arr<Histogram<T>, numDims> m_hists;
//};

//! Utility functions
namespace util {
  //! Accumulates each element of data into corresponding histogram dimension in hists
  template <typename VecT, typename VecHistT>
  void addVecToVecHistogram(const VecT& data, VecHistT* hists) {
    assert(data.size() == hists->size());
    for (int i = 0; i < data.size(); ++i) {
      (*hists)[i].add(data[i]);
    }
  }

  //! Accumulates each element of data into corresponding histogram dimension in hists
  template <typename VecHistT>
  void addVecHistogramToVecHistogram(const VecHistT& h, VecHistT* hists) {
    assert(h.size() == hists->size());
    for (int i = 0; i < h.size(); ++i) {
      (*hists)[i].addAll(h[i]);
    }
  }

  //! Accumulates each element of data into corresponding histogram dimension in hists
  template <typename VecHistT, typename countT>
  void addVecHistogramToVecHistogram(const VecHistT& h, const countT weight, VecHistT* hists) {
    assert(h.size() == hists->size());
    for (int i = 0; i < h.size(); ++i) {
      (*hists)[i].addAll(h[i], weight);
    }
  }

  //! Accumulates each element of data into corresponding histogram dimension in hists
  template <typename VecHistMapT>
  void addVecHistogramMapToVecHistogramMap(const VecHistMapT& h, VecHistMapT* hists) {
    for (const auto& pair : h) {
      if (hists->count(pair.first) == 0) {
        hists->insert(pair);
      } else {
        (*hists)[pair.first].addAll(pair.second);
      }
    }
  }
}

}  // namespace stats
}  // namespace sg


