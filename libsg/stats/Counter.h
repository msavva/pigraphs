#pragma once

#include "libsg.h"  // NOLINT

#include "util/grid.h"
#include "util/index.h"
#include "util/util.h"

namespace sg {
namespace stats {

//! Simple counter class
template <typename T, typename countT = size_t>
struct Counter {
  Counter() : m_counts() { }
  explicit Counter(const Counter<T>& other) : m_counts(other.m_counts) { }

  typedef std::function<bool(const pair<T,countT>& c, const pair<T,countT> bestSoFar)> cmpT;

  bool empty() { 
    return m_counts.empty(); 
  }

  void inc(const T& x) {
    ++m_counts[x];
  }

  void inc(const T& x, const countT c) {
    m_counts[x] += c;
  }

  void copy(const Counter<T, countT>& cts) {
    m_counts.clear();
    for (const auto& it : cts) {
      m_counts[it.first] = it.second;
    }
  }

  template <typename countT2>
  void copy(const Counter<T, countT2>& cts) {
    m_counts.clear();
    for (const auto& it : cts) {
      m_counts[it.first] = static_cast<countT>(it.second);
    }
  }

  void add(const Counter<T, countT>& cts) {
    for (const auto& it : cts) {
      m_counts[it.first] += it.second;
    }
  }

  template <typename countT2>
  void add(const Counter<T, countT2>& cts) {
    for (const auto& it : cts) {
      m_counts[it.first] += static_cast<countT>(it.second);
    }
  }

  void subtract(const Counter<T, countT>& cts) {
    for (const auto& it : cts) {
      m_counts[it.first] -= it.second;
    }
  }

  template <typename countT2>
  void subtract(const Counter<T, countT2>& cts) {
    for (const auto& it : cts) {
      m_counts[it.first] -= static_cast<countT>(it.second);
    }
  }

  template <typename countT1, typename countT2>
  void product(const Counter<T, countT1>& cts1, const Counter<T, countT2>& cts2) {
    m_counts.clear();
    for (const auto& it : cts1) {
      if (cts2.hasCount(it.first)) {
        m_counts[it.first] = static_cast<countT>(it.second * cts2.count(it.first));
      }
    }
  }

  void normalize() {
    countT total = totalCount();
    divideBy(total);
  }

  void divideBy(countT t) {
    for (auto& it : m_counts) {
      m_counts[it.first] /= t;
    }
  }

  void multiplyBy(countT t) {
    for (auto& it : m_counts) {
      m_counts[it.first] *= t;
    }
  }

  void set(const T& x, const countT c) {
    m_counts[x] = c;
  }

  void remove(const T& x) {
    if (m_counts.count(x) > 0) {
      m_counts.erase(x);
    }
  }

  void resetCount(const T& x) {
    m_counts[x] = 0;
  }

  pair<T, countT> maxPair() const {
    T x;
    countT m = std::numeric_limits<countT>::min();
    for (const auto it : m_counts) {
      if (it.second > m) {
        x = it.first;
        m = it.second;
      }
    }
    return pair<T, countT>(x, m);
  }

  T argMax() const {
    return maxPair().first;
  }

  pair<T, countT> minPair() const {
    T x;
    countT m = std::numeric_limits<countT>::min();
    for (const auto it : m_counts) {
      if (it.second < m) {
        x = it.first;
        m = it.second;
      }
    }
    return std::pair<T, countT>(x, m);
  }
  T argMin() const {
    return minPair().first;
  }

  pair<T, countT> bestPair(cmpT cmp) const {
    pair<T, countT> p;
    p.second = std::numeric_limits<countT>::min();
    for (const auto it : m_counts) {
      if (cmp(it, p)) {
        p = it;
      }
    }
    return p;
  }

  countT count(const T& x) const {
    if (m_counts.count(x) > 0) {
      return m_counts.at(x);
    } else {
      return static_cast<countT>(0);
    }
  }

  countT totalCount() const {
    countT sum = static_cast<countT>(0);
    for (const auto& p : m_counts) { sum += p.second; }
    return sum;
  }

  //! Specialized string prefix count: return total of counts with given prefix
  countT countWithPrefix(const string& x) const {
    countT sum = static_cast<countT>(0);
    for (const auto& p : m_counts) {
      if (sg::util::startsWith(p.first, x)) {
        sum += p.second;
      }
    }
    return sum;
  }

  void clear() {
    m_counts.clear();
  }

  bool hasCount(const T& x) const {
    return m_counts.count(x) > 0;
  }

  template <typename PredT = sg::util::predicate_pair_desc2<T,countT>>
  vec<std::pair<T, countT>> toSortedPairs() const {
    PredT pred;
    vec<std::pair<T, countT>> pairs;
    pairs.reserve(m_counts.size());
    for (const auto& p : m_counts) {
      pairs.push_back({p.first, p.second});
    }
    std::sort(pairs.begin(), pairs.end(), pred);
    return pairs;
  }

  typename map<T, countT>::const_iterator begin() const { return m_counts.begin(); }
  typename map<T, countT>::const_iterator end() const { return m_counts.end(); }

private:

  //! boost serialization function
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive& ar, const unsigned int) {  // NOLINT
    ar & m_counts;
  }

  map<T, countT> m_counts;
};

typedef Counter<string, float> CounterSf;
typedef Counter<string, double> CounterSd;

template <typename T, typename num_t>
ostream& operator<<(ostream& os, const Counter<T,num_t>& c) {  // NOLINT
  os << "{totalCount:" << c.totalCount() << "}";
  return os;
}

template <typename T, typename num_t>
ostream& toDelimited(ostream& os, const Counter<T,num_t>& c, const string delimiter = "\t") {
  vec<std::pair<T,num_t>> pairs = c.toSortedPairs();
  return sg::util::pairsToDelimited(os, pairs, delimiter);
}


//! A dense 2D counter
template <typename T, typename countT = size_t>
struct Counter2D {
  Counter2D() 
    : m_index()
    , m_counts()
    , m_totalCount(static_cast<countT>(0)) { }
  explicit Counter2D(const sg::util::Index<T>& index)
    : m_index(index)
    , m_counts(static_cast<int>(index.size()), static_cast<int>(index.size()), static_cast<countT>(0))
    , m_totalCount(static_cast<countT>(0)) { }

  void init(const sg::util::Index<T>& index) {
    m_index = index;
    m_counts.init(static_cast<int>(index.size()), static_cast<int>(index.size()));
    clear();
  }

  void inc(const T& x, const T& y) {
    inc(x,y,static_cast<countT>(1));
  }

  void inc(const T& x, const T& y, const countT c) {
    int i1 = m_index.indexOf(x);
    int i2 = m_index.indexOf(y);
    if (i1 < 0) {
      SG_LOG_ERROR << "[Counter2D] Tried to increment unknown element " << x;
    }
    if(i2 < 0) {
      SG_LOG_ERROR << "[Counter2D] Tried to increment unknown element " << y;
    }

    m_counts(i1, i2) += c;
    m_totalCount += c;
  }

  void clear() {
    m_counts.clear(static_cast<countT>(0));
    m_totalCount = static_cast<countT>(0);
  }

  countT getTotalCount() { return m_totalCount; }

  const sg::util::NumericGrid<countT>& getCounts() const {
    return m_counts;
  }

  const sg::util::Index<T>& getIndex() const {
    return m_index;
  }

private:
  //! boost serialization function
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive& ar, const unsigned int) {  // NOLINT
    ar & m_index;
    ar & m_counts;
    ar & m_totalCount;
  }

  sg::util::Index<T> m_index;
  sg::util::NumericGrid<countT> m_counts;
  countT m_totalCount;
};

template <typename T, typename num_t>
ostream& toDelimited(ostream& os, const Counter2D<T,num_t>& c,
                     const string& delimiter = ",", const string& lineEnd = "\n") {
  const auto labels = c.getIndex().labels();
  for (int i = 0; i < labels.size(); ++i) {
    for (int j = 0; j < labels.size(); ++j) {
      os << labels[i] << delimiter << labels[j] << delimiter << c.getCounts()(i, j) << lineEnd;
    }
  }
  return os;
}

}  // namespace stats
}  // namespace sg


