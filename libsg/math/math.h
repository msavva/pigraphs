#pragma once

#include <numeric>
#include <random>

#include "libsg.h"  // NOLINT

namespace sg {
namespace math {

namespace constants {

const double  PI        = 3.141592653589793238462643383279502884e+00,
              PI2       = 2 * PI,
              INVPI     = 1.0 / PI,
              INVPI2    = 1.0 / PI2,
              LOGPI2    = log(PI2),
              RAD2DEG   = 180.0 / PI,
              DEG2RAD   = 1.0 / RAD2DEG,
              PHI       = (1 + sqrt(5.0)) / 2,
              INVPHI    = 1.0 / PHI,
              POSINF    = std::numeric_limits<double>::infinity(),
              NEGINF    = -std::numeric_limits<double>::infinity();

const float   PIf       = static_cast<float>(PI),
              PI2f      = static_cast<float>(PI2),
              INVPIf    = static_cast<float>(INVPI),
              INVPI2f   = static_cast<float>(INVPI2),
              LOGPI2f   = static_cast<float>(LOGPI2),
              RAD2DEGf  = static_cast<float>(RAD2DEG),
              DEG2RADf  = static_cast<float>(DEG2RAD),
              PHIf      = static_cast<float>(PHI),
              INVPHIf   = static_cast<float>(INVPHI),
              POSINFf   = std::numeric_limits<float>::infinity(),
              NEGINFf   = -std::numeric_limits<float>::infinity();

}  // namespace constants

//! Return whether the given floating point values x and y are equal within a variation of ulp units in the last place
template<class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
almost_equal(T x, T y, int ulp = 2) {
  // Machine epsilon has to be scaled to the magnitude of the values used and multiplied by the desired precision
  // in ULPs (units in the last place)
  return x == y ||
         std::abs(x - y) < std::numeric_limits<T>::epsilon() * std::abs(x + y) * ulp ||
         // unless the result is subnormal
         std::abs(x - y) < std::numeric_limits<T>::min();
}

template <typename T>
T rad2deg(const T rad) {
  return rad * static_cast<T>(constants::RAD2DEG);
}

template <typename T>
T deg2rad(const T deg) {
  return deg * static_cast<T>(constants::DEG2RAD);
}

//! Clamp x to be between a and b
template <typename T>
T clamp(const T x, const T a, const T b) {
  return (x < a) ? a : ((x > b) ? b : x);
}

//! Linear interpolation function: maps range [s1,e1] to [s2,e2], returning value of v in [s2,e2]
template <typename T>
T lerp(const T s1, const T e1, const T s2, const T e2, const T v) {
  const T d1 = e1 - s1,
          d2 = e2 - s2;
  if (d1 == static_cast<T>(1)) {  // Skip division if possible
    return ((v - s1) * d2 + s2);
  } else {
    return ((v - s1) * d2 / d1 + s2);
  }
}

//! Sigmoid function mapping [-inf,inf] to [-1,1])
template <typename T>
T sigmoid(const T x) {
  return tanh(x);
}

//! Inverse Sigmoid function mapping [-1,1] to [-inf,inf])
template <typename T>
T invSigmoid(const T x) {
  return atanh(x);
}

//! Returns whether given integer is power of two
inline bool isPowOf2(int v) {
  return v && !(v & (v - 1));
}

//! Round up a given uint32_t to the nearest power of two
inline uint32_t roundUpPow2(uint32_t v) {
  v--;
  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
  return v + 1;
}

template <typename T>
T acosSafe(T x) {
  if (x <= static_cast<T>(-1)) {
    return static_cast<T>(constants::PI);
  } else if (x >= static_cast<T>(1)) {
    return static_cast<T>(0);
  }
  return std::acos(x);
}

//! Simple Euclidean squared distance between two vector types
template <typename VecT>
float distanceSquared(const VecT& a, const VecT& b) {
  assert(a.size() == b.size());
  typedef typename VecT::value_type T;
  const auto dSq = [ ] (T x, T y) {
    T z = y - x;
    return z * z;
  };
  const T val = std::inner_product(a.begin(), a.end(), b.begin(), static_cast<T>(0), std::plus<T>(), dSq);
  return val;
}

//! Cosine similarity between two vectors: returns sim = cos(\theta) = dot(a, b) / (||a|| ||b||)
//! Range is [-1,1] with -1 being exactly opposite, 0 indicating independence, and 1 exactly the same
template <typename VecT>
float cosineSimilarity(const VecT& a, const VecT& b) {
  assert(a.size() == b.size());
  typedef typename VecT::value_type T;
  const T zero = static_cast<T>(0),
          aSquare = std::inner_product(a.begin(), a.end(), a.begin(), zero),
          bSquare = std::inner_product(b.begin(), b.end(), b.begin(), zero),
          ab = std::inner_product(a.begin(), a.end(), b.begin(), zero),
          cos = clamp(ab / (sqrt(aSquare) * sqrt(bSquare)), static_cast<T>(-1), static_cast<T>(1));
  return cos;
}

//! Angular similarity between two vectors: returns sim = 1 - (acos(cosineSimilarity) / pi)
//! Range is [0,1] with 0 being completely dissimilar and 1 being exactly the same
template <typename VecT>
float angularSimilarity(const VecT& a, const VecT& b) {
  typedef typename VecT::value_type T;
  const T cosSim = cosineSimilarity(a, b),
          angSim = clamp(1 - (acos(cosSim) * static_cast<T>(constants::INVPI)), static_cast<T>(0), static_cast<T>(1));
  return angSim;
}

//! Returns the sum of a vector of values
template <typename VecT>
typename VecT::value_type sum(const VecT& v) {
  return std::accumulate(std::begin(v), std::end(v), static_cast<typename VecT::value_type>(0));
}

//! Returns the mean value of a vector of values
template <typename VecT>
typename VecT::value_type mean(const VecT& v) {
  return sum(v) / v.size();
}

//! Returns the median value of a vector of values
//! NOTE: This is a naive implementation -- does not correctly handle even-sized collections
template <typename VecT>
typename VecT::value_type median(const VecT& v) {
  vec<size_t> sortedIndices;
  util::sortIndices(v.begin(), v.end(), std::greater<typename VecT::value_type>(), sortedIndices);
  const size_t midIdx = v.size() / 2;
  return v[sortedIndices[midIdx]];
}

//! Returns the standard deviation of a vector of values
template <typename VecT>
typename VecT::value_type stdev(const VecT& v) {
  typedef typename VecT::value_type T;
  const T m = mean(v);
  T accum = 0.0;
  std::for_each(std::begin(v), std::end(v), [&](const T d) {
    accum += (d - m) * (d - m);
  });
  return sqrt(accum / (v.size() - 1));
}

//! Returns the covariance of two vectors of values x and y
template <typename VecT>
typename VecT::value_type covariance(const VecT& x, const VecT& y) {
  typedef typename VecT::value_type T;
  assert(x.size() == y.size());
  const T mx = mean(x), my = mean(y);
  const auto n = x.size();
  T accum = 0.0;
  for (auto i = 0; i < n; ++i) {
    accum += (x[i] - mx) * (y[i] - my);
  }
  return accum / (n - 1);
}

//! Returns the circular mean given a vector of angle values in radians. Also stores concentration if pointer is
//! passed in (more +ve for tighter distributions, 0 if uniformly distributed around circle).
//! See: http://en.wikipedia.org/wiki/Mean_of_circular_quantities
template <typename VecT, typename T = typename VecT::value_type>
T circularMean(const VecT& v, T* concentration = nullptr) {
  const size_t n = v.size();
  VecT sinv(n), cosv(n);
  std::transform(std::begin(v), std::end(v), std::begin(sinv), [&](const T& x) { return std::sin(x); });
  std::transform(std::begin(v), std::end(v), std::begin(cosv), [&](const T& x) { return std::cos(x); });
  const T meanSin = sum(sinv) / static_cast<T>(n),
          meanCos = sum(cosv) / static_cast<T>(n),
          meanAng = std::atan2(meanSin, meanCos);
  if (concentration != nullptr) {  // Also save out variance
    // Use simple approximation for concentration (kappa)
    // See https://en.wikipedia.org/wiki/Von_Mises%E2%80%93Fisher_distribution
    const T R = sqrt(meanSin * meanSin + meanCos * meanCos);
    const T R2 = R * R;
    *concentration = R * (2 - R2) / (1 - R2);
  }
  return meanAng;
}

// Select random element from STL range (https://gist.github.com/cbsmith/5538174)
template <typename RandomGenerator = std::default_random_engine>
struct random {
  // On most platforms, you probably want to use
  // std::random_device("/dev/urandom")()
  explicit random(RandomGenerator g = RandomGenerator(std::random_device()()))
    : gen_(g)
    , uniform_float_01_(0, 1)
    , uniform_int_()
    , uniform_uint32_()
    , normal_float_01_(0, 1) { }

  //! Return randomly sampled int distributed uniformly in [0, std::numeric_limits<int>::max()]
  int uniform_int() {
    return uniform_int_(gen_);
  }
  //! Return randomly sampled uint32_t distributed uniformly in [0, std::numeric_limits<uint32_t>::max()]
  uint32_t uniform_uint() {
    return uniform_uint32_(gen_);
  }
  //! Return randomly sampled float distributed uniformly in [0, 1)
  float uniform_float_01() {
    return uniform_float_01_(gen_);
  }
  //! Return randomly sampled float distributed normally (mean=0, std=1)
  float normal_float_01() {
    return normal_float_01_(gen_);
  }
  //! Return randomly sampled float distributed uniformly in [a, b)
  float uniform_float(float a, float b) {
    return uniform_float_01_(gen_)*(b-a)+a;
  }
  //! Return randomly sampled int distributed uniformly in [a, b)
  int uniform_int(int a, int b) {
    return static_cast<int>(uniform_float(static_cast<float>(a), static_cast<float>(b)));
  }

  template <typename Iter>
  Iter select(Iter startIt, Iter endIt) {
    std::uniform_int_distribution<> dis(0, static_cast<int>(std::distance(startIt, endIt)) - 1);
    std::advance(startIt, dis(gen_));
    return startIt;
  }

  template <typename T, typename num_t>
  int selectFromWeighted(const vec<T>& items,
                         const std::function<num_t(const T&)>& weightFunc,
                         pair<T, num_t>* pSampled = nullptr) {
    num_t total = 0;
    for (const auto& element : items) {
      num_t weight = weightFunc(element);
      total += weight;
    }
    return selectFromWeighted(items, total, weightFunc, pSampled);
  }

  template <typename T, typename num_t>
  int selectFromWeighted(const vec<T>& items,
                         num_t total,
                         const std::function<num_t(const T&)>& weightFunc,
                         pair<T, num_t>* pSampled = nullptr) {
    num_t r = static_cast<num_t>(uniform_float_01() * total);
    num_t t = 0;
    for (int i = 0; i < items.size(); ++i) {
      const auto& element = items[i];
      num_t weight = weightFunc(element);
      t += weight;
      if (t > r) {
        if (pSampled) {
          pSampled->first = element;
          pSampled->second = weight;
        }
        return i;
      }
    }
    return -1;
  }

  template <typename T, typename num_t>
  int selectFromWeighted(const vec<pair<T, num_t>>& weighted, 
                         pair<T, num_t>* pSampled = nullptr) {
    num_t total = 0;
    for (const auto& element : weighted) {
      num_t weight = element.second;
      total += weight;
    }
    return selectFromWeighted(weighted, total, pSampled);
  }

  template <typename T, typename num_t>
  int selectFromWeighted(const vec<pair<T, num_t>>& weighted,
                         num_t total,
                         pair<T, num_t>* pSampled = nullptr) {
    num_t r = static_cast<num_t>(uniform_float_01() * total);
    num_t t = 0;
    for (int i = 0; i < weighted.size(); ++i) {
      const auto& element = weighted[i];
      num_t weight = element.second;
      t += weight;
      if (t > r) {
        if (pSampled) {
          (*pSampled) = element;
        }
        return i;
      }
    }
    return -1;
  }

  template <typename Iter, typename T, typename num_t>
  bool selectFromWeighted(Iter iter, const std::function<num_t(const T&)>& weightFunc,
                          pair<T, num_t>* pSampled) {
    num_t total = 0;
    for (const T& element : iter) {
      num_t weight = weightFunc(element);
      total += weight;
    }
    return selectFromWeighted<Iter,T,num_t>(iter, total, weightFunc, pSampled);
  }

  template <typename Iter, typename T, typename num_t>
  bool selectFromWeighted(Iter iter, num_t total, const std::function<num_t(const T&)>& weightFunc,
                          pair<T, num_t>* pSampled) {
    num_t r = static_cast<num_t>(uniform_float_01() * total);
    num_t t = 0;
    for (const T& element : iter) {
      num_t weight = weightFunc(element);
      t += weight;
      if (t > r) {
        pSampled->first = element;
        pSampled->second = weight;
        return true;
      }
    }
    return false;
  }

  template <typename Iter>
  Iter operator()(Iter startIt, Iter endIt) { return select(startIt, endIt); }

  //! Randomly shuffle elements of range [startIt,endIt)
  template <typename Iter>
  void shuffle(Iter startIt, Iter endIt) { std::shuffle(startIt, endIt, gen_); }
  template <typename Container>
  void shuffle(Container& c) { shuffle(begin(c), end(c)); }  // NOLINT

  //! Shuffles within each subrange of given size in [startIt,endIt)
  template <typename Iter>
  void shuffleSubranges(Iter startIt, Iter endIt, size_t rangeSize) {
    const size_t size = std::distance(startIt, endIt);
    assert(rangeSize > 0 && rangeSize <= size);
    //const size_t numSubranges = size / rangeSize;
    Iter s = startIt, e = s;
    while (static_cast<size_t>(std::distance(s, endIt)) < rangeSize) {
      std::advance(e, rangeSize);
      std::shuffle(s, e, gen_);
      s = e;
    }
    if (s < endIt) {
      std::shuffle(s, endIt, gen_);  // One final shuffle of partial range if there are elements left
    }
  }
  template <typename Container>
  void shuffleSubranges(Container& c, size_t rangeSize) {  // NOLINT
    shuffleSubranges(begin(c), end(c), rangeSize);
  }

  // convenience function that works on anything with a sensible begin() and
  // end(), and returns with a ref to the value type
  template <typename Container>
  auto operator()(const Container& c) -> decltype(*begin(c))& {
    return *select(begin(c), end(c));
  }

  //! Return n randomly chosen elements from c with no replacement of picked values (i.e. different)
  template <typename Container>
  Container randomSubset(const Container& c, size_t n) {
    const size_t nAll = c.size();
    if (nAll < 2) { return c; }
    Container c2(c);
    shuffle(begin(c2), end(c2));
    if (n >= nAll) { return c2; }
    const size_t nDrop = nAll - n;
    return Container(begin(c2) + nDrop - 1, end(c2));
  }

  
  // Select k out of 0 to total
  void sampleNumbers(size_t k, size_t total, vec<size_t>* pSampled) {
    size_t n = std::min(static_cast<size_t>(k), total);
    pSampled->clear();
    if (n < total/2) { 
      pSampled->reserve(n);
      set<size_t> s;
      while (s.size() < n) {
        size_t sample = static_cast<size_t>(floor(uniform_float_01() * total));
        s.insert(sample);
        pSampled->emplace_back(sample);
      }
    } else {
      pSampled->resize(total);
      for (size_t i = 0; i < total; i++) {
        (*pSampled)[i] = i;
      }
      std::shuffle(pSampled->begin(), pSampled->end(), gen_);
      pSampled->resize(n);
    }
  }


 private:
  RandomGenerator gen_;
  std::uniform_real_distribution<float> uniform_float_01_;
  std::uniform_int_distribution<int> uniform_int_;
  std::uniform_int_distribution<uint32_t> uniform_uint32_;
  std::normal_distribution<float> normal_float_01_;
};

//! Default statically initialized random value generator
static random<> DEF_RAND;

//! Normalize 
template <typename numT>
void normalize(vec<numT>& v) {
  numT t = 0.0;
  for (numT c : v) {
    ++t;
  }
  for (numT& c : v) {
    c = c/t;
  }
}

template <typename numT>
void normalizeLog(vec<numT>& v) {
  numT m = v[0];
  for (numT c : v) {
    if (c < m) { m = c; }
  }
  numT t = 0.0;
  for (numT& c : v) {
    c = c - m;
    t += exp(c);
  }
  t = log(t);
  for (numT& c : v) {
    c = c - t;
  }
}

//! Modified Bessel function of the first kind v-th order, arg x
double besselI(int v, double x);

//! Beta function B(a, b)
double beta(double a, double b);

}  // namespace math
}  // namespace sg


