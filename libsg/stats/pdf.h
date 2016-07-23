#pragma once

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/density.hpp>
#include <boost/accumulators/statistics/stats.hpp>

#include "libsg.h"  // NOLINT
#include "stats/stats.h"
#include "math/math.h"

namespace sg {
namespace stats {

namespace ba = boost::accumulators;

//! Class that can build a probability density function given samples in a domain
class PDF {
 public:
  //! _cache_size determines how many samples are needed before setting binning range and giving aggregation back
  PDF(int _numBins, size_t _cache_size)
    : numBins(_numBins)
    , m_acc(ba::tag::density::num_bins = numBins, ba::tag::density::cache_size = _cache_size) { }

  void add(float x) {
    m_acc(x);
  }

  double density(int iBin) {
    const auto& hist = ba::density(m_acc);
    return hist[iBin].second;
  }

  double min() {
    return ba::min(m_acc);
  }

  double max() {
    return ba::max(m_acc);
  }

  Interval<double> binInterval(int iBin) {
    const auto& hist = ba::density(m_acc);
    if (iBin == numBins + 1) {  // Overflow bin
      return Interval<double>(hist[iBin].first, math::constants::POSINF);
    } else if (iBin == 0) {  // Underflow bin
      return Interval<double>(math::constants::NEGINF, hist[iBin + 1].first);
    } else {
      return Interval<double>(hist[iBin].first, hist[iBin + 1].first);
    }
  }

  size_t numSamples() {
    return ba::count(m_acc);
  }

  vecd vecNormalized() {
    const auto hist = ba::density(m_acc);
    vecd counts(hist.size());
    std::transform(hist.begin(), hist.end(), counts.begin(), [ ] (const pair<double, double>& p) { return p.second; });
    return counts;
  }

  vecd vecCounts() {
    vecd counts = vecNormalized();
    const size_t total = numSamples();
    std::transform(counts.begin(), counts.end(), counts.begin(), [total] (double x) { return x * total; });
    return counts;
  }

  const int numBins;  // Does not include underflow and overflow bins

 protected:
  ba::accumulator_set<double, ba::features<ba::tag::density> > m_acc;
};

inline void testPDF() {
  PDF hist(11, 100000);
  for (size_t i = 0; i < 10000000; ++i) {
    hist.add(sg::math::DEF_RAND.normal_float_01());
  }

  const sg::vecd normed = hist.vecNormalized(), counts = hist.vecCounts();
  std::cout << "numSamples=" << hist.numSamples() << std::endl;
  for (int i = 0; i < counts.size(); ++i) {
    const Interval<double> range = hist.binInterval(i);
    std::cout << i << ":[" << range.min << "," << range.max << "]: " << normed[i] << ", " << counts[i] << std::endl;
  }
}

}  // namespace stats
}  // namespace sg


