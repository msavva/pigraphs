#pragma once

#include "libsg.h"  // NOLINT

#include "math/math.h"

namespace sg {
namespace stats {

//! A low-discrepancy sampler based on PBRT's implementation which follows the (0,2) sequence method in
//! "Efficient Multidimensional Sampling" [Kollig and Keller 2002] http://www.uni-kl.de/AG-Heinrich/EMS.pdf
class LDSampler {
 public:
  typedef float Sample1D;
  typedef arr<float, 2> Sample2D;

  //! Create LDSampler with at least numRequestedSamples, optionally jittered by numBins (or numBins^2 in 2D)
  LDSampler(uint32_t numRequestedSamples, uint32_t numBins = 1)
    : m_numRequestedSamples(numRequestedSamples)
    , m_numBins(numBins)
    , m_numTotalSamples(math::roundUpPow2(numRequestedSamples)) { }

  //! Get next 1D sample point
  Sample1D next1D() {
    if (m_samples1D.empty()) {
      init1D();
    }
    if (m_currSample1DIt == m_samples1D.end()) {
      cerr << "[LDSampler] Warning: wrapping around samples since requested more than available" << endl;
      m_currSample1DIt = m_samples1D.begin();
    }
    const Sample1D& s = *m_currSample1DIt;
    ++m_currSample1DIt;
    return s;
  }

  //! Get next 2D sample point
  Sample2D next2D() {
    if (m_samples2D.empty()) {
      init2D();
    }
    if (m_currSample2DIt == m_samples2D.end()) {
      cerr << "[LDSampler] Warning: wrapping around samples since requested more than available" << endl;
      m_currSample2DIt = m_samples2D.begin();
    }
    const Sample2D& s = *m_currSample2DIt;
    ++m_currSample2DIt;
    return s;
  }

 private:
  // van der Corput's radical inverse function with bit scrambling in base 2
  float VanDerCorput(uint32_t n, uint32_t scramble) {
    // Reverse bits of _n_
    n = (n << 16) | (n >> 16);
    n = ((n & 0x00ff00ff) << 8) | ((n & 0xff00ff00) >> 8);
    n = ((n & 0x0f0f0f0f) << 4) | ((n & 0xf0f0f0f0) >> 4);
    n = ((n & 0x33333333) << 2) | ((n & 0xcccccccc) >> 2);
    n = ((n & 0x55555555) << 1) | ((n & 0xaaaaaaaa) >> 1);
    n ^= scramble;
    return std::min(((n >> 8) & 0xffffff) / float(1 << 24), 0.9999999403953552f);  // NOLINT
  }

  // Sobol's radical inverse function with bit scrambling in base 2
  float Sobol2(uint32_t n, uint32_t scramble) {
    for (uint32_t v = 1 << 31; n != 0; n >>= 1, v ^= v >> 1) {
      if (n & 0x1) scramble ^= v;
    }
    return std::min(((scramble >> 8) & 0xffffff) / float(1 << 24), 0.9999999403953552f);  // NOLINT
  }

  // Sets the n-th sample from (0,2) sequence, scrambled by bit patterns in scramble into xy
  void Sample02(const uint32_t n, const uint32_t scramble[2], Sample2D* s) {
    (*s)[0] = VanDerCorput(n, scramble[0]);
    (*s)[1] = Sobol2(n, scramble[1]);
  }

  void init1D() {
    m_samples1D.resize(m_numTotalSamples);
    const uint32_t
      scramble = math::DEF_RAND.uniform_uint(),
      numSamplesPerBin = std::max(static_cast<uint32_t>(m_numTotalSamples / m_numBins), static_cast<uint32_t>(1));
    const float binSize = 1.f / m_numBins;

    float binOffset = 0.f; uint32_t baseIdx = 0;
    for (uint32_t iBin = 0; iBin < m_numBins; ++iBin, baseIdx += numSamplesPerBin, binOffset += binSize) {
      for (uint32_t iSample = 0; iSample < numSamplesPerBin; ++iSample) {
        m_samples1D[baseIdx + iSample] = binOffset + VanDerCorput(baseIdx + iSample, scramble) * binSize;
      }
      // shuffle within bin
      const auto start = m_samples1D.begin() + baseIdx;
      math::DEF_RAND.shuffle(start, start + numSamplesPerBin);
    }
    math::DEF_RAND.shuffleSubranges(m_samples1D, numSamplesPerBin);
    m_currSample1DIt = m_samples1D.begin();
  }

  void init2D() {
    m_samples2D.resize(m_numTotalSamples);
    const uint32_t
      scramble[2]       = {math::DEF_RAND.uniform_uint(), math::DEF_RAND.uniform_uint()},
      numBinsPerDim     = static_cast<uint32_t>(std::ceil(std::sqrt(m_numBins))),
      numSamplesPerBin  = std::max(static_cast<uint32_t>(m_numTotalSamples / m_numBins), static_cast<uint32_t>(1));
    const float binSize = 1.f / numBinsPerDim;

    float binOffsetI = 0.f; uint32_t baseIdx = 0; Sample2D sample;
    for (uint32_t iBin = 0; iBin < numBinsPerDim; ++iBin, binOffsetI += binSize) {
      float binOffsetJ = 0.f;
      for (uint32_t jBin = 0; jBin < numBinsPerDim; ++jBin, binOffsetJ += binSize) {
        for (uint32_t iSample = 0; iSample < numSamplesPerBin; ++iSample) {
          Sample02(baseIdx + iSample, scramble, &sample);
          auto& out = m_samples2D[baseIdx + iSample];
          out[0] = binOffsetI + sample[0] * binSize;
          out[1] = binOffsetJ + sample[1] * binSize;
        }  // sample
        // shuffle within bin
        const auto start = m_samples2D.begin() + baseIdx;
        math::DEF_RAND.shuffle(start, start + numSamplesPerBin);
        baseIdx += numSamplesPerBin;
      }  // j
    }  // i
    math::DEF_RAND.shuffleSubranges(m_samples2D, numSamplesPerBin);  // now shuffle bins
    m_currSample2DIt = m_samples2D.begin();
  }

  size_t m_numRequestedSamples, m_numBins, m_numTotalSamples;
  vec<Sample1D> m_samples1D;
  vec<Sample1D>::const_iterator m_currSample1DIt;
  vec<Sample2D> m_samples2D;
  vec<Sample2D>::const_iterator m_currSample2DIt;
};

// TODO: Check!!!
template <typename Iter, typename T, typename num_t = double>
void sampleWeightedWithoutReplacement(Iter& iter, const std::function<num_t(const T&)>& weightFunc,
                                      int nSamples, vec<pair<T, num_t>>* selected, sg::math::random<> rand = sg::math::DEF_RAND) {
  sg::util::predicate_pair_desc2asc1<T, num_t> pair_cmp;
  int n = 0;
  selected->clear();
  if (nSamples > 0) {
    selected->reserve(nSamples);
  }
  for (const T& element : iter) {
    n += 1;
    num_t weight = weightFunc(element);
    if (weight > 0.0) {
      if (selected->size() >= nSamples) {
        double r = rand.uniform_float_01();
        double k = 1.0 / weight * std::log(r);
        num_t s = selected->front().second;
        bool swap = k > s;
        if (k == s) {
          swap = rand.uniform_float_01() > 0.5;
        }
        if (swap) {
          // Swap with element from heap
          std::pop_heap(selected->begin(), selected->end(), pair_cmp);
          selected->pop_back();
          // Add new element
          selected->push_back(std::make_pair(element, weight));
          std::push_heap(selected->begin(), selected->end(), pair_cmp);
        }
      } else {
      // Add new element
        selected->push_back(std::make_pair(element, weight));
        std::push_heap(selected->begin(), selected->end(), pair_cmp);
      }
    } 
  }
}

}  // namespace stats
}  // namespace sg


