#include "common.h"  // NOLINT

#include "vis/Heatmap.h"

#include "core/Skeleton.h"
#include "math/math.h"

namespace sg {
namespace vis {

PoseHeatMap::Sample PoseHeatMap::maxLikelihoodSample() const {
  size_t iEntryMax = 0; int iSkelMax = 0; int iThetaMax = 0; double maxLike = -1.0;
  for (size_t i = 0; i < m_points.size(); ++i) {
    const auto& e = m_points[i];
    int maxSkelIdx, maxThetaIdx;
    const double currLike = e.maxLikelihood(&maxSkelIdx, &maxThetaIdx);
    if (currLike > maxLike) {
      iEntryMax = i;
      iSkelMax = maxSkelIdx;
      iThetaMax = maxThetaIdx;
      maxLike = currLike;
    }
  }
  return { m_points[iEntryMax], iSkelMax, iThetaMax, maxLike };
}

template<class Pred = std::less<double>>
struct sort_sample_likelihood {
  bool operator()(const PoseHeatMap::Sample&left, const PoseHeatMap::Sample&right) {
    return p(left.likelihood, right.likelihood);
  }
private:
  Pred p;
};
static sort_sample_likelihood<std::less<double>> sort_sample_likelihood_asc;
static sort_sample_likelihood<std::greater<double>> sort_sample_likelihood_dsc;

void PoseHeatMap::topKSamples(int k, vec<Sample>* pOut) const {
  vec<Sample>& topK = *pOut;
  topK.clear();
  topK.reserve(k);
  for (size_t i = 0; i < m_points.size(); ++i) {
    const auto& e = m_points[i];
    int maxSkelIdx, maxThetaIdx;
    const double currLike = e.maxLikelihood(&maxSkelIdx, &maxThetaIdx);
    if (topK.size() > k) {
      // Check if this element is better that our worst element
      // Top of the heap is the min element
      const Sample& sample = topK.front();
      if (currLike > sample.likelihood) {
        // Remove the min element
        std::pop_heap(topK.begin(), topK.end(), sort_sample_likelihood_dsc);
        topK.pop_back();
        // Add new element
        topK.push_back({m_points[i], maxSkelIdx, maxThetaIdx, currLike});
        std::push_heap(topK.begin(), topK.end(), sort_sample_likelihood_dsc);
      }
    } else {
      // Add new element
      topK.push_back({m_points[i], maxSkelIdx, maxThetaIdx, currLike});
      std::push_heap(topK.begin(), topK.end(), sort_sample_likelihood_dsc);
    }
  }
  std::sort_heap(topK.begin(), topK.end(), sort_sample_likelihood_dsc);
}

void PoseHeatMap::addPoints(const vec<Point>& points, const vec<float>& thetas) {
  if (points.size() == 0) { return; }
  const auto sizes = points[0].likelihoods.shape();
  const int numSkels = static_cast<int>(sizes[0]), numThetas = static_cast<int>(sizes[1]);
  assert(m_skeletons.size() == numSkels);
  if (m_thetas.size() > 0) {  // Ensure sampled thetas identical, otherwise new points incompatible with existing ones
    assert(thetas.size() == m_thetas.size());
    for (int i = 0; i < thetas.size(); ++i) {
      assert(thetas[i] == m_thetas[i]);
    }
  } else {  // Set sampled thetas
    m_thetas = thetas;
  }
  m_points.insert(m_points.end(), points.begin(), points.end());
}

void PoseHeatMap::getSamples(vec<Sample>* pSamples, Aggregation skelAggr /*= kMean*/,
                         Aggregation thetaAggr /*= kMax*/) const {
  const size_t numSamples = m_points.size();
  pSamples->reserve(numSamples);
  for (size_t iPoint = 0; iPoint < numSamples; ++iPoint) {
    const double likelihood = m_points[iPoint].likelihood(skelAggr, thetaAggr);
    pSamples->emplace_back(Sample{m_points[iPoint], -1, -1, likelihood});
  }
}

vec<geo::BBox2f> PoseHeatMap::topLikelihoodSampleBins(float binSize, float sumLikeRatioThresh /*= 0.5f*/) {
  // Retrieve aggregated heatmap samples
  vec<Sample> samples;
  // Aggregate across skeletons by averaging and aggregate across thetas by taking the max
  getSamples(&samples, Aggregation::kMean, Aggregation::kMax);

  // Compute heatmap domain bbox
  geo::BBox2f bbox;
  for (const auto& s : samples) {
    bbox.extend(s.pos());
  }
  SG_LOG_INFO << "min: " << bbox.min().format(geo::kEigenJSONFormat)
              << "max: " << bbox.max().format(geo::kEigenJSONFormat);

  // Bin index functions
  const auto  bboxLengths = bbox.sizes();
  const float invBinSize  = 1.f / binSize,
              minX        = bbox.min().x(),
              minY        = bbox.min().y();
  const int   numBinsX    = static_cast<int>(std::ceil(bboxLengths.x() * invBinSize)),
              numBinsY    = static_cast<int>(std::ceil(bboxLengths.y() * invBinSize)),
              numBins     = numBinsX * numBinsY;
  const auto  binIndex = [&] (const geo::Vec2f& p) {
    int xIdx = static_cast<int>(std::floor((p.x() - minX) * invBinSize)),
        yIdx = static_cast<int>(std::floor((p.y() - minY) * invBinSize));
    // Include upper boundaries since we are a closed range on both ends
    if (xIdx == numBinsX) { xIdx--; }
    if (yIdx == numBinsY) { yIdx--; }
    assert(xIdx >= 0 && xIdx < numBinsX && yIdx >= 0 && yIdx < numBinsY);
    return std::make_pair(xIdx, yIdx);
  };

  // Initialize heatmap bins
  struct Bin { int xIdx, yIdx; double sumLikelihood; };
  vec<Bin> bins(numBins);
  for (int iX = 0; iX < numBinsX; ++iX) {
    for (int iY = 0; iY < numBinsY; ++iY) {
      Bin& bin = bins[iY * numBinsX + iX];
      bin = {iX, iY, 0};
    }
  }

  // Distribute samples into corresponding bins
  for (const auto& s : samples) {
    int xIdx, yIdx;
    std::tie(xIdx, yIdx) = binIndex(s.pos());
    Bin& bin = bins[yIdx * numBinsX + xIdx];
    bin.sumLikelihood += s.likelihood;
  }

  // Sort bins by total likelihood
  sort(bins.begin(), bins.end(), [] (const Bin& l, const Bin& r) { return l.sumLikelihood > r.sumLikelihood; });

  // Pick bins within threshold ratio of max summed likelihood
  const double threshSumLike = sumLikeRatioThresh * bins[0].sumLikelihood;
  int iTop = 0;
  for (; iTop < numBins; ++iTop) {
    const Bin& b = bins[iTop];
    if (b.sumLikelihood < threshSumLike) { break; }
  }

  // Return bin bboxen
  vec<geo::BBox2f> bboxen(iTop);
  for (int i = 0; i < iTop; ++i) {
    const Bin& b = bins[i];
    const geo::Vec2f minPt(b.xIdx * binSize, b.yIdx * binSize),
                     maxPt(minPt.x() + binSize, minPt.y() + binSize);
    bboxen[i].extend(minPt);
    bboxen[i].extend(maxPt);
    SG_LOG_INFO << i << "\t" << ((minPt + maxPt) * .5f).format(geo::kEigenJSONFormat) 
                << "->" << b.sumLikelihood;
  }
  return bboxen;
}

double PoseHeatMap::Point::likelihood(Aggregation skelAggr /*= kMean*/, Aggregation thetaAggr /*= kMax*/) const {
  typedef boost::multi_array_types::index_range range;
  Grid::index_gen indices;
  const size_t numSkels = likelihoods.shape()[0], numThetas = likelihoods.shape()[1];
  vecd skeletonValues(numSkels);
  for (size_t iSkel = 0; iSkel < numSkels; ++iSkel) {
    skeletonValues[iSkel] = PoseHeatMap::aggregate(likelihoods[iSkel], thetaAggr);
  }
  return PoseHeatMap::aggregate(skeletonValues, skelAggr);
}

double PoseHeatMap::Point::likelihoodByTheta(int iTheta, Aggregation skelAggr /*= kMean*/) const {
  typedef boost::multi_array_types::index_range range;
  Grid::index_gen indices;
  const size_t numSkels = likelihoods.shape()[0];
  const auto& skeletonValues = likelihoods[ indices[range(0, numSkels)][iTheta] ];
  return PoseHeatMap::aggregate(skeletonValues, skelAggr);
}

double PoseHeatMap::Point::maxLikelihood(int* piSkelMax, int* piThetaMax) const {
  size_t iSkelMax = 0, iThetaMax = 0;
  double maxLike = likelihoods[iSkelMax][iThetaMax];
  const size_t numSkels = likelihoods.shape()[0], numThetas = likelihoods.shape()[1];
  for (size_t iSkel = 0; iSkel < numSkels; iSkel++) {
    for (size_t iTheta = 0; iTheta < numThetas; iTheta++) {
      const double currLike = likelihoods[iSkel][iTheta];
      if (currLike > maxLike) {
        maxLike = currLike;
        iSkelMax = iSkel;
        iThetaMax = iTheta;
      }
    }
  }
  if (piSkelMax  != nullptr) { *piSkelMax  = static_cast<int>(iSkelMax);  }
  if (piThetaMax != nullptr) { *piThetaMax = static_cast<int>(iThetaMax); }
  return maxLike;
}

int PoseHeatMap::getTopKSkels(vec<sg::core::TransformedSkeleton>* pPosedSkels, int k) const {
  // Get top K samples from heatmap
  const vec<float>& thetas = m_thetas;
  vec<vis::PoseHeatMap::Sample> samples;
  topKSamples(k, &samples);
  // Convert samples into transformed skeletons
  pPosedSkels->resize(samples.size());
  for (int i = 0; i < samples.size(); ++i) {
    const vis::PoseHeatMap::Sample& sample = samples[i];
    const geo::Vec2f& p = sample.pos();
    const float theta = thetas[sample.iTheta];
    (*pPosedSkels)[i] = sg::core::TransformedSkeleton(m_skeletons[sample.iSkeleton],
                                            ml::mat4f::translation(ml::vec3f(p.x(), p.y(), 0.0f))
                                          * ml::mat4f::rotationZ(math::rad2deg(theta)));
    (*pPosedSkels)[i].score = sample.likelihood;
  }
  return static_cast<int>(samples.size());
}


}  // namespace vis
}  // namespace sg
