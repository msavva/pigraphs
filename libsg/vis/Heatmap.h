#pragma once

#include "libsg.h"  // NOLINT

#include <boost/multi_array.hpp>

#include "geo/geo.h"

namespace sg {
namespace vis {

// How to aggregate over a given heatmap sampling dimension
enum Aggregation { kMax = 0, kMean = 1 };

class PoseHeatMap {
 public:
  struct Point {
    Point(int _numSkels, int _numThetas) : likelihoods(boost::extents[_numSkels][_numThetas]) { }

    //! Returns aggregated likelihood for this Point with given aggregations over skeletons and angles respectively
    double likelihood(Aggregation skelAggr = kMean, Aggregation thetaAggr = kMax) const;

    //! Returns aggregated likelihood for given theta index
    double likelihoodByTheta(int iTheta, Aggregation skelAggr = kMean) const;

    //! Returns maximum likelihood and sets iSkelMax and iThetaMax to be skel and theta indices of max
    double maxLikelihood(int* iSkelMax, int* iThetaMax) const;

    //! position of this sample
    geo::Vec2f pos;

    //! likelihood as a function of skeleton AND theta; access using: likelihoods[iSkel][iTheta]
    typedef boost::multi_array<double, 2> Grid;
    Grid likelihoods;
  };

  PoseHeatMap() : m_skeletons(), m_thetas(), m_points() { }

  //! Set skeletons associated with heat map
  void setSkeletons(const vec<sg::core::TransformedSkeleton>& skels) { m_skeletons = skels; }

  //! Adds the given sample points and corresponding theta values sampled at each point to this PoseHeatMap
  void addPoints(const vec<Point>& points, const vec<float>& thetas);

  //! Clears points
  void clear() { m_points.clear(); }

  //! Convenience struct for returning specific sample
  struct Sample { 
    std::reference_wrapper<const Point> entry;
    int iSkeleton; int iTheta; double likelihood;

    const geo::Vec2f& pos() const { 
      return entry.get().pos;
    };
  };

  //! Return aggregated samples in pSamples using aggregation strategies skellAggr and thetaAggr
  void getSamples(vec<Sample>* pSamples, Aggregation skelAggr = kMean, Aggregation thetaAggr = kMax) const;
  
  //! Bins samples in this Heatmap's domain into bins of dimension binSize x binSize and returns BBoxes of bins
  //! with sum likelihood within sumLikeRatioThresh of the bin with maximum sum likelihood
  vec<geo::BBox2f> topLikelihoodSampleBins(float binSize, float sumLikeRatioThresh = 0.5f);

  //! Returns the sample with max likelihood
  Sample PoseHeatMap::maxLikelihoodSample() const;

  //! Populate pOut with the top K samples (at K different points)
  void topKSamples(int k, vec<Sample>* pOut) const;

  //! Returns the number of skeletons sampled at each point
  int numSkeletons() const { return static_cast<int>(m_skeletons.size()); }

  //! Returns the number of theta orientations sampled at each point
  int numThetaDivisions() const { return static_cast<int>(m_thetas.size()); }

  //! Returns the data vector containing the Points in this Heatmap
  vec<Point>& points() { return m_points; }
  const vec<Point>& points() const { return m_points; }

  //! Return the sampled theta values (maps from iTheta index in Points to sampled theta value in radians)
  const vec<float>& thetas() const { return m_thetas; }

  //! Return the top k posed skeletons
  int getTopKSkels(vec<sg::core::TransformedSkeleton>* pPosedSkels, int k) const;

 private:
  template <typename VecT>
  static double aggregate(const VecT& v, Aggregation aggrType) {
    assert(v.size() > 0);
    if (aggrType == Aggregation::kMean) {
      return math::mean(v);
    } else if (aggrType == Aggregation::kMax) {
      return *std::max_element(v.begin(), v.end());
    } else {
      cerr << "[PoseHeatMap] Unknown aggregation type: " << aggrType << endl;
      return 0;
    }
  }

  vec<sg::core::TransformedSkeleton> m_skeletons;
  vec<float> m_thetas;
  vec<Point> m_points;
};


}  // namespace vis
}  // namespace sg


