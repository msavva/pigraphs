#pragma once

#include "libsg.h"  // NOLINT
#include "geo/geo.h"

namespace sg {
namespace core {

//! Provides sampled poses from a given set of baseSkeletons placed within a region defined by a BBox
class SkelPoseSampler {
 public:
  //! Encapsulates parameters of a posed skeleton
  struct PoseParams { size_t iTheta, iPose, iPoint; };

  //! Requires baseSkeletons are normalized (i.e. centered at zero)
  SkelPoseSampler(const geo::BBox2f& bbox, const vec<TransformedSkeleton>& baseSkeletons, int numSample, int numTheta) {
    init(bbox, baseSkeletons, numSample, numTheta);
  }

  //! Sets next posed TransformedSkeleton into tSkel and its PoseParams into params.
  //! Returns true if tSkel was set, or false if out of skeletons
  bool getNext(TransformedSkeleton* tSkel, PoseParams* params);

  //! Resets this SkelPoseSampler to the initial position
  void reset() {
    m_currPoseParams = { 0, 0, 0 };
  }

  //! Returns number of original skeletons used in the poses of this PoseIterator
  size_t numSkeletons() const {
    return m_pSkels->size();
  }

  //! Returns number of angles sampled at each point
  size_t numThetas() const {
    return m_thetas.size();
  }

  //! Returns number of points sampled
  size_t numPoints() const {
    return m_points.size();
  }

  //! Return sample points by const&
  const vec<geo::Vec2f>& getPoints() const {
    return m_points;
  }

  //! Returns theta orientations to be sampled (wrt to theta=0 of sampled base skeleton coord frame)
  const vec<float>& getThetas() const {
    return m_thetas;
  }

  //! Returns total number of sampled poses
  size_t size() const {
    return numThetas() * numSkeletons() * numPoints();
  }

 private:
  //! Initializes given sampling region bbox, set of baseSkeletons and numbers of angular and spatial samples
  void init(const geo::BBox2f& bbox, const vec<TransformedSkeleton>& baseSkeletons, int numSample, int numTheta);

  const vec<TransformedSkeleton>* m_pSkels;  //! Base skeletons to be sampled in order (indexed by iPose)
  geo::BBox2f m_bbox;                        //! Region to be sampled
  vec<geo::Vec2f> m_points;                  //! Sampling points within region
  vec<float> m_thetas;                       //! Orientations to be sampled
  PoseParams m_currPoseParams;               //! Current pose param
};

}  // namespace core
}  // namespace sg


