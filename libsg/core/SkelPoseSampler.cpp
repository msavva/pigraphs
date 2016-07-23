#include "common.h"  // NOLINT

#include "core/SkelPoseSampler.h"

#include "core/Skeleton.h"
#include "stats/sampler.h"

namespace sg {
namespace core {

void SkelPoseSampler::init(const geo::BBox2f& bbox, const vec<TransformedSkeleton>& baseSkeletons,
                       int numSamples, int numThetas) {
  m_pSkels = &baseSkeletons;
  m_bbox = bbox;
  m_currPoseParams = {0, 0, 0};

  // Populate sample points
  const geo::Vec2f bboxMin = bbox.min(),
                   bboxExtent = bbox.sizes();
  m_points.resize(numSamples, bboxMin);
  stats::LDSampler ldSampler(numSamples, (numSamples > 100) ? 100 : 1);
  for (auto& p : m_points) {
    const auto xySample = ldSampler.next2D();
    p[0] += xySample[0] * bboxExtent[0];
    p[1] += xySample[1] * bboxExtent[1];
  }
  // Populate sample thetas
  m_thetas.resize(numThetas);
  const float thetaStep = math::constants::PI2f / numThetas;
  for (int iTheta = 0; iTheta < numThetas; ++iTheta) {
  	m_thetas[iTheta] = iTheta * thetaStep;
  }
}

bool SkelPoseSampler::getNext(TransformedSkeleton* tSkel, PoseParams* params) {
  auto& p = m_currPoseParams;
  if (p.iPoint == m_points.size()) { return false; }  // Bail out if we've reached the end

  // Construct and set current skeleton
  assert(tSkel != nullptr);
  const auto& point = m_points[p.iPoint];
  const ml::vec3f translation(point[0], point[1], 0.0f);
  const float rotation = math::rad2deg(m_thetas[p.iTheta]);
  // TODO(MS): Store translation and theta into TransformedSkeleton instead of computing transforms all the time
  *tSkel = TransformedSkeleton((*m_pSkels)[p.iPose],
                               ml::mat4f::translation(translation) * ml::mat4f::rotationZ(rotation));
  // Also set params
  *params = m_currPoseParams;

  // Increment params; order from inner loop to outer loop is: iTheta, iPose, iPoint
  p.iTheta++;
  if (p.iTheta == numThetas()) {
    p.iTheta = 0;  p.iPose++;
    if (p.iPose == numSkeletons()) {
      p.iPose = 0;  p.iPoint++;
    }
  }
  return true;
}

}  // namespace core
}  // namespace sg
