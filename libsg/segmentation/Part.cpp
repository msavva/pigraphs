#include "common.h"  // NOLINT

#include "segmentation/Part.h"

#include "geo/OBB.h"

namespace sg {
namespace segmentation {

typedef sg::geo::OBB OBB;
typedef sg::geo::Vec3f Vec3f;

float Part::diagonalLength() const {
  if (m_obb == nullptr) { return 0.f; }
  return m_obb->diagonalLength();
}

template <>
float Part::distance<Vec3f>(const Vec3f& v) const {
  MLIB_ASSERT_STR(m_obb != nullptr, "tried to query Part with no OBB");
  return m_obb->distance(v);
}

template <>
float Part::distance(const ml::vec3f& v) const {
  MLIB_ASSERT_STR(m_obb != nullptr, "tried to query Part with no OBB");
  const Vec3f v2(v[0], v[1], v[2]);
  return m_obb->distance(v2);
}

template <>
Vec3f Part::closestPoint(const Vec3f& v) const {
  MLIB_ASSERT_STR(m_obb != nullptr, "tried to query Part with no OBB");
  return m_obb->closestPoint(v);
}

template <>
ml::vec3f Part::closestPoint(const ml::vec3f& v) const {
  MLIB_ASSERT_STR(m_obb != nullptr, "tried to query Part with no OBB");
  const Vec3f v2(v[0], v[1], v[2]);
  const Vec3f& vC = m_obb->closestPoint(v2);
  return ml::vec3f(vC[0], vC[1], vC[2]);
}

template <>
Vec3f Part::closestSurfacePoint(const Vec3f& v) const {
  MLIB_ASSERT_STR(m_obb != nullptr, "tried to query Part with no OBB");
  return m_obb->closestSurfacePoint(v);
}

template <>
ml::vec3f Part::closestSurfacePoint(const ml::vec3f& v) const {
  MLIB_ASSERT_STR(m_obb != nullptr, "tried to query Part with no OBB");
  const Vec3f v2(v[0], v[1], v[2]);
  const Vec3f& vC = m_obb->closestSurfacePoint(v2);
  return ml::vec3f(vC[0], vC[1], vC[2]);
}


template <>
Vec3f Part::centroid<Vec3f>() const {
  MLIB_ASSERT_STR(m_obb != nullptr, "tried to query Part with no OBB");
  return m_obb->centroid();
}

template <>
ml::vec3f Part::centroid<ml::vec3f>() const {
  MLIB_ASSERT_STR(m_obb != nullptr, "tried to query Part with no OBB");
  const Vec3f& c = m_obb->centroid();
  return ml::vec3f(c[0], c[1], c[2]);
}


template <>
Vec3f Part::axesLengths<Vec3f>() const {
  MLIB_ASSERT_STR(m_obb != nullptr, "tried to query Part with no OBB");
  return m_obb->axesLengths();
}

template <>
ml::vec3f Part::axesLengths<ml::vec3f>() const {
  MLIB_ASSERT_STR(m_obb != nullptr, "tried to query Part with no OBB");
  const Vec3f& l = m_obb->axesLengths();
  return ml::vec3f(l[0], l[1], l[2]);
}


template <>
void Part::normalizedAxes(arr<ml::vec3f, 3>* m) const {
  MLIB_ASSERT_STR(m_obb != nullptr, "tried to query Part with no OBB");
  const sg::geo::Matrix3f& R = m_obb->normalizedAxes();
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      (*m)[i][j] = R.col(i)[j];
    }
  }
}

ml::mat4f Part::localToWorld() const {
  MLIB_ASSERT_STR(m_obb != nullptr, "tried to query Part with no OBB");
  const sg::geo::Transform& T = m_obb->localToWorld();
  return sg::geo::to<ml::mat4f>(T);
}

void Part::computeDominantNormal() {
  const auto obbDomNormal = m_obb->dominantNormal();
  for (int i = 0; i < 3; i++) {
    m_dominantNormal[i] = obbDomNormal(i);
  }
}

}  // namespace segmentation
}  // namespace sg
