#include "common.h"  // NOLINT

#include "geo/OBB.h"

#include <functional>
#include <limits>

#include "geo/minRectangle.h"
#include "math/math.h"
#include "io/io.h"
#include "util/util.h"

namespace sg {
namespace geo {

OBB::OBB(const Vec3f& c, const Vec3f& r, const Matrix3f& R)
  : c_(c)
  , r_(r)
  , R_(R) {
  computeTransforms();
}

OBB::OBB(const OBB& obb) 
  : c_(obb.c_)
  , r_(obb.r_)
  , R_(obb.R_)
  , worldToLocal_(obb.worldToLocal_)
  , localToWorld_(obb.localToWorld_) { }

void OBB::init(const VecVec3fIter start, const VecVec3fIter stop, const FitOptFlags opts) {
  const size_t nPoints = stop - start;
  if (nPoints < 3) {
    SG_LOG_WARN << "Not enough points to compute OBB: only " << nPoints;
    return;
  }

  if (opts[CONSTRAIN_Z]) {
    // Get centroid, z range and x-y points for 2D rect fitting
    vec<Vec2f> projPs(nPoints);
    size_t i = 0;
    float big = std::numeric_limits<float>::max();
    Vec3f pMin(big, big, big), pMax(-big, -big, -big);
    for (VecVec3fIter it = start; it != stop; ++it, ++i) {
      const float x = (*it)[0], y = (*it)[1], z = (*it)[2];
      //if (x < pMin[0]) { pMin[0] = x; } else if (x > pMax[0]) { pMax[0] = x; }
      //if (y < pMin[1]) { pMin[1] = y; } else if (y > pMax[1]) { pMax[1] = y; }
      if (z < pMin[2]) { pMin[2] = z; } else if (z > pMax[2]) { pMax[2] = z; }
      projPs[i][0] = x;  projPs[i][1] = y;
    }

    // Find minimum rectangle in x-y plane
    const vec<Vec2f> rectPts = minRectangle2D(projPs);

    // Set x and y bbox axes from 2D rectangle axes
    const Vec2f center2d = (rectPts[0] + rectPts[2]) * 0.5f;
    Vec2f v0 = rectPts[1] - rectPts[0], v1 = rectPts[2] - rectPts[1];
    const float v0norm2 = v0.squaredNorm(), v1norm2 = v1.squaredNorm();
    // Check for degenerate cases
    if (v0norm2 == 0.0) {
      if (v1norm2 == 0.0) {
        // Hmm, both are zero, oh well, let's pick some random dirs
        v0 = Vec2f(1,0); v1 = Vec2f(0,1);
      } else {
        // Set v0 perpendicular to v1
        v0 = Vec2f(v1.y(), -v1.x());
      }
    } else if (v1norm2 == 0.0) {
      // Set v1 perpendicular to v0
      v1 = Vec2f(-v0.y(), v0.x());
    }
    size_t v0idx = (v0norm2 > v1norm2) ? 0 : 1;
    size_t v1idx = (v0idx + 1) % 2;
    const Vec2f v0n = (v0idx == 0) ? v0.normalized() : -v0.normalized(), v1n = v1.normalized();
    R_.col(v0idx) = Vec3f(v0n[0], v0n[1], 0);  r_[v0idx] = sqrtf(v0norm2) * 0.5f;
    R_.col(v1idx) = Vec3f(v1n[0], v1n[1], 0);  r_[v1idx] = sqrtf(v1norm2) * 0.5f;
    R_.col(2) = Vec3f(0, 0, 1);                r_[2] = (pMax[2] - pMin[2]) * 0.5f;
    c_[0] = center2d[0];
    c_[1] = center2d[1];
    c_[2] = (pMin[2] + pMax[2]) * 0.5f;

    assert(c_.allFinite());
    assert(r_.allFinite());
    assert(R_.allFinite());
  } else {  // !opts[CONSTRAIN_Z]
    const CoordSystem coords(start, stop);
    R_ = coords.R();

    if (opts[MIN_PCA]) {
      // Project points into 2D plane formed by the first two eigenvector
      // in R's columns. The plane normal is the last eigenvector
      vec<Vec2f> projPs(nPoints);
      size_t i = 0;
      const auto Mproj = R_.transpose().topLeftCorner<2, 3>();
      for (VecVec3fIter it = start; it != stop; it++, i++) {
        projPs[i] = Mproj * (*it);
      }

      // Find minimum rectangle in that plane
      const vec<Vec2f> rectPts = minRectangle2D(projPs);

      // Set new bbox axes v0 and v1 from 2D rectangle's axes by first getting
      // back their 3D world space coordinates and then ordering by length so
      // that v0 remains largest OBB dimension, followed by v1
      const Vec2f pV0 = rectPts[1] - rectPts[0], pV1 = rectPts[2] - rectPts[1];
      const Vec3f bv0 = Mproj.transpose() * pV0, bv1 = Mproj.transpose() * pV1;
      const float bv0norm = bv0.squaredNorm(), bv1norm = bv1.squaredNorm();
      R_.col(0) = (bv0norm > bv1norm) ? bv0.normalized() : bv1.normalized();
      R_.col(1) = (bv0norm > bv1norm) ? bv1.normalized() : bv0.normalized();
      R_.col(2) = R_.col(0).cross(R_.col(1));
    }  // opts[MIN_PCA]

    // Regularize basis by swapping axes so that Y always points upwards
    R_ = regularizeBasisMatrix(R_, coords.S().diagonal());

    // Find half-extents and centroid by fitting BBox in local space
    Matrix3f Rt = R_.transpose();
    BBox bbox;
    for (VecVec3fIter pIt = start; pIt != stop; pIt++) {
      bbox.extend(Rt * (*pIt));
    }

    // Can now decide if AABB is a better or almost as good fit and revert to it
    if (opts[AABB_FALLBACK]) {
      BBox aabb;
      for (VecVec3fIter p = start; p != stop; p++) { aabb.extend(*p); }
      const float AABBvolume = aabb.volume();
      const float OBBvolume = bbox.volume();
      // Threshold OBB volume multiplier under which to use AABB instead
      const float eps = 1.1f;
      if (AABBvolume < eps * OBBvolume) {
        sg::util::println("Reverting to AABB");

        // Adjust AABB axes for longest-to-shortest ordering
        float dims[3];
        Eigen::Map<Vec3f> m(dims);
        m = aabb.diagonal();
        vec<size_t> ind;
        sg::util::sortIndices(std::begin(dims), std::end(dims), std::less<float>(), ind);
        const Matrix3f I = Matrix3f::Identity();
        const Vec3f x = I.col(ind[2]);
        const Vec3f y = I.col(ind[1]);
        const Vec3f z = I.col(ind[0]);
        R_.col(0) = x;
        R_.col(1) = y;
        R_.col(2) = z;
        const Vec3f sortedDims(m[ind[2]], m[ind[1]], m[ind[0]]);
        R_ = regularizeBasisMatrix(R_, sortedDims);

        // Recompute dimensions with new AABB basis
        bbox.setEmpty();
        Matrix3f Rt2 = R_.transpose();
        for (VecVec3fIter pIt = start; pIt != stop; pIt++) {
          bbox.extend(Rt2 * (*pIt));
        }
      }
    }  // opts[AABB_FALLBACK]

    // Save half-widths and centroid of BBox
    r_ = (bbox.max() - bbox.min()) * 0.5;
    c_ = R_ * (bbox.max() + bbox.min()) * 0.5;
  }  // !opts[CONSTRAIN_Z]

  // Avoid degenerate boxes by enforcing non-zero width along all dimensions
  const float minWidth = 1e-6f;
  for (size_t i = 0; i < 3; i++) {
    if (r_[i] < minWidth) { r_[i] = minWidth; }
  }
  computeTransforms();
}

// Helper function to compute world to local and local to world transforms
void OBB::computeTransforms() {
  assert(c_.allFinite());
  assert(r_.allFinite());
  assert(R_.allFinite());
  // Local-to-world transform
  for (size_t i = 0; i < 3; i++) {
    localToWorld_.linear().col(i) = R_.col(i) * r_[i];
  }
  localToWorld_.translation() = c_;

  // World-to-local transform. Points within OBB are in [0,1]^3
  for (size_t i = 0; i < 3; i++) {
    worldToLocal_.linear().row(i) = R_.col(i) * (1.0f / r_[i]);
  }
  worldToLocal_.translation() = - worldToLocal_.linear() * c_;
}

// Returns distance from bbox to p. Returns 0 if point p is inside box
float OBB::distance(const Vec3f& p) const {
  if (contains(p)) { return 0; }
  const Vec3f closest = closestPoint(p);
  return (p - closest).norm();
}

bool OBB::contains(const Vec3f& p, float eps /* = 0.01f */) const {
  const Vec3f pLocal = worldToLocal() * p;
  const float bound = 1.0f + eps;
  for (size_t i = 0; i < 3; i++) { if (abs(pLocal[i]) > bound) { return false; } }
  return true;  // Here only if all three coords within bounds
}

Vec3f OBB::closestPoint(const Vec3f& p) const {
  const Vec3f d = p - c_;
  Vec3f closest = c_;
  for (size_t i = 0; i < 3; i++) {
    closest += sg::math::clamp(R_.col(i).dot(d), -r_[i], r_[i]) * R_.col(i);
  }
  return closest;
}

Vec3f OBB::closestSurfacePoint(const Vec3f& p) const {
  const Vec3f pLocal = worldToLocal() * p;
  float maxDim = -1.f;
  for (int i = 0; i < 3; i++) {
    if (pLocal[i] > maxDim) {
      maxDim = pLocal[i];
    }
  }
  const float scaleToSurfRatio = 1.f / maxDim;
  const Vec3f pLocalSurf = pLocal * scaleToSurfRatio;
  const Vec3f pWorldSurf = localToWorld() * pLocalSurf;
  return pWorldSurf;
}

Vec3f OBB::sample() const {
  Vec3f p = c_;
  for (int i = 0; i < 3; ++i) {
    const float r = math::DEF_RAND.uniform_float_01() * 2.f - 1.f;
    p += r * r_[i] * R_.col(i);
  }
  return p;
}

BBox OBB::toAABB() const {
  // TODO: Use matrix
  Vec3f points[8];
  points[0] = Vec3f(-1, -1, -1);
  points[1] = Vec3f(-1, -1, +1);
  points[2] = Vec3f(-1, +1, -1);
  points[3] = Vec3f(-1, +1, +1);
  points[4] = Vec3f(+1, -1, -1);
  points[5] = Vec3f(+1, -1, +1);
  points[6] = Vec3f(+1, +1, -1);
  points[7] = Vec3f(+1, +1, +1);
  BBox bbox;
  for (int i = 0; i < 8; i++) {
    const Vec3f worldPt = c_ + (R_ * (points[i].cwiseProduct(r_)));
    bbox.extend(worldPt);
  }
  return bbox;
}

Matrix3f OBB::worldAxes() const { return localToWorld_.linear(); }

ostream& OBB::toJSON(ostream& os, bool endlines) const {
  const std::function<void(void)> sep = io::put(os, ",", endlines);
  const auto key = [] (const string & id) { return "\"" + id + "\": "; };

  os << "{";
  if (endlines) { os << endl; }
  os << key("centroid"); geo::toJSON(os, centroid()); sep();
  os << key("axesLengths"); geo::toJSON(os, axesLengths()); sep();
  os << key("normalizedAxes"); geo::toJSON(os, normalizedAxes());
  os << "}";
  if (endlines) { os << endl; }
  return os;
}

sg::geo::Vec3f OBB::dominantNormal() const {
  // Pick the R col with the smallest length
  int smallestIdx = 2;
  if (r_[1] < r_[smallestIdx]) smallestIdx = 1;
  if (r_[0] < r_[smallestIdx]) smallestIdx = 0;
  return R_.col(smallestIdx);
}

ostream& operator<<(ostream& os, const OBB& obb) {
  os << "OBB: {c:"; toJSON(os, obb.c_) << ", r:"; toJSON(os, obb.r_) << ", ";
  for (size_t i = 0; i < 3; i++) {
    os << "R" << i << ":" << obb.R_.col(i).format(kEigenJSONFormat);
    os << ((i == 2) ? "}" : ", ");
  }
  return os;
}

Matrix3f flipBasisMatrix(const Matrix3f& R, size_t iRef) {
  Matrix3f R2 = R;

  // This is a reflection, not a rotation so flip one axis
  if (R2.determinant() < 0) {
    R2.col(0) = -R2.col(0);
  }

  size_t nFlips = 0;
  const Vec3f dot = R2.row(iRef);
  float maxAbsDot = -std::numeric_limits<float>::max();
  size_t maxAbsDotIdx;
  for (size_t i = 0; i < 3; i++) {
    const float d = abs(dot[i]);
    if (d > maxAbsDot) { maxAbsDot = d; maxAbsDotIdx = i; }
  }

  if (dot[maxAbsDotIdx] < 0) {
    R2.col(maxAbsDotIdx) = -R2.col(maxAbsDotIdx);
    nFlips++;
  }

  // Perform same for remaining 2 axes: flip one to ensure positive dot
  // product with remaining 2D dimension which is nearest collinearity
  const size_t i2 = (maxAbsDotIdx + 2) % 3;
  const size_t iRef1 = (iRef + 1) % 3;
  const size_t iRef2 = (iRef + 2) % 3;
  const Vec3f v2 = R2.col(i2);
  const size_t maxAbsDotIdx2 = (abs(v2[iRef1]) > abs(v2[iRef2])) ? iRef1 : iRef2;
  if (v2[maxAbsDotIdx2] < 0) {
    R2.col(i2) = -R2.col(i2);
    nFlips++;
  }

  // Now adjust remaining axis to preserve right-handedness (needed if there
  // was only one flip)
  if (nFlips == 1) {
    const size_t i1 = (maxAbsDotIdx + 1) % 3;
    R2.col(i1) = -R2.col(i1);
  }
  return R2;
}

Matrix3f regularizeBasisMatrix(const Matrix3f& R, const Vec3f& diag,
                               size_t iRef /* = 1 */) {
  Matrix3f R2 = reorientBasisMatrix(R, diag);
  return flipBasisMatrix(R2, iRef);
}

Matrix3f reorientBasisMatrix(const Matrix3f& R, const Vec3f& diag) {
  Matrix3f R2 = R;
  //cout << "Reorienting approximately equal axes" << endl;

  // Sort by x, then by y and finally by z
  const auto vec3fless = [](const Vec3f & l0, const Vec3f & r0) {
    const Vec3f l = l0.cwiseAbs();
    const Vec3f r = r0.cwiseAbs();
    if (l[0] == r[0]) {
      if (l[1] == r[1]) {
        return l[2] < r[2];
      } else { return l[1] < r[1]; }
    } else { return l[0] < r[0]; }
  };

  const float percThresh = 0.1f;

  // all three the same
  if (abs(diag[0] - diag[2]) / std::max(diag[0], diag[2]) < percThresh) {
    if (vec3fless(R2.col(0), R2.col(1))) {
      R2.col(0).swap(R2.col(1));
    }
    if (vec3fless(R2.col(1), R2.col(2))) {
      R2.col(1).swap(R2.col(2));
    }
    // 0 and 1 same
  } else if (abs(diag[0] - diag[1]) / std::max(diag[0], diag[1]) < percThresh) {
    if (vec3fless(R2.col(0), R2.col(1))) {
      R2.col(0).swap(R2.col(1));
    }
    // 1 and 2 same
  } else if (abs(diag[1] - diag[2]) / std::max(diag[1], diag[2]) < percThresh) {
    if (vec3fless(R2.col(1), R2.col(2))) {
      R2.col(1).swap(R2.col(2));
    }
  }

  return R2;
}

void pointSetToOBB(const Matrix3Xf& P, OBB* obb, const Transform* xform) {
  assert(obb != nullptr);
  const Matrix3Xf Pxformed = (xform != nullptr) ? (*xform) * P : P;
  const size_t numPts = Pxformed.cols();
  vec<Vec3f> pts(numPts);
  for (size_t iPt = 0; iPt < numPts; ++iPt) {
    pts[iPt] = Pxformed.col(iPt);
  }
  obb->init(pts.begin(), pts.end(), OBB::CONSTRAIN_Z);
}

size_t numPointsInsideOBBs(const VecVec3f& pts, const vec<OBB>& obbs) {
  size_t insidePts = 0;
  const size_t numPts = pts.size();
  if (numPts == 0) {
    return 0;
  }
  for (size_t i = 0; i < numPts; ++i) {
    for (const OBB& obb : obbs) {
      if (obb.contains(pts[i])) {
        ++insidePts;
        break;
      }
    }
  }
  return insidePts;
}

//! Returns number of points contained by OBBs
size_t numPointsInsideOBBs(const Matrix3Xf& pts, const vec<OBB>& obbs) {
  size_t insidePts = 0;
  const size_t numPts = pts.cols();
  if (numPts == 0) {
    return 0;
  }
  for (size_t i = 0; i < numPts; ++i) {
    for (const OBB& obb : obbs) {
      if (obb.contains(pts.col(i))) {
        ++insidePts;
        break;
      }
    }
  }
  return insidePts;
}

size_t sampleOBBOverlapRaw(const OBB& obbA, const OBB& obbB, int kSamples) {
  // Estimate overlap between obbA and obbB by sampling kSamples from obbA
  size_t totalOverlap = 0;
  for (int i = 0; i < kSamples; ++i) {
    const Vec3f s = obbA.sample();
    if (obbB.contains(s)) {
      totalOverlap++;
    }
  }
  return totalOverlap;
}

float sampleOBBOverlap(const OBB& obbA, const OBB& obbB, int kSamples) {
  size_t overlap = sampleOBBOverlapRaw(obbA, obbB, kSamples);
  return static_cast<float>(overlap) / kSamples;
}


}  // namespace geo
}  // namespace sg
