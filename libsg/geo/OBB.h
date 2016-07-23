#pragma once

#include "libsg.h"  // NOLINT
#include "geo/geo.h"
#include "util/FlagSet.h"

namespace sg {
namespace geo {

// Oriented Bounding Box
class OBB {
 public:
  // This is required to make sure pointers to member objects that are Eigen
  // classes are 16-bit aligned (allows SIMD vectorization). Refer to:
  // http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OBB() { }
  OBB(const Vec3f& c, const Vec3f& r, const Matrix3f& R);
  OBB(const OBB& obb);

  //! Options for Oriented Bounding Box fitting
  enum FitOpts {
    PCA               = 1 << 0,  // OBB axes determined through PCA decomposition of point set
    MIN_PCA           = 1 << 1,  // Use minimum PCA axis and determine other axes through 2D project + min rectangle fit
    AABB_FALLBACK     = 1 << 2,  // Fall back to AABB if volume of AABB is within 10% of OBB
    CONSTRAIN_Z       = 1 << 3,  // Constrain OBB z axis to be canonical z axis
    DEFAULT_OPTS      = (MIN_PCA | AABB_FALLBACK)
  };
  typedef sg::util::FlagSet<FitOpts> FitOptFlags;

  //! Fit to points, construct world-to-local and inverse transforms
  OBB(const VecVec3fIter start, const VecVec3fIter stop, const FitOptFlags opts = DEFAULT_OPTS) {
    init(start, stop, opts);
  }

  template <typename Container>
  explicit OBB(const Container& c, const FitOptFlags opts = DEFAULT_OPTS) {
    init(std::begin(c), std::end(c), opts);
  }

  template <typename Vec3Iter>
  OBB(Vec3Iter start, Vec3Iter stop, const FitOptFlags opts = DEFAULT_OPTS) {
    init(start, stop, opts);
  }

  //! Rescale this OBB by scaling each axis length by the multiplier coefficient given in s
  template <typename Vec3>
  void rescale(const Vec3& s) {
    r_[0] *= s[0];
    r_[1] *= s[1];
    r_[2] *= s[2];
    computeTransforms();
  }

  const Transform& worldToLocal() const { return worldToLocal_; }
  const Transform& localToWorld() const { return localToWorld_; }
  Vec3f axesLengths()  const { return 2.0f * r_; }
  float diagonalLength() const { return axesLengths().norm(); }
  const Vec3f& centroid() const { return c_; }
  float volume() const { return axesLengths().prod(); }

  //! Returns dominant normal of OBB
  Vec3f dominantNormal() const;

  //! Return a point sampled uniformly at random from inside this OBB
  Vec3f sample() const;

  //! Returns distance to p from closest point on OBB surface
  //! (0 if point p is inside box)
  float distance(const Vec3f& p) const;

  //! Returns whether world coordinate point p is contained in this OBB
  bool contains(const Vec3f& p, float eps = 0.01f) const;

  //! Return closest point to p within OBB.  If p is inside return p.
  Vec3f closestPoint(const Vec3f& p) const;

  //! Return closest point to p on OBB surface.
  Vec3f closestSurfacePoint(const Vec3f& p) const;

  //! Return a 3x3 matrix containing world-space axes of this OBB as columns
  Matrix3f worldAxes() const;

  //! Return a 3x3 matrix containing normalized world-space axes as columns
  Matrix3f normalizedAxes() const { return R_; }

  //! Returns an axis aligned bounding box bounding this OBB
  BBox toAABB() const;

  //! Outputs this OBB to passed ostream in JSON format and returns the ostream
  ostream& toJSON(ostream&, bool endlines) const;

  //! Stream print out for OBB
  friend ostream& operator<<(ostream& os, const OBB& obb);

  //! Initializes this OBB given iterators to start and end of Vec3-like point range, and fit options in opts
  template <typename Vec3Iter>
  void init(Vec3Iter start, Vec3Iter stop, const FitOptFlags opts) {
    const size_t nPoints = std::distance(start, stop);
    vec<Vec3f> vecVec3f(nPoints);
    size_t i = 0;
    for (Vec3Iter it = start; it != stop; ++it, ++i) {
      vecVec3f[i] = vec3f(*it);
    }
    init(vecVec3f.begin(), vecVec3f.end(), opts);
  }

  //! Initializes this OBB given iterators to start and end of VecVec3f range, and fit options in opts
  void init(const VecVec3fIter start, const VecVec3fIter stop, const FitOptFlags opts);

 private:
  void computeTransforms();

  Vec3f c_;     // Centroid
  Vec3f r_;     // Half-length along each local axis
  Matrix3f R_;  // Rotation matrix normalized world space axis vectors
  Transform worldToLocal_, localToWorld_;
};

//! Write JSON representation of OBB
ostream& toJSON(ostream& os, const geo::OBB& o, bool endlines);

// Check whether basis matrix needs to be re-oriented for tie breaking
// approximately equal length PCA coordinate frames ensuring axes are close to
// global frame axes.  Also flip matrix axes so that they face global frame
Matrix3f regularizeBasisMatrix(const Matrix3f& R, const Vec3f& diag,
                               size_t iRef = 0);

// Given a coordinate frame matrix R with columns representing a set of axes
// "regularize" it by first making it a right-handed system.
// Then choose axis directions such that the dot product of the closest axis to
// to the canonical coordinate indexed by iRef is positive (in same direction).
// This usually useful to ensure a global notion of "up" by making vector close
// to Y point towards positive Y.
// Do the same for the remaining 2 axes while preserving handedness to
// establish a left-right convention.
Matrix3f flipBasisMatrix(const Matrix3f& R, size_t iRef);

Matrix3f reorientBasisMatrix(const Matrix3f& R, const Vec3f& diag);

// Given points in P and optional xform computer OBB around all
void pointSetToOBB(const Matrix3Xf& P, OBB* obb,
                   const Transform* xform = nullptr);

// Convert to mlib OBB
template<typename OBB3f, typename Vec3>
inline OBB3f to(const OBB& obb) {
  const Vec3f c = obb.localToWorld()*Vec3f(-1, -1, -1);
  const Matrix3f axes = obb.worldAxes();
  const Vec3f x = axes.col(0)*2;
  const Vec3f y = axes.col(1)*2;
  const Vec3f z = axes.col(2)*2;
  return OBB3f(to<Vec3>(c),
               to<Vec3>(x),
               to<Vec3>(y),
               to<Vec3>(z));
}

//! Returns number of points contained by OBBs
size_t numPointsInsideOBBs(const VecVec3f& pts, const vec<OBB>& obbs);

//! Returns number of points contained by OBBs
size_t numPointsInsideOBBs(const Matrix3Xf& pts, const vec<OBB>& obbs);

//! Estimate overlap between obbA and obbB by sampling kSamples from obbA
size_t sampleOBBOverlapRaw(const OBB& obbA, const OBB& obbB, int kSamples);

//! Estimate overlap between obbA and obbB by sampling kSamples from obbA
//! and normalizing by the number of samples
float sampleOBBOverlap(const OBB& obbA, const OBB& obbB, int kSamples);

}  // namespace geo
}  // namespace sg


