#pragma once

#include <Eigen/Geometry>

#include "libsg.h"  // NOLINT

namespace sg {
namespace geo {

typedef Eigen::Vector2f Vec2f;
typedef Eigen::Vector3f Vec3f;
typedef Eigen::Vector4f Vec4f;
typedef Eigen::Vector2d Vec2d;
typedef Eigen::Vector3d Vec3d;
typedef Eigen::Vector4d Vec4d;
typedef Eigen::Vector2i Vec2i;
typedef Eigen::Vector3i Vec3i;
typedef Eigen::Vector4i Vec4i;
typedef vec<Vec3f> VecVec3f;
typedef vec<Vec3f>::iterator VecVec3fIter;
typedef vec<Vec2f> VecVec2f;
typedef vec<Vec2f>::iterator VecVec2fIter;
typedef Eigen::MatrixXf MatrixXf;
typedef Eigen::MatrixXd MatrixXd;
typedef Eigen::Matrix3f Matrix3f;
typedef Eigen::Matrix4f Matrix4f;
typedef Eigen::Matrix3Xf Matrix3Xf;
typedef Eigen::Matrix4Xf Matrix4Xf;
typedef Eigen::MatrixX3f MatrixX3f;
typedef Eigen::MatrixX4f MatrixX4f;
typedef Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> Transform;
typedef Eigen::AlignedBox3f BBox;
typedef Eigen::AlignedBox2f BBox2f;
typedef Eigen::Quaternionf Quatf;
typedef Eigen::AngleAxisf AngleAxisf;

// Vector directions for the 6 canonical directions
enum DirectionIndex : int8_t  {
  I_LEFT, I_RIGHT, I_DOWN, I_UP, I_BACK, I_FRONT, I_NONE
};
struct Direction {
  Direction() = default;
  const DirectionIndex index;
  const string name;
  const Vec3f vec;
};
  
const arr<const Direction,6> kDirections = {
  Direction{I_LEFT, "left", Vec3f(-1,0,0)},
  Direction{I_RIGHT, "right", Vec3f(1,0,0)},
  Direction{I_DOWN, "down", Vec3f(0,0,-1)},
  Direction{I_UP, "up", Vec3f(0,0,1)},
  Direction{I_BACK, "back", Vec3f(0,-1,0)},
  Direction{I_FRONT, "front", Vec3f(0,1,0)}
};
// Shorthand names for convenience
extern const Direction& DIR_LEFT;
extern const Direction& DIR_RIGHT;
extern const Direction& DIR_DOWN;
extern const Direction& DIR_UP;
extern const Direction& DIR_BACK;
extern const Direction& DIR_FRONT;
extern const Direction& DIR_NONE;

extern const map<string, DirectionIndex> kDirectionIndexMap;
inline const Direction& lookupDirection(const string& dirName) {
  if (kDirectionIndexMap.count(dirName) > 0) {
    DirectionIndex i = kDirectionIndexMap.at(dirName);
    return kDirections[i];
  } else {
    return kDirections[I_NONE];
  }
}

const Eigen::IOFormat kEigenJSONFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", ",", "", "", "[", "]");
const Eigen::IOFormat kEigenSpacedFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ", "", "", "", "");

//! Write Eigen matrix types out in JSON format
template <typename T, int numR, int numC>
ostream& toJSON(ostream& os, const Eigen::Matrix<T, numR, numC>& m) {
  return os << m.format(kEigenJSONFormat);
}
//! Write Eigen matrix types into ostreams as simple flat spaced format by default
template <typename T, int numR, int numC>
ostream& operator<<(ostream& os, const Eigen::Matrix<T, numR, numC>& m) {
  return os << m.format(kEigenSpacedFormat);
}

inline ostream& toJSON(ostream& os, const BBox& bbox);

inline ostream& operator<<(ostream& os, const BBox& bbox) {
  return toJSON(os, bbox);
}

//! Vec3f conversion operator
template<typename T>
Vec3f vec3f(const T& x) {
  return Vec3f(static_cast<float>(x[0]),
               static_cast<float>(x[1]),
               static_cast<float>(x[2]));
}

//! Vec4f conversion operator
template<typename T>
Vec4f vec4f(const T& x) {
  return Vec4f(static_cast<float>(x[0]),
               static_cast<float>(x[1]),
               static_cast<float>(x[2]),
               static_cast<float>(x[3]));
}

//! Return translation by t
inline Transform toTrans(const Vec3f& t) {
  return Transform(Transform::Identity()).translate(t);
}

//! Return rotation by qR
inline Transform toRot(const Quatf& qR) {
  return Transform(Transform::Identity()).rotate(qR);
}

//! Return rotation by qvec (asssumes x,y,z,w quaternior data order)
inline Transform toRot(const Vec4f& qvec) {
  return Transform(Transform::Identity()).rotate(Quatf(qvec));
}

//! Return transform rotating by qR then translating by t
inline Transform toTransRot(const Quatf& qR, const Vec3f& t) {
  return Transform(Transform::Identity()).rotate(qR).translate(t);
}

//! Return transform translating by t then rotating by qR
inline Transform toRotTrans(const Vec3f& t, const Quatf& qR) {
  return Transform(Transform::Identity()).translate(t).rotate(qR);
}

//! Encoding of rotations as angles
enum AngleEncoding : int8_t {
  LAT_LONG_ROT, EULER_XYZ, EULER_ZYZ, EULER_XZY
};
//! Convert quaternion to given angle encoding
Vec3f toAngleEncoding(const Quatf& q, const AngleEncoding& enc);
//! Convert from given angle encoding to quaternion
Quatf fromAngleEncoding(const Vec3f& v, const AngleEncoding& enc);

//! Return quaternion representation of rotation matrix defined by cols x,y,z
inline Quatf Rquat(const Vec3f& x, const Vec3f& y, const Vec3f& z) {
  Matrix3f R;
  R.col(0) = x;
  R.col(1) = y;
  R.col(2) = z;
  return Quatf(R).normalized();
};

//! Returns normalized cross product vector
inline Vec3f crossNorm(const Vec3f& a, const Vec3f& b) {
  return a.cross(b).normalized();
};

//! Returns perpendicular vector
inline Vec3f getPerp(const Vec3f& v) {
  return (fabs(v.x()) < 0.5)? 
    v.cross(Vec3f::UnitX()).normalized() : v.cross(Vec3f::UnitY()).normalized();
}

//! Conversion from Eigen Transform to ml::mat4f i.e. colwise to rowise 4x4
template<typename T>
T to(const Transform& xf) {
  T t(xf.matrix().data());
  t.transpose();
  return t;
}

//! Conversion from ml::mat4f to Eigen Transform i.e. rowise to colwise 4x4
template<typename T>
Transform from(const T& xf) {
  const Matrix4f m(Matrix4f::Map(xf.getData()));
  return Transform(m.transpose());
}

template<typename Vec2>
Vec2 to(const Vec2f& v) { return Vec2(v.data()); }
template<typename Vec3>
Vec3 to(const Vec3f& v) { return Vec3(v.data()); }
template<typename Vec4>
Vec4 to(const Vec4f& v) { return Vec4(v.data()); }

// Compute centroid of points in given range
template<class Vec3fIter>
Vec3f centroid(Vec3fIter start, Vec3fIter stop) {
  Vec3f c(0, 0, 0);
  Vec3f v;
  for (Vec3fIter curr = start; curr != stop; ++curr) {
    v[0] = (*curr)[0]; v[1] = (*curr)[1]; v[2] = (*curr)[2];
    c += v;
  }
  size_t n_points = stop - start;
  c /= static_cast<float>(n_points);
  return c;
}

//! Returns a vector with random uniform coordinates (i.e. mean 0, variance 1)
Vec3f randomVec3f();

//! Returns a uniformly sampled random point within the sphere with radius r and
//! with center at c
Vec3f randomPointInSphere(const Vec3f& c, float r);

//! Transforms corners of BBox a by xform and returns new BBox around transformed corners
BBox transformBBox(const Transform& xform, const BBox& a);

// Local coordinate system obtained by PCA of 3D point set.
// For a world point p, corresponding local point p' = S * R * (p - c)
class CoordSystem {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef Eigen::DiagonalMatrix<float, 3> ScaleMatrix;

  CoordSystem() : c_(0, 0, 0), R_(Matrix3f::Identity()) {
    S_.diagonal() = Vec3f(1, 1, 1);
  }

  // Coordinate system from PCA of point set (center at centroid, PCA axes)
  template<class Vec3fIter>
  CoordSystem(Vec3fIter start, Vec3fIter stop) {
    c_ = centroid(start, stop);
    const size_t n_points = stop - start;
    assert(n_points > 2);         // At least 3 points for PCA
    MatrixXf A(3, n_points);      // Form 3xN centered point matrix
    size_t i = 0;
    Vec3f v;
    for (Vec3fIter p = start; p != stop; p++, i++) {
      v[0] = (*p)[0]; v[1] = (*p)[1]; v[2] = (*p)[2];
      A.col(i) = v - c_;
    }

    // Rotation from singular vectors and scaling from singular values
    const auto svd = A.jacobiSvd(Eigen::ComputeThinU);
    R_ = svd.matrixU();
    S_.diagonal() = svd.singularValues();
  }
  void init();
  const Vec3f& c()       const { return c_; }
  const Matrix3f& R()    const { return R_; }
  const ScaleMatrix& S() const { return S_; }

 private:
  Vec3f c_;               // Centroid
  Matrix3f R_;            // Rotation matrix
  ScaleMatrix S_;         // Scaling matrix
};

//! Basic triangle class
class Tri {
 public:
  Tri() : v0_(), v1_(), v2_() { }
  Tri(const Vec3f& a, const Vec3f& b, const Vec3f& c) : v0_(a), v1_(b), v2_(c) { }

  Vec3f v0() const { return v0_; }
  Vec3f v1() const { return v1_; }
  Vec3f v2() const { return v2_; }
  float area() const { return areaWeightedNormal().norm(); }
  Vec3f normal() const { return areaWeightedNormal().normalized(); }
  Vec3f centroid() const { return (v0_ + v1_ + v2_) / 3.0; }
  Vec3f areaWeightedNormal() const {
    return 0.5f * ((v1_ - v0_).cross(v2_ - v0_));
  }

  // Uniform random sample from triangle surface (random barycentric coords)
  void sample(float u, float v, Vec3f* p, Vec3f* n = nullptr) const {
    if ((u + v) > 1.0) { u = 1 - u; v = 1 - v; }

    if (p) { *p = u * v0_ + v * v1_ + (1 - u - v) * v2_; }
    if (n) { *n = normal(); }
  }

 protected:
  Vec3f v0_, v1_, v2_;
};

//! Convenience struct representing a triangle associated with a Mesh type composed of triangles.
//! Mesh type is required to contain a FaceHandle type pointing to a triangle.
template<typename FaceHandle>
class MeshBackedTri : public Tri {
 public:
  MeshBackedTri() : Tri(), f_(0) { }
  MeshBackedTri(const Vec3f& a, const Vec3f& b, const Vec3f& c, const FaceHandle f) : Tri(a, b, c), f_(f) { }
  FaceHandle face() const { return f_; }

  //! Get Tri from OpenMesh format Mesh at FaceHandle f
  template <typename Mesh>
  void makeTriFromOpenMesh(const Mesh& mesh, const FaceHandle& f) {
    typename Mesh::ConstFaceVertexIter it = mesh.cfv_iter(f);
    const auto& t0 = mesh.point(*it);
    const auto& t1 = mesh.point(*++it);
    const auto& t2 = mesh.point(*++it);
    f_ = f;
    for (size_t i = 0; i < 3; ++i) {  // Ugh, ugly copy assignment
      v0_[i] = static_cast<float>(t0[i]);
      v1_[i] = static_cast<float>(t1[i]);
      v2_[i] = static_cast<float>(t2[i]);
    }
  }

  //! Get Tri from mLib format TriMesh at FaceHandle (index) f
  template <typename TriMesh>
  void makeTriFromMlibMesh(const TriMesh& mesh, const FaceHandle& f) {
    const auto& I = mesh.getIndices();
    const auto& V = mesh.getVertices();
    const auto& triIs = I[f];

    const auto& t0 = V[triIs[0]];
    const auto& t1 = V[triIs[1]];
    const auto& t2 = V[triIs[2]];
    f_ = f;
    for (int i = 0; i < 3; ++i) {  // Ugh, ugly copy assignment
      v0_[i] = static_cast<float>(t0.position[i]);
      v1_[i] = static_cast<float>(t1.position[i]);
      v2_[i] = static_cast<float>(t2.position[i]);
    }
  }

 protected:
  FaceHandle f_;
};

// Represents a 3D plane defined by a*x + b*y + c*z + d = 0
class Plane {
 public:
  Plane() : a(0), b(0), c(0), d(0), center_(0, 0, 0) { }
  Plane(float _a, float _b, float _c, float _d)
    : a(_a), b(_b), c(_c), d(_d), center_(0, 0, 0) { }
  Plane(const Plane& p) : a(p.a), b(p.b), c(p.c), d(p.d), center_(p.center_) { }
  Plane(const Vec3f& pt, const Vec3f& n) { fromCentroidNormal(pt, n); }
  template <typename Vec3fIter>
  explicit Plane(Vec3fIter b, Vec3fIter e) { leastSquaresFit(b, e); }
  Plane& operator=(const Plane& p) {
    a = p.a; b = p.b; c = p.c; d = p.d; center_ = p.center_;
    return *this;
  }

  const Vec3f& center() const { return center_; }
  Vec3f normal() const { return Vec3f(a, b, c); }

  void fromCentroidNormal(const Vec3f& pt, const Vec3f& n) {
    a = n.x(); b = n.y(); c = n.z();
    d = -pt.dot(n);
    center_ = pt;
  }

  float signedDistance(const Vec3f& p) const {
    return a * p.x() + b * p.y() + c * p.z() + d;
  }

  Vec3f closestPoint(const Vec3f& p) const {
    return p - (normal() * signedDistance(p));
  }

  // Fits plane to points using SVD-based least squares regression
  template <typename Vec3fIter>
  void leastSquaresFit(Vec3fIter start, Vec3fIter stop) {
    assert(stop - start > 2);
    const CoordSystem worldToLocal(start, stop);
    const Vec3f& normal = worldToLocal.R().col(2);
    const Vec3f& c = worldToLocal.c();
    fromCentroidNormal(c, normal);
  }

  // Returns a transform representing a reflection through this plane
  Transform reflection() const {
    Transform t;
    const float taa = -2.f * a * a, tab = -2.f * a * b, tac = -2.f * a * c, tbc = -2.f * b * c,
                tbb = -2.f * b * b, tcc = -2.f * c * c, tad = -2.f * a * d, tbd = -2.f * b * d,
                tcd = -2.f * c * d;
    t.affine() << 1 + taa,   tab,   tac,  tad,
             tab, 1 + tbb,   tbc,  tbd,
             tac,   tbc, 1 + tcc,  tcd;
    return t;
  }

 private:
  float a, b, c, d;
  Vec3f center_;
};

// Transform a mesh by the given transformation matrix
template <typename Mesh>
void transformMesh(const Transform& xform, Mesh* mesh) {
  typedef typename Mesh::Point Point;
  typedef typename Mesh::Normal Normal;

  // These will be our transformed normals and points
  Vec4f p2;
  Vec3f n2;
  // Take inverse transponse to transform normals correctly
  Eigen::Matrix3f normXform = xform.linear().inverse().adjoint();

  for (auto v : mesh->vertices()) {
    auto p = mesh->point(v);
    auto n = mesh->normal(v);
    p2 << static_cast<float>(p[0]),
    static_cast<float>(p[1]),
    static_cast<float>(p[2]),
    1.0f;
    p2 = xform * p2;
    n2 << static_cast<float>(n[0]),
    static_cast<float>(n[1]),
    static_cast<float>(n[2]);
    n2 = (normXform * n2).normalized();
    mesh->set_point(v, Point(p2[0], p2[1], p2[2]));
    mesh->set_normal(v, Normal(n2[0], n2[1], n2[2]));
  }
  for (auto f : mesh->faces()) {
    auto n = mesh->normal(f);
    n2 << static_cast<float>(n[0]),
    static_cast<float>(n[1]),
    static_cast<float>(n[2]);
    n2 = (normXform * n2).normalized();
    mesh->set_normal(f, Normal(n2[0], n2[1], n2[2]));
  }
}

// Returns the total surface area of the mesh
template <typename Mesh, typename fIter>
float totalSurfaceArea(const Mesh& mesh, fIter fBegin, fIter fEnd) {
  MeshBackedTri<typename Mesh::FaceHandle> t;
  float area = 0;
  for (auto fIt = fBegin; fIt != fEnd; ++fIt) {
    t.makeTriFromOpenMesh(mesh, *fIt);
    area += t.area();
  }
  return area;
}
template <typename Mesh, typename Container>
float totalSurfaceArea(const Mesh& mesh, const Container& c) {
  return totalSurfaceArea(mesh, std::begin(c), std::end(c));
}

// Returns vector with of all triangles in Mesh
template <typename Mesh, typename fIter>
vec<MeshBackedTri<Mesh>> triangles(const Mesh& mesh, fIter fBegin, fIter fEnd) {
  const size_t nFaces = std::distance(fBegin, fEnd);
  vec<MeshBackedTri<typename Mesh::FaceHandle>> tris(nFaces);
  size_t i = 0;
  for (auto fIt = fBegin; fIt != fEnd; ++fIt, ++i) {
    tris[i].makeTriFromOpenMesh(mesh, *fIt);
  }
  return tris;
}

// Scratch space for KD tree queries (variables to interface with nanoflann)
struct KNNWorkspace {
  vec<size_t> out_indices;
  vecf out_dists_sq;
  vec<pair<size_t, float>> out_pairs;
};

//! BBox around pts transformed by xform
inline BBox ptsBBox(const VecVec3f& pts, const Transform& xform) {
  BBox bbox;
  for (const auto& p : pts) {
    bbox.extend(xform * p);
  }
  return bbox;
}

//! BBox around pts
inline BBox ptsBBox(const VecVec3f& pts) {
  BBox bbox;
  for (const auto& p : pts) {
    bbox.extend(p);
  }
  return bbox;
}

template <typename Vector3T>
VecVec3f toVecVec3f(const vec<Vector3T>& pts) {
  const size_t numPts = pts.size();
  VecVec3f out(numPts);
  for (size_t i = 0; i < numPts; ++i) {
    out[i] = geo::vec3f(pts[i]);
  }
  return out;
}

// Helper functions to align up/front
inline void getMatrix3f(const Vec3f& v1, const Vec3f& v2, Matrix3f* pM) {
  Vec3f v1n = v1.normalized();
  Vec3f v2n = v2.normalized();
  Vec3f v3n = crossNorm(v1n,v2n);
  pM->col(0) = v1n;
  pM->col(1) = v2n;
  pM->col(2) = v3n;
}

inline void computeAlignToUpFrontAxesMatrix3f(
  const Vec3f& objectUp, const Vec3f& objectFront,
  const Vec3f& targetUp, const Vec3f& targetFront,
  Matrix3f* pM) {
  Matrix3f objM, targetM;
  getMatrix3f(objectUp, objectFront, &objM);
  getMatrix3f(targetUp, targetFront, &targetM);
  (*pM) = targetM * objM.inverse();
}

}  // namespace geo
}  // namespace sg
