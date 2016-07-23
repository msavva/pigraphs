#include "common.h"  // NOLINT

#include "geo/geo.h"

#include "math/math.h"

namespace sg {
namespace geo {

const Direction& DIR_LEFT = kDirections[I_LEFT];
const Direction& DIR_RIGHT = kDirections[I_RIGHT];
const Direction& DIR_DOWN = kDirections[I_DOWN];
const Direction& DIR_UP = kDirections[I_UP];
const Direction& DIR_BACK = kDirections[I_BACK];
const Direction& DIR_FRONT = kDirections[I_FRONT];
const Direction& DIR_NONE = Direction{I_NONE, "none", Vec3f(0, 0, 0)};

map<string, DirectionIndex> createDirectionIndexMap()
{
  map<string, DirectionIndex> map;
  for (size_t i = 0; i < kDirections.size(); ++i) {
    const Direction& dir = kDirections[i];
    map[dir.name] = dir.index;
  }
  return map;
}

const map<string, DirectionIndex> kDirectionIndexMap = createDirectionIndexMap();

ostream& toJSON(ostream& os, const BBox& bbox) {
  os << "BBox: {min:";  toJSON(os, bbox.min());
  os << ", max:";       toJSON(os, bbox.max()) << "}";
  return os;
}

// Implementation described by http://www.euclideanspace.com/maths/geometry/rotations/euler/
// Also refer to https://en.wikipedia.org/wiki/Euler_angles for general discussion

Vec3f toEulerXZY(const Quatf& q) {
  double heading, attitude, bank;
  const double test = q.x()*q.y() + q.z()*q.w();
  if (test > 0.499) { // singularity at north pole
    heading   = 2 * atan2(q.x(), q.w());
    attitude  = math::constants::PI/2;
    bank      = 0;
  } else if (test < -0.499) { // singularity at south pole
    heading   = -2 * atan2(q.x(), q.w());
    attitude  = - math::constants::PI/2;
    bank      = 0;
  } else {
    const double sqx = q.x()*q.x();
    const double sqy = q.y()*q.y();
    const double sqz = q.z()*q.z();
    heading   = atan2(2*q.y()*q.w()-2*q.x()*q.z(), 1 - 2*sqy - 2*sqz);
    attitude  = asin(2*test);
    bank      = atan2(2*q.x()*q.w() - 2*q.y()*q.z(), 1 - 2*sqx - 2*sqz);
  }
  return Vec3f(static_cast<float>(heading),
               static_cast<float>(attitude),
               static_cast<float>(bank));
}

Quatf fromEulerXZY(const Vec3f& hab) {
  const Quatf q(
    AngleAxisf(hab[2], Vec3f::UnitX()) *  //bank last (about x axis -- front-back)
    AngleAxisf(hab[1], Vec3f::UnitZ()) *  //attitude second (about z axis -- along wing)
    AngleAxisf(hab[0], Vec3f::UnitY())    //heading first (about y axis -- up-down)
  );
  return q;
}

Vec3f toEulerZYZ(const Quatf& q) {
  const Vec3f euler = q.toRotationMatrix().eulerAngles(2, 1, 2);
  return euler;
}

Quatf fromEulerZYZ(const Vec3f& v) {
  Matrix3f R;
  R = AngleAxisf(v[0], Vec3f::UnitZ())
    * AngleAxisf(v[1], Vec3f::UnitY())
    * AngleAxisf(v[2], Vec3f::UnitZ());
  const Quatf q(R);
  return q;
}

Vec3f toEulerXYZ(const Quatf& q) {
  const Vec3f euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
  return euler;
}

Quatf fromEulerXYZ(const Vec3f& v) {
  Matrix3f R;
  R = AngleAxisf(v[0], Vec3f::UnitX())
    * AngleAxisf(v[1], Vec3f::UnitY())
    * AngleAxisf(v[2], Vec3f::UnitZ());
  const Quatf q(R);
  return q;
}

Vec3f toLatLongRotAngles(const Quatf& q) {
  double
    cosa = q.w(),
    sina = sqrt(1.0 - cosa * cosa),
    ang  = 2.0 * math::acosSafe(cosa);

  if (fabs(sina) < 0.0005) {
    sina = 1.0;
  }

  double
    tx        = q.x() / sina,
    ty        = q.y() / sina,
    tz        = q.z() / sina,
    latitude  = asin(ty),
    longitude = atan2(tx, tz);

  double txz = tx * tx + tz * tz;

  if (!isfinite(latitude)) {
    latitude = atan2(ty, sqrt(txz));
  }

  if (txz < 0.0005) {
    longitude = 0.0;
  }

  if (longitude < 0.0) {
    longitude += math::constants::PI2f;
  }
  if (!isfinite(latitude) || !isfinite(longitude) || !isfinite(ang)) {
    SG_LOG_WARN << "Invalid rot angles: [" 
      << latitude << "," << longitude << "," << ang << "] from " 
      << q.coeffs().format(kEigenJSONFormat) << " with t=["
      << tx << "," << ty << "," << tz << "], cosa=" << cosa << ", sina=" << sina;
  }

  return Vec3f(static_cast<float>(latitude),
               static_cast<float>(longitude),
               static_cast<float>(ang));
}

Quatf fromLatLongRotAngles(const Vec3f& v) {
  const double
    latitude  = v.x(),
    longitude = v.y(),
    ang       = v.z(),
    sina      = sin(ang / 2),
    cosa      = cos(ang / 2),
    sinlat    = sin(latitude),
    coslat    = cos(latitude),
    sinlon    = sin(longitude),
    coslon    = cos(longitude),
    qx        = sina * coslat * sinlon,
    qy        = sina * sinlat,
    qz        = sina * coslat * coslon,
    qw        = cosa;
  return Quatf(static_cast<float>(qw),
               static_cast<float>(qx),
               static_cast<float>(qy),
               static_cast<float>(qz));
}

Vec3f toAngleEncoding(const Quatf& q, const AngleEncoding& enc) {
  switch(enc) {
    case LAT_LONG_ROT:  return toLatLongRotAngles(q);
    case EULER_XYZ:     return toEulerXYZ(q);
    case EULER_ZYZ:     return toEulerZYZ(q);
    case EULER_XZY:     return toEulerXZY(q);
    default:            return toLatLongRotAngles(q);
  }
}

Quatf fromAngleEncoding(const Vec3f& v, const AngleEncoding& enc) {
  switch(enc) {
    case LAT_LONG_ROT:  return fromLatLongRotAngles(v);
    case EULER_XYZ:     return fromEulerXYZ(v);
    case EULER_ZYZ:     return fromEulerZYZ(v);
    case EULER_XZY:     return fromEulerXZY(v);
    default:            return fromLatLongRotAngles(v);
  }
}

Vec3f randomVec3f() {
  Vec3f r;
  for (size_t i = 0; i < 3; i++) { r[i] = math::DEF_RAND.normal_float_01(); }
  return r;
}

Vec3f randomPointInSphere(const Vec3f& c, float r) {
  const Vec3f X = randomVec3f();
  const float U = math::DEF_RAND.uniform_float_01();
  return (r * std::powf(U, 1.f / 3.f) * X.normalized()) + c;
}

//! Transforms corners of BBox a by xform and returns new BBox around transformed corners
BBox transformBBox(const Transform& xform, const BBox& a) {
  const Vec3f
    m = a.min(),         // 0
    M = a.max();         // 1
  const vec<Vec3f> corners{
    {m[0], m[1], m[2]},  // 000
    {m[0], m[1], M[2]},  // 001
    {m[0], M[1], m[2]},  // 010
    {m[0], M[1], M[2]},  // 011
    {M[0], m[1], m[2]},  // 100
    {M[0], m[1], M[2]},  // 101
    {M[0], M[1], m[2]},  // 110
    {M[0], M[1], M[2]}   // 111
  };
  BBox bbox;
  for (const Vec3f& p : corners) {
    bbox.extend(xform * p);
  }
  return bbox;
}

}  // namespace geo
}  // namespace sg
