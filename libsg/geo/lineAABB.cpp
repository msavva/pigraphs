#include "common.h"  // NOLINT

#include "geo/geo.h"

#include "math/math.h"

namespace sg {
namespace geo {

// Intersection code taken from
// http://www.geometrictools.com/GTEngine/Include/Mathematics/GteIntrRay3AlignedBox3.h
// http://www.geometrictools.com/GTEngine/Include/Mathematics/GteIntrLine3AlignedBox3.h

// Helper function for findIntersectingPointsLine
template <typename num_t>
bool clip(num_t denom, num_t numer, num_t& t0, num_t& t1) {
  if (denom > (num_t) 0) {
    if (numer > denom*t1) {
      return false;
    } 
    if (numer > denom*t0) {
      t0 = numer / denom;
    }
    return true;
  } else if (denom < (num_t) 0) {
    if (numer > denom*t0) {
      return false;
    }
    if (numer > denom*t1) {
      t1 = numer / denom;
    }
    return true;
  } else {
    return numer <= (num_t) 0;
  }
}

struct IntersectResult
{
  bool intersect;
  int numPoints;
  float lineParameter[2];
  Vec3f point[2];
};

// Helper function for finding intersections points along line
void findIntersectionPointsLineCenteredBBox(const Vec3f& lineOrigin,
                                            const Vec3f& lineDirection,
                                            const Vec3f& boxExtent,
                                            IntersectResult& result) {
  // The line t-values are in the interval (-infinity,+infinity).  Clip the
  // line against all six planes of an aligned box in centered form.  The
  // result.numEndpoints is
  //   0, no intersection
  //   1, intersect in a single point (t0 is line parameter of point)
  //   2, intersect in a segment (line parameter interval is [t0,t1])
  float t0 = -std::numeric_limits<float>::max();
  float t1 = std::numeric_limits<float>::max();
  if (clip(+lineDirection[0], -lineOrigin[0] - boxExtent[0], t0, t1) &&
      clip(-lineDirection[0], +lineOrigin[0] - boxExtent[0], t0, t1) &&
      clip(+lineDirection[1], -lineOrigin[1] - boxExtent[1], t0, t1) &&
      clip(-lineDirection[1], +lineOrigin[1] - boxExtent[1], t0, t1) &&
      clip(+lineDirection[2], -lineOrigin[2] - boxExtent[2], t0, t1) &&
      clip(-lineDirection[2], +lineOrigin[2] - boxExtent[2], t0, t1))
  {
    result.intersect = true;
    if (t1 > t0) {
      result.numPoints = 2;
      result.lineParameter[0] = t0;
      result.lineParameter[1] = t1;
    } else {
      result.numPoints = 1;
      result.lineParameter[0] = t0;
      result.lineParameter[1] = t0;  // Used for ray
    }
    return;
  }

  result.intersect = false;
  result.numPoints = 0;  
}

// Finds intersecting point along line
vec<Vec3f> findIntersectingPointsLine(const BBox& box,
                                      const Vec3f& origin,
                                      const Vec3f& dir) {
  // Get the centered form of the aligned box.
  Vec3f boxCenter = box.center(), boxExtent = box.sizes() / 2;

  // Transform the line to the aligned-box coordinate system.
  Vec3f lineOrigin = origin - boxCenter;

  IntersectResult result;
  findIntersectionPointsLineCenteredBBox(lineOrigin, dir, boxExtent, result);

  vec<Vec3f> intersectPoints;
  intersectPoints.resize(result.numPoints);
  for (int i = 0; i < result.numPoints; ++i) {
    result.point[i] = origin + result.lineParameter[i] * dir;
    intersectPoints.push_back(result.point[i]);
  }
  return intersectPoints;
}

// Helper function for finding intersections points along ray
void findIntersectionPointsRayCenteredBBox(const Vec3f& rayOrigin,
                                           const Vec3f& rayDirection,
                                           const Vec3f& boxExtent,
                                           IntersectResult& result) {
  findIntersectionPointsLineCenteredBBox(rayOrigin, rayDirection, boxExtent, result);

  if (result.intersect) {
    // The line containing the ray intersects the box; the t-interval is
    // [t0,t1].  The ray intersects the box as long as [t0,t1] overlaps
    // the ray t-interval (0,+infinity).
    if (result.lineParameter[1] >= 0) {
      if (result.lineParameter[0] < 0) {
        result.lineParameter[0] = 0;
      }
    } else {
      result.intersect = false;
      result.numPoints = 0;
    }
  }
}

vec<Vec3f> findIntersectingPointsRay(const BBox& box,
                                     const Vec3f& origin,
                                     const Vec3f& dir) {
  // Get the centered form of the aligned box.
  Vec3f boxCenter = box.center(), boxExtent = box.sizes() / 2;

  // Transform the ray to the aligned-box coordinate system.
  Vec3f rayOrigin = origin - boxCenter;

  IntersectResult result;
  findIntersectionPointsRayCenteredBBox(rayOrigin, dir, boxExtent, result);

  vec<Vec3f> intersectPoints;
  intersectPoints.resize(result.numPoints);
  for (int i = 0; i < result.numPoints; ++i) {
    result.point[i] = origin + result.lineParameter[i] * dir;
    intersectPoints.push_back(result.point[i]);
  }
  return intersectPoints;
}

boost::optional<Vec3f> findClosestIntersectingPoint(const BBox& b, const Vec3f& p,
                                                    const Vec3f& dir, bool checkOppositeDir) {
  
  vec<Vec3f> intersecting = (checkOppositeDir)? 
    findIntersectingPointsLine(b, p, dir) : findIntersectingPointsRay(b, p, dir);
  boost::optional<Vec3f> res;
  float bestDist = math::constants::POSINFf;
  for (int i = 0; i < intersecting.size(); ++i) {
    float dist = (intersecting[i] - p).norm();
    //SG_LOG_DEBUG << "Got " << intersecting[i].format(kEigenJSONFormat) << " with dist " << dist;
    if (dist < bestDist) {
      bestDist = dist;
      res = intersecting[i];
    }
  }
  return res;
}

}  // namespace geo
}  // namespace sg
