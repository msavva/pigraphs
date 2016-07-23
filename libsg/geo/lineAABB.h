#pragma once

#include "libsg.h"  // NOLINT

#include "geo/geo.h"
#include <boost/optional.hpp>

namespace sg {
namespace geo {

vec<Vec3f> findIntersectingPointsLine(const BBox& box,
                                      const Vec3f& origin,
                                      const Vec3f& dir);

vec<Vec3f> findIntersectingPointsRay(const BBox& box,
                                     const Vec3f& origin,
                                     const Vec3f& dir);

boost::optional<Vec3f> findClosestIntersectingPoint(const BBox& b, const Vec3f& p,
                                                    const Vec3f& dir, bool checkOppositeDir);


}  // namespace geo
}  // namespace sg
