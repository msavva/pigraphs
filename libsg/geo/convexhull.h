#pragma once

#include "libsg.h"  // NOLINT
#include "geo/geo.h"

namespace sg {
namespace geo {

vec<Vec3f> convexHull(const VecVec3fIter pBegin, const VecVec3fIter pEnd);

template <typename Container>
vec<Vec3f> convexHull(Container c) { return convexHull(std::begin(c), std::end(c)); }

}  // namespace geo
}  // namespace sg


