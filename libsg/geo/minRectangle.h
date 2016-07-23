#pragma once

#include "libsg.h"  // NOLINT
#include "geo/geo.h"

namespace sg {
namespace geo {

vec<Vec2f> minRectangle2D(const VecVec2fIter pBegin, const VecVec2fIter pEnd);

template <typename Container>
vec<Vec2f> minRectangle2D(Container c) { return minRectangle2D(std::begin(c), std::end(c)); }

}  // namespace geo
}  // namespace sg


