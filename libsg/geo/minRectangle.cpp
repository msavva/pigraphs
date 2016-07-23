#include "common.h"  // NOLINT

#include "geo/minRectangle.h"

#include <CGAL/convex_hull_2.h>
#include <CGAL/min_quadrilateral_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Simple_cartesian.h>

namespace sg {
namespace geo {

vec<Vec2f> minRectangle2D(const VecVec2fIter pBegin, const VecVec2fIter pEnd) {
  typedef CGAL::Simple_cartesian<float>        K;
  typedef K::Point_2                           Point_2;
  typedef K::Line_2                            Line_2;
  typedef CGAL::Polygon_2<K>                   Polygon_2;

  const size_t nPoints = pEnd - pBegin;
  vec<Point_2> cgalPoints;
  cgalPoints.reserve(nPoints);
  for (VecVec2fIter it = pBegin; it != pEnd; ++it) {
    const auto& p = *it;
    cgalPoints.push_back(Point_2(p[0], p[1]));
  }

  Polygon_2 p;
  CGAL::convex_hull_2(begin(cgalPoints), end(cgalPoints),
                      std::back_inserter(p));

  Polygon_2 p_m;
  CGAL::min_rectangle_2(p.vertices_begin(), p.vertices_end(),
                        std::back_inserter(p_m));

  vec<Vec2f> out(p_m.size());
  size_t i = 0;
  for (auto it = p_m.vertices_begin(); it != p_m.vertices_end(); ++it, ++i) {
    Vec2f& v = out[i];
    v[0] = static_cast<float>(it->x());
    v[1] = static_cast<float>(it->y());
  }
  return out;
}

}  // namespace geo
}  // namespace sg
