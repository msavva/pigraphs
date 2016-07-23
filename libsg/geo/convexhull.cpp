#include "common.h"  // NOLINT

#include "geo/convexhull.h"

#include <CGAL/convex_hull_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>

namespace sg {
namespace geo {

vec<Vec3f> convexHull(const VecVec3fIter pBegin, const VecVec3fIter pEnd) {
  typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
  typedef CGAL::Polyhedron_3<K>                                Polyhedron_3;
  typedef K::Point_3                                           Point_3;

  const size_t nPoints = pEnd - pBegin;
  vec<Point_3> cgalPoints;
  cgalPoints.reserve(nPoints);
  for (VecVec3fIter it = pBegin; it != pEnd; ++it) {
    const auto& p = *it;
    cgalPoints.push_back(Point_3(p[0], p[1], p[2]));
  }

  Polyhedron_3 poly;
  CGAL::convex_hull_3(begin(cgalPoints), end(cgalPoints), poly);
  poly.vertices_begin();
  vec<Vec3f> out(poly.size_of_vertices());

  size_t i = 0;
  for (auto it = poly.vertices_begin(); it != poly.vertices_end(); ++it, ++i) {
    Vec3f& v = out[i];
    const auto& p = it->point();
    v[0] = static_cast<float>(p[0]);
    v[1] = static_cast<float>(p[1]);
    v[2] = static_cast<float>(p[2]);
  }

  return out;
}

}  // namespace geo
}  // namespace sg
