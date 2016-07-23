#include "common.h"  // NOLINT

#include "mesh/MeshSamples.h"

#include "math/math.h"

namespace sg {
namespace mesh {

MeshSamples::MeshSamples(const Mesh& mesh, cfIter fBegin, cfIter fEnd,
                         size_t nSamples, bool addCentroids,
                         bool addVertices) {
  // Initialization
  const size_t nFaces  = fEnd - fBegin;
  const size_t nPoints = nSamples + (addCentroids ? nFaces : 0)
                       + (addVertices ? 3 * nFaces : 0);
  resize(nPoints);
  area_ = geo::totalSurfaceArea(mesh, fBegin, fEnd);
  const double area_scale = 1.0 / area_;
  const double n_inv = 1.0 / nSamples;

  cfIter f_it = fBegin;
  geo::MeshBackedTri<Mesh::FaceHandle> tri;
  tri.makeTriFromOpenMesh(mesh, *f_it);
  double triA = tri.area() * area_scale;
  double areaAcc = triA;
  ++f_it;

  for (size_t pI = 0; pI < nSamples; pI++) {
    const double iMin = pI * n_inv;
    const double iMax = (pI + 1) * n_inv;
    const double r = iMin + math::DEF_RAND.uniform_float_01() * (iMax - iMin);
    const float u = math::DEF_RAND.uniform_float_01();
    const float v = math::DEF_RAND.uniform_float_01();
    while (areaAcc < r && f_it != fEnd) {
      tri.makeTriFromOpenMesh(mesh, *f_it);
      triA =  tri.area() * area_scale;
      areaAcc += triA;
      ++f_it;
    }
    tri.sample(u, v, &(p_[pI]), &(n_[pI]));
    f_[pI] = tri.face();
    c_[pI] = mesh.color(tri.face());
  }

  // Also force-add face centroids if required
  if (addCentroids) {
    Mesh::Point p;
    size_t f_i = nSamples;  // Start after sampled points and fill until end
    for (f_it = fBegin; f_it != fEnd; ++f_it, ++f_i) {
      const Mesh::FaceHandle f = *f_it;
      mesh.calc_face_centroid(f, p);
      p_[f_i] = geo::vec3f(p);
      n_[f_i] = geo::vec3f(mesh.calc_face_normal(f));
      f_[f_i] = f;
      c_[f_i] = mesh.color(f);
    }
  }

  // And vertices
  if (addVertices) {
    Mesh::Point p;
    size_t f_i = 0;
    size_t f_iBase = nSamples + (addCentroids ? nFaces : 0);

    for (f_it = fBegin; f_it != fEnd; ++f_it, ++f_i) {
      const Mesh::FaceHandle f = *f_it;
      Mesh::ConstFaceVertexIter fv_it = mesh.cfv_iter(f);
      const auto n = geo::vec3f(mesh.calc_face_normal(f));
      const auto c = mesh.color(f);
      size_t v_i = f_iBase + 3 * f_i;
      for (size_t j = 0; j < 3; j++, ++fv_it, ++v_i) {
        p_[v_i] = geo::vec3f(mesh.point(*fv_it));
        n_[v_i] = n;
        f_[v_i] = f;
        c_[v_i] = c;
      }
    }
  }
}

}  // namespace mesh
}  // namespace sg
