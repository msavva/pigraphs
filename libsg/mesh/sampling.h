#pragma once

#include "libsg.h"  // NOLINT
#include "geo/geo.h"
#include "math/math.h"

namespace sg {
namespace mesh {
namespace sampling {

//! Uniform point sampling over the surface of the TriMeshT mesh using area-weighted method. TriMeshT must provide
//! getVertices() and getIndices() functions. Sampled points are saved as vec3 types into ps. Sampled normals are also
//! saved in ns if pointer to vec3 container is given.
template <typename TriMeshT, typename VecVec3>
void sampleTris(const TriMeshT& m, size_t numSamples, VecVec3* ps, VecVec3* ns = nullptr) {
  const auto& V = m.getVertices();
  const auto& I = m.getIndices();
  const size_t numTris = I.size();

  vecf cumArea(numTris);
  geo::MeshBackedTri<size_t> tri;
  tri.makeTriFromMlibMesh(m, 0);
  cumArea[0] = tri.area();
  for (size_t i = 1; i < numTris; i++) {
    tri.makeTriFromMlibMesh(m, i);
    cumArea[i] = cumArea[i - 1] + tri.area();
  }
  const float totalArea = cumArea[numTris - 1];
  const auto cumAreaBegin = cumArea.begin();
  const auto cumAreaEnd = cumArea.end();

  ps->resize(numSamples);
  if (ns) {
    ns->resize(numSamples);
  }
  for (size_t i = 0; i < numSamples; i++) {
    const float r = math::DEF_RAND.uniform_float_01() * totalArea;
    const float u = math::DEF_RAND.uniform_float_01();
    const float v = math::DEF_RAND.uniform_float_01();
    const size_t j = lower_bound(cumAreaBegin, cumAreaEnd, r) - cumAreaBegin;
    tri.makeTriFromMlibMesh(m, j);
    if (ns != nullptr) {
      tri.sample(u, v, &((*ps)[i]), &((*ns)[i]));
    } else {
      tri.sample(u, v, &((*ps)[i]));
    }
  }
}

//! TODO: This function is mostly a copy of the function above and can probably be merged with it
//! Version of sampling of the surface of the TriMeshT mesh with a set of accepted triIndices
//! Uniform point sampling over the surface of the TriMeshT mesh using area-weighted method. TriMeshT must provide
//! getVertices() and getIndices() functions. Sampled points are saved as vec3 types into ps. Sampled normals are also
//! saved in ns if pointer to vec3 container is given.
template <typename TriMeshT, typename VecVec3>
void sampleTris(const TriMeshT& m, vec<size_t> triIndices, size_t numSamples, VecVec3* ps, VecVec3* ns = nullptr) {
//  const auto& V = m.getVertices();
//  const auto& I = m.getIndices();
  const size_t numTris = triIndices.size();

  vecf cumArea(numTris);
  geo::MeshBackedTri<size_t> tri;
  tri.makeTriFromMlibMesh(m, triIndices.at(0));
  cumArea[0] = tri.area();
  for (size_t i = 1; i < numTris; i++) {
    tri.makeTriFromMlibMesh(m, triIndices.at(i));
    cumArea[i] = cumArea[i - 1] + tri.area();
  }
  const float totalArea = cumArea[numTris - 1];
  const auto cumAreaBegin = cumArea.begin();
  const auto cumAreaEnd = cumArea.end();

  ps->resize(numSamples);
  if (ns) {
    ns->resize(numSamples);
  }
  for (size_t i = 0; i < numSamples; i++) {
    const float r = math::DEF_RAND.uniform_float_01() * totalArea;
    const float u = math::DEF_RAND.uniform_float_01();
    const float v = math::DEF_RAND.uniform_float_01();
    const size_t j = lower_bound(cumAreaBegin, cumAreaEnd, r) - cumAreaBegin;
    tri.makeTriFromMlibMesh(m, triIndices.at(j));
    if (ns != nullptr) {
      tri.sample(u, v, &((*ps)[i]), &((*ns)[i]));
    } else {
      tri.sample(u, v, &((*ps)[i]));
    }
  }
}

template <typename TriMeshT, typename VecIndices, typename VecVec3>
void sampleVertices(const TriMeshT& m, const VecIndices& vertIndices, size_t numSamples, VecVec3* ps) {
  const auto& V = m.getVertices();
  const size_t numVertices = vertIndices.size();
  ps->resize(numSamples);
  for (size_t i = 0; i < numSamples; i++) {
    const float r = math::DEF_RAND.uniform_float_01() * numVertices;
    auto vi = vertIndices[static_cast<size_t>(floor(r))];
    (*ps)[i] = V[vi];
  }
}

template <typename TriMeshT, typename VecVec3>
void sampleVertices(const TriMeshT& m, size_t numSamples, VecVec3* ps) {
  const auto& V = m.getVertices();
  const size_t numVertices = V.size();
  ps->resize(numSamples);
  for (size_t i = 0; i < numSamples; i++) {
    const float r = math::DEF_RAND.uniform_float_01() * numVertices;
    auto vi = static_cast<size_t>(floor(r));
    (*ps)[i] = V[vi].position;
  }
}

}  // namespace sampling
}  // namespace mesh
}  // namespace sg


