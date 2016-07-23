#include "common.h"  // NOLINT

#include "segmentation/MeshSegment.h"

#include "geo/OBB.h"
#include "mesh/sampling.h"

namespace sg {
namespace segmentation {

typedef sg::geo::OBB OBB;
typedef sg::geo::Vec3f Vec3f;

MeshSegment::MeshSegment(const ml::TriMeshf& _mesh, const vec<size_t>& _elements, int _id,
                         bool constrainZup, float shrinkPercentile /* = 0.0f */,
                         const geo::Transform& _xform /* = geo::Transform::Identity() */,
                         const vec<size_t>& _triIndices /* = vec<size_t>() */,
                         const size_t nPointsToSampleForObb /* = 0 */)
  : Part(kPartSegment, _id, "")
  , mesh(&_mesh)
  , transform(_xform)
  , elements(_elements)
  , triIndices(_triIndices) {
  init(constrainZup, shrinkPercentile, nPointsToSampleForObb);
}

MeshSegment::~MeshSegment() {
}

void MeshSegment::init(bool constrainZup, float shrinkPercentile, size_t nPointsToSampleForObb) {
  size_t nSamples = (!triIndices.empty())? nPointsToSampleForObb : 0;
  points.resize(elements.size() + nPointsToSampleForObb);
  const auto& V = mesh->getVertices();
  const size_t numVerts = V.size();
  const bool needTransform = !transform.isApprox( geo::Transform::Identity() );
  const ml::mat4f mlxf = geo::to<ml::mat4f>(transform);
  for (size_t i = 0; i < elements.size(); i++) {
    const size_t elIdx = elements[i];
    if (elIdx > numVerts) {
      SG_LOG_ERROR << "[MeshSegment] Out of bounds element index: " << elIdx;
    } else {
      points[i] = V[elIdx].position;
      if (needTransform) {
        points[i] = mlxf * points[i];
      }
    }
  }
  if (nSamples > 0) {
    vec<geo::Vec3f> sampledPoints;
    samplePoints(nSamples, &sampledPoints);
    for (size_t i = 0; i < nSamples; i++) {
      points[i+elements.size()] = geo::to<ml::vec3f>(sampledPoints[i]);
    }
  }
  if (points.size() > 2) {
    m_obb = std::make_shared<OBB>(points, (constrainZup) ? OBB::FitOpts::CONSTRAIN_Z : OBB::FitOpts::MIN_PCA);

    // Remove outlier points from OBB dimension estimate
    if (shrinkPercentile > 0) {
      assert(shrinkPercentile < 1);
      const size_t numPoints = points.size();
      vecf projectedPts(numPoints);
      const ml::vec3f centroid = sg::geo::to<ml::vec3f>(m_obb->centroid());
      const auto &m = m_obb->normalizedAxes();

      for (int axis = 0; axis < 3; axis++) {
        const ml::vec3f v(m(0, axis), m(1, axis), m(2, axis));
        for (size_t i = 0; i < numPoints; i++) {
          projectedPts[i] = ml::vec3f::dot(v, points[i] - centroid);
        }
        std::sort(projectedPts.begin(), projectedPts.end());

        const float minProj = projectedPts[static_cast<size_t>(numPoints * shrinkPercentile)];
        const float maxProj = projectedPts[static_cast<size_t>(numPoints * (1.0f - shrinkPercentile))];
        shrunkOBBaxes[axis] = maxProj - minProj;
      }
    }

    // Set the dominant normal direction (normal of plane fit through points)
    if (constrainZup) {
      // Since Z was constrained to be up, we need to find the min-PCA axis on the points
      const sg::geo::CoordSystem C(points.begin(), points.end());
      for (int i = 0; i < 3; i++) {
        m_dominantNormal[i] = C.R()(i, 2);
      }
    } else {
      // For efficiency, we use the OBB's existing z axis as the dominant normal since it was computed through min-PCA
      for (int i = 0; i < 3; i++) {
        m_dominantNormal[i] = m_obb->normalizedAxes()(i, 2);
      }
    }

    areaWeightedNormal = computeAreaWeightedNormal();
    segNormal = areaWeightedNormal;
    if (areaWeightedNormal.length() > 0) {
      segNormal = areaWeightedNormal.getNormalized();
    }
  }
}

void MeshSegment::samplePoints(size_t numSamples, vec<Vec3f>* pPoints) const {
  assert(pPoints);
  mesh::sampling::sampleTris(*mesh, triIndices, numSamples, pPoints);
  const bool needTransform = !transform.isApprox( geo::Transform::Identity() );
  if (needTransform) {
    // Apply mesh segment transform on sampled points
    for (size_t i = 0; i < pPoints->size(); ++i) {
      (*pPoints)[i] = transform * (*pPoints)[i];
    }
  }
}

ml::vec3f MeshSegment::computeAreaWeightedNormal() {
  ml::vec3f norm;
  const std::vector<ml::vec3ui>& meshIndices = mesh->getIndices();
  const auto& V = mesh->getVertices();
  const bool needTransform = !transform.isApprox(geo::Transform::Identity());
  const ml::mat4f mlxf = geo::to<ml::mat4f>(transform);
  for (size_t triIndex : triIndices) {
    const auto& t = meshIndices[triIndex];
    ml::vec3f v0 = V[t[0]].position, v1 = V[t[1]].position, v2 = V[t[2]].position;
    if (needTransform) {
      v0 = mlxf*v0;
      v1 = mlxf*v1;
      v2 = mlxf*v2;
    }
    norm += 0.5f * ((v1 - v0) ^ (v2 - v0));
    //geo::Tri tri(v0, v1, v2);
    //norm += tri.getAreaWeightedNormal();
  }
  return norm;
}

}  // namespace segmentation
}  // namespace sg
