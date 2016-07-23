#include "common.h"  // NOLINT

#include "core/ModelInstance.h"

#include "core/Model.h"
#include "geo/OBB.h"
#include "geo/voxels.h"
#include "segmentation/MeshSegment.h"

namespace sg {
namespace core {

geo::BBox ModelInstance::computeWorldBBoxFast() const {
  vec<ml::vec3f> bboxPts = model.bbox.getVertices();
  geo::BBox bbox;
  for (const auto& p : bboxPts) {
    bbox.extend(m_localToParent * geo::vec3f(p));
  }
  return bbox;
}

geo::BBox ModelInstance::computeWorldBBox() const {
  const auto& V = model.flattenedMesh.getVertices();
  geo::BBox bbox;
  for (const auto& p : V) {
    bbox.extend(m_localToParent * geo::vec3f(p.position));
  }
  return bbox;
}

geo::OBB ModelInstance::computeWorldOBB() const {
  geo::OBB obb;
  if (model.voxelCenters.size() > 0) {
    pointSetToOBB(model.voxelCenters, &obb, &m_localToParent);
  } else {
    SG_LOG_ERROR << "No voxels loaded for model " << model.id
      << ": Please call ensureVoxelization before computeWorldOBB";
  }
  return obb;
}

bool ModelInstance::getSegments(VecSegPtr* pSegs, bool constrainZup, float shrinkPercentile /*= 0*/) const {
  if (model.segments == nullptr) {
    SG_LOG_WARN << "No segments for model " << model.id;
    return false;
  }
  const VecSegPtr& rawSegs = *model.segments;

  // We need to get a copy of the mInst model segments with a transform applied
  pSegs->clear();
  pSegs->reserve(rawSegs.size());
  for (const SegPtr rawSeg : rawSegs) {
    SegPtr seg = rawSeg->copy(constrainZup, shrinkPercentile, m_localToParent);
    pSegs->emplace_back(seg);
  }
  return true;
}

float voxelOverlapRaw(const ModelInstance& mInstA, const ModelInstance& mInstB, int kSamples /*=inf*/) {
  // Voxel center points from mInstA
  const int numModelVoxels = std::min(kSamples, static_cast<int>(mInstA.model.voxelCenters.cols()));
  const geo::Matrix3Xf& voxelPtsAModelSplace = mInstA.model.voxelCenters.leftCols(numModelVoxels);

  // Transfrom from mInstA model space to mInstB voxel space
  const geo::Transform xformA2B = geo::from(mInstB.model.modelToVoxel) * mInstB.getParentToLocal() * mInstA.getLocalToParent();
  const geo::Matrix3Xf voxelPtsA = xformA2B * voxelPtsAModelSplace;

  // Compute and return overlap
  const ml::BinaryGrid3& voxelsB = mInstB.model.solidVoxels;
  const int numVoxelsAinB = geo::numPointsInVoxels(voxelPtsA, voxelsB);
  const float voxelVolA = mInstA.model.voxelVolume();
  const float overlapInMetersCubed = voxelVolA * numVoxelsAinB;
  return overlapInMetersCubed;
}

float voxelOverlapNormalized(const ModelInstance& mInstA, const ModelInstance& mInstB, int kSamples /*=inf*/) {
  const float volA = mInstA.model.solidVoxels.getNumOccupiedEntries() * mInstA.model.voxelVolume();
  return voxelOverlapRaw(mInstA, mInstB, kSamples) / volA;
}

float voxelOverlapRaw(const vec<ModelInstance>& modelInstances, int kSamples /*=inf*/) {
  // Compute raw overlap between all models
  float totalOverlap = 0.0;
  for (size_t i = 0; i < modelInstances.size(); ++i) {
    for (size_t j = i+1; j < modelInstances.size(); ++j) {
      float overlap = voxelOverlapRaw(modelInstances.at(i), modelInstances.at(j), kSamples);
      totalOverlap += overlap;
    }
  }
  return totalOverlap;
}



}  // namespace core
}  // namespace sg
