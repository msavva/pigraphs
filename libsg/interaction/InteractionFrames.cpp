#include "common.h"  // NOLINT

#include "interaction/InteractionFrames.h"

// ReSharper disable once CppUnusedIncludeDirective
#include <ext-boost/serialization.h>

#include "core/Model.h"
#include "core/ModelInstance.h"
#include "core/OccupancyGrid.h"
#include "core/Scan.h"
#include "interaction/SkeletonInteraction.h"

// ReSharper disable once CppUnusedIncludeDirective
#include "util/eigen_boost_serialization.h"

namespace sg {
namespace interaction {

BOOST_BINARY_SERIALIZABLE_IMPL(InteractionFrames)

InteractionFrames::InteractionFrames() {
  m_iframes.resize(static_cast<size_t>(InteractionFrameType::kCount));
  for (size_t i = 0; i < m_iframes.size(); ++i) {
    m_iframes[i] = nullptr;
  }
  m_iframeWeights.resize(static_cast<size_t>(InteractionFrameType::kCount));
  for (size_t i = 0; i < m_iframeWeights.size(); ++i) {
    m_iframeWeights[i] = 1;
  }
  m_iframeWeights[static_cast<int>(InteractionFrameType::kSkeleton)] = 0.1;
}

InteractionFrames::InteractionFrames(const geo::Vec3f& halfwidths, int numBinsPerDim) : InteractionFrames() {
  for (size_t i = 0; i < m_iframes.size(); ++i) {
    m_iframes[i] = std::make_shared<InteractionFrame>(halfwidths, numBinsPerDim);
  }
}

void InteractionFrames::clear() {
  for (size_t i = 0; i < m_iframes.size(); ++i) {
    if (m_iframes[i] != nullptr) {
      m_iframes[i]->clearPoints();
    }
  }
}

void InteractionFrames::reposition(const Skeleton& skel) {
  for (size_t i = 0; i < m_iframes.size(); ++i) {
    if (m_iframes[i] != nullptr) {
      m_iframes[i]->reposition(skel);
    }
  }
}

void InteractionFrames::recenter(const geo::Vec3f& center) {
  for (size_t i = 0; i < m_iframes.size(); ++i) {
    if (m_iframes[i] != nullptr) {
      m_iframes[i]->recenter(center);
    }
  }
}

double InteractionFrames::support(const Skeleton& skel, const core::Scan* pScan) {
  if (!pScan) { return 0.0; }

  const ml::BinaryGrid3& scanVoxelGrid = pScan->getOccupancyGrid().unknownOrOccupied();
  const ml::mat4f& worldToGrid = pScan->getOccupancyGrid().worldToGrid();

  return support(skel, scanVoxelGrid, worldToGrid, true);
}

double InteractionFrames::support(const Skeleton& skel, const core::ModelInstance& modelInstance) {
  const ml::BinaryGrid3& voxelGrid = modelInstance.model.solidVoxels;
  const ml::mat4f worldToModel = geo::to<ml::mat4f>(modelInstance.getParentToLocal());
  const ml::mat4f worldToGrid =  modelInstance.model.modelToVoxel * worldToModel;

  return support(skel, voxelGrid, worldToGrid, false);
}

double InteractionFrames::support(const Skeleton& skel, 
                                  const ml::BinaryGrid3& voxelGrid,
                                  const ml::mat4f& worldToGrid,
                                  bool skipIfOutsideGrid) {
  // Sum up support from individual interaction frames
  double totalSupport = 0;
  for (size_t i = 0; i < m_iframes.size(); ++i) {
    double w = m_iframeWeights[i];
    if (m_iframes[i] != nullptr && abs(w) > 0) {
      double iframeSupport = m_iframes[i]->support(skel, voxelGrid, worldToGrid, skipIfOutsideGrid);
      totalSupport += iframeSupport * w;
    }
  }
  return totalSupport;
}

}  // namespace interaction
}  // namespace sg
