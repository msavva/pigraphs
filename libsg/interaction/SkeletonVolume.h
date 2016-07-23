#pragma once

#include "libsg.h"  // NOLINT
#include "core/Skeleton.h"

#include <functional>
#include <list>

#include <mLibCore.h>

namespace sg {
namespace interaction {

class SkeletonVolume {
 public:
  using Skeleton = Skeleton;

  //! Create a SkeletonVolume integration grid with given voxelSize
  SkeletonVolume(float voxelSize = 0.01f) {
    m_voxelSize = voxelSize;
    m_invVoxelSize = 1.f / voxelSize;
  }

  //! Encapsulates observed counts of skeleton joints at each voxel grid location
  struct Voxel {
    Voxel() {
      jointCount.assign(0);
      count = 0;
    }
    int count;
    std::array<int, Skeleton::kNumJoints> jointCount;
  };

  //! Clear this SkeletonVolume
  void clear() {
    m_grid.clear();
  }

  //! Add the given Skeleton observation into this SkeletonVolume
  void add(const Skeleton& skel) {
    for (int i = 0; i < Skeleton::kNumJoints; ++i) {
      const ml::vec3f pos = skel.jointPositions[i];
      Voxel& v = m_grid[toVirtualVoxelCoords(pos)];
      v.jointCount[i]++;
      v.count++;
    }
  }

  //! Retrieve a list of voxel positions in real world coordinates that match the given predicate lambda
  std::list<ml::vec3f> getVoxelPositionsByPredicate(const std::function<bool(const Voxel&)>& predicate) const {
    std::list<ml::vec3f> res;
    for (ml::SparseGrid3<Voxel>::const_iterator iter = m_grid.begin(); iter != m_grid.end(); iter++) {
      if (predicate(iter->second)) {
        res.push_back(toRealWorldCoords(iter->first));
      }
    }
    return res;
  }

  //! Return the voxel closest to the world space position pos
  const Voxel& operator[](const ml::vec3f& pos) const {
    const ml::vec3i voxelCoords = toVirtualVoxelCoords(pos);
    return m_grid[voxelCoords];
  }

 private:
  //! from meters (realWorldCoords) to our virtual voxel space
  ml::vec3i toVirtualVoxelCoords(const ml::vec3f& realWorldCoords) const {
    return static_cast<ml::vec3i>(realWorldCoords * m_invVoxelSize + ml::vec3f(0.5f));
  }

  //! from virtual voxel space to realWorldCoords (in meters)
  ml::vec3f toRealWorldCoords(const ml::vec3i& virtualVoxelCoords) const {
    return static_cast<ml::vec3f>(virtualVoxelCoords) * m_voxelSize;
  }

  float m_voxelSize;
  float m_invVoxelSize;
  ml::SparseGrid3<Voxel> m_grid;
};

}  // namespace interaction
}  // namespace sg


