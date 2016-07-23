#pragma once

#include "libsg.h"  // NOLINT

namespace sg {
namespace geo {

// Basic utility functions for working with grids
template <typename BinaryGridT>
inline bool addBinaryGrid(const BinaryGridT& V, BinaryGridT* pGrid, size_t* pAdded = nullptr) {
  // Check the grid dimensions match
  const ml::vec3ul& gridDims = pGrid->getDimensions();
  const ml::vec3ul& vDims = V.getDimensions();
  size_t added = 0;
  if (gridDims == vDims) {
    for (size_t x = 0; x < vDims.x; ++x) {
      for (size_t y = 0; y < vDims.y; ++y) {
        for (size_t z = 0; z < vDims.z; ++z) {
          if (V.isVoxelSet(x,y,z)) {
            if (!pGrid->isVoxelSet(x,y,z)) {
              pGrid->setVoxel(x,y,z);
              added++;
            }
          }
        }
      }
    }
    if (pAdded) {
      (*pAdded) = added;
    }
    return true;
  } else {
    return false;
  }
}

}  // namespace geo
}  // namespace sg

