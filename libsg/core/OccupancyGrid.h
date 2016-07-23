#pragma once

#include "libsg.h"  // NOLINT

namespace sg {
namespace core {

//! Represents occupied vs free space observations over a volume covered by a voxel grid
class OccupancyGrid {
 public:
  enum OccupancyType {
    OccupancyType_Free = 0,
    OccupancyType_Unknown = 1,
    OccupancyType_Occupied = 2,
    OccupancyType_UnknownOrOccupied = 3,
    OccupancyType_Count = 4
  };
  //! Returns a BinaryGrid3 which is set at each voxel based on the occupancy type
  const ml::BinaryGrid3& get(OccupancyType occType) const {
    switch (occType) {
    case OccupancyType_Free: return free();
    case OccupancyType_Unknown: return unknown();
    case OccupancyType_Occupied: return occupied();
    case OccupancyType_UnknownOrOccupied: return unknownOrOccupied();
    default: throw std::invalid_argument("Invalid occupancy type: " + occType); 
    }
  }

  ml::vec3ul getDimensions() const {
    return m_grid.getDimensions();
  }

  float voxelSize() const {
    return m_voxelSize;
  }

  //! Returns a BinaryGrid3 which is set at each voxel if the voxel was observed to be occupied at least once
  const ml::BinaryGrid3& occupied() const;

  //! Returns a BinaryGrid3 which is set at each voxel if the voxel was observed to be free at least once
  const ml::BinaryGrid3& free() const;

  //! Returns a BinaryGrid3 which is set at each voxel if the voxel had no observations
  const ml::BinaryGrid3& unknown() const;

  //! Returns a BinaryGrid3 which is set at each voxel if the voxel is unobserved, or if was observed occupied
  const ml::BinaryGrid3& unknownOrOccupied() const;

  //! Read an occupancy grid file and returns whether successful
  bool readFromGridFile(const string& file);

  //! Save occupancy grid to file
  void saveToFile(const string& filename) const;

  //! Returns transform from grid voxel coordinates to world space
  const ml::mat4f& gridToWorld() const { return m_gridToWorld; }

  //! Returns transform from world space to grid voxel coordinates
  const ml::mat4f& worldToGrid() const { return m_worldToGrid; }

 private:
  // Encapsulates sensor data
  struct OccupancyVoxel {
    uint16_t freeCtr = 0;
    uint16_t occupiedCtr = 0;
    uint16_t unkCtr = 0;
    uint16_t label = 0;
  };

  static const int kFormatVersion = 1;

  float m_voxelSize;        //! Real world size (cube side length) of each voxel
  float m_depthMin;         //! Minimum sensor integration distance
  float m_depthMax;         //! Maximum sensor integration distance
  ml::mat4f m_worldToGrid;  //! Transform from world coordinates to grid voxel coordinates
  ml::mat4f m_gridToWorld;  //! Transform from grid voxel coordinates to world coordinates
  ml::Grid3<OccupancyVoxel> m_grid;     //! Original occupancy grid
  mutable ml::BinaryGrid3 m_free;       //! Free space voxels
  mutable ml::BinaryGrid3 m_occupied;   //! Occupied voxels
  mutable ml::BinaryGrid3 m_unknown;    //! Unobserved/unknown voxels
  mutable ml::BinaryGrid3 m_unkOrOcc;   //! Unknown OR occupied voxels
  mutable vec<string> m_labels;         //! label set for voxels
};

}  // namespace core
}  // namespace sg


