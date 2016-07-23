#include "common.h"  // NOLINT

#include "core/OccupancyGrid.h"

#include <mLibCore.h>

#include <io/io.h>

namespace sg {
namespace core {

const unsigned kThresh = 1;

const ml::BinaryGrid3& OccupancyGrid::occupied() const {
  if (m_occupied.getNumElements()) { return m_occupied; }
  const auto dim = m_grid.getDimensions();
  m_occupied.allocate(dim.x, dim.y, dim.z);
  m_occupied.clearVoxels();
  for (size_t k = 0; k < dim.z; k++) {
    for (size_t j = 0; j < dim.y; j++) {
      for (size_t i = 0; i < dim.x; i++) {
        const OccupancyVoxel& voxel = m_grid(i, j, k);
        if (voxel.occupiedCtr >= kThresh) {  // occupied
          m_occupied.setVoxel(i, j, k);
        }
      }
    }
  }
  return m_occupied;
}

const ml::BinaryGrid3& OccupancyGrid::free() const {
  if (m_free.getNumElements()) { return m_free; }
  const auto dim = m_grid.getDimensions();
  m_free.allocate(dim.x, dim.y, dim.z);
  m_free.clearVoxels();
  for (size_t k = 0; k < dim.z; k++) {
    for (size_t j = 0; j < dim.y; j++) {
      for (size_t i = 0; i < dim.x; i++) {
        const OccupancyVoxel& voxel = m_grid(i, j, k);
        if (voxel.freeCtr >= kThresh) {  // free
          m_free.setVoxel(i, j, k);
        }
      }
    }
  }
  return m_free;
}

const ml::BinaryGrid3& OccupancyGrid::unknown() const {
  if (m_unknown.getNumElements()) { return m_unknown; }
  const auto dim = m_grid.getDimensions();
  m_unknown.allocate(dim.x, dim.y, dim.z);
  m_unknown.clearVoxels();
  for (size_t k = 0; k < dim.z; k++) {
    for (size_t j = 0; j < dim.y; j++) {
      for (size_t i = 0; i < dim.x; i++) {
        const OccupancyVoxel& voxel = m_grid(i, j, k);
        if ((voxel.freeCtr == 0 && voxel.occupiedCtr == 0)) {  // unknown is not free or occupied
          if (voxel.unkCtr >= kThresh) {
            m_unknown.setVoxel(i, j, k);
          }
        }
      }
    }
  }
  return m_unknown;
}

const ml::BinaryGrid3& OccupancyGrid::unknownOrOccupied() const {
  if (m_unkOrOcc.getNumElements()) { return m_unkOrOcc; }
  const auto dim = m_grid.getDimensions();
  m_unkOrOcc.allocate(dim.x, dim.y, dim.z);
  m_unkOrOcc.clearVoxels();
  for (size_t k = 0; k < dim.z; k++) {
    for (size_t j = 0; j < dim.y; j++) {
      for (size_t i = 0; i < dim.x; i++) {
        const OccupancyVoxel& voxel = m_grid(i, j, k);
        if (voxel.occupiedCtr >= kThresh ||
            (voxel.unkCtr >= kThresh && voxel.freeCtr == 0 && voxel.occupiedCtr == 0)) {  // occupied or unknown
          m_unkOrOcc.setVoxel(i, j, k);
        }
      }
    }
  }
  return m_unkOrOcc;
}

//! Read an occupancy grid file. File starts with ASCII header of following format (excluding comments after #):
//!   voxelgrid\t1
//!   dimensions\tdimX dimY dimZ  # vec3ul
//!   worldToGrid\tfloat00 float01 float02 floa03 float10 ... float33  # mat4f
//!   depthMin\tfloat
//!   depthMax\tfloat
//!   voxelSize\tfloat
//!   labels\tId1,Id2,Id3,...
//!   data:
//! Binary section follows with pairs of uint32 (freeCtr,occCtr) in iteration order x, y, z (outermost to innermost)
bool OccupancyGrid::readFromGridFile(const string& file) {
  if (!io::fileExists(file)) { return false; }

  const auto readerFun = [&] (istream& is) {
    // Parse header
    string symbol, line;
    size_t dimX, dimY, dimZ;
    getline(is, line);
    if (line.compare("voxelgrid\t1") != 0) {
      SG_LOG_ERROR << "Unknown VoxelGrid format. "
        << "First line should be 'voxelgrid\t1'. "
        << "Line is: " << line << endl;
    }
    while (line.compare("data:") != 0) {
      getline(is, line);
      std::stringstream ss(line);
      ss >> symbol;
      if (symbol == "dimensions") {
        ss >> dimX >> dimY >> dimZ;
      } else if (symbol == "worldToGrid") {
        for (int i = 0; i < 16; ++i) {
          ss >> m_worldToGrid.matrix[i];
        }
      } else if (symbol == "depthMin") {
        ss >> m_depthMin;
      } else if (symbol == "depthMax") {
        ss >> m_depthMax;
      } else if (symbol == "voxelSize") {
        ss >> m_voxelSize;
      } else if (symbol == "labels") {
        string labelsString;
        ss >> labelsString;
        m_labels = ml::util::split(labelsString, ',');
      }
    }

    m_gridToWorld = m_worldToGrid.getInverse();

    // Allocate and read grid binary section
    m_grid.allocate(dimX, dimY, dimZ);
    is.read(reinterpret_cast<char*>(m_grid.getData()), sizeof(OccupancyVoxel) * m_grid.getNumElements());
  };

  io::readFile(file, readerFun, std::ios::binary);

  return true;
}

void OccupancyGrid::saveToFile(const string& filename) const {
  ofstream outFile(filename, std::ios::binary);
  outFile << "voxelgrid\t" << kFormatVersion << endl;
  outFile << "dimensions\t" << m_grid.getDimX() << " " << m_grid.getDimY() << " " << m_grid.getDimZ() << endl;
  outFile << "worldToGrid\t";
  for (int i = 0; i < 15; ++i) {
    outFile << m_worldToGrid.getData()[i] / m_voxelSize << " ";
  }
  outFile << m_worldToGrid.getData()[15] << endl;
  outFile << "depthMin\t" << m_depthMin << endl;
  outFile << "depthMax\t" << m_depthMax << endl;
  outFile << "voxelSize\t" << m_voxelSize << endl;
  outFile << "labels\t";
  const int numLabels = static_cast<int>(m_labels.size());
  for (int i = 0; i < numLabels; ++i) {
    outFile << m_labels[i];
    if (i == numLabels - 1) {
      outFile << endl;
    } else {
      outFile << ",";
    }
  }
  outFile << "data:" << endl;

  outFile.write(reinterpret_cast<const char*>(m_grid.getData()), sizeof(OccupancyVoxel) * m_grid.getNumElements());

  outFile.close();
}

}  // namespace core
}  // namespace sg
