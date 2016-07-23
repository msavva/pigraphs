#pragma once

#include "libsg.h"  // NOLINT

#include "eval/ConfusionMatrix.h"
#include "stats/Counter.h"
#include "util/index.h"

namespace sg {
namespace core {

const string CATEGORY_NONE = "";
const string CATEGORY_ANY = "any";
const string CATEGORY_UNKNOWN = "unknown";

//! Represents labeled voxels
class LabeledGrid {
 public:
  LabeledGrid(float voxelSize, const ml::vec3ui& dims, const ml::mat4f& world2grid)
    : m_voxelSize(voxelSize), m_gridDims(dims), m_world2grid(world2grid) { }
  explicit LabeledGrid(const string& file) { load(file); }

  //! Clears the grid
  void clear() {
    m_labelIndex.clear();
    m_labeledGrid.clear();
  }

  //! Label a voxel
  void setLabel(const ml::vec3i& v, const string& label) {
    int i = m_labelIndex.indexOf(label, true);
    m_labeledGrid[v] = i;
  }

  //! Get the label of a voxel
  const string& getLabel(const ml::vec3i& v) const {
    int i = getLabelIndex(v);
    if (i >= 0) {
      return m_labelIndex[static_cast<size_t>(i)];
    }
    return CATEGORY_NONE;
  }

  //! Get the label index of a voxel
  int getLabelIndex(const ml::vec3i& v) const {
    if (m_labeledGrid.exists(v)) {
      return m_labeledGrid(v);
    }
    return -1;
  }

  //! Return true if the voxel is in the grid
  bool contains(const ml::vec3i& v) const {
    return m_labeledGrid.exists(v);
  }

  //! Get the number of labels
  size_t getNumLabels() const {
    return m_labelIndex.size();
  }

  //! Get the label index 
  const util::Index<string>& getLabelIndex() const {
    return m_labelIndex;
  }

  //! Return grid
  const ml::SparseGrid3<int>& getGrid() const {
    return m_labeledGrid;
  }

  const ml::mat4f& worldToGrid() const { return m_world2grid; }
  float voxelSize() const { return m_voxelSize; }
  const ml::vec3ui& gridDims() const { return m_gridDims; }

  bool load(const string& file);
  bool save(const string& file) const;

 private:
  static const int kFormatVersion = 1;
  util::Index<string>   m_labelIndex;  //! Converts labels into integers
  ml::SparseGrid3<int>  m_labeledGrid; //! Sparse grid of labels
  float m_voxelSize;                   //! side dimension of voxels
  ml::vec3ui m_gridDims;               //! total dimensions of grid
  ml::mat4f m_world2grid;              //! transform from world to grid coords
};

struct LabeledGridComparison {
  LabeledGridComparison(const string& predictedDir, const string& truthDir);
  util::Index<string> labels;
  stats::Counter2D<string, int> confusionMatrix;
  eval::BinaryConfusionMatrixSet binaryConfusionPerLabel;
};

//! Compares predictions and ground truth with matching ids
LabeledGridComparison compare(const string& predictedDir, const string& groundTruthDir);

//! Represents labeled voxels with scores 
class ScoredLabeledGrid : public LabeledGrid {
 private:
  ml::SparseGrid3<double> m_scoredGrid; //! Sparse grid of scores
};

}  // namespace core
}  // namespace sg


