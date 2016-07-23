#pragma once

#include "libsg.h"  // NOLINT
#include "geo/geo.h"
#include "io/io.h"
#include "util/index.h"
#include "util/util.h"

namespace sg {
namespace util {

template <typename T>
class Grid {
 public:
  //! Constructors
  Grid() : m_rows(0), m_cols(0), m_data(nullptr) { }
  Grid(int rows, int cols) : m_data(nullptr) {
    allocate(rows, cols);
  }
  Grid(int rows, int cols, const T& clearVal) : Grid(rows, cols) {
    clear(clearVal);
  }
  Grid(const Grid<T>& o) : Grid(o.rows(), o.cols()) {
    std::copy(o.begin(), o.end(), begin());
  }
  Grid<T>& operator=(const Grid<T>& o) {
    allocate(o.m_rows, o.m_cols);
    std::copy(o.begin(), o.end(), begin());
    return *this;
  }
  ~Grid() {
    deallocate();
  }

  //! Initialize the Grid to have numRows rows and numCols columns. Warning: this will deallocate all existing memory
  void init(int numRows, int numCols) {
    allocate(numRows, numCols);
  }

  //! Return by reference element at (row,col)
  T& operator() (int iRow, int iCol) {
    return get(iRow, iCol);
  }
  //! Return by const-reference element at (row,col)
  const T& operator() (int iRow, int iCol) const {
    return get(iRow, iCol);
  }
  //! Return number of rows (1st dimension) in this grid
  int rows() const {
    return m_rows;
  }
  //! Return number of columns (2nd dimension) in this grid
  int cols() const {
    return m_cols;
  }

  //! Set all elements to val
  void clear(const T& val) {
    std::fill(begin(), end(), val);
  }

  //! Return const pointer to first element
  const T* begin() const {
    return m_data;
  }

  //! Return pointer to first element
  T* begin() {
    return m_data;
  }

  //! Return const pointer to end (one beyond last element)
  const T* end() const {
    return m_data + m_rows * m_cols;
  }

  //! Return pointer to end (one beyond last element)
  T* end() {
    return m_data + m_rows * m_cols;
  }

  //! Return number of elements in this grid
  size_t size() const {
    return m_rows * m_cols;
  }

  //! Return copy of row with given index
  vec<T> row(int iRow) const {
    const int iBase = iRow * m_cols;
    return vec<T>(&m_data[iBase], &m_data[iBase + m_cols]);
  }

  //! Return copy of column with given index
  vec<T> col(int iCol) const {
    vec<T> result(m_rows);
    for (int iRow = 0; iRow < m_rows; ++iRow) {
      result[iRow] = get(iRow, iCol);
    }
    return result;
  }

 protected:
  void allocate(int numRows, int numCols) {
    deallocate();
    m_rows = numRows;
    m_cols = numCols;
    m_data = new T[m_rows * m_cols];
  }

  void deallocate() {
    m_rows = 0;
    m_cols = 0;
    if (m_data != nullptr) {
      delete[] m_data;
      m_data = nullptr;
    }
  }

  T& get(int iRow, int iCol) {
    assert(iRow >= 0 && iRow < m_rows && iCol >= 0 && iCol < m_cols);
    return m_data[iRow * m_cols + iCol];
  }

  const T& get(int iRow, int iCol) const {
    assert(iRow >= 0 && iRow < m_rows && iCol >= 0 && iCol < m_cols);
    return m_data[iRow * m_cols + iCol];
  }

  int m_rows, m_cols;
  T* m_data;

 private:
  // boost serialization support
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {  // NOLINT
    boost::serialization::split_member(ar, *this, version);
  }
  template <typename Archive>
  void save(Archive& ar, const unsigned int) const {  // NOLINT
    ar << m_rows;
    ar << m_cols;
    ar << boost::serialization::make_array(m_data, m_rows * m_cols);
  }
  template <typename Archive>
  void load(Archive& ar, const unsigned int) {  // NOLINT
    ar >> m_rows;
    ar >> m_cols;
    allocate(m_rows, m_cols);
    ar >> boost::serialization::make_array(m_data, m_rows * m_cols);
  }
};


//! A Grid over numeric values.  Assumes that T supports operator> and operator<
template <typename T>
class NumericGrid : public Grid<T> {
 public:
  //! Returns maximum element in this Grid.  If pointers piRowMax or piColMax are given, also sets row and column
  //! indices at which maximum was found.
  T maxElement(int* piRowMax = nullptr, int* piColMax = nullptr) const {
    assert(m_rows * m_cols > 0);
    int iRowMax = 0, iColMax = 0;  T maxVal = m_data[0];
    for (int iRow = 0; iRow < m_rows; ++iRow) {
      const int iBase = iRow * m_cols;
      for (int iCol = 0; iCol < m_cols; ++iCol) {
        const T& currVal = m_data[iBase + iCol];
        if (currVal > maxVal) {
          iRowMax = iRow;  iColMax = iCol;  maxVal = currVal;
        }
      }
    }
    if (piRowMax != nullptr) { *piRowMax = iRowMax; }
    if (piColMax != nullptr) { *piColMax = iColMax; }
    return maxVal;
  }

  //! Returns minimum element in this Grid.  If pointers piRowMax or piColMax are given, also sets row and column
  //! indices at which minimum was found.
  T minElement(int* piRowMin = nullptr, int* piColMin = nullptr) const {
    assert(m_rows * m_cols > 0);
    int iRowMin = 0, iColMin = 0;  T minVal = m_data[0];
    for (int iRow = 0; iRow < m_rows; ++iRow) {
      const int iBase = iRow * m_cols;
      for (int iCol = 0; iCol < m_cols; ++iCol) {
        const T& currVal = m_data[iBase + iCol];
        if (currVal < minVal) {
          iRowMin = iRow;  iColMax = iCol;  minVal = currVal;
        }
      }
    }
    if (piRowMin != nullptr) { *piRowMin = iRowMin; }
    if (piColMin != nullptr) { *piColMin = iColMin; }
    return minVal;
  }

 private:
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {  // NOLINT
    ar & boost::serialization::base_object< Grid<T> >(*this);
  }
};

template <typename T>
void saveGrid(const string& filename,
              const string& rowIdxName, const Index<string>& rowIdx,
              const string& colIdxName, const Index<string>& colIdx,
              const Grid<T>& grid) {
  io::ensurePathToFileExists(filename);
  ofstream ofs(filename);
  ofs << rowIdxName << "_" << colIdxName << "," << join(colIdx.labels(), ",") << endl;
  for (int i = 0; i < rowIdx.size(); ++i) {
    const string& v = rowIdx[i];
    const vec<T> row = grid.row(i);
    ofs << v << "," << join(row, ",") << endl;
  }
  ofs.close();
  SG_LOG_INFO << "Saved " << filename;
}

template <typename T>
void saveGrid(const string& filename,
              const string& idxName, const Index<string>& idx,
              const Grid<T>& grid) {
  saveGrid(filename, idxName, idx, idxName, idx, grid);
}

template <typename T>
void saveGridAsRows(const string& filename,
                    const string& rowIdxName, const Index<string>& rowIdx,
                    const string& colIdxName, const Index<string>& colIdx,
                    const Grid<T>& grid) {
  io::ensurePathToFileExists(filename);
  ofstream ofs(filename);
  ofs << rowIdxName << "," << colIdxName << ",prob" << endl;
  for (int i = 0; i < rowIdx.size(); ++i) {
    const string& iName = rowIdx[i];
    const vec<T> row = grid.row(i);
    for (int j = 0; j < colIdx.size(); ++j) {
      const string& jName = colIdx[j];
      ofs << iName << "," << jName << "," << row[j] << endl;
    }
  }
  ofs.close();
  SG_LOG_INFO << "Saved " << filename;
}

template <typename T>
void saveVector(const string& filename,
                const string& idxName, const Index<string>& index,
                const string& valName, const vec<T>& vals) {
  io::ensurePathToFileExists(filename);
  ofstream ofs(filename);
  ofs << idxName << "," << valName << endl;
  for (int i = 0; i < vals.size(); ++i) {
    ofs << index[i] << "," << vals[i] << endl;
  }
  ofs.close();
}

template <typename T>
void loadGrid(const string& filename,
              std::function<T(const string&)> fromStrFn,
              Grid<T>* pGrid) {
  ifstream ifs(filename);
  const auto lines = io::getLines(filename);
  if (lines.size() < 1) {
    throw ml::MLibException("Unexpected number of lines in " + filename);
  }
  vec<string> fields;
  const string& header = lines[0];
  tokenize(header, ",", &fields);
  if (fields.size() < 1) {
    throw ml::MLibException("Unexpected number of fields in " + filename);
  }
  pGrid->init(static_cast<int>(lines.size()) - 1, static_cast<int>(fields.size()) - 1);
  for (int i = 1; i < lines.size(); i++) {
    tokenize(lines[i], ",", &fields);
    for (int j = 1; j < fields.size(); j++) {
      (*pGrid)(i - 1, j - 1) = fromStrFn(fields[j]);
    }
  }
  ifs.close();
}

//! Grid which bins a continuous 2D domain over NumT into a discrete 2D bins of a given homogeneous size
template <typename T, typename NumT = float>
class BinnedGrid : public Grid<T> {
 public:
  //! Create BinnedGrid spanning domain (minX, minY) x (maxX, maxY) with binSize bins
  BinnedGrid(NumT minX, NumT minY, NumT maxX, NumT maxY, NumT binSize) {
    init(minX, minY, maxX, maxY, binSize);
  }

  //! Default constructor creates single [0,1]^2 bin
  BinnedGrid() {
    init(0, 0, 1, 1, 1);
  }

  //! Initialize the Grid to span (minX, minY) x (maxX, maxY) domain with bins of size binSize.
  //! Warning: this will deallocate all existing memory.
  void init(NumT minX, NumT minY, NumT maxX, NumT maxY, NumT binSize) {
    m_min[0] = minX;  m_min[1] = minY;  m_max[0] = maxX;  m_max[1] = maxY; m_binSize = binSize;
    const int numRows = static_cast<int>(ceil((maxX - minX) / binSize)),
              numCols = static_cast<int>(ceil((maxY - minY) / binSize));
    allocate(numRows, numCols);
  }

  //! Return whether the point (x,y) is within this grid's bounds
  bool contains(NumT x, NumT y) const {
    return (x >= m_min[0] && x <= m_max[0] && y >= m_min[1] && y <= m_max[1]);
  }

  //! Get the (iRow,iCol) coordinates of a given point
  pair<int, int> binIndices(NumT x, NumT y) const {
    assert(x >= m_min[0] && x <= m_max[0] && y >= m_min[1] && y <= m_max[1]);
    int iRow = static_cast<int>(floor((x - m_min[0]) / m_binSize + static_cast<NumT>(0.5)));
    if (iRow == m_rows) { --iRow; }  // Include top boundary
    int iCol = static_cast<int>(floor((y - m_min[1]) / m_binSize + static_cast<NumT>(0.5)));
    if (iCol == m_cols) { --iCol; }  // Include top boundary
    return std::make_pair(iRow, iCol);
  }

  //! Return the element in the bin that contains the given point
  const T& get(NumT x, NumT y) const {
    const auto iRowCol = binIndices(x, y);
    return Grid<T>::get(iRowCol.first, iRowCol.second);
  }

  //! Return the element in the bin that contains the given point
  T& get(NumT x, NumT y) {
    const auto iRowCol = binIndices(x, y);
    return Grid<T>::get(iRowCol.first, iRowCol.second);
  }

  //! Return the element in the bin that contains the given point
  template <typename Vec2>
  const T& get(const Vec2& pos) const {
    const auto iRowCol = binIndices(pos[0], pos[1]);
    return Grid<T>::get(iRowCol.first, iRowCol.second);
  }

  //! Return the element in the bin that contains the given point
  template <typename Vec2>
  T& get(const Vec2& pos) {
    const auto iRowCol = binIndices(pos[0], pos[1]);
    return Grid<T>::get(iRowCol.first, iRowCol.second);
  }

  //! Return the minimum point of this BinnedGrid's domain
  const geo::Vec2f& min() const {
    return m_min;
  }

  //! Return the maximum point of this BinnedGrid's domain
  const geo::Vec2f& max() const {
    return m_max;
  }

  //! Return the dimensions of this BinnedGrid's domain
  geo::Vec2f dimensions() const {
    return m_max - m_min;
  }

  //! Return size of bins
  NumT binSize() const {
    return m_binSize;
  }

 protected:
  geo::Vec2f m_min, m_max;
  NumT m_binSize;
};

}  // namespace util
}  // namespace sg


