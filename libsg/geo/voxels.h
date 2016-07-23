#pragma once

#include "libsg.h"  // NOLINT

#include "geo/geo.h"
#include "geo/lineAABB.h"

#include <boost/optional.hpp>

namespace sg {
namespace geo {

//! Utility functions for working with voxels

//! Return number of pts in P (3 x n matrix), that fall within occupied voxels in V.
//! If pBbox given, only consider points inside pBbox.
template <typename BinaryGridT>
int numPointsInVoxels(const Matrix3Xf& P, const BinaryGridT& V, const BBox* pBbox = nullptr) {
  // Count how many points are both in bbox and overlap with set voxels
  const int numModelVoxels = static_cast<int>(P.cols());
  int numOverlap = 0;
  for (int iPt = 0; iPt < numModelVoxels; ++iPt) {
    const Vec3f& p = P.col(iPt);
    if (pBbox == nullptr || pBbox->contains(p)) {
      const size_t
        x = static_cast<size_t>(p[0]),
        y = static_cast<size_t>(p[1]),
        z = static_cast<size_t>(p[2]);
      if (V.isValidCoordinate(x, y, z) && V.isVoxelSet(x, y, z)) {
        numOverlap++;
      }
    }
  }
  return numOverlap;
}

//! Return [0,1] proportion of pts in P (3 x n matrix), that are in occupied voxels in V.
//! If pBbox given, only consider points inside pBbox.
template <typename BinaryGridT>
float ratioPointsInVoxels(const Matrix3Xf& P, const BinaryGridT& V, const BBox* pBbox = nullptr) {
  const int numModelVoxels = static_cast<int>(P.cols());
  const int numOverlap = numPointsInVoxels(P, V, pBbox);
  const float voxelOverlap = (numModelVoxels > 0) ? static_cast<float>(numOverlap) / numModelVoxels : 0.f;
  assert(voxelOverlap >= 0.f && voxelOverlap <= 1.f);
  return voxelOverlap;
}

template <typename BinaryGridT>
inline bool isValidCoordinate(const BinaryGridT& V, const Vec3f& p) {
  return V.isValidCoordinate(static_cast<size_t>(round(p.x())), 
                             static_cast<size_t>(round(p.y())),
                             static_cast<size_t>(round(p.z())));
}

template <typename BinaryGridT>
inline bool isVoxelSet(const BinaryGridT& V, const Vec3f& p) {
  return V.isVoxelSet(static_cast<size_t>(round(p.x())), 
                      static_cast<size_t>(round(p.y())),
                      static_cast<size_t>(round(p.z())));
}

template <typename BinaryGridT>
inline bool isValidAndSet(const BinaryGridT& V, const Vec3f& p) {
  return isValidCoordinate(V, p) && isVoxelSet(V, p);
}

template <typename BinaryGridT>
inline bool isValidAndSet(const BinaryGridT& V, size_t x, size_t y, size_t z) {
  return V.isValidCoordinate(x, y, z) && V.isVoxelSet(x, y, z);
}

template <typename BinaryGridT>
inline bool isVoxelSet(const BinaryGridT& V, const Vec3f& p,
                       const Vec3f& dir1, const Vec3f& dir2,
                       const float r, const float threshold) {
  // Consider a square of radius r around p (with dir1/dir2)
  // and return true is more than the specified threshold
  int n = 0;
  int t = 0;
  int d = static_cast<int>(ceilf(r));
  for (int i = -d; i < d; i++) {
    for (int j = -d; j < d; j++) {
      Vec3f pij = p + static_cast<float>(i)*dir1 + static_cast<float>(j)*dir2;
      n += isValidAndSet(V,
                         static_cast<size_t>(round(pij.x())), 
                         static_cast<size_t>(round(pij.y())),
                         static_cast<size_t>(round(pij.z())));
      t++;      
    }
  }
  float sratio = static_cast<float>(n)/t;
  return sratio > threshold;
}

class VoxelSlice1 {
public:
  VoxelSlice1() : m_selected(-1) { }
  VoxelSlice1(const vec<bool>& _arr, const Vec3f _s, const Vec3f& _dir, int _sel) {
    init(_arr, _s, _dir, _sel);
  }
  void init(const vec<bool>& _arr, const Vec3f _s, const Vec3f& _dir, int _sel) {
    m_occupancy = _arr;
    m_selected = _sel;
    m_start = _s;
    m_dir = _dir;
    m_end = _s + static_cast<float>(m_occupancy.size())*m_dir;
  }
  void clear() {
    m_occupancy.clear();
    m_selected = -1;
    m_start.setZero();
    m_end.setZero();
    m_dir.setZero();
  }
  Vec3f getPoint(int s) {
    return m_start + static_cast<float>(s)*m_dir;
  }
  bool isSet(int s) {
    return m_occupancy[s];
  }
  int getSelected() {
    return m_selected;
  }
  const vec<bool>& getOccupancy() {
    return m_occupancy;
  }

private:
  vec<bool>  m_occupancy;
  int m_selected;
  Vec3f m_start;
  Vec3f m_end;
  Vec3f m_dir;
};

template <typename BinaryGridT>
void getVoxelSlice1(const BinaryGridT& V, const Vec3f& gridPt, const Vec3f& gridDir,
                    const float radius, const float thresh, VoxelSlice1* pSlice) {
  const Vec3f gridPerpDir1 = geo::getPerp(gridDir);
  const Vec3f gridPerpDir2 = geo::crossNorm(gridDir, gridPerpDir1);
  if (!isValidCoordinate(V,gridPt)) {
    pSlice->clear();
    return;
  }
  // Give a 1D slice representing voxels along gridDir (with gridPt being selected)
  vec<bool> beforeOcc;
  vec<bool> afterOcc;
  Vec3f first = gridPt;
  Vec3f last = gridPt;
  Vec3f g = gridPt + gridDir;
  while (isValidCoordinate(V,g)) {
    last = g;
    afterOcc.push_back(isVoxelSet(V,g,gridPerpDir1,gridPerpDir2,radius,thresh));
    g += gridDir;
  }
  g = gridPt - gridDir;
  while (isValidCoordinate(V,g)) {
    first = g;
    beforeOcc.push_back(isVoxelSet(V,g,gridPerpDir1,gridPerpDir2,radius,thresh));
    g -= gridDir;
  }
  vec<bool> occ;
  int sel = static_cast<int>(beforeOcc.size());
  occ.resize(beforeOcc.size() + 1 + afterOcc.size());
  std::copy(beforeOcc.rbegin(), beforeOcc.rend(), occ.begin());
  occ[sel] = isVoxelSet(V,gridPt);
  std::copy(afterOcc.begin(), afterOcc.end(), occ.begin() + beforeOcc.size() + 1);
  pSlice->init(occ, first, gridDir, sel);
  SG_LOG_INFO << "Got occupied slice: " << util::join(pSlice->getOccupancy());
}

template <typename BinaryGridT>
boost::optional<Vec3f> findIntersectingPoint(const BinaryGridT& V, const Vec3f& gridPt, const Vec3f& gridDir) {
  // Find closest point from grid from gridPt along gridDir
  if (isValidCoordinate(V, gridPt)) {
    return gridPt;
  } else {
    // Create bounding box and intersect in gridDir
    BBox bbox;
    bbox.extend(Vec3f::Zero());
    bbox.extend(Vec3f(static_cast<float>(V.getDimX()-1), 
      static_cast<float>(V.getDimY()-1), static_cast<float>(V.getDimZ()-1)));
    boost::optional<Vec3f> res =
      findClosestIntersectingPoint(bbox, gridPt, gridDir, true);
    if (res) {
      SG_LOG_INFO << "Got closest intersecting point " << res.get().format(kEigenJSONFormat) 
        << " for " << gridPt.format(kEigenJSONFormat);
    }
    return res;
  }
}

template <typename BinaryGridT>
boost::optional<Vec3f> findClosestSupportPoint(const BinaryGridT& V, 
                                               const Vec3f& gridPt, 
                                               const Vec3f& gridDir,
                                               const float radius = 1.0f,
                                               const float thresh = 0.2f,
                                               bool searchBefore = true,
                                               bool searchAfter = true
) {
  // Find closest point with a solid voxel followed by an empty voxel in the direction of gridDir
  // Returns the point corresponding to the empty voxel
  VoxelSlice1 vs;
  boost::optional<Vec3f> validGridPt = findIntersectingPoint(V ,gridPt, gridDir);
  if (!validGridPt) {
    SG_LOG_WARN << "Cannot find intersecting point in grid with p=" 
      << gridPt << " dir=" << gridDir;
    return boost::none;
  }
  getVoxelSlice1(V, validGridPt.get(), gridDir, radius, thresh, &vs);
  const vec<bool>& occs = vs.getOccupancy();
  if (!occs.empty()) {
    int iSel = vs.getSelected(); 
    // Check voxels after this one
    int iAfter = -1;
    if (searchAfter) {
      bool prevOcc = (iSel > 0)? occs[iSel-1] : false;
      for (int i = iSel; i < occs.size(); ++i) {
        bool occ = occs[i];
        if (prevOcc && !occ) {
          iAfter = i;
          break;
        }
        prevOcc = occ;
      }
      if (iAfter < 0 && prevOcc) iAfter = static_cast<int>(occs.size());
    }
    // Check voxels before this one
    int iBefore = -1;
    if (searchBefore) {
      bool nextOcc = occs[iSel];
      for (int i = iSel-1; i >= 0; --i) {
        bool occ = occs[i];
        if (occ && !nextOcc) {
          iBefore = i+1;
          break;
        }
        nextOcc = occ;
      }
    }
    // Check which one is closer
    int iClosest = -1;
    if (iAfter >= 0) { 
      if (iBefore >= 0) {
        if (iSel - iBefore < iAfter - iSel) {
          iClosest = iBefore;
        } else {
          iClosest = iAfter;
        }
      } else {
        iClosest = iAfter;
      }
    } else if (iBefore >= 0) {
      iClosest = iBefore;
    }
    if (iClosest >= 0) {
      return vs.getPoint(iClosest);
    } else {
      SG_LOG_WARN << "Cannot find closest support point";
    }
  } else {
    SG_LOG_WARN << "No voxel slice";
  }
  return boost::none;
}

template <typename BinaryGridT>
boost::optional<Vec3f> findClosestSupportPoint(const BinaryGridT& V, const Vec3f& worldPt, 
                                               const Transform& worldToGrid, 
                                               const Vec3f& worldDir,
                                               const boost::optional<float> worldRadius = boost::none,
                                               const float thresh = 0.4) {
  const Vec3f gridPt = worldToGrid*worldPt;
  const Vec3f gridDir = worldToGrid.rotation()*worldDir;
  const Vec3f worldPerpDir = geo::getPerp(worldDir);
  const float gridRadius = (worldRadius)?
    (worldToGrid.linear() * (worldRadius.get()*worldPerpDir)).norm() : 1.0f;
  SG_LOG_INFO << "Find closest with pt " << worldPt.format(kEigenJSONFormat) << "," << gridPt.format(kEigenJSONFormat)
    << " dir " << worldDir.format(kEigenJSONFormat) << ", " << gridDir.format(kEigenJSONFormat)
    << " perp " << worldPerpDir.format(kEigenJSONFormat)
    << " radius " << worldRadius << ", " << gridRadius;
  boost::optional<Vec3f> gridClosest = findClosestSupportPoint(V, gridPt, gridDir, gridRadius, thresh);
  if (gridClosest) {
    const Vec3f worldClosest = worldToGrid.inverse() * gridClosest.get();
    SG_LOG_INFO << "Found closest " << worldClosest.format(kEigenJSONFormat) << ", " << gridClosest.get().format(kEigenJSONFormat);
    return worldClosest;
  } else return boost::none;
}

template <typename BinaryGridT>
boost::optional<Vec3f> findClosestSupportPoint(const BinaryGridT& solidVoxels,
                                               const BinaryGridT& surfaceVoxels,
                                               const Vec3f& worldPt, 
                                               const Transform& worldToGrid, 
                                               const Vec3f& worldDir,
                                               const boost::optional<float> worldRadius = boost::none,
                                               const float thresh = 0.4) {
  const Vec3f gridPt = worldToGrid*worldPt;
  const Vec3f gridDir = worldToGrid.rotation()*worldDir;
  const Vec3f worldPerpDir = geo::getPerp(worldDir);
  const float gridRadius = (worldRadius)?
    (worldToGrid.linear() * (worldRadius.get()*worldPerpDir)).norm() : 1.0f;
  SG_LOG_INFO << "Find closest with pt " << worldPt.format(kEigenJSONFormat) << "," << gridPt.format(kEigenJSONFormat)
    << " dir " << worldDir.format(kEigenJSONFormat) << ", " << gridDir.format(kEigenJSONFormat)
    << " perp " << worldPerpDir.format(kEigenJSONFormat)
    << " radius " << worldRadius << ", " << gridRadius;
  boost::optional<Vec3f> gridClosest = 
    findClosestSupportPoint(solidVoxels, gridPt, gridDir, gridRadius, thresh);
  if (gridClosest) {
    // Sometimes solid voxels are filled in too much,
    //  so let's go down to the surface ones
    if (!geo::isValidAndSet(surfaceVoxels, gridClosest.get())) {
      boost::optional<Vec3f> gc2 = findClosestSupportPoint(
        surfaceVoxels, gridClosest.get(), gridDir, gridRadius, thresh, true, false);
      if (gc2) {
        gridClosest = gc2;
      }
    }

    const Vec3f worldClosest = worldToGrid.inverse() * gridClosest.get();
    SG_LOG_INFO << "Found closest " << worldClosest.format(kEigenJSONFormat) << ", " << gridClosest.get().format(kEigenJSONFormat);
    return worldClosest;
  } else return boost::none;
}



}  // namespace geo
}  // namespace sg
