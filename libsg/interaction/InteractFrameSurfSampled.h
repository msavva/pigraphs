#pragma once

#include "libsg.h"  // NOLINT
#include "geo/OBB.h"
#include "io/io.h"
#include "stats/Counter.h"

namespace sg {

namespace vis { struct IFVisParams; }

namespace interaction {

//! A coordinate frame in which interactions take place
class InteractionFrame : public io::Serializable {
 public:
  BOOST_BINARY_SERIALIZABLE_FUNCS
  //! Create an InteractionFrame with the given world halfwidth dimensions and numBinsPerDim along each axis
  explicit InteractionFrame(const geo::Vec3f& halfwidths = geo::Vec3f(1, 1, 1), int numBinsPerDim = 10);
  InteractionFrame(float h, int numBinsPerDim) : InteractionFrame(geo::Vec3f(h, h , h), numBinsPerDim) { }

  //! Return the total support of this InteractionFrame centered on skel within scene
  //! NOTE: Repositions InteractionFrame at skel before support computation
  double support(const core::Skeleton& skel, const core::Scan* pScan);

  //! Return the support of this InteractionFrame centered on skel wrt to a model instance
  //! NOTE: Repositions InteractionFrame at skel before support computation
  double support(const core::Skeleton& skel, const core::ModelInstance& modelInst);

  //! Return the support of this InteractionFrame centered on skel wrt to a voxel grid with the given world to grid transform
  //! NOTE: Repositions InteractionFrame at skel before support computation
  double support(const core::Skeleton& skel, const ml::BinaryGrid3& voxelGrid, 
                 const ml::mat4f& worldToGrid, bool skipIfOutsideGrid);

  //! Clear all stored interaction points
  void clearPoints();
  //! Clear all stored interaction points with given id
  void clearPoints(const string& id);

  //! Repositions this InteractionFrame so that it contains Skeleton skel and axis Y faces in the body normal direction
  void reposition(const core::Skeleton& skel);
  //! Recenters InteractionFrame so that it is centered at center with front along positive x axis
  void recenter(const geo::Vec3f& center);

  //! Returns current world space OBB of this InteractionFrame
  const geo::OBB& getWorldOBB() const {
    return m_obb;
  }

  //! Return number of bins per dimension
  int getNumBinsPerDim() const { return m_numBinsPerDim; }

  //! Convenience struct for accessing counters along with their center positions in OBB space
  struct Bin { const stats::Counter<string>& counter; const geo::Vec3f center; };
  //! Returns a vector of all bins with non-zero counts in this InteractionFrame
  vec<Bin> getBins() const;

  //! Returns frame bins colored by joint and weighted by joint weight
  struct ColoredBin { ml::vec3f pos;  ml::vec4f val; };
  typedef ml::SparseGrid3<vec<ColoredBin>> ColoredBinGrid;
  void getWeightedColorBins(const vis::IFVisParams& p, ColoredBinGrid* out, bool dropLowWeightBins = true) const;

  //! Computes vector of bin centroid positions in world space
  void getBinWorldCentroids(vec<geo::Vec3f>* pCentroids) const;

  //! Return the size along each dimension of bins in this InteractionFrame
  float getBinSize() const {
    return m_binDim;
  }

  //! Return index of bin with maximum total count and total count for that bin
  std::pair<int,size_t> getBinWithMaxTotalCount() const;
  //! Return index of bin with maximum total count for given labels and total count for that bin
  std::pair<int,size_t> getBinWithMaxTotalCount(const vec<string>& labels) const;
  std::pair<int,size_t> getBinWithMaxTotalCount(const string& label) const {
    vec<string> v(1, label);
    return getBinWithMaxTotalCount(v);
  }
  //! Return index of bin with maximum total count for given filter and total count for that bin
  std::pair<int,size_t> getBinWithMaxTotalCount(std::function<bool(const string&)> filterFn) const;

  //! Returns vector of sampled interaction points with given id in world space
  vec<ml::vec3f> getWorldPoints(const string& id) const;

  //! Returns vector of sampled points with given id in OBB space
  const vec<geo::Vec3f>& getPoints(const string& id) const {
    return m_points.at(id);
  }

  const stats::Counter<string>& getTotalsCounter() const {
    return m_observationsCount;
  }

  const vec<stats::Counter<string>>& getAllBinCounters() const {
    return m_bins;
  }

  //! String id to use for Skeleton points
  static const string kSkeletonId;

  //! Add worldPoint with given id to this InteractionFrame
  void addPoint(const string& id, const geo::Vec3f& worldPoint, bool boundChecks = false);
  void addPoint(const string& id, const ml::vec3f& worldPoint, bool boundChecks = false);

  ml::vec3i binIndexToCoords(int idx) const {
    assert(idx >= 0);
    const int xIdx = idx / m_numBinsPerDimSq,
              rest = idx % m_numBinsPerDimSq,
              yIdx = rest / m_numBinsPerDim,
              zIdx = rest % m_numBinsPerDim;
    return ml::vec3i(xIdx, yIdx, zIdx);
  }

 private:
  //! Returns flattened bin index corresponding to point with obbCoords, or returns -1 if outside InteractionFrame
  int binIndex(const geo::Vec3f& obbCoords) const {
    const int xIdx = binIndex(obbCoords[0], -m_halfDims[0], m_halfDims[0]),
              yIdx = binIndex(obbCoords[1], -m_halfDims[1], m_halfDims[1]),
              zIdx = binIndex(obbCoords[2], -m_halfDims[2], m_halfDims[2]);
    if (xIdx < 0 || yIdx < 0 || zIdx < 0) {
      return -1;
    } else {
      return xIdx * m_numBinsPerDimSq + yIdx * m_numBinsPerDim + zIdx;
    }
  }
  //! Returns bin index for x given min and max of dimension, or returns -1 if outside range
  int binIndex(float x, float min, float max) const {
    if (x < min || x > max) {  // Underflow or overflow
      return -1;
    }
    int idx = static_cast<int>(floor((x - min) * m_invBinSize));
    if (idx == m_numBinsPerDim) { idx--; }  // Include upper boundary since we are a closed range on both ends
    assert(idx >= 0 && idx < m_numBinsPerDim);
    return idx;
  }

  //! Returns the center of a given bin index in local OBB coordinates
  geo::Vec3f binCenter(int idx) const {
    const auto coo = binIndexToCoords(idx);
    geo::Vec3f c = -m_halfDims;
    c[0] += (coo.x + 0.5f) * m_binDim;
    c[1] += (coo.y + 0.5f) * m_binDim;
    c[2] += (coo.z + 0.5f) * m_binDim;
    return c;
  }

  //! Returns the center of a given bin index in world coordinates
  geo::Vec3f binCenterWorld(int idx) const {
    return m_obb.localToWorld() * binCenter(idx);
  }

  const geo::Vec3f m_halfDims;                  //! Half-dimensions of this InteractionFrame
  const int m_numBinsPerDim;                    //! Number of bins along each dimension
  const float m_binDim;                         //! bin dimensions
  const float m_invBinSize;                     //! cached inverse bin size
  const int m_numBinsPerDimSq;                  //! cached numBins*numBins
  geo::OBB m_obb;                               //! Bounding box of frame in world space
  map<string, vec<geo::Vec3f>> m_points;        //! Interaction points inside this frame (expressed in OBB coordinates)
  vec<stats::Counter<string>> m_bins;           //! Volume bins with per-id counter
  stats::Counter<string> m_observationsCount;   //! Total count of point observations for each id

  friend class boost::serialization::access;

  //! boost serialization function
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {  // NOLINT
    boost::serialization::split_member(ar, *this, version);
  }

  template <typename Archive>
  void save(Archive& ar, const unsigned int version) const {  // NOLINT
    ar & m_halfDims;
    ar & m_numBinsPerDim;
    ar & m_binDim;
    // Skip serialization of m_invBinSize, m_numBinsPerDimSq, m_obb
    //ar & m_points;  // huge -- don't store until really needed
    ar & m_bins;
    ar & m_observationsCount;
  }

  template <typename Archive>
  void load(Archive& ar, const unsigned int version) {  // NOLINT
    ar & const_cast<geo::Vec3f&>(m_halfDims);
    ar & const_cast<int&>(m_numBinsPerDim);
    ar & const_cast<float&>(m_binDim);
    //ar & m_points;  // huge -- don't store until really needed
    ar & m_bins;
    ar & m_observationsCount;
    // Recompute m_invBinSize, m_numBinsPerDimSq, m_obb
    const_cast<float&>(m_invBinSize) = 1 / m_binDim;
    const_cast<int&>(m_numBinsPerDimSq) = m_numBinsPerDim * m_numBinsPerDim;
    m_obb = geo::OBB(geo::Vec3f::Zero(), m_halfDims, geo::Matrix3f::Identity());
  }
};

}  // namespace interaction
}  // namespace sg


