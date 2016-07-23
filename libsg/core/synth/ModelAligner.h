#pragma once

#include "libsg.h"  // NOLINT

#include "geo/geo.h"

namespace sg {
namespace core {
namespace synth {

//! Encapsulates alignment parameters for placing a model
struct Alignment {
  float theta;            //! rotation around normal
  float scale;            //! model scale factor
  geo::Matrix3f R;        //! rotation matrix
  geo::Vec3f t;           //! translation vector
  geo::Transform xform;   //! total transform (s*R,t), wasteful but convenient
};

//! Encapsulates a scored alignment
struct ScoredAlignment : public Alignment {
  float
    gos,    //! geometric overlap score for known voxels
    gus,    //! geometric overlap score for unknown voxels
    fos,    //! face orientation score
    sos,    //! skeleton score nonoverlap with models
    ovs,    //! obb volume score
    score;  //! overall score
};
inline ostream& operator<<(ostream& os, const ScoredAlignment& p) {
  os << "ScoredAlignment: {s:" << p.scale << ", rot:" << p.theta
    << ", trans:" << p.t.format(geo::kEigenJSONFormat) << ", gos: "
    << p.gos << ", gus: " << p.gus << ", fos: " << p.fos << ", sos:"
    << p.sos << ", ovs:" << p.ovs << ", ts:" << p.score << "}";
  return os;
}

//! Encapsulates alignment state for debugging and visualization
struct AlignmentState {
  geo::BBox searchBBox, segsBBox, modelBBox;
  ModelInstance* pInst;
  geo::Matrix3Xf modelVoxelPts;
  geo::Transform xform;
};

//! Parameters for performing an alignment
struct ModelAlignerParams {
  ModelAlignerParams() = default;
  explicit ModelAlignerParams(const util::Params& p);
  float
    minStep,        // minimum XYZ step size
    minScale,       // minimum scale factor
    maxScale,       // maximum scale factor
    scaleStep,      // step size for scaling in [minScale,maxScale]
    supportZ,       // height at which support plane for this alignment exists (not used if < 0)
    wGeo,           // occupied voxel overlap weight
    wGeoUnk,        // unknown voxel overlap weight
    wSkel,          // skeleton non-overlap weight
    wOBBVol,        // OBB volume to model volume match weight (penalizes model volumes smaller than OBB)
    wFace;          // model-front to skeleton gaze match weight (penalizes models facing away from gaze)
  int
    thetaDivs,      // thetas to check (thetaStep = 2pi / thetaDivs)
    numVoxels,      // model voxel random samples for volumetric overlap
    numSkelSamples; // skeleton random samples for skeleton range non-overlap (collision avoidance)

  //! In-the-loop render callback for debugging alignment process
  typedef std::function<void(const AlignmentState& state)> RenderAlignmentStateFn;
  RenderAlignmentStateFn* pRenderFun = nullptr;
};

//! Aligns models given constraints
class ModelAligner {
 public:
  void init(const ModelAlignerParams& params) {
    m_params = params;
  }

  //! Return Transform for mInst that has max overlap with region of scan in
  //! obb, and min overlap with skels in skelRange.
  geo::Transform alignToScan(const Scan& scan,
                             const SkelRange& skelRange,
                             const ModelInstance& mInst,
                             const geo::OBB& obb) const;

  //! Return Transform for mInst that minimizes overlap with skelRange
  geo::Transform align(const ModelInstance& mInst,
                       const SkelRange& skelRange,
                       const geo::OBB& obb) const;

  //! Return Transform for mInst that minimizes overlap with skelRange
  geo::Transform orient(const ModelInstance& mInst,
                        const SkelRange& skelRange) const;

 private:
  ModelAlignerParams m_params;
};

}  // namespace synth
}  // namespace core
}  // namespace sg
