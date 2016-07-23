#pragma once

#include <mLibCore.h>

#include "libsg.h"  // NOLINT

#include "geo/geo.h"

namespace sg {
namespace segmentation {

//! Helper class for identifying important surfaces
//! This class supports two types of functions:
//!   predictXxx returns a score for each segPtr as to how likely it is to satisfy the required criterion
//!   identifyXxx returns a filtered set that with a score that is above some threshold
class SurfacePredictor {
public:
  //! Predict flat surfaces with matching surface normal
  void predictFlatSurfaces(const VecSegPtr& segPtrs, const geo::Vec3f surfaceNormal, vec<double>* pScores) const;

  //! Predict likely support surfaces for the given OBB
  void predictSupportSurfaces(const VecSegPtr& segPtrs, const geo::OBB* pObb, vec<double>* pScores) const;

  void identifySupportSurfaces(const VecSegPtr& segPtrs, const geo::OBB* pObb, 
                               const double threshold, const int limit,
                               vec<pair<SegPtr, double>>* pSurfaces) const;

  void addFiltered(const VecSegPtr& segPtrs, const vec<double>& scores, const double threshold, 
                   vec<pair<SegPtr, double>>* pFiltered) const;
private:
};

}  // namespace segmentation
}  // namespace sg


