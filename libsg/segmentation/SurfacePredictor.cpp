#include "common.h"  // NOLINT

#include "segmentation/SurfacePredictor.h"

#include "segmentation/MeshSegment.h"
#include "geo/OBB.h"
#include "math/math.h"
#include "util/util.h"

namespace sg {
namespace segmentation {

void SurfacePredictor::predictFlatSurfaces(const VecSegPtr& segPtrs, const geo::Vec3f surfaceNormal, 
                                           vec<double>* pScores) const {
  pScores->clear();
  for (const SegPtr segPtr : segPtrs) {
    const auto pObb = segPtr->obb();
    // Check aspect ratio to see if the segment is relatively flat
    double flatness = 1.0; //pObb->axesLengths()
    // How similar is the dominant normal to the desired surface normal?
    //const auto& segNorm = pObb->dominantNormal();
    const auto& segNorm = segPtr->segNormal;
    double normSim = std::max(0.0f, surfaceNormal.dot( geo::vec3f(segNorm) ));
    double score = flatness * normSim;
    pScores->push_back(score);
  }
}

double estimateFaceArea(const geo::BBox& bbox,
                        const geo::DirectionIndex di) {
  const geo::Vec3f s = bbox.sizes();
  switch (di) {
    case geo::DirectionIndex::I_UP:
    case geo::DirectionIndex::I_DOWN:
      return s.x()*s.y();
    case geo::DirectionIndex::I_FRONT:
    case geo::DirectionIndex::I_BACK:
      return s.x()*s.z();
    case geo::DirectionIndex::I_LEFT:
    case geo::DirectionIndex::I_RIGHT:
      return s.y()*s.z();
    default:
      return 0.0;
  }
}

double estimateFaceArea(const geo::OBB& obb,
                        const geo::DirectionIndex d) {
  // Estimate surface area of a obb
  const geo::BBox aabb = obb.toAABB();
  return estimateFaceArea(aabb, d);
}

double evaluateSurfaceSpace(const SegPtr pParentSurface, 
                            const geo::OBB& obb,
                            const geo::DirectionIndex dir) {
  double score = 1.0;
  double parentSurfaceArea = estimateFaceArea(*pParentSurface->obb(), dir);
  double childBBFaceArea = estimateFaceArea(obb, dir);
  bool acceptable = false;
  if (parentSurfaceArea <= 0.0) {
    score = 0.0;
  } else if (parentSurfaceArea > 0.0 && parentSurfaceArea < childBBFaceArea) {
    double r = (childBBFaceArea - parentSurfaceArea)/parentSurfaceArea;
    if (r > 0.5) {
      score = 0.0;
    } else {
      score = math::lerp(0.0, 0.5, 1.0, 0.0, r);
    }
  }
  if (score > 0) {
    SG_LOG_DEBUG << "Surface area score of " << score 
      << " from parent area " << parentSurfaceArea 
      << " and child area " << childBBFaceArea;
//    ml::vec3f parentBBDims;// = pParentSurface->parentBBFaceDims;
//    ml::vec3f childBBFaceDim;
//    // Make sure the parent surface width and height is large enough
//    // TODO: Allow some tolerance
//    if (parentBBDims.x > childBBFaceDim.x * 0.90f && parentBBDims.y > childBBFaceDim.y * 0.90f) {
//      float z = std::max(childBBFaceDim.z, 0.1f);
//      //bool isInternal = isInternalSurface(parent.node, parentSurface.meshSurface, z, 1.0f);
//      //acceptable = !isInternal;
//      acceptable = true;
//    } else {
//      SG_LOG_INFO << "Parent surface is too small: parent surface " <<
//        pParentSurface->id  + " has dimensions " << parentBBDims <<
//        ", child face has dimensions " << childBBFaceDim;
//    }
  } else {
    SG_LOG_DEBUG << "Parent surface is too small: parent surface " <<
      pParentSurface->id << " has area " << parentSurfaceArea <<
      ", child face has area " << childBBFaceArea;
  }
  return score;
}

void SurfacePredictor::predictSupportSurfaces(const VecSegPtr& segPtrs, const geo::OBB* pObb, 
                                              vec<double>* pScores) const {
  // Lets see what are the upward facing flat surfaces
  predictFlatSurfaces(segPtrs, geo::DIR_UP.vec, pScores);
  if (pObb != nullptr) {
    // Filter by surfaces that are large enough and have enough clearance
    for (size_t i = 0; i < pScores->size(); ++i) {
      const SegPtr segPtr = segPtrs[i];
      if ((*pScores)[i] > 0.005) {
        double surfaceSpaceScore = evaluateSurfaceSpace(segPtr, *pObb, geo::DIR_UP.index);
        (*pScores)[i] *=  surfaceSpaceScore;
      }
    }
  }
}

void SurfacePredictor::identifySupportSurfaces(const VecSegPtr& segPtrs, const geo::OBB* pObb, 
                                               const double threshold, const int limit, 
                                               vec<pair<SegPtr, double>>* pSurfaces) const {
  vec<double> scores;
  util::predicate_pair_desc2asc1<SegPtr, double> pred;
  predictSupportSurfaces(segPtrs, pObb, &scores);
  addFiltered(segPtrs, scores, threshold, pSurfaces);
  // Sort by score
  std::sort(pSurfaces->begin(), pSurfaces->end(), pred);
  // Trim to limit
  if (limit >= 0 && pSurfaces->size() > limit) {
    pSurfaces->resize(limit);
  }
}

void SurfacePredictor::addFiltered(const VecSegPtr& segPtrs, const vec<double>& scores, 
                                   const double threshold, vec<pair<SegPtr, double>>* pFiltered) const {
  assert(segPtrs.size() == scores.size());
  for (size_t i = 0; i < scores.size(); ++i) {
    if (scores[i] > threshold) {
      pFiltered->push_back(std::make_pair(segPtrs[i], scores[i]));
    }
  }
}

}  // namespace segmentation
}  // namespace sg
