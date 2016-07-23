#include "common.h"  // NOLINT

#include "core/synth/ModelAligner.h"

#include <mLibCore.h>

#include "core/Model.h"
#include "core/ModelInstance.h"
#include "core/OccupancyGrid.h"
#include "core/Scan.h"
#include "core/Skeleton.h"
#include "core/synth/synth.h"
#include "geo/OBB.h"
#include "geo/voxels.h"
#include "math/math.h"
#include "util/timer.h"
#include "util/Params.h"

namespace sg {
namespace core {
namespace synth {

using ml::vec3f;  using ml::vec4f;  using ml::mat4f;  using math::DEF_RAND;
using geo::Vec3f;  using geo::OBB;  using geo::BBox;  using geo::Matrix3f;
using geo::Transform;  using geo::Matrix3Xf;

//! Computes alignment search domain BBox given model and scan region bboxen, supportZ constraint and minStep
BBox computeSearchBox(const BBox& modelBox, const BBox& scanBox, const float supportZ, const float minStep) {
  // Figure out the min and max of the positions we need to place the model
  const Vec3f
    hwidths = modelBox.sizes() * 0.5f,
    m1 = scanBox.max() - hwidths,  // coincident max face of model with max face of scan box
    m2 = scanBox.min() + hwidths;  // coincident min face of model with min face of scan box
  Vec3f
    min = m1.cwiseMin(m2),
    max = m1.cwiseMax(m2);

  // Clamp z close to support height if height is known
  if (supportZ >= 0.f) {
    min.z() = supportZ - minStep + hwidths.z();
    max.z() = supportZ + minStep + hwidths.z();
  }

  SG_LOG_TRACE << "[computeSearchBox]" << min.format(geo::kEigenJSONFormat) << " to "
                                       << max.format(geo::kEigenJSONFormat);
  SG_LOG_TRACE << "[computeSearchBox][hWidth]" << hwidths.format(geo::kEigenJSONFormat);
  return BBox(min, max);
}

// Scores the alignment of model, scan and skeleton given by alignment parameters
// in align, and records the scores in align score fields
void scoreAlignment(const Matrix3Xf& modelVoxelPts, const ml::BinaryGrid3& modelVoxels,
                    const Transform& modelToVoxel, const Scan& scan, const BBox& scoredRegion,
                    const Matrix3Xf& skelPts,  const ModelAlignerParams& p,
                    const Vec3f& modelFrontDir, const Vec3f& skelGazeDir,
                    const float mInstVolNormalizedByOBB, ScoredAlignment* align) {
  const bool
    useGeo    = p.wGeo > 0,
    useGeoUnk = p.wGeoUnk > 0,
    useSkel   = p.wSkel > 0,
    useOBBvol = p.wOBBVol > 0,
    useFace   = p.wFace != 0;

  // occupied geometry score
  align->gos = 0.f;
  if (useGeo) {
    const ml::BinaryGrid3& sceneVoxelsOcc = scan.getOccupancyGrid().occupied();
    align->gos = geo::ratioPointsInVoxels(modelVoxelPts, sceneVoxelsOcc, &scoredRegion);
  }

  // unknown geometry score
  align->gus = 0.f;
  if (useGeoUnk) {
    const ml::BinaryGrid3& sceneVoxelsUnk = scan.getOccupancyGrid().unknown();
    align->gus = geo::ratioPointsInVoxels(modelVoxelPts, sceneVoxelsUnk, &scoredRegion);
  }

  // skeleton non overlap score
  align->sos = 1.f;
  if (useSkel) {
    const Transform xformInv = align->xform.inverse();
    align->sos = 1.f - geo::ratioPointsInVoxels((modelToVoxel * xformInv) * skelPts, modelVoxels);
  }

  // model facing orientation score
  align->fos = 1.f;
  if (useFace) {
    const Vec3f currModelFront = (align->R * modelFrontDir).normalized();
    const float frontFaceScore = 1.f - ((currModelFront.dot(skelGazeDir) + 1.f) * .5f);  // 1 = towards, 0 = away
    if (p.wFace > 0) {
      align->fos = std::pow(frontFaceScore, p.wFace);
    } else {
      align->fos = std::pow(1.0f - frontFaceScore, -p.wFace);
    }
  }

  // OBB volume match score
  align->ovs = 0.f;
  if (useOBBvol) {
    const float sCubed = align->scale * align->scale * align->scale;
    const float currModelVolRatio = sCubed * mInstVolNormalizedByOBB;
    align->ovs = std::min(currModelVolRatio, 1.f);
  }

  // total score
  align->score = (   (useGeo     ? p.wGeo * align->gos      : 0.f)
                   + (useGeoUnk  ? p.wGeoUnk * align->gus   : 0.f) )
                   * (useFace    ? p.wFace * align->fos              : 1.f)
                   * (useSkel    ? std::pow(align->sos, p.wSkel)     : 1.f)
                   * (useOBBvol  ? std::pow(align->ovs, p.wOBBVol)   : 1.f);
}

// Returns AABB around segsOBB and also transforms to scan grid space in segsBBoxGrid
BBox computeSegsBBox(const OBB& segsOBB, float rescale, const Transform& world2scanGrid,
                     BBox* segsBBoxGrid) {
  OBB obbEnlarged = segsOBB;
  obbEnlarged.rescale(Vec3f(rescale, rescale, rescale));
  const BBox segsBBox = obbEnlarged.toAABB();

  // transform segs bbox into scan grid space
  *segsBBoxGrid = geo::transformBBox(world2scanGrid, segsBBox);
  for (int i = 0; i < 3; ++i) {
    segsBBoxGrid->min()[i] = floorf(segsBBoxGrid->min()[i] - 0.5f);
    segsBBoxGrid->max()[i] =  ceilf(segsBBoxGrid->max()[i] + 0.5f);
  }
  SG_LOG_TRACE << "[getSegsBBoxen]" << segsBBoxGrid->min().format(geo::kEigenJSONFormat)
    << " to " << segsBBoxGrid->max().format(geo::kEigenJSONFormat);
  
  return segsBBox;
}

Transform ModelAligner::alignToScan(const Scan& scan, const SkelRange& skelRange,
                                    const ModelInstance& mInst, const OBB& segsOBB) const {
  util::Timer totalTimer("[alignToScan]");
  AlignmentState state;  // for debugging
  state.pInst = const_cast<ModelInstance*>(&mInst);

  const float mInstVolNormalizedByOBB = mInst.computeWorldOBB().volume() / segsOBB.volume();
  const vec<Vec3f> modelBBoxVerts = geo::toVecVec3f(mInst.model.bbox.getVertices());
  const Matrix3f mInstSR = mInst.getLocalTransform().linear();
  const Matrix3Xf skelPtsM = getSkelPoints(skelRange, m_params.numSkelSamples);
  const Vec3f
    skelGazeDir   = geo::vec3f(skelRange[0].get().gazeDirection),  // TODO(MS): Use average gaze direction!
    modelFrontDir = geo::vec3f(mInst.model.front);

  // For subsampling voxel overlap computation
  const int numModelVoxels = std::min(m_params.numVoxels, static_cast<int>(mInst.model.voxelCenters.cols()));
  const Matrix3Xf& modelVoxelPtsInModelSpace = mInst.model.voxelCenters.leftCols(numModelVoxels);

  const ml::BinaryGrid3& modelVoxels = mInst.model.solidVoxels;
  const Transform modelToVoxel = geo::from(mInst.model.modelToVoxel);
  Transform world2scanGrid = geo::from(scan.getOccupancyGrid().worldToGrid());
  world2scanGrid.translation() += Vec3f(0.5f, 0.5f, 0.5f);  // for correct rounding of floating point voxel coords to grid

  // slightly enlarge segs bbox for a bit of padding around target region
  BBox segsBBoxInScanGrid;  // this will hold BBox in scan grid space
  const BBox segsBBox = computeSegsBBox(segsOBB, 1.05f, world2scanGrid, &segsBBoxInScanGrid);
  state.segsBBox = segsBBox;

  // compute steps
  const Vec3f transStep = (segsOBB.axesLengths() * 0.1f).cwiseMax(m_params.minStep);
  const int numThetas = m_params.thetaDivs;
  const float thetaStep = math::constants::PI2f / numThetas;
  vec<float> thetas(numThetas);
  for (int iTheta = 0; iTheta < numThetas; ++iTheta) {
    thetas[iTheta] = iTheta * thetaStep;
  }

  // loop temp variables
  ScoredAlignment maxAlign; float maxAlignScore = -10.f;  // max scoring alignment
  // iterate over possible scales and rotations for the model
  for (float scale = m_params.minScale; scale < m_params.maxScale; scale += m_params.scaleStep) {
    #pragma omp parallel for
    for (int iTheta = 0; iTheta < numThetas; ++iTheta) {
      util::Timer timer(to_string(iTheta));
      // create new alignment for current rotation and scale
      ScoredAlignment currAlign;
      currAlign.theta = thetas[iTheta];
      currAlign.scale = scale;
      currAlign.R = Eigen::AngleAxisf(currAlign.theta, Vec3f::UnitZ()).toRotationMatrix() * mInstSR;
      currAlign.xform.linear() = currAlign.scale * currAlign.R;
      currAlign.xform.translation().setZero();

      // precompute rotated and scaled model voxels in scan voxel space
      const Transform model2scanGrid = world2scanGrid.linear() * currAlign.xform;
      Matrix3Xf modelVoxelPts = model2scanGrid * modelVoxelPtsInModelSpace;

      // get the size of the (scaled and rotated) model bbox and search bbox
      const BBox modelBBox = geo::ptsBBox(modelBBoxVerts, currAlign.xform);
      if (m_params.pRenderFun != nullptr) { state.modelBBox = modelBBox; }
      const BBox searchBox = computeSearchBox(modelBBox, segsBBox, m_params.supportZ, m_params.minStep);
      if (m_params.pRenderFun != nullptr) { state.searchBBox = searchBox; }

      // iterate over possible positions for model
      float& tx = currAlign.t[0]; float& ty = currAlign.t[1]; float& tz = currAlign.t[2];
      const Vec3f& min = searchBox.min(); const Vec3f& max = searchBox.max();
      for (tx = min[0]; tx < max[0]; tx += transStep[0]) {
        for (ty = min[1]; ty < max[1]; ty += transStep[1]) {
          for (tz = min[2]; tz < max[2]; tz += transStep[2]) {

            // update translation component of model xform and model voxels
            currAlign.xform.translation() = currAlign.t;
            const Vec3f modelVoxelTranslation = world2scanGrid.linear() * currAlign.t + world2scanGrid.translation();

            // add current translation to model voxels
            modelVoxelPts.colwise() += modelVoxelTranslation;

            // compute current alignment score
            scoreAlignment(modelVoxelPts, modelVoxels, modelToVoxel, scan, segsBBoxInScanGrid,
                           skelPtsM, m_params, modelFrontDir, skelGazeDir,
                           mInstVolNormalizedByOBB, &currAlign);

            // set new max alignment if score is higher
            #pragma omp flush(maxAlignScore)
            if (currAlign.score > maxAlignScore) {
              #pragma omp critical (saveMaxAlign)
              {
                if (currAlign.score > maxAlignScore) {
                  maxAlignScore = currAlign.score;
                  maxAlign = currAlign;
                }
              }
            }

            // if debug render callback given update state and call
            if (m_params.pRenderFun != nullptr) {
              state.modelVoxelPts = currAlign.xform * modelVoxelPtsInModelSpace;
              state.modelBBox = modelBBox;
              state.modelBBox.translate(currAlign.t);
              state.xform = currAlign.xform;
              (*m_params.pRenderFun)(state);
            }

            // remove current translation for next loop
            modelVoxelPts.colwise() -= modelVoxelTranslation;
          }
        }
      }
      // Debugging
      const auto diff = (max - min);
      SG_LOG_TRACE << "[Retrieval]" << "xSteps="   << diff[0] / transStep[0]
                                   << "\tySteps=" << diff[1] / transStep[1]
                                   << "\tzSteps=" << diff[2] / transStep[2];
    }
  }
  SG_LOG_TRACE << "[Retrieval]" << "max: " << maxAlign;

  return maxAlign.xform;
}

Transform ModelAligner::align(const ModelInstance& mInst, const SkelRange& skelRange,
                              const OBB& obb) const {
  const float
    wSkel = m_params.wSkel,
    wFace = m_params.wFace;
  const bool
    useSkel = wSkel > 0,
    useFace = wFace != 0;

  const vec<Vec3f> modelBBoxVerts = geo::toVecVec3f(mInst.model.bbox.getVertices());
  const Matrix3f mInstSR = mInst.getLocalTransform().linear();
  const Matrix3Xf skelPtsM = getSkelPoints(skelRange, m_params.numSkelSamples);
  const Vec3f
    skelGazeDir = geo::vec3f(skelRange[0].get().gazeDirection),  // TODO(MS): Use average gaze direction!
    modelFrontDir = geo::vec3f(mInst.model.front);

  ScoredAlignment maxAlign; maxAlign.score = -10.f;
  Matrix3f R; Vec3f t;
  Transform xform = mInst.getLocalTransform();
  const ml::BinaryGrid3& modelVoxels = mInst.model.solidVoxels;
  const Transform modelToVoxel = geo::from(mInst.model.modelToVoxel);

  const BBox gridBBox = obb.toAABB();
  const Vec3f stepSize = (obb.axesLengths() * 0.1f).cwiseMax(m_params.minStep);
  const float thetaStep = math::constants::PI2f / m_params.thetaDivs;
  // Iterate over possible scales and rotations for the model
  for (float scale = m_params.minScale; scale < m_params.maxScale; scale += m_params.scaleStep) {
    for (float dTheta = 0.f; dTheta < math::constants::PI2f; dTheta += thetaStep) {
      R = Eigen::AngleAxisf(dTheta, Vec3f::UnitZ()).toRotationMatrix() * mInstSR;
      const Vec3f currModelFront = (R * modelFrontDir).normalized();
      const float frontFaceScore = 1.f - ((currModelFront.dot(skelGazeDir) + 1.f) * .5f);  // 1 = towards, 0 = away

      util::Timer timer(to_string(dTheta));

      // Compute scale and rotate matrix for our model 
      xform.linear() = scale * R;
      xform.translation().setZero();
      // Get the size of the model bounding box (scaled and rotated)
      const BBox mBbox = geo::ptsBBox(modelBBoxVerts, xform);

      // Figure out the min and max of the positions we need to place the model
      const Vec3f
        hwidths = mBbox.sizes() * 0.5f,
        m1 = gridBBox.max() - hwidths,  // Pos for coincident max face of model AABB with max face of seg AABB
        m2 = gridBBox.min() + hwidths,  // Pos for coincident min face of model AABB with min face of seg AABB
        min = m1.cwiseMin(m2),
        max = m1.cwiseMax(m2);
      SG_LOG_TRACE << "[Retrieval][SearchOBB]" << min.format(geo::kEigenJSONFormat) << " to "
                  << max.format(geo::kEigenJSONFormat);
      SG_LOG_TRACE << "[Retrieval][SearchOBB][hWidth]" << hwidths.format(geo::kEigenJSONFormat);

      // Iterate over possible positions for model
      for (float dX = min[0]; dX < max[0]; dX += stepSize[0]) {
        t[0] = dX;
        for (float dY = min[1]; dY < max[1]; dY += stepSize[1]) {
          t[1] = dY;
          for (float dZ = min[2]; dZ < max[2]; dZ += stepSize[2]) {
            t[2] = dZ;

            // Update translation component of model xform
            xform.translation() = t;

            // Compute score of model transform as product of weighted component scores
            float sos = 0.f;
            if (useSkel) {
              sos = 1.f - geo::ratioPointsInVoxels((modelToVoxel * xform.inverse()) * skelPtsM, modelVoxels);
            }

            float faceScore = 1.f;
            if (useFace) {
              if (wFace > 0) {
                faceScore = std::pow(frontFaceScore, wFace);
              } else {
                faceScore = std::pow(1.0f - frontFaceScore, -wFace);
              }
            }
            const float currScore = faceScore * (useSkel ? std::pow(sos, wSkel) : 1.f);

            // Set new max alignment if score is higher
            if (currScore > maxAlign.score) {
              maxAlign.theta = dTheta;
              maxAlign.scale = scale;
              maxAlign.R = R;
              maxAlign.t = t;
              maxAlign.xform = xform;
              maxAlign.gos = 1.f;
              maxAlign.gus = 1.f;
              maxAlign.sos = sos;
              maxAlign.fos = faceScore;
              maxAlign.ovs = 1.f;
              maxAlign.score = currScore;
              SG_LOG_TRACE << "[Retrieval]" << "updated max: " << maxAlign;
            }
          }
        }
      }
    }
  }
  SG_LOG_TRACE << "[Retrieval]" << "max: " << maxAlign;

  return maxAlign.xform;
}

Transform ModelAligner::orient(const ModelInstance& mInst,
                               const SkelRange& skelRange) const {
  const Vec3f origPos = mInst.getLocalTransform().translation();

  const float
    wSkel = m_params.wSkel,
    wFace = m_params.wFace;
  const bool
    useSkel = wSkel > 0,
    useFace = wFace != 0;

  const Matrix3f mInstSR = mInst.getLocalTransform().linear();
  const Matrix3Xf skelPtsM = getSkelPoints(skelRange, m_params.numSkelSamples);
  const Vec3f
    skelGazeDir = geo::vec3f(skelRange[0].get().gazeDirection),  // TODO(MS): Use average gaze direction!
    modelFrontDir = geo::vec3f(mInst.model.front);

  ScoredAlignment maxAlign; maxAlign.score = -10.f;
  Matrix3f R; //Vec3f t;
  Transform xform = mInst.getLocalTransform();
  const ml::BinaryGrid3& modelVoxels = mInst.model.solidVoxels;
  const Transform modelToVoxel = geo::from(mInst.model.modelToVoxel);

  const float thetaStep = math::constants::PI2f / m_params.thetaDivs;
  // Iterate over possible scales and rotations for the model
  for (float scale = m_params.minScale; scale < m_params.maxScale; scale += m_params.scaleStep) {
    for (float dTheta = 0.f; dTheta < math::constants::PI2f; dTheta += thetaStep) {
      R = Eigen::AngleAxisf(dTheta, Vec3f::UnitZ()).toRotationMatrix() * mInstSR;
      const Vec3f currModelFront = (R * modelFrontDir).normalized();
      const float frontFaceScore = 1.f - ((currModelFront.dot(skelGazeDir) + 1.f) * .5f);  // 1 = towards, 0 = away

      //util::Timer timer(to_string(dTheta));

      // Compute scale and rotate matrix for our model 
      xform.linear() = scale * R;

      // Compute score of model transform as product of weighted component scores
      float ss = 0.f;
      if (useSkel) {
        ss = 1.f - geo::ratioPointsInVoxels((modelToVoxel * xform.inverse()) * skelPtsM, modelVoxels);
      }

      float faceScore = 1.f;
      if (useFace) {
        if (wFace > 0) {
          faceScore = std::pow(frontFaceScore, wFace);
        } else {
          faceScore = std::pow(1.0f - frontFaceScore, -wFace);
        }
      }
      const float currScore = faceScore * (useSkel ? std::pow(ss, wSkel) : 1.f);

      // Set new max alignment if score is higher
      if (currScore > maxAlign.score) {
        maxAlign.theta = dTheta;
        maxAlign.scale = scale;
        maxAlign.R = R;
        maxAlign.t = origPos;
        maxAlign.xform = xform;
        maxAlign.gos = 1.f;
        maxAlign.gus = 1.f;
        maxAlign.sos = ss;
        maxAlign.fos = faceScore;
        maxAlign.ovs = 1.f;
        maxAlign.score = currScore;
        SG_LOG_TRACE << "[Retrieval]" << "updated max: " << maxAlign;
      }
    }
  }
  SG_LOG_TRACE << "[Retrieval]" << "max: " << maxAlign;

  return maxAlign.xform;
}

ModelAlignerParams::ModelAlignerParams(const util::Params& globalParams) {
  const util::Params p = globalParams.extract("ModelAligner");
  minStep         = p.getWithDefault("minStep",       0.05f);
  minScale        = p.getWithDefault("minScale",      1.0f );
  maxScale        = p.getWithDefault("maxScale",      1.51f);
  scaleStep       = p.getWithDefault("scaleStep",     0.25f);
  supportZ        = p.getWithDefault("supportZ",     -1.0f );
  wGeo            = p.getWithDefault("wGeo",          1.0f );
  wGeoUnk         = p.getWithDefault("wGeoUnk",       0.5f );
  wSkel           = p.getWithDefault("wSkel",         1.0f );
  wOBBVol         = p.getWithDefault("wOBBVol",       1.0f );
  wFace           = p.getWithDefault("wFace",         0.0f );
  thetaDivs       = p.getWithDefault("thetaDivs",        16);
  numVoxels       = p.getWithDefault("numVoxels",     10000);
  numSkelSamples  = p.getWithDefault("numSkelSamples", 1000);
}

}  // namespace synth
}  // namespace core
}  // namespace sg
