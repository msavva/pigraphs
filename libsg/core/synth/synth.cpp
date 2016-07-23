#include "common.h"  // NOLINT

#include "core/synth/synth.h"

#include <mLibCore.h>

#include "core/Model.h"
#include "core/ModelInstance.h"
#include "core/Scan.h"
#include "core/Skeleton.h"
#include "geo/OBB.h"
#include "math/math.h"
#include "mesh/sampling.h"
#include "segmentation/segmentation.h"
#include "segmentation/MeshSegment.h"

namespace sg {
namespace core {
namespace synth {

using ml::vec3f;  using ml::vec4f;  using ml::mat4f;  using math::DEF_RAND;
using geo::Vec3f; using geo::OBB; using geo::BBox; using geo::Matrix3f; using geo::Transform;
using geo::Matrix3Xf;
using segmentation::ConstSegPtr; using segmentation::VecConstSegPtr;

//! Returns ratio of points contained by OBBs of segs
float ratioPointsInsideSegs(const vec<Vec3f>& pts, const VecConstSegPtr& segs) {
  size_t insidePts = 0;
  const size_t numPts = pts.size();
  if (numPts == 0) {
    return 0;
  }
  for (size_t i = 0; i < numPts; ++i) {
    for (const ConstSegPtr& pSeg : segs) {
      const segmentation::MeshSegment& seg = *pSeg;
      if (seg.obb()->contains(pts[i])) {
        ++insidePts;
        break;
      }
    }
  }
  const float insideRatio = insidePts / static_cast<float>(numPts);
  return insideRatio;
}

// helper for computing skeleton overlap with hard linear rescaling [0,0.05] -> [1,0]
float skelScore(const vec<Vec3f>& pts, const vec<OBB>& skelOBBs) {
  const auto numPts = pts.size();
  const float ratio = numPointsInsideOBBs(pts, skelOBBs) / static_cast<float>(numPts);
  return 1 - ((ratio > 0.3f) ? 1.f : ratio * 0.7f);
}

vec<Vec3f> sampleMesh(const ml::TriMeshf& mesh, size_t numSamples) {
  vec<Vec3f> samplePts;
  mesh::sampling::sampleTris(mesh, numSamples, &samplePts);
  return samplePts;
}

vec<Vec3f> sampleMeshSegment(const SegPtr segPtr, size_t numSamples) {
  vec<Vec3f> samplePts;
  segPtr->samplePoints(numSamples, &samplePts);
  return samplePts;
}

float placementScore(const ModelInstance& mInst, const VecConstSegPtr& segs, const SkelRange& skelRange,
                     size_t numSamples /*= 1000*/) {
  vec<Vec3f> samplePts = sampleMesh(mInst.model.flattenedMesh, numSamples);
  const Transform& xform = mInst.getLocalTransform();
  for (size_t i = 0; i < numSamples; ++i) {
    samplePts[i] = xform * samplePts[i];
  }
  vec<OBB> skelOBBs;
  for (const Skeleton& skel : skelRange) {
    const auto& myOBBs = skel.getBoneOBBs();
    skelOBBs.insert(skelOBBs.end(), myOBBs.begin(), myOBBs.end());
  }
  const float
    ratioModelPtsInSegOBBs = ratioPointsInsideSegs(samplePts, segs),
    skelRatio = skelScore(samplePts, skelOBBs),
    score = ratioModelPtsInSegOBBs * skelRatio;
  cout << "geoScore=" << ratioModelPtsInSegOBBs << "\tskelScore=" << skelRatio << "\tscore=" << score << endl;
  return score;
}

void placeModelInstance(const Model& model, const Scan& scene, vec<ModelInstance>* out) {
  const float
    r0 = DEF_RAND.uniform_float_01() * 1.8f - .9f,
    r1 = DEF_RAND.uniform_float_01() * 1.8f - .9f;
  const Vec3f
    randomFloorPos(r0, r1, 0.f),
    scenePos = geo::from(scene.bbox.cubeToWorldTransform()) * randomFloorPos;
  const vec4f modelColor = ml::ColorUtils::colorById<vec4f>(static_cast<int>(out->size()));
  Transform xformModelToFloor = Transform::Identity();
  xformModelToFloor.translate(Vec3f(scenePos[0], scenePos[1], model.bbox.getExtentZ() / 2));
  out->emplace_back(model, xformModelToFloor, modelColor);
}

BBox placeModelInstance(const Model& model, const VecConstSegPtr& segs, vec<ModelInstance>* out) {
  const BBox bbox = computeSegsBBox(segs);
  Transform xform = Transform::Identity();
  xform.translate(bbox.center());
  const vec4f modelColor = ml::ColorUtils::colorById<vec4f>(static_cast<int>(out->size()));
  out->emplace_back(model, xform, modelColor);
  return bbox;
}

//! Helper to return a set of numPoints points randomly sampled from the volumes of the skeletons in skelRange
Matrix3Xf getSkelPoints(const SkelRange& skelRange, int numPoints) {
  const int
    numSkels      = static_cast<int>(skelRange.size()),
    numPtsPerSkel = numPoints / numSkels;
  vec<vec3f> pts;
  pts.reserve(numPoints);
  for (const Skeleton& skel : skelRange) {
    const auto& skelPts = skel.getPointsInOBBs(numPtsPerSkel);
    pts.insert(pts.end(), skelPts.begin(), skelPts.end());
  }
  const Matrix3Xf skelPtsM = Matrix3Xf::Map(&pts[0][0], 3, pts.size());
  return skelPtsM;
}

}  // namespace synth
}  // namespace core
}  // namespace sg
