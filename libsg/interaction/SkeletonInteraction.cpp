#include "common.h"  // NOLINT

#include "interaction/SkeletonInteraction.h"
#include "core/OccupancyGrid.h"
#include "geo/OBB.h"
#include "math/math.h"
#include "segmentation/MeshSegment.h"
#include "segmentation/Part.h"

namespace sg {
namespace interaction {

using ml::vec2f;
using ml::vec3f;

void contactFeats(const Skeleton& skel, const Joint& joint, const ConstPartPtr part, vecf* feats) {
  const vec3f
    pS = part->closestSurfacePoint(joint.position),
    pB = joint.position,
    nS = part->dominantNormal();
  const vec3f& nB = skel.bodyPlaneNormal;
  relFeats(pB, nB, pS, nS, feats);
}

void contactFeats(const Skeleton& skel, const Joint& joint, const geo::OBB* pOBB, vecf* feats) {
  geo::Vec3f jp = geo::Vec3f(joint.position);
  const vec3f
    pS = geo::to<vec3f>(pOBB->closestSurfacePoint(jp)),
    pB = joint.position,
    nS = geo::to<vec3f>(pOBB->dominantNormal());
  const vec3f& nB = skel.bodyPlaneNormal;
  relFeats(pB, nB, pS, nS, feats);
}

void gazeFeats(const Skeleton& skel, const ConstPartPtr part, vecf* feats) {
  const vec3f& pHead  = skel.jointPositions[Skeleton::JointType_Head];
  const vec3f& nHead  = skel.bodyPlaneNormal;
  const vec3f  pS     = part->closestSurfacePoint(pHead);
  const vec3f  nS     = part->dominantNormal();
  relFeats(pHead, nHead, pS, nS, feats);
}

void gazeFeats(const Skeleton& skel, const geo::OBB* pOBB, vecf* feats) {
  const vec3f& pHead = skel.jointPositions[Skeleton::JointType_Head];
  const vec3f& nHead = skel.bodyPlaneNormal;
  geo::Vec3f ph = geo::Vec3f(pHead);
  const vec3f  pS = geo::to<vec3f>(pOBB->closestSurfacePoint(ph));
  const vec3f  nS = geo::to<vec3f>(pOBB->dominantNormal());
  relFeats(pHead, nHead, pS, nS, feats);
}

void partFeats(const ConstPartPtr part, vecf* feats) {
  const auto& axes = part->axesLengths<vec3f>();
  vecf& f = *feats;
  f.resize(kFeats.at(kSeg).numDims);
  f[0] = sig(part->centroid<vec3f>().z);
  f[1] = sig(sqrtf(axes.x * axes.x + axes.y * axes.y));
  f[2] = sig(sqrtf(axes.x * axes.y));
  f[3] = sig(axes.z);
  f[4] = fabsf(part->dominantNormal().z);
}

void segToSegFeats(const segmentation::ConstSegPtr a, const segmentation::ConstSegPtr b, vecf* feats) {
  float minDistSq = math::constants::POSINFf;
  //vec3f minDistPb;
  const size_t bNumPoints = b->points.size();
  size_t bNumPointsInsideA = 0;
  for (size_t iPoint = 0; iPoint < bNumPoints; ++iPoint) {
    const vec3f& pb = b->points[iPoint];
    const float distSq = a->distance(pb);
    if (distSq == 0) { bNumPointsInsideA++; }
    if (distSq < minDistSq) {
      minDistSq = distSq;
      //minDistPb = pb;
    }
  }

  float bInAMeasure = static_cast<float>(bNumPointsInsideA) / bNumPoints;
  vecf& f = *feats;
  f.resize(kFeats.at(kSpatial).numDims);
  f[0] = sqrtf(minDistSq);
  f[1] = fabsf(vec3f::dot(a->dominantNormal(), b->dominantNormal()));
  f[2] = bInAMeasure;
}


void partToPartFeats(const ConstPartPtr a, const ConstPartPtr b, vecf* feats) {
  if (a->partType == segmentation::kPartSegment && b->partType == segmentation::kPartSegment) {
    segToSegFeats(std::static_pointer_cast<const segmentation::MeshSegment>(a), 
                  std::static_pointer_cast<const segmentation::MeshSegment>(b),
                  feats);
  } else {
    // TODO: Implement with OBBS
    float minDistSq = 0; //computeMinDistSq(a, b);
    float bInAMeasure = 0; // static_cast<float>(bNumPointsInsideA) / bNumPoints
    vecf& f = *feats;

    f.resize(kFeats.at(kSpatial).numDims);
    f[0] = sqrtf(minDistSq);
    f[1] = fabsf(vec3f::dot(a->dominantNormal(), b->dominantNormal()));
    f[2] = bInAMeasure;
  }
}

void boneFeats(const Skeleton& skel, const Joint& joint1, const Joint& joint2, vecf* feats) {
  const vec3f& pi = joint1.position;
  const vec3f& pj = joint2.position;
  auto& f = *feats;
  f.resize(kFeats.at(kBone).numDims);
  f[0] = sig(vec3f::dist(pi, pj));  // Distance along bone [0,max_d] -> [0,1]
  f[1] = sig(pj.z - pi.z);          // Vertical distance between joint positions [-z,z] -> [-1,1]
}

void jointFeats(const Skeleton& skel, const Joint& joint, vecf* feats) {
  const vec3f& p = joint.position;
  auto& f = *feats;
  f.resize(kFeats.at(kJoint).numDims);
  f[0] = sig(p.z < 0 ? 0 : p.z);
}

void comFeats(const Skeleton& skel, vecf* feats) {
  const vec3f& p = skel.centerOfMass();
  auto& f = *feats;
  f.resize(kFeats.at(kCOM).numDims);
  f[0] = sig(p.z < 0 ? 0 : p.z);
}

void comLinkFeats(const Skeleton& skel, const ConstPartPtr part, vecf* feats) {
  const vec3f
    pB  = skel.centerOfMass(),             // center of mass point
    pS = part->closestPoint(pB),           // point of contact
    nS = part->dominantNormal();           // part dominant normal
  const vec3f& nB = skel.bodyPlaneNormal;  // body normal
  relFeats(pB, nB, pS, nS, feats);
}

void comLinkFeats(const Skeleton& skel, const geo::OBB* pOBB, vecf* feats) {
  const geo::Vec3f
    pB  = geo::vec3f(skel.centerOfMass()),  // center of mass point
    pS = pOBB->closestPoint(pB),            // point of contact
    nS = pOBB->dominantNormal(),            // part dominant normal
    nB = geo::vec3f(skel.bodyPlaneNormal);  // body normal
  relFeats(pB, nB, pS, nS, feats);
}

void jointVolumeFeats(const core::OccupancyGrid& occupancyGrid, const vec3f& point, InteractionNode& node) {
  node.heightHistogramOccupied = stats::Histogram<float>(stats::Interval<float>(0.0f, 3.0f), 30, "heightHistogramOccupied");
  node.heightHistogramTotal = stats::Histogram<float>(stats::Interval<float>(0.0f, 3.0f), 30, "heightHistogramTotal");

  unsigned int count = 0, occupied = 0;
  const ml::BinaryGrid3& voxelGrid = occupancyGrid.unknownOrOccupied();
  const int radius = 10;  // num samples*2 ^ 2
  const float stepSize = 0.03f;  // 1cm step size; 30cm overall
  const float maxRadius = stepSize * stepSize * radius * radius;
  const vec3f startPosWorld = point - (radius * stepSize);

  vec3f samplePosWorld;
  samplePosWorld.z = startPosWorld.z;
  for (int z = -radius; z <= radius; z++, samplePosWorld.z += stepSize) {
    samplePosWorld.y = startPosWorld.y;
    for (int y = -radius; y <= radius; y++, samplePosWorld.y += stepSize) {
      samplePosWorld.x = startPosWorld.x;
      for (int x = -radius; x <= radius; x++, samplePosWorld.x += stepSize) {
        if ((samplePosWorld - point).lengthSq() <= maxRadius) {
          const ml::vec3ui voxelCoord(occupancyGrid.worldToGrid() * samplePosWorld);
          const float zPos = samplePosWorld.z < 0.f ? 0.f : samplePosWorld.z;
          if (voxelGrid.isValidCoordinate(voxelCoord)) { //check if inside of the scan
            if (voxelGrid.isVoxelSet(voxelCoord)) {
              occupied++;
              node.heightHistogramOccupied.add(zPos);
            }
          }
          node.heightHistogramTotal.add(zPos);
          count++;
        }
      }
    }
  }

  const float countInv = 1.f / count;
  node.free = (count - occupied) * countInv;
  node.occupied = occupied * countInv;
}

void jointVolumeFeats(const core::OccupancyGrid& occupancyGrid, const Skeleton& skel, const Joint& joint, InteractionNode& node) {
  jointVolumeFeats(occupancyGrid, joint.position, node);
}

vec<string> InteractionNode::getTargetedByVerbs() const {
  return util::tokenize(targetedBy, ",");
}

}  // namespace interaction
}  // namespace sg
