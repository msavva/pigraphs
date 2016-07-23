#include "common.h"  // NOLINT

#include "vis/vis.h"

#include "core/Skeleton.h"
#include "core/SkelState.h"
#include "geo/OBB.h"
#include "interaction/InteractionSet.h"
#include "interaction/SkeletonInteraction.h"
#include "math/math.h"
#include "vis/ColorIndex.h"

namespace sg {
namespace vis {

using interaction::kSkelParams;
using core::Skeleton;
using namespace geo;
using S = ml::Shapesf;

ml::TriMeshf toTriMesh(const Skeleton& skel, bool showJoints /*=true*/, bool showBones /*=true*/,
                       bool showGaze /*=true*/, bool showBodyPlane /*=true*/, bool showCenterOfMass /*=true*/,
                       bool showBoneCoords /*=false*/, vec<ml::vec4f>* pColors /*=nullptr*/) {
  //return toTriMesh2(skel);
  const vec<ml::vec3f> jointPts(std::begin(skel.jointPositions), std::end(skel.jointPositions));
  vec<ml::vec4f> jointColors(Skeleton::kNumJoints);
  if (pColors != nullptr) {
    jointColors = *pColors;
  } else {
    for (int i = 0; i < Skeleton::kNumJoints; ++i) {
      jointColors[i] = kSkelParams.kJointColors[kSkelParams.kJointMap[i]];
    }
  }

  vec<ml::TriMeshf> meshes;

  // Create joint spheres
  if (showJoints) {
    for (int i = 0; i < Skeleton::kNumJoints; ++i) {
      if (   i == Skeleton::JointType_Neck
          || i == Skeleton::JointType_ThumbLeft
          || i == Skeleton::JointType_ThumbRight
          || i == Skeleton::JointType_HandTipLeft
          || i == Skeleton::JointType_HandTipRight) { continue; }
      const float radius = i == Skeleton::JointType_Head ? 0.16f : 0.09f;
      //jointColors[i] *= skel.jointConfidences[i] > 0.8f ? 1.f : 0.f;
      meshes.push_back(S::sphere(radius, jointPts[i], 10, 10, jointColors[i]));
    }
  }

  if (showBoneCoords) {
    for (int iJoint = 0; iJoint < Skeleton::kNumJoints; ++iJoint) {
      const auto& oi = skel.jointOrientations[iJoint];
      const Quatf qi(oi.w, oi.x, oi.y, oi.z);
      const Vec3f ti = vec3f(jointPts[iJoint]);
      const Transform xform = toRotTrans(ti, qi);
      const auto mesh = coordAxesWidget(to<ml::mat4f>(xform), 0.25f);
      meshes.push_back(mesh);
    }
  }

  // Create bone cylinders
  if (showBones) {
    for (const auto& bone : skel.kBigBones) {
      meshes.push_back(S::line(skel.jointPositions[bone[0]],
                               skel.jointPositions[bone[1]],
                               Skeleton::kBoneColor, 0.05f));
    }
  }

  //// Show bone OBBs
  //if (showBones) {
  //  for (const auto& obb : skel.getBoneOBBs()) {
  //    const auto boneMesh = OBBbox(obb, Skeleton::kBoneColor);  // OBBwireframe(obb, Skeleton::kBoneColor);
  //    meshes.push_back(boneMesh);
  //  }
  //}

  // Compute view direction
  if (showGaze) {
    //const float s = skel.gazeConfidence > 0.f ? 1.f : 0.f;
    const float cosTheta = static_cast<float>(std::cos(1.f * ml::math::PI * 0.25f));
    const ml::mat3f frame = ml::mat3f::frame(skel.gazeDirection);
    ml::RNG rng(140117053);
    const int numSamples = 50;
    const ml::vec4f& gazeColor = kSkelParams.kJointColors[kSkelParams.kGazeJoint];
    for (int i = 0; i < numSamples; i++) {
      ml::vec3f dir = frame * ml::Samplef::squareToUniformCone(rng.uniform2D(), cosTheta);
      
      meshes.push_back(S::line(skel.gazePosition, skel.gazePosition + dir * 2.f,
        ml::vec4f(gazeColor.x, gazeColor.y, gazeColor.z, 0.3f), 0.002f));
    }
  }
  // Create center of mass box
  if (showCenterOfMass) {
    const ml::vec3f com = skel.centerOfMass(), comFloor(com.x, com.y, 0.f);
    meshes.push_back(S::sphere(.05f, com, 4, 4));
    meshes.push_back(S::line(com, comFloor, ml::vec4f(1, 1, 1, .5), .01f));
  }

  // Compute body plane and draw its normal
  if (showBodyPlane) {
    const ml::vec3f c = skel.bodyPlaneCentroid, n = skel.bodyPlaneNormal;
    meshes.push_back(S::line(c, c + n * 2.f, ml::vec4f(1, 1, 1, .5), .01f));
  }

  return ml::meshutil::createUnifiedMesh(meshes);
}

ml::TriMeshf toTriMesh2(const Skeleton& skel, bool showBoneCoords) {
  using core::SkelState;
  SkelState ss;
  skel2state(skel, &ss);
  vec<ml::TriMeshf> meshes;

  const AngleEncoding& enc = ss.angleEncoding;
  const auto& boneDirs = ss.getBoneDirs();
  const auto body2world = toRotTrans(ss.bodyFrame.p, fromAngleEncoding(ss.bodyFrame.q, enc));
  const auto bodyQ = fromAngleEncoding(ss.bodyFrame.q, enc);
  // Create bone cylinders
  for (const auto& bone : Skeleton::kBones) {
    const auto& src = ss.joints[bone[0]];
    const auto& tgt = ss.joints[bone[1]];
    //const auto srcQ = fromAngleEncoding(src.q, enc);
    //const auto tgtQ = fromAngleEncoding(tgt.q, enc);
    const auto absQ = (bodyQ * fromAngleEncoding(tgt.q, enc)).normalized();
    // Use bone length to compute target joint position
    const auto boneDir = vec3f(boneDirs[bone[1]][0]);
    const float boneLength = tgt.l * ss.bodyFrame.l;
    const Vec3f srcP = body2world * src.p;
    const Vec3f tgtP = boneLength * (absQ * -boneDir) + srcP;

    meshes.push_back(S::line(to<ml::vec3f>(srcP), to<ml::vec3f>(tgtP), Skeleton::kBoneColor, 0.05f));
    if (bone[1] == Skeleton::JointType::JointType_Head) {
      const Vec3f gazeP = boneLength * (absQ * Vec3f::UnitX()) + tgtP;
      meshes.push_back(S::line(to<ml::vec3f>(tgtP), to<ml::vec3f>(gazeP), Skeleton::kGazeColor, 0.05f));
    }
    //meshes.push_back(coordAxesWidget(xform, 1.2f));
  }
  ml::TriMeshf mesh = ml::meshutil::createUnifiedMesh(meshes);
  mesh.computeNormals();
  return mesh;
}

ml::TriMeshf OBBwireframe(const OBB& obb, const ml::vec4f& color, float jitter /*=0*/,
                          float thickness /*=0.01f*/) {
  ml::mat4f xform = to<ml::mat4f>(obb.localToWorld());
  if (jitter > 0) {
    for (int i = 0; i < 3; i++) { xform(i, 3) += (math::DEF_RAND.uniform_float_01() * 2.f - 1.f) * jitter; }
  }
  return S::wireframeBox(xform, color, thickness);
}

ml::TriMeshf OBBbox(const OBB& obb, const ml::vec4f& color) {
  const ml::mat4f xform = to<ml::mat4f>(obb.localToWorld());
  const ml::BoundingBox3f bbox(ml::vec3f(-1, -1, -1), ml::vec3f(1, 1, 1));
  ml::TriMeshf cube(bbox, color);
  cube.transform(xform);
  return cube;
}

ml::TriMeshf iframeWidget(const interaction::InteractionSet& iset, 
                          const interaction::InteractionFrameType iframeType,
                          const IFVisParams& p) {
  if (!iset.protoInteractionFrame) {
    SG_LOG_WARN << "Tried to visualize InteractionSet with no InteractionFrame";
    return ml::TriMeshf();
  }
  const interaction::InteractionFrames& frames = *iset.protoInteractionFrame;
  // TODO: Select appropriate frame
  return iframeWidget(*frames.getInteractionFrame(iframeType), p);
}

using interaction::InteractionFrame;

ml::TriMeshf iframeWidget(const InteractionFrame& frame, const IFVisParams& p) {
  vec<ml::TriMeshf> meshes;
  //sg::stats::toDelimited(cout, frame.getTotalsCounter());

  //const auto obbMesh = OBBwireframe(frame.getWorldOBB(), p.color, 0.f, p.thickness);
  //meshes.push_back(obbMesh);
  //const auto frameXform = frame.getWorldOBB().localToWorld();
  //meshes.push_back(coordAxesWidget(to<ml::mat4f>(frameXform)));

  InteractionFrame::ColoredBinGrid bins;
  frame.getWeightedColorBins(p, &bins);
  const auto worldXform = to<ml::mat4f>(frame.getWorldOBB().localToWorld());
//  vec<ml::vec3f> binCenters;
//  vec<ml::vec4f> binColors;
  for (const auto& pair : bins) {
    const auto& bin = pair.second;
    const auto maxVal = *max_element(bin.begin(), bin.end(), [] (const InteractionFrame::ColoredBin& l,
    const InteractionFrame::ColoredBin& r) { return l.val.w < r.val.w; });
    const ml::vec3f pos = worldXform * maxVal.pos;
    const float binSize = frame.getBinSize() * 0.5f * maxVal.val.w * maxVal.val.w;
    ml::TriMeshf binBox = S::box(binSize, maxVal.val);
    binBox.transform(ml::mat4f::translation(pos));
    meshes.push_back(binBox);
  }

//  const float binSize = frame.getBinSize() * 0.5f;
//  const auto binBox = S::box(binSize, p.color);
//  const auto pointCloud = ml::meshutil::createPointCloudTemplate(binBox, binCenters, binColors);
//  meshes.push_back(pointCloud);

  auto result = ml::meshutil::createUnifiedMesh(meshes);
  return result;
}

ml::TriMeshf iframePointsWidget(const interaction::InteractionFrame& frame, const IFVisParams& p) {
  vec<ml::TriMeshf> meshes;
  const auto obbMesh = OBBwireframe(frame.getWorldOBB(), p.color, 0.f, p.thickness);
  meshes.push_back(obbMesh);
  const auto binBox = S::box(0.005f, p.color);

  vec<ml::vec3f> binCenters;
  vec<ml::vec4f> binColors;
  for (int iJoint = 0; iJoint < kSkelParams.kNumJoints; ++iJoint) {
    const string id = kSkelParams.kJointNames[iJoint];
    const ml::vec4f color = kSkelParams.kJointColors[iJoint];
    const vec<ml::vec3f> points = frame.getWorldPoints(id);
    for (const auto& pt : points) {
      binCenters.push_back(pt);
      binColors.push_back(color);
    }
  }
  const auto pointCloud = ml::meshutil::createPointCloudTemplate(binBox, binCenters, binColors);
  meshes.push_back(pointCloud);

  auto result = ml::meshutil::createUnifiedMesh(meshes);
  return result;
}

ml::TriMeshf coordAxesWidget(const ml::mat4f& xform, float scaleFactor /*=1*/) {
  const float s = scaleFactor * 0.01f;
  const ml::vec4f r(1, 0, 0, 1), g(0, 1, 0, 1), b(0, 0, 1, 1);
  const ml::BoundingBox3f box(ml::vec3f(0, -s, -s), ml::vec3f(scaleFactor, s, s));

  // get transform for each axis
  const Transform Tx = from(xform);
  auto T = Tx.matrix();
  T.col(0).swap(T.col(2));
  T.col(0).swap(T.col(1));
  const Transform Ty(T);
  T.col(0).swap(T.col(2));
  T.col(0).swap(T.col(1));
  const Transform Tz(T);

  // create axes meshes
  ml::TriMeshf xAxis(box, r);
  xAxis.transform(to<ml::mat4f>(Tx));
  ml::TriMeshf yAxis(box, g);
  yAxis.transform(to<ml::mat4f>(Ty));
  ml::TriMeshf zAxis(box, b);
  zAxis.transform(to<ml::mat4f>(Tz));

  return ml::meshutil::createUnifiedMesh({xAxis, yAxis, zAxis});
}

ml::TriMeshf coordAxesWidget(const OBB& obb, float scaleFactor /*=1*/) {
  return coordAxesWidget(to<ml::mat4f>(obb.localToWorld()), scaleFactor);
}

void setCameraToLookAt(int windowWidth, int windowHeight,
                       const ml::vec3f& eye, const ml::vec3f& target,
                       ml::Cameraf* pCamera) {
  ml::vec3f worldUp(0.0f, 0.0f, 1.0f);
  ml::vec3f dir = target - eye;
  dir.normalize();
  ml::vec3f camRight = worldUp ^ dir;
  camRight.normalize();

  float dirDotUp = ml::vec3f::dot(dir, worldUp);
  if (abs(abs(dirDotUp) - 1.0f) < 0.01f) {
    camRight = ml::vec3f::eX;
  }
  ml::vec3f camUp = dir ^ camRight;
  camUp.normalize();

  *pCamera = ml::Cameraf(eye, dir, camUp, 60.0f, 
                         static_cast<float>(windowWidth) / windowHeight,
                         0.01f, 1000.0f);
}


}  // namespace vis
}  // namespace sg
