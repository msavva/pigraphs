#include "common.h"  // NOLINT

#include "core/Skeleton.h"

#include "core/Scan.h"
#include "core/SkelState.h"
#include "geo/geo.h"
#include "geo/OBB.h"
#include "io/json.h"
#include "math/math.h"
#include "util/util.h"

namespace sg {
namespace core {

const arr<Skeleton::JointType[2], Skeleton::kNumBones> Skeleton::kBones = {{
  {JointType_SpineBase, JointType_SpineMid},  // 0
  {JointType_SpineMid, JointType_SpineShoulder},  // 1
  {JointType_SpineShoulder, JointType_Neck},  // 2
  {JointType_Neck, JointType_Head},  // 3
  {JointType_SpineShoulder, JointType_ShoulderLeft},  // 4
  {JointType_ShoulderLeft, JointType_ElbowLeft},  // 5
  {JointType_ElbowLeft, JointType_WristLeft},  // 6
  {JointType_WristLeft, JointType_HandLeft},  // 7
  {JointType_HandLeft, JointType_HandTipLeft},  // 8
  {JointType_HandLeft, JointType_ThumbLeft},  // 9
  {JointType_SpineShoulder, JointType_ShoulderRight},  // 10
  {JointType_ShoulderRight, JointType_ElbowRight},  // 11
  {JointType_ElbowRight, JointType_WristRight},   // 12
  {JointType_WristRight, JointType_HandRight},    // 13
  {JointType_HandRight, JointType_HandTipRight},  // 14
  {JointType_HandRight, JointType_ThumbRight},  // 15
  {JointType_SpineBase, JointType_HipLeft},     // 16
  {JointType_HipLeft, JointType_KneeLeft},      // 17
  {JointType_KneeLeft, JointType_AnkleLeft},    // 18
  {JointType_AnkleLeft, JointType_FootLeft},    // 19
  {JointType_SpineBase, JointType_HipRight},    // 20
  {JointType_HipRight, JointType_KneeRight},    // 21
  {JointType_KneeRight, JointType_AnkleRight},  // 22
  {JointType_AnkleRight, JointType_FootRight}   // 23
}};

const Skeleton::BoneDirs Skeleton::kKinectBoneDirs = [] () {
  Skeleton::BoneDirs b;
  b.fill({{ml::vec3f::eY, ml::vec3f::eZ}});
  return b;
}();

//const arr<Skeleton::JointType[2], Skeleton::kNumBigBones> Skeleton::kBigBones = {{
//  {JointType_SpineShoulder, JointType_SpineBase},  // Head-to-hip bone should stay first
//  {JointType_Neck, JointType_ShoulderLeft},
//  {JointType_Neck, JointType_ShoulderRight},
//  {JointType_ShoulderLeft, JointType_ElbowLeft},
//  {JointType_ShoulderRight, JointType_ElbowRight},
//  {JointType_ElbowLeft, JointType_WristLeft},
//  {JointType_ElbowRight, JointType_WristRight},
//  {JointType_SpineBase, JointType_HipLeft},
//  {JointType_SpineBase, JointType_HipRight},
//  {JointType_HipLeft, JointType_KneeLeft},
//  {JointType_HipRight, JointType_KneeRight},
//  {JointType_KneeLeft, JointType_AnkleLeft},
//  {JointType_KneeRight, JointType_AnkleRight}
//}};

const arr<Skeleton::JointType[2], Skeleton::kNumBigBones> Skeleton::kBigBones = {{
  {JointType_SpineBase, JointType_SpineShoulder},  // 0 Head-to-hip bone should stay first
// {JointType_SpineShoulder, JointType_Neck},
  {JointType_Neck, JointType_ShoulderLeft},  // 1
  {JointType_ShoulderLeft, JointType_ElbowLeft},  // 2
  {JointType_ElbowLeft, JointType_WristLeft},  // 3
  {JointType_Neck, JointType_ShoulderRight},  // 4
  {JointType_ShoulderRight, JointType_ElbowRight},  // 5
  {JointType_ElbowRight, JointType_WristRight},  // 6
  {JointType_SpineBase, JointType_HipLeft},  // 7
  {JointType_SpineBase, JointType_HipRight},  // 8
  {JointType_HipLeft, JointType_KneeLeft},  // 9
  {JointType_KneeLeft, JointType_AnkleLeft},  // 10
  {JointType_HipRight, JointType_KneeRight},  // 11
  {JointType_KneeRight, JointType_AnkleRight}  // 12
}};

const arr<Skeleton::JointGroupLR[2], Skeleton::kNumBonesSimple> Skeleton::kBonesSimple = {{
  {JointGroupLR_Head, JointGroupLR_Gaze},
  {JointGroupLR_Head, JointGroupLR_Torso},
  {JointGroupLR_Torso, JointGroupLR_Hips},
  {JointGroupLR_Torso, JointGroupLR_ElbowL},
  {JointGroupLR_ElbowL, JointGroupLR_HandL},
  {JointGroupLR_Torso, JointGroupLR_ElbowR},
  {JointGroupLR_ElbowR, JointGroupLR_HandR},
  {JointGroupLR_Hips, JointGroupLR_KneeL},
  {JointGroupLR_KneeL, JointGroupLR_FootL},
  {JointGroupLR_Hips, JointGroupLR_KneeR},
  {JointGroupLR_KneeR, JointGroupLR_FootR}
}};

const arr<string, Skeleton::kNumJoints + 1> Skeleton::kJointNames = {
  "SpineBase",      // 0
  "SpineMid",       // 1
  "Neck",           // 2
  "Head",           // 3
  "ShoulderLeft",   // 4
  "ElbowLeft",      // 5
  "WristLeft",      // 6
  "HandLeft",       // 7
  "ShoulderRight",  // 8
  "ElbowRight",     // 9
  "WristRight",     // 10
  "HandRight",      // 11
  "HipLeft",        // 12
  "KneeLeft",       // 13
  "AnkleLeft",      // 14
  "FootLeft",       // 15
  "HipRight",       // 16
  "KneeRight",      // 17
  "AnkleRight",     // 18
  "FootRight",      // 19
  "SpineShoulder",  // 20
  "HandTipLeft",    // 21
  "ThumbLeft",      // 22
  "HandTipRight",   // 23
  "ThumbRight",     // 24
  "Gaze"            // 25
};

const arr<string, Skeleton::kNumJointGroups> Skeleton::kJointGroupNames = {
  "Head",           // 0
  "Torso",          // 1
  "Hips",           // 2
  "Knees",          // 3
  "Feet",           // 4
  "Hands",          // 5
  "Elbows",         // 6
  "Gaze"            // 7
};

const arr<string, Skeleton::kNumJointGroupsLR> Skeleton::kJointGroupLRNames = {
  "Head",           // 0
  "Torso",          // 1
  "Hips",           // 2
  "KneeL",          // 3
  "FootL",          // 4
  "ElbowL",         // 5
  "HandL",          // 6
  "KneeR",          // 7
  "FootR",          // 8
  "ElbowR",         // 9
  "HandR",          // 10
  "Gaze"            // 11
};

const arr<Skeleton::JointGroup, Skeleton::kNumJointGroupsLR> Skeleton::kJointGroupLRToJointGroup = {
  JointGroup_Head,    //"Head",           // 0
  JointGroup_Torso,   //"Torso",          // 1
  JointGroup_Hips,    //"Hips",           // 2
  JointGroup_Knees,   //"KneeL",          // 3
  JointGroup_Feet,    //"FootL",          // 4
  JointGroup_Elbows,  //"ElbowL",         // 5
  JointGroup_Hands,   //"HandL",          // 6
  JointGroup_Knees,   //"KneeR",          // 7
  JointGroup_Feet,    //"FootR",          // 8
  JointGroup_Elbows,  //"ElbowR",         // 9
  JointGroup_Hands,   //"HandR",          // 10
  JointGroup_Gaze     //"Gaze"            // 11
};

const arr<vec<Skeleton::JointGroupLR>, Skeleton::kNumJointGroups> Skeleton::kJointGroupToJointGroupLR = {
  vec<Skeleton::JointGroupLR>{JointGroupLR_Head},  //"Head",           // 0
  vec<Skeleton::JointGroupLR>{JointGroupLR_Torso}, //"Torso",          // 1
  vec<Skeleton::JointGroupLR>{JointGroupLR_Hips},  //"Hips",           // 2
  vec<Skeleton::JointGroupLR>{JointGroupLR_KneeL, JointGroupLR_KneeR},  //"Knees",          // 3
  vec<Skeleton::JointGroupLR>{JointGroupLR_FootL, JointGroupLR_FootR},  //"Feet",           // 4
  vec<Skeleton::JointGroupLR>{JointGroupLR_HandL, JointGroupLR_HandR},  //"Hands",          // 5
  vec<Skeleton::JointGroupLR>{JointGroupLR_ElbowL, JointGroupLR_ElbowR},  //"Elbows",         // 6
  vec<Skeleton::JointGroupLR>{JointGroupLR_Gaze}   //"Gaze"            // 7
};

const arr<Skeleton::JointGroup, Skeleton::kNumJoints + 1> Skeleton::kJointToJointGroup = {
  JointGroup_Hips,   //"SpineBase",      // 0
  JointGroup_Torso,  //"SpineMid",       // 1
  JointGroup_Torso,  //"Neck",           // 2
  JointGroup_Head,   //"Head",           // 3
  JointGroup_Torso,  //"ShoulderLeft",   // 4
  JointGroup_Elbows, //"ElbowLeft",      // 5
  JointGroup_Hands,  //"WristLeft",      // 6
  JointGroup_Hands,  //"HandLeft",       // 7
  JointGroup_Torso,  //"ShoulderRight",  // 8
  JointGroup_Elbows, //"ElbowRight",     // 9
  JointGroup_Hands,  //"WristRight",     // 10
  JointGroup_Hands,  //"HandRight",      // 11
  JointGroup_Hips,   //"HipLeft",        // 12
  JointGroup_Knees,  //"KneeLeft",       // 13
  JointGroup_Feet,   //"AnkleLeft",      // 14
  JointGroup_Feet,   //"FootLeft",       // 15
  JointGroup_Hips,   //"HipRight",       // 16
  JointGroup_Knees,  //"KneeRight",      // 17
  JointGroup_Feet,   //"AnkleRight",     // 18
  JointGroup_Feet,   //"FootRight",      // 19
  JointGroup_Torso,  //"SpineShoulder",  // 20
  JointGroup_Hands,  //"HandTipLeft",    // 21
  JointGroup_Hands,  //"ThumbLeft",      // 22
  JointGroup_Hands,  //"HandTipRight",   // 23
  JointGroup_Hands,  //"ThumbRight"      // 24
  JointGroup_Gaze    //"Gaze"            // 25
};

const arr<Skeleton::JointGroupLR, Skeleton::kNumJoints + 1> Skeleton::kJointToJointGroupLR = {
  JointGroupLR_Hips,    //"SpineBase",      // 0
  JointGroupLR_Torso,   //"SpineMid",       // 1
  JointGroupLR_Torso,   //"Neck",           // 2
  JointGroupLR_Head,    //"Head",           // 3
  JointGroupLR_Torso,   //"ShoulderLeft",   // 4
  JointGroupLR_ElbowL,  //"ElbowLeft",      // 5
  JointGroupLR_HandL,   //"WristLeft",      // 6
  JointGroupLR_HandL,   //"HandLeft",       // 7
  JointGroupLR_Torso,   //"ShoulderRight",  // 8
  JointGroupLR_ElbowR,  //"ElbowRight",     // 9
  JointGroupLR_HandR,   //"WristRight",     // 10
  JointGroupLR_HandR,   //"HandRight",      // 11
  JointGroupLR_Hips,    //"HipLeft",        // 12
  JointGroupLR_KneeL,   //"KneeLeft",       // 13
  JointGroupLR_FootL,   //"AnkleLeft",      // 14
  JointGroupLR_FootL,   //"FootLeft",       // 15
  JointGroupLR_Hips,    //"HipRight",       // 16
  JointGroupLR_KneeR,   //"KneeRight",      // 17
  JointGroupLR_FootR,   //"AnkleRight",     // 18
  JointGroupLR_FootR,   //"FootRight",      // 19
  JointGroupLR_Torso,   //"SpineShoulder",  // 20
  JointGroupLR_HandL,   //"HandTipLeft",    // 21
  JointGroupLR_HandL,   //"ThumbLeft",      // 22
  JointGroupLR_HandR,   //"HandTipRight",   // 23
  JointGroupLR_HandR,   //"ThumbRight"      // 24
  JointGroupLR_Gaze     //"Gaze"            // 25
};

arr<vec<size_t>, Skeleton::kNumJointGroups> createJointGroupToJointsMap()
{
  arr<vec<size_t>, Skeleton::kNumJointGroups> jointsForJointGroups;
  for (size_t iJointGroup = 0; iJointGroup < Skeleton::kNumJointGroups; iJointGroup++) {
    vec<size_t>& jointsForJointGroup = jointsForJointGroups[iJointGroup];
    for (size_t iJoint = 0; iJoint < Skeleton::kNumJoints+1; iJoint++) {
      if (Skeleton::kJointToJointGroup[iJoint] == iJointGroup) {
        jointsForJointGroup.push_back(iJoint);
      }
    }
  }
  return jointsForJointGroups;
}
const arr<vec<size_t>, Skeleton::kNumJointGroups>
Skeleton::kJointGroupToJoints = createJointGroupToJointsMap();


const arr<Skeleton::JointType, Skeleton::kNumBodyPlaneJoints> Skeleton::kBodyPlaneJoints = {
  JointType_Neck,
  JointType_ShoulderLeft,
  JointType_ShoulderRight,
  JointType_SpineShoulder,
  JointType_SpineMid,
  JointType_SpineBase,
  JointType_HipLeft,
  JointType_HipRight
};

const arr<Skeleton::JointType, Skeleton::kNumMovableJoints> Skeleton::kMovableJoints = {
//  JointType_Head,
  JointType_ElbowLeft,
  JointType_WristLeft,
  JointType_HandLeft,
  JointType_ElbowRight,
  JointType_WristRight,
  JointType_HandRight,
  JointType_KneeLeft,
  JointType_AnkleLeft,
  JointType_FootLeft,
  JointType_KneeRight,
  JointType_AnkleRight,
  JointType_FootRight,
  JointType_HandTipLeft,
  JointType_ThumbLeft,
  JointType_HandTipRight,
  JointType_ThumbRight
};

const arr<Skeleton::JointType, Skeleton::kNumLegJoints> Skeleton::kLegJoints = {
  JointType_KneeLeft,
  JointType_KneeRight,
  JointType_AnkleLeft,
  JointType_AnkleRight,
  JointType_FootLeft,
  JointType_FootRight
};

const arr<Skeleton::JointType, 4> Skeleton::kFeetJoints = {
  JointType_FootLeft,
  JointType_FootRight,
  JointType_AnkleLeft,
  JointType_AnkleRight
};

const arr<Skeleton::JointType, 3> Skeleton::kHipJoints = {
  JointType_HipLeft,
  JointType_HipRight,
  JointType_SpineBase
};

const arr<ml::vec4f, Skeleton::kNumJoints + 1> Skeleton::kJointBaseColors
= ml::ColorUtils::colorArrayByIdSeq<ml::vec4f, kNumJoints + 1>(-22);
const arr<ml::vec4f, Skeleton::kNumJointGroups + 1> Skeleton::kJointGroupBaseColors
= ml::ColorUtils::colorArrayByIdSeq<ml::vec4f, kNumJointGroups + 1>(-22);
const arr<ml::vec4f, Skeleton::kNumJointGroupsLR> Skeleton::kJointGroupLRBaseColors = {{
  {0.58f, 0.40f, 0.74f, 1.f},  // Head = purple
  {0.84f, 0.15f, 0.16f, 1.f},  // Torso = red
  {0.17f, 0.63f, 0.17f, 1.f},  // Hips = green
  {0.62f, 0.85f, 0.90f, 1.f},  // KneeL = cyan
  {0.45f, 0.62f, 0.81f, 1.f},  // FootL = blue
  {0.80f, 0.80f, 0.36f, 1.f},  // ElbowL = yellow
  {1.00f, 0.73f, 0.47f, 1.f},  // HandL = orange
  {0.09f, 0.75f, 0.81f, 1.f},  // KneeR = dark cyan
  {0.12f, 0.47f, 0.71f, 1.f},  // FootR = dark blue
  {0.74f, 0.74f, 0.13f, 1.f},  // ElbowR = dark yellow
  {1.00f, 0.50f, 0.05f, 1.f},  // HandR = dark orange
  {0.77f, 0.69f, 0.84f, 1.f}   // Gaze = light purple
}};
const ml::vec4f Skeleton::kBoneColor = ml::vec4f(1.0f, 242.f / 255.f, 181.f / 255.f, 1.0f);
const ml::vec4f Skeleton::kGazeColor = ml::vec4f(0.2f, 0.2f, 0.7f, 1.0f);

const SkeletonParamsT<Skeleton::JointGroupLR, Skeleton::kNumJointGroupsLR> kSkelParamsJointGroupLR(
  Skeleton::kBonesSimple,
  Skeleton::kJointGroupLRNames,
  Skeleton::kJointGroupLRBaseColors,
  Skeleton::kJointToJointGroupLR
);

// TODO(MS): Add other SkeletonParamsT instantiations and hide members from header

using geo::Vec3f; using geo::OBB; using geo::Quatf;

//! Apply joint angle delta offset to s
inline void applyJointAngleDeltas(const vec<float>& angDeltas, Skeleton* s) {
  if (angDeltas.empty()) { return; }
  SkelState ss;
  skel2state(*s, &ss);
  ss = makeHierarchical(ss);
  for (int i = 0; i < Skeleton::kNumJoints; ++i) {
    ss.joints[i].q.z() += angDeltas[i];
  }
  ss = makeAbsolute(ss);
  state2skel(ss, s);
}

void TransformedSkeleton::makeSkeleton() const {
  m_xformedSkel = *m_skel;
  applyJointAngleDeltas(m_jointAngDeltas, &m_xformedSkel);
  const Quatf Rq(geo::from(m_xform).rotation());  // TODO(MS): Cache this!
  for (int iJoint = 0; iJoint < Skeleton::kNumJoints; ++iJoint) {
    ml::vec3f& p = m_xformedSkel.jointPositions[iJoint];
    p = m_xform * p;
    ml::vec4f& o = m_xformedSkel.jointOrientations[iJoint];
    const Quatf q = Rq * Quatf(o.w, o.x, o.y, o.z);
    o.x = q.x();
    o.y = q.y();
    o.z = q.z();
    o.w = q.w();
  }
  if (!m_skel->gazeBodyPlaneOkay) {
    SG_LOG_WARN << "Gaze body plane not computed!!!";
  }
  assert(m_skel->gazeBodyPlaneOkay);
  m_xformedSkel.gazePosition      =  m_xform * m_skel->gazePosition;
  m_xformedSkel.gazeDirection     = (m_xform * ml::vec4f(m_skel->gazeDirection, 0)).getVec3();
  m_xformedSkel.bodyPlaneCentroid =  m_xform * m_skel->bodyPlaneCentroid;
  m_xformedSkel.bodyPlaneNormal   = (m_xform * ml::vec4f(m_skel->bodyPlaneNormal, 0)).getVec3();
  m_xformedSkel.m_pBoneOBBs       = nullptr;
  m_xformedSkel.m_pBonePts        = nullptr;

  m_xformedSkelComputed = true;
}

ml::vec3f TransformedSkeleton::transformedJoint(size_t jointIndex) const {
  return m_xform * m_skel->jointPositions[jointIndex];
}

void Skeleton::computeBodyPlaneCentroidNormal() {
  vec<ml::vec3f> pts;
  vecf confs;
  for (const JointType joint : kBodyPlaneJoints) {
    if (jointConfidences[joint] > 0.f) {
      pts.push_back(jointPositions[joint]);
      confs.push_back(jointConfidences[joint]);
    }
  }
  if (pts.size() < 3) {
    bodyPlaneDist = 0.0f;
    bodyPlaneConf = 0.0f;
    bodyPlaneNormal = ml::vec3f::origin;
    bodyPlaneCentroid = ml::vec3f::origin;
    return;
  }  // Cannot fit plane

  // Get centroid and normal of least squares plane fit
  geo::CoordSystem C(pts.begin(), pts.end());
  for (int i = 0; i < 3; i++) {
    bodyPlaneNormal[i] = C.R()(i, 2);
    bodyPlaneCentroid[i] = C.c()[i];
  }

  ml::vec3f center = ml::vec3f::origin;
  float distSq = 0.0f;
  for (const JointType joint : kMovableJoints) {
    if (jointConfidences[joint] > 0.0f) {
      center += jointPositions[joint];
      float currDist = (jointPositions[joint] - bodyPlaneCentroid) | bodyPlaneNormal;
      if (currDist < 0.0f) {
        distSq -= currDist*currDist;
      } else {
        distSq += currDist*currDist;
      }
    }
  }
  center /= static_cast<float>(kMovableJoints.size());
  //float dist = (center - *centroid) | *normal;
  float dist = distSq;

  if (dist < 0) {
    bodyPlaneNormal = -bodyPlaneNormal;
    dist *= -1;
  }
  bodyPlaneDist = dist;

  if (dist < 0.15f) {
    dist = 0.0f;
  };

  // Return average joint confidence
  float confSum = accumulate(confs.begin(), confs.end(), 0.0f);
  float confMean;
  if (confs.size() == 0) {
    confMean = 0;
  } else {
    confMean = confSum / static_cast<float>(confs.size());
  }

  bodyPlaneDist = dist;
  bodyPlaneConf = sqrt(confMean * dist);


  // Flip normal if majority of movable joints are behind
  //unsigned int countInFront = 0, totalCount = 0;
  //for (const JointType joint : kMovableJoints) {
  //  //if (jointConfidences[joint] > 0.f) {
  //    totalCount++;
  //  //} else { continue; }
  //  if (((jointPositions[joint] - *centroid) | *normal) > 0.f) { countInFront++; }
  //}
  //if (static_cast<float>(countInFront) / totalCount < (kNumMovableJoints / 2)) { *normal = -*normal; }
}

void Skeleton::computeGaze() {
  computeBodyPlaneCentroidNormal();
  gazeDirection = bodyPlaneNormal;
  gazeConfidence = bodyPlaneConf;
  gazePosition = jointPositions[JointType_Head];
  gazeBodyPlaneOkay = true;
}

bool Skeleton::isSitting() const {
  const auto& p = jointPositions;
  const float
    hipHeight    = (p[JointType_HipLeft].z + p[JointType_HipRight].z + p[JointType_SpineBase].z) * 0.333f,
    kneeHeight   = (p[JointType_KneeLeft].z + p[JointType_KneeRight].z) * 0.5f,
    calfVertDisp = fabsf(((p[JointType_KneeLeft].z  - p[JointType_AnkleLeft].z) +
                          (p[JointType_KneeRight].z - p[JointType_AnkleRight].z)) * 0.5f);
  return fabsf(kneeHeight - hipHeight) < .5f * calfVertDisp;
}

float Skeleton::avgDistToSupport(const Scan& scan) const {
  const auto& vStack = scan.getVertexStackGrid();
  const auto& p = jointPositions;

  float sumDistFromScene = 0;  int numSupports = 0;
  const auto distFun = [&] (const JointType joint) {
    const auto& jpos = p[joint];
    if (!vStack.contains(jpos.x, jpos.y)) {
      return;
    }
    const auto& v = vStack.get(jpos).verts;
    if (v.size() > 0) {
      const float
        sceneZ = v.front().position.z,
        dZ = jpos.z - sceneZ;
      sumDistFromScene += fabsf(dZ);
      numSupports++;
    }
  };

  if (isSitting()) {
    for (const JointType joint : kHipJoints) {
      distFun(joint);
    }
  } else {
    for (const JointType joint : kFeetJoints) {
      distFun(joint);
    }
  }

  if (numSupports > 0) {
    return sumDistFromScene / numSupports;
  } else {
    return math::constants::POSINFf;
  }
}

float Skeleton::minDistToSupport(const Scan& scan) const {
  const auto& vStack = scan.getVertexStackGrid();
  const auto& p = jointPositions;

  float minDistFromScene = math::constants::POSINFf;
  const auto distFun = [&] (const JointType joint) {
    const auto& jpos = p[joint];
    if (!vStack.contains(jpos.x, jpos.y)) {
      return;
    }
    const auto& v = vStack.get(jpos).verts;
    if (v.size() > 0) {
      const float
        sceneZ = v.front().position.z,
        dZ = jpos.z - sceneZ;
      if (fabsf(dZ) < fabsf(minDistFromScene)) {
        minDistFromScene = dZ;
      }
    }
  };

  if (isSitting()) {
    for (const JointType joint : kHipJoints) {
      distFun(joint);
    }
  } else {
    for (const JointType joint : kFeetJoints) {
      distFun(joint);
    }
  }
  return minDistFromScene;
}

float Skeleton::maxDistToSupport(const Scan& scan) const {
  const auto& vStack = scan.getVertexStackGrid();
  const auto& p = jointPositions;

  float maxDistFromScene = math::constants::NEGINFf;
  const auto distFun = [&] (const JointType joint) {
    const auto& jpos = p[joint];
    if (!vStack.contains(jpos.x, jpos.y)) {
      return;
    }
    const auto& v = vStack.get(jpos).verts;
    if (v.size() > 0) {
      const float
        sceneZ = v.front().position.z,
        dZ = jpos.z - sceneZ;
      if (fabsf(dZ) > maxDistFromScene) {
        maxDistFromScene = dZ;
      }
    }
  };

  if (isSitting()) {
    for (const JointType joint : kHipJoints) {
      distFun(joint);
    }
  } else {
    for (const JointType joint : kFeetJoints) {
      distFun(joint);
    }
  }
  return fabsf(maxDistFromScene);
}

float Skeleton::unfloat(const Scan& scan) {
  const float minDistFromScene = minDistToSupport(scan);
  float zAdjustment = 0;

  // Offset skeleton joints
  if (minDistFromScene < math::constants::POSINFf) {
    const float absDist = fabsf(minDistFromScene);
    const float minThresh = 0.05f;   // don't move when less than this
    const float maxThresh = 0.30f;   // don't move when more than this
    if (absDist > minThresh && absDist < maxThresh) {
      //SG_LOG_INFO << "Unfloat skeleton " << minDistFromScene;
      zAdjustment = minDistFromScene * 0.9f;
      for (auto& jointPos : jointPositions) {
        jointPos.z -= zAdjustment;
      }
      gazePosition.z -= zAdjustment;
      bodyPlaneCentroid.z -= zAdjustment;
    }
  }
  if (m_pBoneOBBs) m_pBoneOBBs->clear();
  if (m_pBonePts) m_pBonePts->clear();
  return -zAdjustment;
}

void Skeleton::move(const ml::vec3f& delta) {
  for (auto& jointPos : jointPositions) {
    jointPos += delta;
  }
  gazePosition += delta;
  bodyPlaneCentroid += delta;
  if (m_pBoneOBBs) m_pBoneOBBs->clear();
  if (m_pBonePts) m_pBonePts->clear();
}

void Skeleton::move(const int idx, const float delta) {
  for (auto& jointPos : jointPositions) {
    jointPos[idx] += delta;
  }
  gazePosition[idx] += delta;
  bodyPlaneCentroid[idx] += delta;
  if (m_pBoneOBBs) m_pBoneOBBs->clear();
  if (m_pBonePts) m_pBonePts->clear();
}

ml::vec3f Skeleton::centerOfMass() const {
  // Body part mass ratios from p59 of http://www.dtic.mil/dtic/tr/fulltext/u2/710622.pdf
  const float mHead = 0.073f, mTrunk = 0.507f, mUpperArm = 0.026f, mForearm = 0.016f,
              mHand = 0.007f, mThigh = 0.103f, mCalf = 0.043f, mFoot = 0.015f;
  const auto& p = jointPositions;
  const ml::vec3f
    pHead       = p[JointType_Head],
    pTrunk      = p[JointType_SpineMid],
    pUpperArmL  = (p[JointType_ShoulderLeft] + p[JointType_ElbowLeft]) * 0.5f,
    pUpperArmR  = (p[JointType_ShoulderRight] + p[JointType_ElbowRight]) * 0.5f,
    pForearmL   = (p[JointType_ElbowLeft] + p[JointType_HandLeft]) * 0.5f,
    pForearmR   = (p[JointType_ElbowRight] + p[JointType_HandRight]) * 0.5f,
    pHandL      = (p[JointType_HandLeft] + p[JointType_HandTipLeft] + p[JointType_ThumbLeft]) / 3.0f,
    pHandR      = (p[JointType_HandRight] + p[JointType_HandTipRight] + p[JointType_ThumbRight]) / 3.0f,
    pThighL     = (p[JointType_HipLeft] + p[JointType_KneeLeft]) * 0.5f,
    pThighR     = (p[JointType_HipRight] + p[JointType_KneeRight]) * 0.5f,
    pCalfL      = (p[JointType_KneeLeft] + p[JointType_AnkleLeft]) * 0.5f,
    pCalfR      = (p[JointType_KneeRight] + p[JointType_AnkleRight]) * 0.5f,
    pFootL      = (p[JointType_AnkleLeft] + p[JointType_FootLeft]) * 0.5f,
    pFootR      = (p[JointType_AnkleRight] + p[JointType_FootRight]) * 0.5f;
  const ml::vec3f
    pCOM  = mHead * pHead
          + mTrunk * pTrunk
          + mUpperArm * (pUpperArmL + pUpperArmR)
          + mForearm * (pForearmL + pForearmR)
          + mHand * (pHandL + pHandR)
          + mThigh * (pThighL + pThighR)
          + mCalf * (pCalfL + pCalfR)
          + mFoot * (pFootL + pFootR);
  return pCOM;
}

const vec<OBB>& Skeleton::getBoneOBBs() const {
  if (m_pBoneOBBs != nullptr) {
    return *m_pBoneOBBs;
  }

  const bool usePositions = false;  // TODO(MS): Parameterize
  const BoneDirs& boneDirs = isKinectSkel ? kKinectBoneDirs : SkelState::kBoneDirs;

  m_pBoneOBBs = std::make_shared<vec<OBB>>();
  m_pBoneOBBs->resize(kNumBigBones);
  geo::Matrix3f R;
  for (int iBone = 0; iBone < kNumBigBones; ++iBone) {
    const auto& bone = kBigBones[iBone];
    const float
      dm1 = (iBone == 0) ? 0.1f : 0.045f,   // "depth" (front-back size)
      dm2 = (iBone == 0) ? 0.15f : 0.045f;  // "length" (left-right size)
    const Vec3f
      p0  = geo::vec3f(jointPositions[bone[0]]),
      p1  = geo::vec3f(jointPositions[bone[1]]),
      c   = (p1 + p0) * 0.5f,
      d   = p1 - p0,
      hw  = Vec3f(d.norm() * 0.5f, dm1, dm2);
    if (usePositions) {
      const Vec3f
        dn  = d.normalized(),
        bn  = geo::vec3f(bodyPlaneNormal),
        prp  = bn.cross(dn).normalized(),  // perpendicular to bodyNormal
        fro = dn.cross(prp).normalized();  // mutually perpendicular to dn and bodyNormal
      R.col(0) = dn;
      R.col(1) = fro;
      R.col(2) = -prp;
    } else {
      const ml::vec4f& v = jointOrientations[bone[1]];
      const Quatf q(v.w, v.x, v.y, v.z);  // bone orientation quaternion
      const auto& bDir = boneDirs[bone[1]];
      R.col(0) = q * geo::vec3f(bDir[0]);
      R.col(1) = q * geo::vec3f(bDir[1]);
      R.col(2) = R.col(0).cross(R.col(1)).normalized();
      //R = q.toRotationMatrix();  // x = normal, y = bone dir, z = binormal
      //R.col(0).swap(R.col(1));   // OBB expects bone dir first
      //R.col(2) = -R.col(2);      // negate to retain right-handedness
    }
    (*m_pBoneOBBs)[iBone] = OBB(c, hw, R);
  }
  return *m_pBoneOBBs;
}

void sampleOBBs(const vec<OBB>& obbs, vec<ml::vec3f>* pts, int numPoints) {
  // get bone obbs
  const size_t numOBBs = obbs.size();
  if (numOBBs == 0) {
    SG_LOG_ERROR << "Cannot sample obbs: No OBBs!!!";
    return;
  }

  // compute volume of each OBB
  vec<float> volumes(numOBBs, 0);
  float totalVolume = 0.f;
  for (size_t iOBB = 0; iOBB < numOBBs; ++iOBB) {
    auto& obb = obbs[iOBB];
    const float vol = obb.axesLengths().prod();
    totalVolume += vol;
    volumes[iOBB] = vol;
  }
  // sort by volume
  vec<size_t> obbIdxDescByVol;
  util::sortIndices(volumes.begin(), volumes.end(), std::greater<float>(), obbIdxDescByVol);

  // now normalize volumes, and use to assign random points for each OBB
  const float invVol = 1.f / totalVolume;
  int pointsSoFar = 0;
  pts->resize(numPoints);
  for (size_t iOBB = 0; iOBB < numOBBs; ++iOBB) {
    const size_t obbIdx = obbIdxDescByVol[iOBB];
    const int myNumPoints = std::min(static_cast<int>(roundf(volumes[obbIdx] * invVol * numPoints)),
                                     numPoints - pointsSoFar);
    const auto& obb = obbs[obbIdx];
    for (int iPoint = 0; iPoint < myNumPoints; ++iPoint) {
      (*pts)[pointsSoFar] = geo::to<ml::vec3f>(obb.sample());
      pointsSoFar++;
    }
    if (pointsSoFar >= numPoints) break;
  }
  while (pointsSoFar < numPoints) {  // If not enough points yet, grab some more from largest OBB
    const auto& obb = obbs[0];
    (*pts)[pointsSoFar] = geo::to<ml::vec3f>(obb.sample());
    pointsSoFar++;
  }
  assert(numPoints == pts->size());
}

const vec<ml::vec3f>& Skeleton::getPointsInOBBs(int numPoints /*= 100*/) const {
  if (m_pBonePts == nullptr) {
    m_pBonePts = std::make_shared<vec<ml::vec3f>>();
  }
  // return cached points if available
  if (m_pBonePts->size() == numPoints) {
    return *m_pBonePts;
  }

  sampleOBBs(getBoneOBBs(), m_pBonePts.get(), numPoints);

  return *m_pBonePts;
}

int numPointsInsideOBBs(const vec<ml::vec3f>& pts, const vec<OBB>& obbs, int ignoreOBBIdx) {
  int insidePts = 0;
  for (int i = 0; i < pts.size(); ++i) {
    for (int iOBB = 0; iOBB < obbs.size(); ++iOBB) {
      if (iOBB == ignoreOBBIdx) { continue; }
      const OBB& obb = obbs[iOBB];
      if (obb.contains(geo::vec3f(pts[i]))) {
        ++insidePts;
        break;
      }
    }
  }
  return insidePts;
}

double Skeleton::getSelfCollisionRatio() const {
  const auto& obbs = getBoneOBBs();
  const size_t numPoints = 100;
  const size_t totalPoints = (obbs.size() - 2) * numPoints;  // hips ignored
  int collidingPts = 0;
  vec<ml::vec3f> pts;
  for (int iBone = 0; iBone < obbs.size(); ++iBone) {
    if (iBone == 7 || iBone == 8) { continue; }  // hip bones are embedded in torso so always "collide"
    pts.clear();
    sampleOBBs({obbs[iBone]}, &pts, numPoints);
    collidingPts += numPointsInsideOBBs(pts, obbs, iBone);
  }
  return static_cast<double>(collidingPts) / totalPoints;
}

// Returns transform that will take skeleton to normalized skeleton
ml::mat4f Skeleton::getNormalizingTransform() const {
  if (!gazeBodyPlaneOkay) {
    SG_LOG_WARN << "Gaze body plane not computed!!!";
  }
  assert(gazeBodyPlaneOkay);
  using ml::vec3f; using ml::mat4f; using ml::mat3f;
  const vec3f& center = jointPositions[JointType_SpineBase];
  const mat4f translation = mat4f::translation(-center.x, -center.y, 0.0f);
  const vec3f front = vec3f(bodyPlaneNormal[0], bodyPlaneNormal[1], 0.f).getNormalized();
  const vec3f right = vec3f::cross(front, vec3f::eZ);
  const mat3f rotation = mat3f(right, front, vec3f::eZ);
  mat4f xform;
  xform.setIdentity();
  xform.setMatrix3x3(rotation);
  xform = xform * translation;
  return xform;
}

// TODO(MS): Integrate into SkeletonParams
Skeleton::SegmentsByJointGroup Skeleton::groupSegmentsByJointGroup(const SegmentsByJointPlusGaze& segsByJoint) {
  SegmentsByJointGroup segsByJointGroup;
  for (unsigned int iJoint = 0; iJoint < kNumJoints + 1; iJoint++) {
    const auto& segs = segsByJoint[iJoint];
    auto& jointGroupSegs = segsByJointGroup[kJointToJointGroup[iJoint]];
    // TODO(MS): Account for duplicates here?
    copy(segs.begin(), segs.end(), back_inserter(jointGroupSegs));
  }
  return segsByJointGroup;
}

// TODO(MS): Integrate into SkeletonParams
Skeleton::SegmentsByJointGroupLR Skeleton::groupSegmentsByJointGroupLR(const SegmentsByJointPlusGaze& segsByJoint) {
  SegmentsByJointGroupLR segsByJointGroupLR;
  for (unsigned int iJoint = 0; iJoint < kNumJoints + 1; iJoint++) {
    const auto& segs = segsByJoint[iJoint];
    auto& jointGroupLRSegs = segsByJointGroupLR[kJointToJointGroupLR[iJoint]];
    // TODO(MS): Account for duplicates here?
    copy(segs.begin(), segs.end(), back_inserter(jointGroupLRSegs));
  }
  return segsByJointGroupLR;
}

ostream& Skeleton::toJSON(ostream& os, bool endlines) const {  // NOLINT(*)
  using io::toJSON;
  const std::function<void(void)> sep = io::put(os, ",", endlines);

  const auto key = [ ] (const string& id) { return "\"" + id + "\": "; };

  os << "{";                      if (endlines) { os << endl; }
  os << key("trackingId")         << trackingId; sep();
  os << key("jointPositions");    toJSON(os, jointPositions, kNumJoints); sep();
  os << key("jointConfidences");  toJSON(os, jointConfidences, kNumJoints); sep();
  os << key("jointOrientations"); toJSON(os, jointOrientations, kNumJoints); sep();
  os << key("handState") << "["   << handLeftState << "," << handLeftConfidence << ","
                                  << handRightState << "," << handRightConfidence << "]"; sep();
  os << key("activities");        toJSON(os, activities, Activity_Count); sep();
  os << key("leanState") << "["   << leanLeftRight << "," << leanForwardBack
                                  << "," << leanConfidence << "]"; sep();
  os << key("clippedEdges")       << clippedEdges; sep();
  os << key("timestamp")          << timestamp;  if (endlines) { os << endl; }
  os << "}";                      if (endlines) { os << endl; }

  return os;
}

}  // namespace core
}  // namespace sg
