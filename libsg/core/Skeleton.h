#pragma once

#include <mLibCore.h>

#include "libsg.h"  // NOLINT
#include "util/util.h"

#include <boost/serialization/version.hpp>

namespace sg {
namespace core {

// Skeleton struct similar to KinectOne's API
struct Skeleton {
  enum JointType {
    JointType_SpineBase = 0,
    JointType_SpineMid  = 1,
    JointType_Neck  = 2,
    JointType_Head  = 3,
    JointType_ShoulderLeft  = 4,
    JointType_ElbowLeft = 5,
    JointType_WristLeft = 6,
    JointType_HandLeft  = 7,
    JointType_ShoulderRight = 8,
    JointType_ElbowRight  = 9,
    JointType_WristRight  = 10,
    JointType_HandRight = 11,
    JointType_HipLeft = 12,
    JointType_KneeLeft  = 13,
    JointType_AnkleLeft = 14,
    JointType_FootLeft  = 15,
    JointType_HipRight  = 16,
    JointType_KneeRight = 17,
    JointType_AnkleRight  = 18,
    JointType_FootRight = 19,
    JointType_SpineShoulder = 20,
    JointType_HandTipLeft = 21,
    JointType_ThumbLeft = 22,
    JointType_HandTipRight  = 23,
    JointType_ThumbRight  = 24,
    JointType_Count = 25
  };
  enum HandState {
    HandState_Unknown = 0,
    HandState_NotTracked  = 1,
    HandState_Open  = 2,
    HandState_Closed  = 3,
    HandState_Lasso = 4
  };
  enum TrackingConfidence {
    TrackingConfidence_Low  = 0,
    TrackingConfidence_High = 1
  };
  enum TrackingState {
    TrackingState_NotTracked  = 0,
    TrackingState_Inferred  = 1,
    TrackingState_Tracked = 2
  };
  enum DetectionResult {
    DetectionResult_Unknown = 0,
    DetectionResult_No  = 1,
    DetectionResult_Maybe = 2,
    DetectionResult_Yes = 3
  };
  enum Activity : int16_t {
    Activity_EyeLeftClosed  = 0,
    Activity_EyeRightClosed = 1,
    Activity_MouthOpen  = 2,
    Activity_MouthMoved = 3,
    Activity_LookingAway  = 4,
    Activity_Count  = (Activity_LookingAway + 1)
  };
  enum FrameEdges {
    FrameEdge_None  = 0,
    FrameEdge_Right = 0x1,
    FrameEdge_Left  = 0x2,
    FrameEdge_Top = 0x4,
    FrameEdge_Bottom  = 0x8
  };
  //! Coarser semantic grouping of joints
  enum JointGroup : int8_t {
    JointGroup_Head = 0,
    JointGroup_Torso = 1,
    JointGroup_Hips = 2,
    JointGroup_Knees = 3,
    JointGroup_Feet = 4,
    JointGroup_Hands = 5,
    JointGroup_Elbows = 6,
    JointGroup_Gaze = 7,
    JointGroup_Count = 8
  };
  //! Coarser semantic grouping of joints with left/right distinction
  enum JointGroupLR : int8_t {
    JointGroupLR_Head = 0,
    JointGroupLR_Torso = 1,
    JointGroupLR_Hips = 2,
    JointGroupLR_KneeL = 3,
    JointGroupLR_FootL = 4,
    JointGroupLR_ElbowL = 5,
    JointGroupLR_HandL = 6,
    JointGroupLR_KneeR = 7,
    JointGroupLR_FootR = 8,
    JointGroupLR_ElbowR = 9,
    JointGroupLR_HandR = 10,
    JointGroupLR_Gaze = 11,
    JointGroupLR_Count = 12
  };

  static const int
    kVersionNumber        = 1,
    kNumJoints            = JointType_Count,
    kNumBones             = JointType_Count - 1,
    kNumBigBones          = 13,
    kNumBonesSimple       = JointGroupLR_Count - 1,
    kNumJointGroups       = JointGroup_Count,
    kNumJointGroupsLR     = JointGroupLR_Count,
    kNumBodyPlaneJoints   = 8,
    kNumMovableJoints     = 16,
    kNumLegJoints         = 6;

  //! Bones (joint pairs)
  static const arr<JointType[2], kNumBones> kBones;
  static const arr<JointType[2], kNumBigBones> kBigBones;  //! Big bones, beginning with head-to-hip spine bone
  static const arr<JointGroupLR[2], kNumBonesSimple> kBonesSimple;
  typedef arr<arr<ml::vec3f, 2>, kNumJoints> BoneDirs;  // <boneDir, boneFront> pairs
  static const BoneDirs kKinectBoneDirs;  // default kinect bone dirs per joint

  //! Bone colors and names
  static const arr<ml::vec4f, kNumJoints + 1> kJointBaseColors;
  static const arr<string, kNumJoints + 1> kJointNames;
  static const arr<ml::vec4f, kNumJointGroups + 1> kJointGroupBaseColors;
  static const arr<string, kNumJointGroups> kJointGroupNames;
  static const arr<ml::vec4f, kNumJointGroupsLR> kJointGroupLRBaseColors;
  static const arr<string, kNumJointGroupsLR> kJointGroupLRNames;

  //! Map from joint to coarser joint group (e.g. hips)
  static const arr<JointGroup, kNumJoints + 1> kJointToJointGroup;
  static const arr<JointGroupLR, kNumJoints + 1> kJointToJointGroupLR;

  //! Map from joint group to individual joints
  static const arr<vec<size_t>, kNumJointGroups> kJointGroupToJoints;
  static const arr<vec<size_t>, kNumJointGroupsLR> kJointGroupLRToJoints;

  //! Map from JointGroupLR to JointGroup (collapses left/right)
  static const arr<JointGroup, kNumJointGroupsLR> kJointGroupLRToJointGroup;
  //! Map from JointGroup to JointGroupLR (uncollapses left/right)
  static const arr<vec<JointGroupLR>, kNumJointGroups> kJointGroupToJointGroupLR;

  //! List of all joints considered to be part of the body plane (torso and hips)
  static const arr<JointType, kNumBodyPlaneJoints> kBodyPlaneJoints;

  //! List of all joints considered to be movable (i.e not part of the body plane)
  static const arr<JointType, kNumMovableJoints> kMovableJoints;

  //! List of all joints that belong to feet and legs
  static const arr<JointType, kNumLegJoints> kLegJoints;

  //! List of all feet joints
  static const arr<JointType, 4> kFeetJoints;

  //! List of all hip joints
  static const arr<JointType, 3> kHipJoints;

  static const ml::vec4f kBoneColor, kGazeColor;

  // TODO(MS): Integrate below functions into SkeletonParams
  template <typename T>
  static string jointName(T t) {
    if (t < 0 || t >= kNumJoints) { return to_string(t); }
    return kJointNames[t];
  }
  template <typename T>
  static string jointGroupName(T t) {
    if (t < 0 || t >= kNumJointGroups) { return to_string(t); }
    return kJointGroupNames[t];
  }
  template <typename T>
  static string jointGroupLRName(T t) {
    if (t < 0 || t >= kNumJointGroupsLR) { return to_string(t); }
    return kJointGroupLRNames[t];
  }

  // TODO(MS): Integrate into SkeletonParams
  typedef arr<segmentation::VecSegPtr, kNumJointGroups>   SegmentsByJointGroup;
  typedef arr<segmentation::VecSegPtr, kNumJoints + 1>    SegmentsByJointPlusGaze;
  typedef arr<segmentation::VecSegPtr, kNumJointGroupsLR> SegmentsByJointGroupLR;
  //typedef arr<sg::segmentation::VecConstPartPtr, Skeleton::kNumJointGroups>   PartsByJointGroup;
  typedef arr<segmentation::VecConstPartPtr, kNumJoints + 1>    PartsByJointPlusGaze;
  //typedef arr<sg::segmentation::VecConstPartPtr, Skeleton::kNumJointGroupsLR> PartsByJointGroupLR;

  //! Reduces active segments by grouping them by JointGroup
  static SegmentsByJointGroup groupSegmentsByJointGroup(const SegmentsByJointPlusGaze& segsByJoint);

  //! Reduces active segments by grouping them by JointGroupLR
  static SegmentsByJointGroupLR groupSegmentsByJointGroupLR(const SegmentsByJointPlusGaze& segsByJoint);

  //! Returns normalizing transform that will this skeleton and 
  //! translate so that SpineBase joint x-y is at origin,
  //! and orient such that body normal is facing +y
  ml::mat4f getNormalizingTransform() const;

  //! Computes a gaze direction and confidence for this Skeleton
  void computeGaze();

  //! Whether Skeleton appears to be in sitting position (hip height is close to knee height)
  bool isSitting() const;

  //! Find average distance to support surfaces in the scan (from hips when sitting, feet when standing).
  //! Returns positive infinity if no support was found.
  float avgDistToSupport(const Scan& scan) const;

  //! Find minimum distance to support surfaces in the scan (from hips when sitting, feet when standing).
  //! Returns positive infinity if no support was found.
  float minDistToSupport(const Scan& scan) const;

  //! Find maximum distance to support surfaces in the scan (from hips when sitting, feet when standing).
  //! Returns positive infinity if no support was found.
  float maxDistToSupport(const Scan& scan) const;

  //! If Skeleton is floating, detect closest support points in scene, and adjust skeleton to be supported on them
  //! Return vertical offset moved or zero if not moved
  float unfloat(const Scan& scan);

  //! Moves skeleton by delta
  void move(const ml::vec3f& delta);
  void move(const int idx, const float d);

  //! Return position of center of mass of this Skeleton
  ml::vec3f centerOfMass() const;

  //! Returns OBBs containing this Skeleton's bones
  const vec<geo::OBB>& getBoneOBBs() const;

  //! Returns [0,1] estimated ratio of Skeleton volume self-collisions (0 if none, 1 if collapsed)
  double getSelfCollisionRatio() const;

  //! Returns set of randomly sampled points within this Skeleton's OBBs (point expressed in Skeleton coordinate frame)
  //! VIP: Point set is cached between calls and regenerated only when different numPoints requested
  //! NOTE: Points are distributed to each Skeleton bone OBB proportional to volume of that bone
  const vec<ml::vec3f>& getPointsInOBBs(int numPoints = 100) const;

  //! Outputs this Skeleton to passed ostream in JSON format and returns the ostream
  ostream& toJSON(ostream&, bool endlines) const;

  // STORED STATE MEMBER VARIABLES
  uint64_t            trackingId = 0;
  ml::vec3f           jointPositions[kNumJoints];
  float               jointConfidences[kNumJoints];
  ml::vec4f           jointOrientations[kNumJoints];
  HandState           handLeftState, handRightState;
  TrackingConfidence  handLeftConfidence, handRightConfidence;
  DetectionResult     activities[Activity_Count];
  float               leanConfidence, leanLeftRight, leanForwardBack;
  unsigned long       clippedEdges;  // Mask indicating Skeleton clips edge (active bit according to FrameEdge) NOLINT
  int64_t             timestamp = 0;
  // if true observed by Kinect, otherwise generated from SkelState (orientation conventions differ)
  bool                isKinectSkel = false;

  // VOLATILE STATE - NOT STORED
  const Recording*    rec = nullptr;
  ml::vec3f           bodyPlaneNormal;
  ml::vec3f           bodyPlaneCentroid;
  float               bodyPlaneConf = 0.0f;
  float               bodyPlaneDist = 0.0f;
  ml::vec3f           gazePosition;
  float               gazeConfidence = 0.0f;
  ml::vec3f           gazeDirection;
  bool                gazeBodyPlaneOkay = false;  // Has body plane and gaze been computed?

  friend struct SkelState;

private:
  //! Compute the best fit body plane through all the torso and hip joints and set the centroid and normal
  void computeBodyPlaneCentroidNormal();

  mutable std::shared_ptr<vec<geo::OBB>> m_pBoneOBBs = nullptr;   // Cached OBBs containing bones of this Skeleton
  mutable std::shared_ptr<vec<ml::vec3f>> m_pBonePts = nullptr;   // Cached randomly sampled points within bone OBBs
  friend struct TransformedSkeleton;  // for access to above mutable state when it needs to be reset

  //! boost serialization function
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive& ar, const unsigned int version) {  // NOLINT
    ar & trackingId & jointPositions & jointConfidences & jointOrientations & handLeftState & handRightState
      & handLeftConfidence & handRightConfidence & activities & leanConfidence & leanLeftRight & leanForwardBack
      & clippedEdges & timestamp;
    if (version > 0) {
      ar & isKinectSkel;
    } else if (Archive::is_loading::value) {
      isKinectSkel = true;  // assume old serialized Skeletons are Kinect format
      // Make sure pointers are set to nullptr
      rec = nullptr;
      m_pBonePts = nullptr;
      m_pBoneOBBs = nullptr;
      // NOTE: Also need to call computeGaze()
      gazeBodyPlaneOkay = false;
    }
  }
};

inline ostream& toJSON(ostream& os, const Skeleton& s, bool endlines = true) {  // NOLINT
  return s.toJSON(os, endlines);
}
inline ostream& operator<<(ostream& os, const Skeleton& s) {
  return s.toJSON(os, true);
}

//! Templatized container of Skeleton parameters for convenient reference by external code
//! numJoints is a count of the number of joints (including the fake gaze joint)
template<typename JointT, size_t numJoints>
struct SkeletonParamsT {
  // Typedef member types
  typedef arr<JointT[2], numJoints - 1>         BonesT;     //! joint pairs defining bones
  typedef arr<string, numJoints>                NamesT;     //! name for each joint
  typedef arr<ml::vec4f, numJoints>             ColorsT;    //! color for each joint
  typedef arr<JointT, Skeleton::kNumJoints + 1> JointMapT;  //! indexed by raw Skel joint, giving corresponding joint
  typedef arr<vec<int>, numJoints>       JointInverseMapT;  //! Map from JointT to all contained raw joints

  // Member constants
  const int          kNumJoints;
  const int          kNumBones;
  const JointT       kGazeJoint;
  const BonesT       kBones;
  const NamesT       kJointNames;
  const ColorsT      kJointColors;
  const JointMapT    kJointMap;
  JointInverseMapT   kJointInvMap;
  
  SkeletonParamsT(const BonesT& bones, const NamesT& jointNames, const ColorsT& jointColors, const JointMapT& jointMap)
    : kNumJoints(numJoints)
    , kNumBones(numJoints - 1)
    , kGazeJoint(static_cast<JointT>(numJoints - 1))  // gaze assumed to always be last joint
    , kBones(bones)
    , kJointNames(jointNames)
    , kJointColors(jointColors)
    , kJointMap(jointMap) { 
    populateJointInvMap();
  }

  //! Segments grouped by JointType
  typedef arr<segmentation::VecSegPtr, numJoints> GroupedSegments;

  //! Returns segments grouped by JointGroup (by mapping from raw joints through kJointMap)
  GroupedSegments groupSegments(const Skeleton::SegmentsByJointPlusGaze& segsByJoint) const {
    GroupedSegments groupedSegs;
    for (unsigned int iJoint = 0; iJoint < Skeleton::kNumJoints; ++iJoint) {
      const auto& segs = segsByJoint[iJoint];
      auto& segGroup = groupedSegs[kJointMap[iJoint]];
      // TODO(MS): Account for duplicates here?
      std::copy(segs.begin(), segs.end(), std::back_inserter(segGroup));
    }
    return groupedSegs;
  }

  //! Returns name of joint
  template <typename T>
  string jointName(T t) const {
    if (t < 0 || t >= kNumJoints) { return to_string(t); }
    return kJointNames[t];
  }

 private:
  // Helper function to populate the inverse map
  void populateJointInvMap() {
    for (int iJoint = 0; iJoint < kNumJoints; iJoint++) {
      vec<int>& rawJoints = kJointInvMap[iJoint];
      for (int iSkelJoint = 0; iSkelJoint < Skeleton::kNumJoints + 1; iSkelJoint++) {
        if (kJointMap[iSkelJoint] == iJoint) {
          rawJoints.push_back(iSkelJoint);
        }
      }
    }
  }
};

extern const SkeletonParamsT<Skeleton::JointGroupLR, Skeleton::kNumJointGroupsLR> kSkelParamsJointGroupLR;

//! Defines a contiguous range of skeletons
struct SkelRange {
  typedef vec<std::reference_wrapper<const Skeleton>> container;
  SkelRange() : m_skels() { }
  explicit SkelRange(const container& skels) : m_skels(skels) { }
  explicit SkelRange(const Skeleton& skel) {
    container skels;
    skels.push_back(skel);
    m_skels = skels;
  }
  container::iterator begin() { return m_skels.begin(); }
  container::iterator end() { return m_skels.end(); }
  container::const_iterator begin() const { return m_skels.begin(); }
  container::const_iterator end() const { return m_skels.end(); }
  size_t size() const { return m_skels.size(); }
  template <typename Idx> const std::reference_wrapper<const Skeleton>& operator[](Idx i) const { return m_skels[i]; }
  const container& get() const { return m_skels; }
private:
  container m_skels;
};


//! Represents a Skeleton that was transformed by a particular transform matrix
struct TransformedSkeleton {
  TransformedSkeleton()
    : m_skel(nullptr) { }
  explicit TransformedSkeleton(std::shared_ptr<const Skeleton> pSkel,
                               const ml::mat4f& _transform = ml::mat4f::identity(),
                               double _score = 1.0)
    : score(_score), m_skel(pSkel), m_xform(_transform) { }
  TransformedSkeleton(const Skeleton& _skel, const ml::mat4f& _transform)
    : m_skel(util::ptr_to_shared<const Skeleton,const Skeleton>(&_skel))
    , m_xform(_transform) { }
  TransformedSkeleton(const TransformedSkeleton& base, const ml::mat4f& _transform)
    : weight(base.weight), score(base.score)
    , m_skel(base.m_skel), m_xform(_transform * base.m_xform) { }

  ml::vec3f transformedJoint(size_t jointIndex) const;

  //! Returns raw Skeleton transformed by this TransformedSkeleton's transform
  const Skeleton& getSkeleton() const {
    if (!m_xformedSkelComputed) {
      makeSkeleton();
    }
    return m_xformedSkel;
  }

  const Skeleton* getOriginalSkeleton() const {
    return m_skel.get();
  }

  TransformedSkeleton getNormalized() const {
    // Returns a transformed skeleton that is normalized
    ml::mat4f xform = m_skel->getNormalizingTransform();
    return TransformedSkeleton(m_skel, xform);
  }

  const ml::mat4f& transform() const { 
    return m_xform;
  }

  //! Parameters to be filled in by our various algorithms
  //! Weight of this transformed skeleton
  double weight = 1.0;
  //! Score of the transformed skeleton
  double score = 1.0;

  void setTransform(const ml::mat4f& xform) {
    m_xform = xform;
    m_xformedSkelComputed = false;
  }

  const vec<float>& getJointAngleDeltas() const {
    return m_jointAngDeltas;
  }

  void setJointAngleDelta(int iJoint, float ang) const {
    if (m_jointAngDeltas.empty()) {
      m_jointAngDeltas.resize(Skeleton::kNumJoints);
      fill(m_jointAngDeltas.begin(), m_jointAngDeltas.end(), 0.f);
    }
    m_jointAngDeltas[iJoint] = ang;
    m_xformedSkelComputed = false;  // need to recompute
  }

 private:
  void makeSkeleton() const;

  std::shared_ptr<const Skeleton> m_skel = nullptr;
  ml::mat4f m_xform;
  mutable bool m_xformedSkelComputed = false;
  mutable Skeleton m_xformedSkel;
  mutable vec<float> m_jointAngDeltas;  //! Joint angle deltas
};

}  // namespace core
}  // namespace sg

BOOST_CLASS_VERSION(sg::core::Skeleton, 1)

