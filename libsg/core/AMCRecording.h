#pragma once

#include "libsg.h"  // NOLINT

#include "core/SkelState.h"
#include "geo/geo.h"

// Based largely on CMU mocapPlayer: http://mocap.cs.cmu.edu/tools.php
// See for ASF/AMC format description: http://run.usc.edu/cs520-s12/assign2/
namespace sg {
namespace core {

// Adjust size of skeleton. Default value is 0.06 = human 1.7 m height (approximately)
#define MOCAP_SCALE 0.06f
#define MAX_BONES_IN_ASF_FILE 256

class AMCRecording;

// this structure defines the property of each bone segment, including its connection to other bones,
// DOF (degrees of freedom), relative orientation and distance to the outboard bone
struct Bone {
  struct Bone *sibling;  // Pointer to the sibling (branch bone) in the hierarchy tree
  struct Bone *child;    // Pointer to the child (outboard bone) in the hierarchy tree
  struct Bone *parent;   // Pointer to the parent bone in the hierarchy trees

  int idx;          // Bone index
  geo::Vec3f dir;   // Unit vector dir from local origin to origin of child bone (in local coords of bone)
  float length;     // Bone length
  geo::Vec3f axis;  // orientation of bone local coords as specified in ASF axis field

  int dof; // number of bone's degrees of freedom
  int dofrx, dofry, dofrz; // rotational degree of freedom mask in x, y, z axis
  int doftx, dofty, doftz; // translational degree of freedom mask in x, y, z axis
  int doftl;
  // dofrx=1 if this bone has x rotational degree of freedom, otherwise dofrx=0.

  // bone names
  char name[256];
  // rotation matrix from the local coordinate of this bone to the local coordinate system of it's parent
  geo::Quatf local2parent;
  geo::Quatf parent2local;

  //Rotation angles for this bone at a particular time frame (as read from AMC file) in local coordinate system, 
  //they are set in the setPosture function before display function is called
  geo::Vec3f r;
  geo::Vec3f t;
  double tl;
  int dofo[8];

  geo::Quatf local2world;  // world space orientation of this bone
  geo::Vec3f tgt_pos;  // position of this bone's endpoint in global coordinate system
};

//Root position and all bone rotation angles (including root) 
struct Posture {
  geo::Vec3f root_pos;  //Root position (x, y, z)
  //Euler angles (thetax, thetay, thetaz) of all bones in their local coordinate system.
  //If a bone does not have a certain degree of freedom, corresponding rotation is set to 0.
  //The order of the bones in the array corresponds to their ids in .ASf file: root, lhipjoint, lfemur,
  geo::Vec3f bone_rotation[MAX_BONES_IN_ASF_FILE];
  // bones that are translated relative to parents (resulting in gaps) (rarely used)
  geo::Vec3f bone_translation[MAX_BONES_IN_ASF_FILE];
  // bones that change length during the motion (rarely used)
  float bone_length[MAX_BONES_IN_ASF_FILE];
};

class ASFSkeleton {
public:
  ASFSkeleton(const string& file, float scale);
  int readASFfile(const string& file, float scale);

  //Get root node's address; for accessing bone data
  Bone* getRoot() const;

  //Set the skeleton's pose based on the given posture
  void setPosture(const Posture& posture);

  //Initial posture Root at (0,0,0) - All bone rotations are set to 0
  void setBasePosture();

  int name2idx(const char*) const;
  char* idx2name(int);
  int numBonesInSkel(Bone bone) const;
  int movBonesInSkel(Bone bone) const;

  //Recurse on skeleton hierarchy and return pointer to bone with index bIndex
  //ptr should be root when function first called
  Bone* getBone(Bone* ptr, int bIndex) const;

  vec<const Bone*> getBones() const;

  typedef arr<geo::Quatf, SkelState::kNumJoints> JointQuats;
  // quaternions representing coords of bone corresponding to each SkelState joint
  const JointQuats& getBoneQuats() const { return m_boneQuats; }

  //This function sets sibling or child for parent bone
  //If parent bone does not have a child, then pChild is set as parent's child
  //else pChild is set as a sibling of parents already existing child
  int setChildrenAndSibling(int parent, Bone* pChild) const;

  static void computeLocalToParentRotations(Bone* parent, Bone* child);
  void ComputeRotationToParentCoordSystem(Bone* bone) const;
  void ComputeBoneWorldPositions(Bone* bone) const;

 protected:
  JointQuats computeSkelStateQuaternions() const;

  friend class AMCRecording;
  // root position in world coordinate system
  geo::Vec3f m_RootPos;

  int NUM_BONES_IN_ASF_FILE;
  int MOV_BONES_IN_ASF_FILE;

  JointQuats m_boneQuats;
  JointQuats m_boneRs;
  Bone *m_pRootBone;  // Pointer to the root bone, m_RootBone = &bone[0]
  Bone  m_pBoneList[MAX_BONES_IN_ASF_FILE];   // Array with all skeleton bones
};

class AMCRecording {
 public:
  // parse AMC file with given scale and skeleton
  AMCRecording(const string& file, float scale, ASFSkeleton* pSkeleton);
  ~AMCRecording();

  // scale is a parameter to adjust the translationalal scaling
  // the value of scale should be consistent with the scale parameter used in Skeleton()
  int writeAMCfile(const string& file, float scale) const;

  //Set all postures to default posture
  //Root position at (0,0,0), orientation of each bone to (0,0,0)
  void SetPosturesToDefault() const;

  //Set the entire posture at specified frame (posture = root position and all bone rotations)
  void SetPosture(int frameIndex, const Posture& InPosture) const;

  //Set root position at specified frame
  void SetRootPos(int frameIndex, const geo::Vec3f& vPos) const;
  //Set specified bone rotation at specified frame
  void SetBoneRotation(int frameIndex, int boneIndex, const geo::Vec3f& vRot) const;

  int GetNumFrames() const { return m_NumFrames; }
  Posture* GetPosture(int frameIndex) const;
  ASFSkeleton* GetSkeleton() const { return pSkeleton; }

  geo::Transform getRecordingToWorldXform() const;

  vec<SkelState> toSkelStates() const;

 protected:
  int m_NumFrames; //number of frames in the motion 
  ASFSkeleton* pSkeleton;
  //Root position and all bone rotation angles for each frame (as read from AMC file)
  Posture* m_pPostures;

  mutable vec<int> kinect2ASFidx;  // maps kinect index to asf bone idx
  mutable vec<int> asfIdx2kinectIdx;  // maps asf bone index to kinect idx

  // The default value is 0.06
  int readAMCfile(const string& file, float scale);
};

}  // namespace core
}  // namespace sg

