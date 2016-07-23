#include "common.h"  // NOLINT

#include "core/AMCRecording.h"

#include "core/Skeleton.h"
#include "core/SkelState.h"

// Based largely on CMU mocapPlayer: http://mocap.cs.cmu.edu/tools.php

using namespace sg::geo;

using AAf = Eigen::AngleAxisf;
const Vec3f eX = Vec3f::UnitX(), eY = Vec3f::UnitY(), eZ = Vec3f::UnitZ();
const float deg2rad = static_cast<float>(M_PI) / 180.f;
AAf Rx(float x) { return AAf(x * deg2rad, eX); }
AAf Ry(float y) { return AAf(y * deg2rad, eY); }
AAf Rz(float z) { return AAf(z * deg2rad, eZ); }
Quatf EulerXYZ2Quat(const Vec3f& v) {
  return Rz(v.z()) * Ry(v.y()) * Rx(v.x());
}

void removeCR(char* str) {
  if (str[strlen(str) - 1] == '\r') { str[strlen(str) - 1] = 0; }
}

namespace sg {
namespace core {

const map<Skeleton::JointType, string> kKinectBones2ASFBones = {
  {Skeleton::JointType_SpineBase, "root"},
  {Skeleton::JointType_SpineMid, "lowerback"},
  {Skeleton::JointType_SpineShoulder, "lowerneck"},
  {Skeleton::JointType_Neck, "upperneck"},
  {Skeleton::JointType_Head, "head"},
  {Skeleton::JointType_HipLeft, "lhipjoint"},
  {Skeleton::JointType_HipRight, "rhipjoint"},
  {Skeleton::JointType_KneeLeft, "lfemur"},
  {Skeleton::JointType_KneeRight, "rfemur"},
  {Skeleton::JointType_AnkleLeft, "ltibia"},
  {Skeleton::JointType_AnkleRight, "rtibia"},
  {Skeleton::JointType_FootLeft, "lfoot"},
  {Skeleton::JointType_FootRight, "rfoot"},
  {Skeleton::JointType_ShoulderLeft, "lclavicle"},
  {Skeleton::JointType_ShoulderRight, "rclavicle"},
  {Skeleton::JointType_ElbowLeft, "lhumerus"},
  {Skeleton::JointType_ElbowRight, "rhumerus"},
  {Skeleton::JointType_WristLeft, "lradius"},
  {Skeleton::JointType_WristRight, "rradius"},
  {Skeleton::JointType_ThumbLeft, "lthumb"},
  {Skeleton::JointType_ThumbRight, "rthumb"},
  {Skeleton::JointType_HandLeft, "lwrist"},
  {Skeleton::JointType_HandRight, "rwrist"},
  {Skeleton::JointType_HandTipLeft, "lhand"},
  {Skeleton::JointType_HandTipRight, "rhand"}
};

int ASFSkeleton::numBonesInSkel(Bone bone) const {
  Bone* tmp = bone.sibling;
  int numBones = 0;
  while (tmp != nullptr) {
    if (tmp->child != nullptr) {
      numBones += numBonesInSkel(*(tmp->child));
    }
    numBones++;
    tmp = tmp->sibling;
  }
  if (bone.child != nullptr) {
    return numBones + 1 + numBonesInSkel(*bone.child);
  } else {
    return numBones + 1;
  }
}

int ASFSkeleton::movBonesInSkel(Bone bone) const {
  Bone * tmp = bone.sibling;
  int numBones = 0;

  if (bone.dof > 0) { numBones++; }

  while (tmp != nullptr) {
    if (tmp->child != nullptr)
      numBones += movBonesInSkel(*(tmp->child));
    if (tmp->dof > 0)
      numBones++;
    tmp = tmp->sibling;
  }

  if (bone.child != nullptr) { return (numBones + movBonesInSkel(*bone.child)); }
  else { return numBones; }
}

// helper function to convert ASF part name into bone index
int ASFSkeleton::name2idx(const char*name) const {
  int i = 0;
  while (strcmp(m_pBoneList[i].name, name) != 0 && i++ < NUM_BONES_IN_ASF_FILE) { }
  return m_pBoneList[i].idx;
}

char* ASFSkeleton::idx2name(int idx) {
  int i = 0;
  while (m_pBoneList[i].idx != idx && i++ < NUM_BONES_IN_ASF_FILE) { }
  return m_pBoneList[i].name;
}

int ASFSkeleton::readASFfile(const string& file, float scale) {
  //open file
  ifstream is(file);
  if (is.fail()) { return -1; }

  // ignore header information
  char str[2048], keyword[256];
  while (1) {
    is.getline(str, 2048);
    removeCR(str);
    sscanf(str, "%s", keyword);
    if (strcmp(keyword, ":bonedata") == 0)
      break;
  }

  // read bone information: global orientation and translation, DOF.
  is.getline(str, 2048);
  removeCR(str);
  char	part[256], *token;
  float length;

  bool done = false;
  for (int i = 1; (!done) && (i < MAX_BONES_IN_ASF_FILE); i++) {
    m_pBoneList[i].dof = 0;
    m_pBoneList[i].dofrx = m_pBoneList[i].dofry = m_pBoneList[i].dofrz = 0;
    m_pBoneList[i].doftx = m_pBoneList[i].dofty = m_pBoneList[i].doftz = 0;
    m_pBoneList[i].doftl = 0;
    m_pBoneList[i].sibling = nullptr;
    m_pBoneList[i].child = nullptr;
    m_pBoneList[i].parent = nullptr;
    NUM_BONES_IN_ASF_FILE++;
    MOV_BONES_IN_ASF_FILE++;
    while (1) {
      is.getline(str, 2048);
      removeCR(str);
      sscanf(str, "%s", keyword);

      if (strcmp(keyword, "end") == 0)
        break;

      if (strcmp(keyword, ":hierarchy") == 0) {
        MOV_BONES_IN_ASF_FILE -= 1;
        NUM_BONES_IN_ASF_FILE -= 1;
        done = true;
        break;
      }

      //id of bone
      if (strcmp(keyword, "id") == 0)
        m_pBoneList[i].idx = NUM_BONES_IN_ASF_FILE - 1;

      //name of the bone
      if (strcmp(keyword, "name") == 0) {
        sscanf(str, "%s %s", keyword, part);
        sscanf(str, "%s %s", keyword, m_pBoneList[i].name);
      }

      //this line describes the bone's direction vector in global coordinates
      //it will later be converted to local coorinate system
      if (strcmp(keyword, "direction") == 0) {
        float dx, dy, dz;
        sscanf(str, "%s %f %f %f", keyword, &dx, &dy, &dz);
        m_pBoneList[i].dir = {dx, dy, dz};
      }

      //length of the bone
      if (strcmp(keyword, "length") == 0)
        sscanf(str, "%s %f", keyword, &length);

      //this line describes the orientation of bone's local coordinate 
      //system relative to the world coordinate system
      if (strcmp(keyword, "axis") == 0) {
        float ax, ay, az;
        sscanf(str, "%s %f %f %f", keyword, &ax, &ay, &az);
        m_pBoneList[i].axis = {ax, ay, az};
      }

      // this line describes the bone's dof 
      if (strcmp(keyword, "dof") == 0) {
        token = strtok(str, " ");
        m_pBoneList[i].dof = 0;
        while (token != nullptr) {
          int tdof = m_pBoneList[i].dof;
          if (strcmp(token, "rx") == 0) {
            m_pBoneList[i].dofrx = 1; m_pBoneList[i].dofo[tdof] = 1;
          } else if (strcmp(token, "ry") == 0) {
            m_pBoneList[i].dofry = 1; m_pBoneList[i].dofo[tdof] = 2;
          } else if (strcmp(token, "rz") == 0) {
            m_pBoneList[i].dofrz = 1; m_pBoneList[i].dofo[tdof] = 3;
          } else if (strcmp(token, "tx") == 0) {
            m_pBoneList[i].doftx = 1; m_pBoneList[i].dofo[tdof] = 4;
          } else if (strcmp(token, "ty") == 0) {
            m_pBoneList[i].dofty = 1; m_pBoneList[i].dofo[tdof] = 5;
          } else if (strcmp(token, "tz") == 0) {
            m_pBoneList[i].doftz = 1; m_pBoneList[i].dofo[tdof] = 6;
          } else if (strcmp(token, "l") == 0) {
            m_pBoneList[i].doftl = 1; m_pBoneList[i].dofo[tdof] = 7;
          } else if (strcmp(token, "dof") == 0) {
            goto end;
          } else {
            printf("UNKNOWN %s\n", token);
          }
          m_pBoneList[i].dof++;
          m_pBoneList[i].dofo[m_pBoneList[i].dof] = 0;
        end:
          token = strtok(nullptr, " ");
        }
        // printf("Bone %d DOF: ", i);
        //for (int x = 0; (x < 7) && (m_pBoneList[i].dofo[x] != 0); x++)
        //  printf("%d ", m_pBoneList[i].dofo[x]);
        //printf("\n");
      }
    }

    //store all the info we read from the file into the data structure
    //		m_pBoneList[i].idx = name2idx(part);
    if ((!m_pBoneList[i].dofrx) && (!m_pBoneList[i].dofry) && (!m_pBoneList[i].dofrz))
      MOV_BONES_IN_ASF_FILE -= 1;
    m_pBoneList[i].length = length * scale;
  }

  //SG_LOG_INFO << "[ASFSkeleton] Read " << file << ", numBones=" << NUM_BONES_IN_ASF_FILE;

  //read and build the hierarchy of the skeleton
  char* part_name;
  int j, parent = 0;

  //find "hierarchy" string in the ASF file
  /*	while(1)
  {
  is.getline(str, 2048);	sscanf(str, "%s", keyword);
  if(strcmp(keyword, ":hierarchy") == 0)
  break;
  } */

  //skip "begin" line
  is.getline(str, 2048);
  removeCR(str);

  //Assign parent/child relationship to the bones
  while (1) {
    //read next line
    is.getline(str, 2048);
    removeCR(str);

    sscanf(str, "%s", keyword);

    //check if we are done
    if (strcmp(keyword, "end") == 0) {
      break;
    } else {
      //parse this line, it contains parent followed by children
      part_name = strtok(str, " ");
      j = 0;
      while (part_name != nullptr) {
        if (j == 0)
          parent = name2idx(part_name);
        else
          setChildrenAndSibling(parent, &m_pBoneList[name2idx(part_name)]);
        part_name = strtok(nullptr, " ");
        j++;
      }
    }
  }

  is.close();

  return 0;
}

// This recursive function traverces skeleton hierarchy and returns a pointer to the bone with index - bIndex
// ptr should be a pointer to the root node when this function first called
Bone* ASFSkeleton::getBone(Bone *ptr, int bIndex) const {
  static Bone *theptr;
  if (ptr == nullptr)
    return(nullptr);
  else if (ptr->idx == bIndex) {
    theptr = ptr;
    return(theptr);
  } else {
    getBone(ptr->child, bIndex);
    getBone(ptr->sibling, bIndex);
    return(theptr);
  }
}

vec<const Bone*> ASFSkeleton::getBones() const {
  vec<const Bone*> bones;
  for (int i = 0; i < NUM_BONES_IN_ASF_FILE; ++i) {
    bones.push_back(&m_pBoneList[i]);
  }
  return bones;
}

/*
This function sets sibling or child for parent bone
If parent bone does not have a child, then pChild is set as parent's child
else pChild is set as a sibling of parents already existing child
*/
int ASFSkeleton::setChildrenAndSibling(int parent, Bone *pChild) const {
  Bone *pParent;

  //Get pointer to root bone
  pParent = getBone(m_pRootBone, parent);

  if (pParent == nullptr) {
    SG_LOG_ERROR << "[ASFSkeleton] inboard bone is undefined";
    return 0;
  } else {
    //if pParent bone does not have a child set pChild as parent bone child
    if (pParent->child == nullptr) {
      pParent->child = pChild;
    } else {
      //if pParent bone already has child set pChild as pParent's child sibling
      Bone* pBone = pParent->child;
      while (pBone->sibling != nullptr)
        pBone = pBone->sibling;

      pBone->sibling = pChild;
    }
    pChild->parent = pParent;
    return 1;
  }
}

Bone* ASFSkeleton::getRoot() const {
  return m_pRootBone;
}

void ASFSkeleton::computeLocalToParentRotations(Bone *p, Bone *c) {
  if (c != nullptr) {
    const auto tmp1 = Rx(-p->axis.x()) * Ry(-p->axis.y()) * Rz(-p->axis.z());
    const auto tmp2 = Rz(c->axis.z())  * Ry(c->axis.y())  * Rx(c->axis.x());
    const auto tmp = tmp1 * tmp2;
    c->local2parent = tmp;
    c->parent2local = tmp.inverse();
  }
}

void ASFSkeleton::ComputeRotationToParentCoordSystem(Bone *bone) const {
  // root joint local coordinate system orientation to world xform
  bone[0].local2parent = EulerXYZ2Quat(bone[0].axis);
  bone[0].parent2local = bone[0].local2parent.inverse();

  //Compute local2parent for all other bones
  int numbones = numBonesInSkel(bone[0]);
  for (int i = 0; i < numbones; i++) {
    if (bone[i].child != nullptr) {
      computeLocalToParentRotations(&bone[i], bone[i].child);
      // compute parent child siblings...
      Bone* b = nullptr;
      if (bone[i].child != nullptr) b = (bone[i].child)->sibling;
      while (b != nullptr) {
        computeLocalToParentRotations(&bone[i], b);
        b = b->sibling;
      }
    }
  }
}

void ASFSkeleton::ComputeBoneWorldPositions(Bone *bone) const {
  const auto xform = [] (Bone* b) {
    const Quatf qR = EulerXYZ2Quat(b->r);
    if (b->parent) {
      b->local2world = b->parent->local2world * b->local2parent * qR;
    } else {
      b->local2world = b->local2parent * qR;
    }
    b->tgt_pos = b->length * (b->local2world * b->dir);
    if (b->parent) { b->tgt_pos += b->parent->tgt_pos; }
  };
  xform(bone);
  int numbones = numBonesInSkel(bone[0]);
  for (int i = 0; i < numbones; i++) {
    if (bone[i].child != nullptr) {
      xform(bone[i].child);

      Bone* b = nullptr;
      if (bone[i].child != nullptr) b = (bone[i].child)->sibling;
      while (b != nullptr) {
        xform(b);
        b = b->sibling;
      }
    }
  }
}

//Initial posture Root at (0,0,0). All bone rotations are set to 0
void ASFSkeleton::setBasePosture() {
  m_RootPos.setZero();
  for (int i = 0; i < NUM_BONES_IN_ASF_FILE; ++i) {
    m_pBoneList[i].r.setZero();
    m_pBoneList[i].t.setZero();
  }
  ComputeBoneWorldPositions(getRoot());
}

// set the skeleton's pose based on the given posture
void ASFSkeleton::setPosture(const Posture& posture) {
  m_RootPos = posture.root_pos;

  for (int j = 0; j < NUM_BONES_IN_ASF_FILE; j++) {
    // if the bone has rotational degree of freedom in x direction
    if (m_pBoneList[j].dofrx)
      m_pBoneList[j].r.x() = posture.bone_rotation[j][0];

    if (m_pBoneList[j].doftx)
      m_pBoneList[j].t.x() = posture.bone_translation[j][0];

    // if the bone has rotational degree of freedom in y direction
    if (m_pBoneList[j].dofry)
      m_pBoneList[j].r.y() = posture.bone_rotation[j][1];

    if (m_pBoneList[j].dofty)
      m_pBoneList[j].t.y() = posture.bone_translation[j][1];

    // if the bone has rotational degree of freedom in z direction
    if (m_pBoneList[j].dofrz)
      m_pBoneList[j].r.z() = posture.bone_rotation[j][2];

    if (m_pBoneList[j].doftz)
      m_pBoneList[j].t.z() = posture.bone_translation[j][2];

    if (m_pBoneList[j].doftl)
      m_pBoneList[j].tl = posture.bone_length[j];
  }

  ComputeBoneWorldPositions(getRoot());
}

ASFSkeleton::ASFSkeleton(const string& file, float scale) {
  sscanf("root", "%s", m_pBoneList[0].name);
  NUM_BONES_IN_ASF_FILE = 1;
  MOV_BONES_IN_ASF_FILE = 1;
  m_pBoneList[0].dofo[0] = 4;
  m_pBoneList[0].dofo[1] = 5;
  m_pBoneList[0].dofo[2] = 6;
  m_pBoneList[0].dofo[3] = 1;
  m_pBoneList[0].dofo[4] = 2;
  m_pBoneList[0].dofo[5] = 3;
  m_pBoneList[0].dofo[6] = 0;
  //Initialization
  m_pBoneList[0].idx = 0;  // root of hierarchy
  m_pRootBone = &m_pBoneList[0];
  m_pBoneList[0].sibling = nullptr;
  m_pBoneList[0].child = nullptr;
  m_pBoneList[0].parent = nullptr;
  m_pBoneList[0].dir.setZero();
  m_pBoneList[0].axis.setZero();
  m_pBoneList[0].length = 0.05f;
  m_pBoneList[0].dof = 6;
  m_pBoneList[0].dofrx = m_pBoneList[0].dofry = m_pBoneList[0].dofrz = 1;
  m_pBoneList[0].doftx = m_pBoneList[0].dofty = m_pBoneList[0].doftz = 1;
  m_pBoneList[0].doftl = 0;
  m_RootPos.setZero();

  // build hierarchy and read in each bone's DOF information
  if (readASFfile(file, scale) != 0) {
    SG_LOG_ERROR << "[ASFSkeleton] Failure reading ASF format file: " << file;
  }

  //transform the direction for each bone from world coordinates to its local coords
  for (int i = 1; i < NUM_BONES_IN_ASF_FILE; ++i) {
    //Rotate vector v by a, b, c in X,Y,Z order. v_out = Rz(c)*Ry(b)*Rx(a)*v_in
    //TODO: Check order (this matched original code!)
    Quatf Rq = Rx(-m_pBoneList[i].axis.x()) * Ry(-m_pBoneList[i].axis.y()) * Rz(-m_pBoneList[i].axis.z());
    m_pBoneList[i].dir = Rq * m_pBoneList[i].dir;
  }

  //Calculate rotation from bone local to the parent bone local (stored in local2parent)
  ComputeRotationToParentCoordSystem(m_pRootBone);
  setBasePosture();
  m_boneQuats = computeSkelStateQuaternions();
}

ASFSkeleton::JointQuats ASFSkeleton::computeSkelStateQuaternions() const {
  JointQuats qs;
  using J = Skeleton::JointType;
  const auto p = [&] (const J& iJoint) {
    const string& boneName = kKinectBones2ASFBones.at(iJoint);
    const int bIdx = name2idx(boneName.c_str());
    const Bone& b = *getBone(getRoot(), bIdx);
    const Vec3f bpBody = m_pRootBone->parent2local * b.tgt_pos;
    return bpBody;
  };
  const Vec3f
    // canonical ASFSkeleton frame is x-left, y-up, z-forward
    up = Vec3f::UnitY(),
    right = -Vec3f::UnitX(),
    /// TORSO ///
    // spine base canonical frame
    yb = up,                                    // pelvic up
    xb = crossNorm(yb, right),                  // pelvic front
    zb = crossNorm(xb, yb),                     // pelvic right

    // pelvis frame - origin = ow, child = ove
    ove = p(J::JointType_SpineMid),             // mid-spine vertebrae
    yp = (ove - p(J::JointType_SpineBase)).normalized(),    // pelvic up
    xp = crossNorm(yp, right),                  // pelvic front
    zp = crossNorm(xp, yp),                     // pelvic right

    // torso frame - origin = ove, child = osc
    osc = p(J::JointType_SpineShoulder),        // sternoclavicular joint
    yto = (osc - ove).normalized(),             // torso up
    xto = crossNorm(yto, right),                // torso front
    zto = crossNorm(xto, yto),                  // torso right

    // shoulder-mid frame - origin = osc, child = ocv
    ocv = p(J::JointType_Neck),                 // cervical vertebrae (neck)
    ysh = (ocv - osc).normalized(),             // shoulder-mid up
    xsh = crossNorm(ysh, right),                // shoulder-mid front
    zsh = crossNorm(xsh, ysh),                  // shoulder-mid right

    // neck frame - origin = ocv, child = ohe
    zne = zp,                                   // neck right
   _xne = crossNorm(ysh, zne),                  // neck front temporary
    yne = crossNorm(zne, _xne),                 // neck up
    xne = crossNorm(yne, zne),                  // neck front

    // L clavicle frame - origin osc, child = ghl
    ghl = p(J::JointType_ShoulderLeft),         // glenohumeral rotation center L
    zcl = -(ghl - osc).normalized(),            // clavicle right L
    xcl = crossNorm(ysh, zcl),                  // clavicle front L
    ycl = crossNorm(zcl, xcl),                  // clavicle up L

    // R clavicle frame - origin osc, child = ghr
    ghr = p(J::JointType_ShoulderRight),        // glenohumeral rotation center R
    zcr = (ghr - osc).normalized(),             // clavicle right R
    xcr = crossNorm(ysh, zcr),                  // clavicle front R
    ycr = crossNorm(zcr, xcr),                  // clavicle up R

    /// ARMS AND HANDS ///

    // L arm frame common
    ecl = p(J::JointType_ElbowLeft),            // epicondyle L
    yhl = (ghl - ecl).normalized(),             // humerus up L
    scl = p(J::JointType_WristLeft),            // radial+ulnar styloid L
    yfl = (ecl - scl).normalized(),             // forearm up L

    // L humerus frame - origin = ghl, child = ecl
    // yhl computed in common arm frame
    _zhl = crossNorm(yfl, yhl),
    zhl = (_zhl.dot(right) > 0) ? _zhl : -_zhl, // humerus right L
    xhl = crossNorm(yhl, zhl),                  // humerus front L
    // TODO(MS): degenerate case: yhl-yfl parallel

    // L forearm frame - origin = scl, child = ecl
    // yfl computed in common arm frame
    zfl = zhl,                                  // forearm right L
    xfl = crossNorm(yfl, zfl),                  // forearm front L

    // L palm frame - origin = scl, child = pal
    xrl = xfl,                                  // trapezium front L
    yrl = yfl,                                  // trapezium up L
    zrl = zfl,                                  // trapezium right L

    // L thumb frame - origin = pal, child = thl
    xxl = xfl,                                  // thumb front L
    yxl = yfl,                                  // thumb up L
    zxl = zfl,                                  // thumb right L

    // L fingertip frame - origin = pal, child = phl
    xpl = xfl,                                  // distal phalanges front L
    zpl = zfl,                                  // distal phalanges right L
    ypl = yfl,                                  // distal phalanges up L

    // R arm frame common
    ecr = p(J::JointType_ElbowRight),           // epicondyle R
    yhr = (ghr - ecr).normalized(),             // humerus up R
    scr = p(J::JointType_WristRight),           // radial+ulnar styloid R
    yfr = (ecr - scr).normalized(),             // forearm up R

    // R humerus frame - origin = ghr, child = ecr
    // yhl computed in common arm frame
    _zhr = crossNorm(yfr, yhr),
    zhr = (_zhr.dot(right) > 0) ? _zhr : -_zhr, // humerus right R
    xhr = crossNorm(yhr, zhr),                  // humerus front R
    // TODO(MS): degenerate case: yhr-yfr parallel

    // R forearm frame - origin = scr, child = ecr
    // yfl computed in common arm frame
    zfr = zhr,                                  // forearm right R
    xfr = crossNorm(yfr, zfr),                  // forearm front R

    // R palm frame - origin = scr, child = par
    xrr = xfr,                                  // trapezium front R
    yrr = yfr,                                  // trapezium up R
    zrr = zfr,                                  // trapezium right R

    // R thumb frame - origin = par, child = thr
    xxr = xfr,                                  // thumb front R
    yxr = yfr,                                  // thumb up R
    zxr = zfr,                                  // thumb right R

    // R fingertip frame - origin = par, child = thr
    xpr = xfr,                                  // distal phalanges front R
    zpr = zfr,                                  // distal phalanges right R
    ypr = yfr,                                  // distal phalanges up R

    /// LEGS AND FEET ///

    // L ilium/hip - origin = ove, child = fhl
    fhl = p(J::JointType_HipLeft),              // hip femur head L
    zil = -fhl.normalized(),                    // ilium/hip right L
    xil = crossNorm(yp, zil),                   // ilium/hip front L
    yil = crossNorm(zil, xil),                  // ilium/hip up L

    // L thigh - origin = fhl, child = otl
    otl = p(J::JointType_KneeLeft),             // knee L
    ytl = (fhl - otl).normalized(),             // thigh up L
    xtl = crossNorm(ytl, right),                // thigh front L
    ztl = crossNorm(xtl, ytl),                  // thigh right L

    // L shank - origin = otl, child osl
    osl = p(J::JointType_AnkleLeft),            // ankle L
    ysl = (otl - osl).normalized(),             // shank up L
    xsl = crossNorm(ysl, right),                // shank front L
    zsl = crossNorm(xsl, ysl),                  // shank right L

    // L foot
    yml = -xsl,                                 // foot toe up L
    zml = zsl,                                  // foot toe right L
    xml = crossNorm(yml, zml),                  // foot toe front L

    // R ilium/hip - origin = ove, child = fhr
    fhr = p(J::JointType_HipRight),             // hip femur head R
    zir = fhr.normalized(),                     // ilium/hip right R
    xir = crossNorm(yp, zir),                   // ilium/hip front R
    yir = crossNorm(zir, xir),                  // ilium/hip up R

    // R thigh - origin = fhr, child = otr
    otr = p(J::JointType_KneeRight),            // knee R
    ytr = (fhr - otr).normalized(),             // thigh up R
    xtr = crossNorm(ytr, right),                // thigh front R
    ztr = crossNorm(xtr, ytr),                  // thigh right R
 
    // R shank - origin = otr, child osr
    osr = p(J::JointType_AnkleRight),           // ankle R
    ysr = (otr - osr).normalized(),             // shank up R
    xsr = crossNorm(ysr, right),                // shank front R
    zsr = crossNorm(xsr, ysr),                  // shank right R

    // R foot
    ymr = -xsr,                                 // foot toe up R
    zmr = zsr,                                  // foot toe right R
    xmr = crossNorm(ymr, zmr);                  // foot toe front R

  qs[J::JointType_SpineBase]      = Rquat(xb,  yb,  zb),   // base
  qs[J::JointType_SpineMid]       = Rquat(xp,  yp,  zp),   // pelvis
  qs[J::JointType_SpineShoulder]  = Rquat(xto, yto, zto),  // torso
  qs[J::JointType_Neck]           = Rquat(xsh, ysh, zsh),  // neck
  qs[J::JointType_Head]           = Rquat(xne, yne, zne),  // head
  qs[J::JointType_ShoulderLeft]   = Rquat(xcl, ycl, zcl),  // clavicle L
  qs[J::JointType_ShoulderRight]  = Rquat(xcr, ycr, zcr),  // clavicle R
  qs[J::JointType_ElbowLeft]      = Rquat(xhl, yhl, zhl),  // humerus L
  qs[J::JointType_WristLeft]      = Rquat(xfl, yfl, zfl),  // forearm L
  qs[J::JointType_HandLeft]       = Rquat(xrl, yrl, zrl),  // trapezium L
  qs[J::JointType_ThumbLeft]      = Rquat(xxl, yxl, zxl),  // thumb L
  qs[J::JointType_HandTipLeft]    = Rquat(xpl, ypl, zpl),  // hand tips L
  qs[J::JointType_ElbowRight]     = Rquat(xhr, yhr, zhr),  // humerus R
  qs[J::JointType_WristRight]     = Rquat(xfr, yfr, zfr),  // forearm R
  qs[J::JointType_HandRight]      = Rquat(xrr, yrr, zrr),  // trapezium R
  qs[J::JointType_ThumbRight]     = Rquat(xxr, yxr, zxr),  // thumb R
  qs[J::JointType_HandTipRight]   = Rquat(xpr, ypr, zpr),  // hand tips R
  qs[J::JointType_HipLeft]        = Rquat(xil, yil, zil),  // hip L
  qs[J::JointType_KneeLeft]       = Rquat(xtl, ytl, ztl),  // thigh L
  qs[J::JointType_AnkleLeft]      = Rquat(xsl, ysl, zsl),  // shank L
  qs[J::JointType_FootLeft]       = Rquat(xml, yml, zml),  // foot L
  qs[J::JointType_HipRight]       = Rquat(xir, yir, zir),  // hip R
  qs[J::JointType_KneeRight]      = Rquat(xtr, ytr, ztr),  // thigh R
  qs[J::JointType_AnkleRight]     = Rquat(xsr, ysr, zsr),  // shank R
  qs[J::JointType_FootRight]      = Rquat(xmr, ymr, zmr);  // foot R

  for (int i = 0; i < Skeleton::kNumJoints; ++i) {
    const J ji = static_cast<J>(i);
    const int bIdx = name2idx(kKinectBones2ASFBones.at(ji).c_str());
    const Bone& b = *getBone(getRoot(), bIdx);
    qs[i] = b.local2world.inverse() * qs[i];
  }

  return qs;
}

AMCRecording::AMCRecording(const string& file, float scale, ASFSkeleton* pSkeleton_) {
  pSkeleton = pSkeleton_;
  m_NumFrames = 0;
  m_pPostures = nullptr;

  if (readAMCfile(file, scale) < 0) {
    SG_LOG_ERROR << "[AMCRecording] Error reading AMC format file: " << file;
  }
}

AMCRecording::~AMCRecording() {
  if (m_pPostures != nullptr)
    delete[] m_pPostures;
}

//Set all postures to default posture
void AMCRecording::SetPosturesToDefault() const {
  for (int frame = 0; frame < m_NumFrames; frame++) {
    //set root position to (0,0,0)
    m_pPostures[frame].root_pos.setZero();
    //set each bone orientation to (0,0,0)
    for (int j = 0; j < MAX_BONES_IN_ASF_FILE; j++)
      m_pPostures[frame].bone_rotation[j].setZero();
  }
}

//Set posture at spesified frame
void AMCRecording::SetPosture(int frameIndex, const Posture& InPosture) const {
  m_pPostures[frameIndex] = InPosture;
}

void AMCRecording::SetBoneRotation(int frameIndex, int boneIndex, const Vec3f& vRot) const {
  m_pPostures[frameIndex].bone_rotation[boneIndex] = vRot;
}

void AMCRecording::SetRootPos(int frameIndex, const Vec3f& vPos) const {
  m_pPostures[frameIndex].root_pos = vPos;
}

Posture* AMCRecording::GetPosture(int frameIndex) const {
  if (frameIndex < 0 || frameIndex >= m_NumFrames) {
    SG_LOG_ERROR << "Error in Motion::GetPosture: illegal frame index: " << frameIndex;
    return nullptr;
  }
  return &(m_pPostures[frameIndex]);
}

Transform AMCRecording::getRecordingToWorldXform() const {
  const Transform xform = toTransRot(Quatf(Rx(90)), Vec3f::Zero());
  return xform;
}

vec<SkelState> AMCRecording::toSkelStates() const {
  // build bone correspondences
  kinect2ASFidx.resize(Skeleton::kNumJoints);
  fill(kinect2ASFidx.begin(), kinect2ASFidx.end(), -1);
  asfIdx2kinectIdx.resize(pSkeleton->NUM_BONES_IN_ASF_FILE);
  fill(asfIdx2kinectIdx.begin(), asfIdx2kinectIdx.end(), -1);
  for (const auto& pair : kKinectBones2ASFBones) {
    int asfBoneIdx = pSkeleton->name2idx(pair.second.c_str());
    if (asfBoneIdx >= 0) {
      kinect2ASFidx[pair.first] = asfBoneIdx;
      asfIdx2kinectIdx[asfBoneIdx] = pair.first;
    } else {
      SG_LOG_WARN << "Bone with name " << pair.second << " not found in ASF file";
    }
  }

  const auto asfSkel2state = [&] (const ASFSkeleton& skel) {
    SkelState o;
    const auto& bDirs = skel.getBoneQuats();
    o.bodyFrame.p = skel.m_RootPos;
    o.bodyFrame.q = toAngleEncoding(skel.getRoot()->local2world * Ry(-90), o.angleEncoding);

    for (const Bone* b : skel.getBones()) {
      const int kIdx = asfIdx2kinectIdx[b->idx];
      if (kIdx >= 0) {
        o.joints[kIdx].q = toAngleEncoding(b->local2world * bDirs[kIdx], o.angleEncoding);
        o.joints[kIdx].l = b->length;
        o.joints[kIdx].p = b->tgt_pos;
      }
    }
    o.isHierarchical = false;
    return o;
  };

  vec<SkelState> out;
  //pSkeleton->setBasePosture();
  //const SkelState& base = asfSkel2state(*pSkeleton);
  //out.push_back(base);
  for (int i = 0; i < m_NumFrames; ++i) {
    const Posture& pi = *GetPosture(i);
    pSkeleton->setPosture(pi);
    const SkelState& o = asfSkel2state(*pSkeleton);
    out.push_back(o);
  }

  return out;
}

int AMCRecording::readAMCfile(const string& amcFile, float scale) {
  Bone* bone = pSkeleton->getRoot();

  ifstream file(amcFile, std::ios::in);
  if (file.fail()) {
    return -1;
  }

  int n = 0;
  char str[2048];

  // count the number of lines
  while (!file.eof()) {
    file.getline(str, 2048);
    if (file.eof()) break;
    //We do not want to count empty lines
    if (strcmp(str, "") != 0)
      n++;
  }

  file.close();

  //Compute number of frames. 
  //Subtract 3 to  ignore the header
  //There are (NUM_BONES_IN_ASF_FILE - 2) moving bones and 2 dummy bones (lhipjoint and rhipjoint)
  int numbones = pSkeleton->numBonesInSkel(bone[0]);
  int movbones = pSkeleton->movBonesInSkel(bone[0]);
  n = (n - 3) / ((movbones)+1);

  m_NumFrames = n;

  //Allocate memory for state vector
  m_pPostures = new Posture[m_NumFrames];

  //Set all postures to default posture
  SetPosturesToDefault();

  file.open(amcFile);

  // process the header
  while (1) {
    file >> str;
    if (strcmp(str, ":DEGREES") == 0) { break; }
  }

  for (int i = 0; i < m_NumFrames; i++) {
    //read frame number
    int frame_num;
    file >> frame_num;

    //There are (NUM_BONES_IN_ASF_FILE - 2) movable bones and 2 dummy bones (lhipjoint and rhipjoint)
    for (int j = 0; j < movbones; j++) {
      //read bone name
      file >> str;

      //fine the bone index corresponding to the bone name
      int bone_idx;
      for (bone_idx = 0; bone_idx < numbones; bone_idx++)
        if (strcmp(str, pSkeleton->idx2name(bone_idx)) == 0)
          break;

      //init rotation angles for this bone to (0, 0, 0)
      m_pPostures[i].bone_rotation[bone_idx].setZero();

      for (int x = 0; x < bone[bone_idx].dof; x++) {
        float tmp;
        file >> tmp;
        // printf("%d %f\n",bone[bone_idx].dofo[x],tmp);
        switch (bone[bone_idx].dofo[x]) {
        case 0:
          printf("FATAL ERROR in bone %d not found %d\n", bone_idx, x);
          x = bone[bone_idx].dof;
          break;
        case 1:
          m_pPostures[i].bone_rotation[bone_idx][0] = tmp;
          break;
        case 2:
          m_pPostures[i].bone_rotation[bone_idx][1] = tmp;
          break;
        case 3:
          m_pPostures[i].bone_rotation[bone_idx][2] = tmp;
          break;
        case 4:
          m_pPostures[i].bone_translation[bone_idx][0] = tmp * scale;
          break;
        case 5:
          m_pPostures[i].bone_translation[bone_idx][1] = tmp * scale;
          break;
        case 6:
          m_pPostures[i].bone_translation[bone_idx][2] = tmp * scale;
          break;
        case 7:
          m_pPostures[i].bone_length[bone_idx] = tmp;// * scale;
          break;
        }
      }
      if (strcmp(str, "root") == 0) {
        m_pPostures[i].root_pos[0] = m_pPostures[i].bone_translation[0][0];// * scale;
        m_pPostures[i].root_pos[1] = m_pPostures[i].bone_translation[0][1];// * scale;
        m_pPostures[i].root_pos[2] = m_pPostures[i].bone_translation[0][2];// * scale;
      }
      // read joint angles, including root orientation
    }
  }

  file.close();
  SG_LOG_INFO << "[AMCRecording] Read " << amcFile << ", numFrames: " << n;
  return n;
}

int AMCRecording::writeAMCfile(const string& file, float scale) const {
  Bone * bone = pSkeleton->getRoot();

  ofstream os(file);

  // header lines
  os << ":FULLY-SPECIFIED" << endl;
  os << ":DEGREES" << endl;

  int numbones = pSkeleton->numBonesInSkel(bone[0]);

  for (int f = 0; f < m_NumFrames; f++) {
    os << f + 1 << endl;
    os << "root "
      << m_pPostures[f].root_pos[0] / scale << " "
      << m_pPostures[f].root_pos[1] / scale << " "
      << m_pPostures[f].root_pos[2] / scale << " "
      << m_pPostures[f].bone_rotation[0][0] << " "
      << m_pPostures[f].bone_rotation[0][1] << " "
      << m_pPostures[f].bone_rotation[0][2];

    for (int j = 2; j < numbones; j++) {
      //output bone name
      if (bone[j].dof != 0) {
        os << endl << pSkeleton->idx2name(j);

        //output bone rotation angles
        for (int d = 0; d < bone[j].dof; d++) {
          // traverse all DOFs

          // is this DOF rx ?
          if (bone[j].dofo[d] == 1) {
            // if enabled, output the DOF
            if (bone[j].dofrx == 1)
              os << " " << m_pPostures[f].bone_rotation[j][0];
          }

          // is this DOF ry ?
          if (bone[j].dofo[d] == 2) {
            // if enabled, output the DOF
            if (bone[j].dofry == 1)
              os << " " << m_pPostures[f].bone_rotation[j][1];
          }

          // is this DOF rz ?
          if (bone[j].dofo[d] == 3) {
            // if enabled, output the DOF
            if (bone[j].dofrz == 1)
              os << " " << m_pPostures[f].bone_rotation[j][2];
          }
        }
      }
    }
    os << endl;
  }

  os.close();
  SG_LOG_INFO << "[AMCRecording] Wrote " << file << ", numFrames=" << m_NumFrames;
  return 0;
}

}  // namespace core
}  // namespace sg
