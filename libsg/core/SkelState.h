#pragma once

#include <mLibCore.h>

#include "libsg.h"  // NOLINT

#include "core/Skeleton.h"
#include "geo/geo.h"

namespace sg {
namespace core {

//! Skeletal joint coords encapsulates position, orientation, and bone length
struct JointCoords {
  geo::Vec3f p;      // position of child joint (x, y, z)
  geo::Vec3f q;      // orientation (latitude, longitude, rotation angle)
  float l;           // child bone length

  //! flattened JointCoords data (7 floats per joint: px, py, pz, qlat, qlon, qang, l)
  typedef arr<vec<float>, 7> Data;
  void flatten(Data* out) const;
};
ostream& operator<<(ostream& os, const JointCoords& jc);

//! State representation of Skeleton based on JCS (Joint Coordinate System)
struct SkelState {
  static const int kNumJoints = 25;                   // == Skeleton::kNumJoints
  static const arr<string, kNumJoints> kJointNames;   // joint names
  static const Skeleton::BoneDirs kBoneDirs;          // default directions of bones in joint coord frame

  arr<JointCoords, kNumJoints> joints;                // joint coordinate frames
  JointCoords bodyFrame;                              // global body frame
  float spineAngle;                                   // spine "uprightness" = angle with horizontal
  bool isHierarchical;                                // is the joints represented hierarchically
  geo::AngleEncoding angleEncoding;                   // encoding used for joint coord angles
  std::shared_ptr<Skeleton::BoneDirs> boneDirs;       // current bone dirs

  //! flatten into 26x7=182 floats (seq of 25 x joint + body joint + dummy join with spineAngle at end)
  typedef arr<JointCoords::Data, kNumJoints + 2> Data;
  void flatten(Data* out) const;

  const Skeleton::BoneDirs& getBoneDirs() const { return boneDirs ? *boneDirs : kBoneDirs; }

  static string getCSVHeader();
  ostream& toCSV(ostream& os, const string& prefix) const;
  ostream& toJSON(ostream& os, bool endlines) const;
};
inline ostream& operator<<(ostream& os, const SkelState& ss) {
  return ss.toJSON(os, false);
}

//! Converts skeleton to reduced SkelState representation
void skel2state(const Skeleton& skel, SkelState* pState);

//! Convert SkelState to Skeleton
void state2skel(const SkelState& state, Skeleton* pSkel);

//! Returns version of state with hierarchically encoded joint positions and orientations
SkelState makeHierarchical(const SkelState& state);

//! Returns version of state with absolute joint positions and orientations
//! VIP: Assumes state joint parameters are specified hierarchically
SkelState makeAbsolute(const SkelState& state);

//! Return transformation matrices of joint coordinate frames
umap<string, geo::Transform> jointCoordFrames(const SkelState& state);

}  // namespace core
}  // namespace sg


