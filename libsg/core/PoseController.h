#pragma once

#include "libsg.h"  // NOLINT

#include "interaction/InteractionDatabase.h"

namespace sg {
namespace core {

const int kNumControlJoints = 14;
const arr<Skeleton::JointType, kNumControlJoints> kControlJoints = {{
  Skeleton::JointType_SpineMid,

  Skeleton::JointType_HipLeft,
  Skeleton::JointType_HipRight,
  Skeleton::JointType_ShoulderLeft,
  Skeleton::JointType_ShoulderRight,

  Skeleton::JointType_Head,
  Skeleton::JointType_KneeLeft,
  Skeleton::JointType_KneeRight,
  Skeleton::JointType_ElbowLeft,
  Skeleton::JointType_ElbowRight,

  Skeleton::JointType_AnkleLeft,
  Skeleton::JointType_AnkleRight,
  Skeleton::JointType_WristLeft,
  Skeleton::JointType_WristRight,
}};

//! Exploring skeleton poses by interactive manipulation
class PoseController {
 public:
  explicit PoseController(const Scan* pScan = nullptr);
  virtual ~PoseController() { }

  bool update(int key);  //! handle key event and return whether update made

  //! Increments or decrements current x, y, theta, and pose. Optionally unfloat skeleton after repositioning
  void movePose(float deltaX, float deltaY, float deltaTheta, float deltaAng,
                int deltaSkel, int deltaJoint, int deltaJointDim, bool unfloat = false);

  //! Sets pose at absolute position (x,y) with angle theta in radians, and using skeleton with index iPose
  //! Optionally unfloat skeleton after repositioning
  void setPose(float x, float y, float theta, int iPose, bool unfloat = false);

  //! Returns current Skeleton
  const Skeleton& currentSkeleton() const { return m_currXformedSkel.getSkeleton(); }
  //! Returns current Skeleton
  const TransformedSkeleton& currentTransformedSkeleton() const { return m_currXformedSkel; }

  //! Returns current pose index
  int currentPoseIndex() const { return m_currPoseParams.iPose; }

  virtual void setBaseSkeleton(const Skeleton& s) {
    m_baseSkel = s;
    updateSkeleton();
  }

  void setAutoupdateSkeleton(bool isAuto) { m_autoupdateSkel = isAuto; }

 protected:
  virtual void setBaseSkeleton() { }
  void updateSkeleton();  //! Updates current Skeleton using pose params
  void unfloatSkeleton();  //! Unfloat skeleton to nearby support surface in scene

  //! Encapsulates parameters of a posed skeleton
  struct PoseParams {
    float theta; int iPose; float x, y;
    int iCtrlJoint; int iCtrlDim;
    arr<geo::Vec3f, Skeleton::kNumJoints> jointAngles;
  };
  PoseParams m_currPoseParams;
  Skeleton m_baseSkel;
  TransformedSkeleton m_currXformedSkel;
  const Scan* m_pScan;
  int m_numPoses = 1;  // how many poses we can iterate over
  bool m_autoupdateSkel = true;
};

//! Exploring skeleton poses by interactive manipulation
class ISetPoseController : public PoseController {
 public:
  explicit ISetPoseController(const interaction::InteractionDatabase& db,
                              const Scan* pScan = nullptr);

  const interaction::InteractionSet& currentInteractionSet() const {
    return m_interactionSetIt->second;
  }

  const string& currentInteractionSetType() const {
    return interaction::InteractionSet::kInteractionSetNames.at(m_interactionSetType);
  }

  //! Goes to the next interaction set
  void nextInteractionSet();

  //! Set the current interaction set
  bool setInteractionSet(const string& type, const string& id);

  //! Goes to the next pose
  void nextPose();

  using PoseController::setBaseSkeleton;

 protected:
  void setBaseSkeleton() override;

  const interaction::InteractionDatabase& m_interactions;
  interaction::InteractionSetType m_interactionSetType;
  interaction::InteractionSetMap::const_iterator m_interactionSetIt;
};

}  // namespace core
}  // namespace sg


