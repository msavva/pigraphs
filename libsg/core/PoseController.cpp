#include "common.h"  // NOLINT

#include "core/PoseController.h"

#include "core/Database.h"
#include "core/SkelState.h"

namespace sg {
namespace core {

PoseController::PoseController(const Scan* pScan) : m_pScan(pScan) {
  const int iCtrlJoint = 0;
  const int iCtrlDim = 2;
  if (pScan) {
    const auto& bbox = pScan->bbox;
    m_currPoseParams = {0, 0, bbox.getMinX() + 0.5f * bbox.getExtentX(),
                              bbox.getMinY() + 0.5f * bbox.getExtentY(),
                        iCtrlJoint, iCtrlDim};
  } else {
    m_currPoseParams = {0, 0, 0, 0, iCtrlJoint, iCtrlDim};
  }
}

void PoseController::movePose(float deltaX, float deltaY, float deltaTheta,
                              float deltaAng, int deltaSkel, int deltaJoint,
                              int deltaJointDim, bool unfloat /*=false*/) {
  auto& p = m_currPoseParams;
  p.x += deltaX;
  p.y += deltaY;
  p.theta = math::deg2rad(fmodf(math::rad2deg(p.theta) + deltaTheta, 360.0f));
  p.iCtrlJoint = ml::math::mod(p.iCtrlJoint + deltaJoint, kNumControlJoints);
  p.iCtrlDim = ml::math::mod(p.iCtrlDim + deltaJointDim, 3);
  p.theta = math::deg2rad(fmodf(math::rad2deg(p.theta) + deltaTheta, 360.0f));
  float& ang = p.jointAngles[kControlJoints[p.iCtrlJoint]][p.iCtrlDim];
  ang = fmodf(ang + deltaAng, math::constants::PI2f);
  p.iPose = ml::math::mod(p.iPose + deltaSkel, m_numPoses);
  updateSkeleton();
  if (unfloat) { unfloatSkeleton(); }
  SG_LOG_INFO << "SCSCORE:" << 1.0 - m_currXformedSkel.getSkeleton().getSelfCollisionRatio();
}

void PoseController::setPose(float x, float y, float theta, int iPose, bool unfloat /* = false */) {
  m_currPoseParams = {theta, iPose, x, y};
  updateSkeleton();
  if (unfloat) { unfloatSkeleton(); }
}

void PoseController::updateSkeleton() {
  if (m_autoupdateSkel) { setBaseSkeleton(); }
  auto& p = m_currPoseParams;
  SkelState ss;
  skel2state(m_baseSkel, &ss);
  ss = makeHierarchical(ss);
  for (int i = 0; i < Skeleton::kNumJoints; ++i) {
    ss.joints[i].q += p.jointAngles[i];
  }
  const int currJoint = kControlJoints[p.iCtrlJoint];
  SG_LOG_INFO << Skeleton::kJointNames[currJoint] << "\t"
    << ss.joints[currJoint].q.format(geo::kEigenJSONFormat);
  ss = makeAbsolute(ss);
  std::shared_ptr<Skeleton> pS = std::make_shared<Skeleton>();
  state2skel(ss, pS.get());
  const auto xform = ml::mat4f::translation(ml::vec3f(p.x, p.y, 0.0f))
                   * ml::mat4f::rotationZ(math::rad2deg(p.theta));
  m_currXformedSkel = TransformedSkeleton(pS, xform);
}

void PoseController::unfloatSkeleton() {
  if (!m_pScan) { return; }
  Skeleton skel = m_currXformedSkel.getSkeleton();
  const float zOffset = skel.unfloat(*m_pScan);
  const auto xform = ml::mat4f::translation(ml::vec3f(0, 0, zOffset));
  m_currXformedSkel.setTransform(xform * m_currXformedSkel.transform());
}

bool PoseController::update(int key) {
  int deltaX = 0, deltaY = 0, deltaTheta = 0, deltaAng = 0,
    deltaPose = 0, deltaJoint = 0, deltaJointDim = 0;
  switch (key) {
    // case KEY_N:     nextSkeleton();       break;
    // case KEY_I:     nextInteractionSet(); break;
    case KEY_RIGHT: deltaX = 1;           break;
    case KEY_LEFT:  deltaX = -1;          break;
    case KEY_UP:    deltaY = 1;           break;
    case KEY_DOWN:  deltaY = -1;          break;
    case KEY_PGUP:  deltaTheta = 1;       break;
    case KEY_PGDN:  deltaTheta = -1;      break;
    case KEY_HOME:  deltaPose = 1;        break;
    case KEY_END:   deltaPose = -1;       break;
    case KEY_1:     deltaJoint = -1;      break;
    case KEY_2:     deltaAng = 1;         break;
    case KEY_3:     deltaAng = -1;        break;
    case KEY_4:     deltaJoint = 1;       break;
    default: return false;  // Bail out since no updates
  }

  // Update pose placement and return
  movePose(deltaX * 0.01f, deltaY * 0.01f, deltaTheta * 5.f,
           deltaAng * 0.1f, deltaPose, deltaJoint, deltaJointDim, true);
  return true;
}


ISetPoseController::ISetPoseController(const interaction::InteractionDatabase& idb,
                                       const Scan* pScan)
  : PoseController(pScan)
  , m_interactions(idb)
  , m_interactionSetType()
  , m_interactionSetIt(idb.getInteractionSets().begin()) {
  m_numPoses = static_cast<int>(m_interactionSetIt->second.get().sampledSkeletons.size());
  m_interactionSetType = m_interactionSetIt->second.get().type;
  updateSkeleton();
}

void ISetPoseController::setBaseSkeleton() {
  m_baseSkel = currentInteractionSet().sampledSkeletons[m_currPoseParams.iPose].getSkeleton();
}

void ISetPoseController::nextInteractionSet() {
  ++m_interactionSetIt;
  const auto& interactionSets = m_interactions.getInteractionSets(m_interactionSetType);
  if (m_interactionSetIt == interactionSets.end()) {
    m_interactionSetIt = interactionSets.begin();
  }
  m_interactionSetType = m_interactionSetIt->second.get().type;
  m_currPoseParams.iPose = 0;
  m_numPoses = static_cast<int>(m_interactionSetIt->second.get().sampledSkeletons.size());
  updateSkeleton();
}

bool ISetPoseController::setInteractionSet(const string& type, const string& id) {
  using interaction::InteractionSet;
  using interaction::InteractionSetType;
  // Check the type...
  if (InteractionSet::kInteractionSetTypeLookupByName.count(type) == 0) {
    SG_LOG_ERROR << "Unknown interaction set type: " << type;
    return false;
  }
  const InteractionSetType iType = InteractionSet::kInteractionSetTypeLookupByName.at(type);
  const auto& interactionSets = m_interactions.getInteractionSets(iType);
  for (auto it = interactionSets.begin(); it != interactionSets.end(); ++it) {
    if (it->first == id) {
      m_interactionSetType = iType;
      m_interactionSetIt = it;
      m_currPoseParams.iPose = 0;
      m_numPoses = static_cast<int>(it->second.get().sampledSkeletons.size());
      updateSkeleton();
      return true;
    }
  }
  SG_LOG_WARN << "Cannot find pose for interaction set type: " << id;
  return false;
}

void ISetPoseController::nextPose() {
  movePose(0, 0, 0, 0, 1, 0, 0);
}

}  // namespace core
}  // namespace sg
