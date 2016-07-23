#include "common.h"  // NOLINT

#include "core/Scene.h"

#include "core/ModelDatabase.h"
#include "core/SceneSerialized.h"
#include "core/SkelState.h"

namespace sg {
namespace core {

Scene::Scene(const SceneSerialized& ss, ModelDatabase* pModelDb) {
  load(ss, pModelDb);
}

Scene::Scene(const string& file, ModelDatabase* pModelDb) : Scene(SceneSerialized(file), pModelDb) { }

bool Scene::serialize(SceneSerialized* pSs) const {
  // check and clear
  if (pSs == nullptr) {
    return false;
  }
  pSs->models.clear();
  pSs->cameras.clear();
  pSs->skeletons.clear();

  // header elements
  pSs->id = m_id;
  pSs->name = m_name;
  pSs->up = geo::to<ml::vec3f>(m_up);
  pSs->front = geo::to<ml::vec3f>(m_front);
  pSs->unit = m_unit;
  pSs->score = m_score;

  // translate model entries
  for (int iModel = 0; iModel < static_cast<int>(m_models.size()); ++iModel) {
    const ModelInstance& mInst = m_models[iModel];
    int parentIndex = -1;
    if (mInst.pParent) {  // get parent index
      const auto mInstIt = find(m_models.begin(), m_models.end(), *mInst.pParent);
      if (mInstIt != m_models.end()) {
        parentIndex = static_cast<int>(mInstIt - m_models.begin());
      } else {  // parent not found - this shouldn't happen!
        SG_LOG_ERROR << "[Scene] Parent model not found!";
        return false;
      }
    }
    // TODO(MS): This is currently hacked to map no source prefix -> wss.
    pSs->models.push_back({"wss." + mInst.model.id, iModel, parentIndex,
                          geo::to<ml::mat4f>(mInst.getLocalTransform()) * mInst.model.modelTransform});
  }

  // translate skeleton entries
  for (int iSkel = 0; iSkel < static_cast<int>(m_skels.size()); ++iSkel) {
    const Skeleton& s = m_skels[iSkel];
    SkelState ss;
    skel2state(s, &ss);
    const string id = "skel_" + to_string(s.trackingId) + "_" + to_string(s.timestamp);
    SceneSerialized::SkelEntry se;
    se.id = id;
    se.worldPosition = ml::vec3f(ss.bodyFrame.p.x(), ss.bodyFrame.p.y(), ss.bodyFrame.p.z());
    const geo::Quatf qb = fromAngleEncoding(ss.bodyFrame.q, ss.angleEncoding);
    se.worldOrientation = ml::vec4f(qb.x(), qb.y(), qb.z(), qb.w());
    se.jointPositionsKinect = {s.jointPositions, s.jointPositions + Skeleton::kNumJoints};
    se.jointOrientationsKinect = {s.jointOrientations, s.jointOrientations + Skeleton::kNumJoints};
    se.jointConfidencesKinect = {s.jointConfidences, s.jointConfidences + Skeleton::kNumJoints};
    se.bonePositions.resize(SkelState::kNumJoints);
    se.boneOrientations.resize(SkelState::kNumJoints);
    se.boneLengths.resize(SkelState::kNumJoints);
    for (int iJoint = 0; iJoint < SkelState::kNumJoints; ++iJoint) {
      const auto& j = ss.joints[iJoint];
      se.bonePositions[iJoint] = geo::to<ml::vec3f>(j.p);
      const geo::Quatf q = fromAngleEncoding(j.q, ss.angleEncoding);
      se.boneOrientations[iJoint] = ml::vec4f(q.x(), q.y(), q.z(), q.w());
      se.boneLengths[iJoint] = j.l;
    }
    pSs->skeletons.push_back(se);
  }

  // cameras would go here

  return true;
}

bool Scene::serialize(const string& file) const {
  SceneSerialized ss;
  serialize(&ss);
  return ss.save(file);
}

bool Scene::load(const SceneSerialized& ss, ModelDatabase* pModelDb) {
  clear();

  // header elements
  m_id = ss.id;
  m_name = ss.name;
  m_up = geo::vec3f(ss.up);
  m_front = geo::vec3f(ss.front);
  m_unit = 1.f;  // we'll deal with re-scaling down below
  geo::Transform xformScale;
  xformScale.setIdentity();
  xformScale.linear() *= ss.unit;

  // instantiate models
  for (const auto& mEntry : ss.models) {
    // TODO(MS): This is currently hacked to map wss. -> no source prefix
    const string
      defSrc = "wss.",
      fullId = mEntry.id,
      id = (!fullId.compare(0, defSrc.size(), defSrc)) ? fullId.substr(defSrc.size()) : fullId;
    if (pModelDb->modelExists(id)) {
      const Model& model = pModelDb->getModel(id).get();
      const int iModel = static_cast<int>(m_models.size());
      assert(iModel == mEntry.index);  // assume index is ordered and continuous
      const auto xform = xformScale * geo::from<ml::mat4f>(mEntry.transform * model.modelTransform.getInverse());
      m_models.emplace_back(model, xform, ml::ColorUtils::colorById<ml::vec4f>(iModel));
    } else {
      SG_LOG_WARN << "Ignoring model with unrecognized id: " << id;
    }
  }

  assert(m_models.size() == ss.models.size());

  // hook up parent and children
  for (int iModel = 0; iModel < m_models.size(); ++iModel) {
    const auto& mEntry = ss.models[iModel];
    ModelInstance& mInst = m_models[iModel];
    if (mEntry.parentIndex < 0) { continue; }  // no parent so skip
    ModelInstance& parent = m_models[mEntry.parentIndex];
    parent.add(&mInst);
  }

  // instantiate skeletons
  for (const auto& sEntry : ss.skeletons) {
    Skeleton s;
    for (int iJoint = 0; iJoint < Skeleton::kNumJoints; ++iJoint) {
      s.jointPositions[iJoint]    = sEntry.jointPositionsKinect[iJoint];
      s.jointOrientations[iJoint] = sEntry.jointOrientationsKinect[iJoint];
      s.jointConfidences[iJoint]  = sEntry.jointConfidencesKinect[iJoint];
    }
    s.isKinectSkel = true;
    s.computeGaze();
    m_skels.push_back(s);
  }

  return true;
}

bool Scene::load(const string& file, ModelDatabase* pModelDb) {
  SceneSerialized ss;
  ss.load(file);
  return load(ss, pModelDb);
}

}  // namespace core
}  // namespace sg

