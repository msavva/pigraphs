#include "common.h"  // NOLINT

#include "core/SkeletonDatabase.h"

// ReSharper disable once CppUnusedIncludeDirective
#include <ext-boost/serialization.h>

#include "interaction/Interaction.h"
#include "interaction/InteractionSet.h"
#include "interaction/InteractionDatabase.h"
#include "core/MotionDatabase.h"
#include "core/Recording.h"
#include "core/RecordingDatabase.h"
#include "core/Skeleton.h"
#include "core/SkelState.h"
#include "math/math.h"

using sg::interaction::InteractionDatabase;

namespace sg {
namespace core {

BOOST_BINARY_SERIALIZABLE_IMPL(SkeletonDatabase)

bool loadJointWeights(const string& file, map<string, arr<double, Skeleton::kNumJoints + 1>>* out) {
  if (!io::fileExists(file)) { return false; }
  // read header
  const vec<vec<string>> lines = io::getTokenizedLines(file, ",");
  arr<int, Skeleton::kNumJoints> col2joint;
  const vec<string>& header = lines[0];
  for (int i = 2; i < header.size(); ++i) {
    const string jointName = header[i];
    auto iJoint = find(begin(Skeleton::kJointNames), end(Skeleton::kJointNames), jointName);
    col2joint[distance(begin(Skeleton::kJointNames), iJoint)] = i;
  }
  // read weights
  for (int iRow = 1; iRow < lines.size(); ++iRow) {
    const vec<string>& l = lines[iRow];
    const string verb = l[0];
    auto& verbWeights = (*out)[verb];
    for (int iJoint = 0; iJoint < Skeleton::kNumJoints; ++iJoint) {
      verbWeights[iJoint] = stof(l[col2joint[iJoint]]);
    }
    verbWeights[Skeleton::kNumJoints] = stof(l[1]);
  }
  return true;
}


SkeletonDatabase::SkeletonDatabase(const util::Params& params)
  : m_params(params) {
  const string file = params.get<string>("localDataDir") + "joint-logit-weights.csv";
  loadJointWeights(file, &m_logitJointWeightsPerVerb);
}

void SkeletonDatabase::init(const RecordingDatabase& recs, InteractionDatabase& interactionDb) {
  const string cacheFile = m_params.get<string>("dataCacheDir") + "skeletonsDb.cached";
  if (io::fileExists(cacheFile)) {
    SG_LOG_INFO << "Loading skeleton database from " << cacheFile;
    Serializable::load(cacheFile);
  } else {
    SG_LOG_INFO << "Creating skeleton database from recordings";
    loadFromRecordingsAndInteractions(recs, interactionDb);
    Serializable::save(cacheFile);
  }
  SG_LOG_INFO << "Finalizing skeleton database";
  finalizeLoad();
}

void SkeletonDatabase::init(const MotionDatabase& mdb, const vec<string>& keywords) {
  for (const string mId : mdb.getMotionIds()) {
    const MotionInfo& mi = mdb.getMotionInfo(mId);
    const Recording& rec = mdb.getMotion(mId);
    const string motionId = mi.motionId;
    const string subjectId = mi.subjectId;
    const size_t interactionIdx = m_interactionIds.size();
    const auto& skels = rec.skeletons;
    m_interactionIds.push_back(motionId);
    for (const Skeleton& skel : skels) {
      const size_t skelIdx = m_skeletons.size();
      m_skeletons.push_back(skel);
      m_interIdToSkelIndices[motionId].push_back(skelIdx);
      m_isetIdToInteractionIndex[subjectId].push_back(interactionIdx);
      m_isetIdToSkelIndices[subjectId].push_back(skelIdx);
    }
  }
}

void SkeletonDatabase::finalizeLoad() {
  SkelState::Data skelStateDataAll;
  for (const string& isetId : getInteractionSetIds()) {
    SkelState::Data skelStateData;
    for (const size_t iSkel : m_isetIdToSkelIndices.at(isetId)) {
      Skeleton& s = m_skeletons[iSkel];
      s.computeGaze();
      SkelState ss;
      skel2state(s, &ss);
      ss = makeHierarchical(ss);
      ss.flatten(&skelStateData);
      ss.flatten(&skelStateDataAll);
    }
    m_isetIdToSkelDists[isetId] = SkeletonDistribution(skelStateData);
  }
  m_isetIdToSkelDists[""] = SkeletonDistribution(skelStateDataAll);
  SG_LOG_INFO << "[SkeletonDatabase] Loaded " << m_skeletons.size() << " skeletons, "
    << m_isetIdToSkelDists.size() << " distributions";
}

void SkeletonDatabase::saveSkelFeats(const string& dir) const {
  SG_LOG_INFO << "Saving Skeleton features...";
  io::ensureDirExists(dir);

  ofstream ofs(dir + "/ALL.skels-feats.csv");
  string header = "recId,interactionId,isetId,timestamp";
  for (int i = 0; i < Skeleton::kNumJoints; ++i) {
    const string joint = Skeleton::kJointNames[i];
    header += "," + joint + "_lat," + joint + "_long," + joint + "_ang";
  }
  ofs << header << endl;

  map<string, int> recIdToInteractionCounter;
  for (const auto& pair : m_interIdToSkelIndices) {
    const string interactionStringId = pair.first;
    const vec<string> parts = ml::util::split(interactionStringId, '_');
    const string recId = util::join(vec<string>(parts.begin(), parts.begin() + 3), "_");
    const string isetId = util::join(vec<string>(parts.begin() + 4, parts.end()), "_");
    const int interactionId = recIdToInteractionCounter[recId]++;
    //const vec<string>& vns =  util::tokenize(isetId, "-");
    for (size_t iSkel : pair.second) {
      const Skeleton& s = m_skeletons[iSkel];
      SkelState ss;
      skel2state(s, &ss);
      ss = makeHierarchical(ss);
      ofs << recId << "," << interactionId << "," << isetId << "," << s.timestamp;
      for (int i=0; i<Skeleton::kNumJoints;++i) {
        const geo::Vec3f& q = ss.joints[i].q;
        ofs << "," << q[0] << "," << q[1] << "," << q[2];
      }
      ofs << endl;
    }
  }

  ofs.close();

  SG_LOG_INFO << "Done computing stats for numSkels=" << m_skeletons.size();
}

vec<string> SkeletonDatabase::getInteractionSetIds() const {
  vec<string> iSetIds;
  util::mapToKeyVec(m_isetIdToInteractionIndex, iSetIds);
  return iSetIds;
}

bool SkeletonDatabase::getRandomObservedSkeleton(const string& isetId,
                                                 Skeleton* pSkel, 
                                                 double* pScore /*= nullptr*/) const {
  if (m_skeletons.empty()) {
    SG_LOG_WARN << "No skeletons available";
    return false;
  }
  if (isetId.size()) {  // get Skeleton from iset
    if (m_isetIdToSkelIndices.count(isetId) > 0 && m_isetIdToSkelIndices.at(isetId).size() > 0) {
      const auto& iset = m_isetIdToSkelIndices.at(isetId);
      const size_t iSkel = *math::DEF_RAND.select(iset.begin(), iset.end());
      *pSkel = m_skeletons[iSkel];
      if (pScore != nullptr) {
        *pScore = predictInteractionSetLikelihood(isetId, *pSkel);
      }
    } else {
      SG_LOG_WARN << "No skeletons for isetId " << isetId;
      return false;
    }
  } else {  // choose from all skeletons
    *pSkel = *math::DEF_RAND.select(m_skeletons.begin(), m_skeletons.end());
  }
  return true;
}

bool SkeletonDatabase::getAverageSkeleton(const string& isetId,
                                          Skeleton* pSkel, 
                                          double* pScore /*= nullptr*/) const {
  if (m_isetIdToSkelDists.count(isetId) > 0) {
    const SkeletonDistribution& skelDist = m_isetIdToSkelDists.at(isetId);
    const SkelState ssMean = skelDist.mean();
    state2skel(ssMean, pSkel);
    if (pScore != nullptr) {
      SkelState ssh = makeHierarchical(ssMean);
      *pScore = skelDist.logprob(ssh);
    }
    return true;
  } else {
    SG_LOG_WARN << "Cannot get average skeleton for unknown isetId " << isetId;
    return false;
  }
}

bool SkeletonDatabase::sampleSkeleton(const string& isetId,
                                      Skeleton* pSkel, 
                                      double* pScore /*= nullptr*/) const {
  if (m_isetIdToSkelDists.count(isetId) > 0) {
    const SkeletonDistribution& skelDist = m_isetIdToSkelDists.at(isetId);
    const pair<SkelState, double> ss = skelDist.sampleMax(10000);
    state2skel(ss.first, pSkel);
    if (pScore != nullptr) {
      *pScore = ss.second;
    }
    SG_LOG_INFO << "sampled skeleton logprob: " << ss.second;
    return true;
  } else {
    SG_LOG_WARN << "Cannot sample skeleton for unknown isetId " << isetId;
    return false;
  }
}

size_t SkeletonDatabase::sampleSkeletons(const string& isetId, 
                                         size_t k, 
                                         vec<pair<Skeleton, double>>* pScoredSkeletons) const {
  if (m_isetIdToSkelDists.count(isetId) > 0) {
    const SkeletonDistribution& skelDist = m_isetIdToSkelDists.at(isetId);
    Skeleton skel; 
    for (size_t i = 0; i < k; ++i) {
      const pair<SkelState, double> ss = skelDist.sampleMax(10000);
      state2skel(ss.first, &skel);
      pScoredSkeletons->emplace_back(skel, ss.second);
    }
    return k;
  } else {
    SG_LOG_WARN << "Cannot sample skeleton for unknown isetId " << isetId;
    return 0;
  }
}

size_t SkeletonDatabase::getFirstKObservedSkeletons(const string& isetId, 
                                                    size_t k, 
                                                    vec<pair<Skeleton, double>>* pScoredSkeletons) const {
  if (m_skeletons.empty()) {
    SG_LOG_WARN << "No skeletons available";
    return 0;
  }
  if (isetId.size()) {  // get first k from iset
    if (m_isetIdToSkelIndices.count(isetId) > 0 && m_isetIdToSkelIndices.at(isetId).size() > 0) {
      const auto& iset = m_isetIdToSkelIndices.at(isetId);
      size_t actualK = std::min(iset.size(), k);
      for (size_t i = 0; i < actualK; i++) {
        const Skeleton& skel = m_skeletons[iset[i]];
        double score = predictInteractionSetLikelihood(isetId, skel);
        pScoredSkeletons->emplace_back(skel, score);
      }
      return actualK;
    } else {
      SG_LOG_WARN << "No skeletons for isetId " << isetId;
      return 0;
    }
  } else {  // get first k from all skeletons
    size_t actualK = std::min(m_skeletons.size(), k);
    for (size_t i = 0; i < actualK; i++) {
      const Skeleton& skel = m_skeletons[i];
      double score = predictInteractionSetLikelihood(isetId, skel);
      pScoredSkeletons->emplace_back(skel, score);
    }
    return actualK;
  }
}

size_t SkeletonDatabase::getRandomObservedSkeletons(const string& isetId,
                                                    size_t k,
                                                    vec<pair<Skeleton, double>>* pScoredSkeletons) const {
  if (m_skeletons.empty()) {
    SG_LOG_WARN << "No skeletons available";
    return 0;
  }
  if (isetId.size()) {  // get first k from iset
    if (m_isetIdToSkelIndices.count(isetId) > 0 && m_isetIdToSkelIndices.at(isetId).size() > 0) {
      const auto& iset = m_isetIdToSkelIndices.at(isetId);
      vec<size_t> indices;
      math::DEF_RAND.sampleNumbers(k, iset.size(), &indices);
      for (size_t i = 0; i < indices.size(); i++) {
        const Skeleton& skel = m_skeletons[iset[indices[i]]];
        double score = predictInteractionSetLikelihood(isetId, skel);
        pScoredSkeletons->emplace_back(skel, score);
      }
      return indices.size();
    } else {
      SG_LOG_WARN << "No skeletons for isetId " << isetId;
      return 0;
    }
  } else {  // get first k from all skeletons
    vec<size_t> indices;
    math::DEF_RAND.sampleNumbers(k, m_skeletons.size(), &indices);
    for (size_t i = 0; i < indices.size(); i++) {
      const Skeleton& skel = m_skeletons[indices[i]];
      double score = predictInteractionSetLikelihood(isetId, skel);
      pScoredSkeletons->emplace_back(skel, score);
    }
    return indices.size();
  }
}

vec<pair<string, double>> SkeletonDatabase::predictInteractionSetLikelihoods(const Skeleton& s) const {
  typedef pair<string, double> ScoredISet;
  vec<ScoredISet> isets;
  SkelState ss;
  skel2state(s, &ss);
  ss = makeHierarchical(ss);

  for (const auto& iset : m_isetIdToSkelDists) {
    //double logitp = 0;
    // get all active verbs
    const string isetId = iset.first;
    if (isetId.empty() || !ml::util::startsWith(isetId, "v/")) { continue; }
    const auto& vns = util::tokenize(isetId.substr(2), "-");
    vec<string> vs;
    for (const string& vn : vns) {
      const string v = util::tokenize(vn, "_")[0];
      vs.push_back(v);
      //if (m_logitJointWeightsPerVerb.count(v) == 0) { continue; }
      //const JointLogitWeights& w = getJointWeights(v);
      //logitp += w[Skeleton::kNumJoints];  // intercept
      //for (int i = 0; i < Skeleton::kNumJoints; ++i) {
      //  logitp += w[i] * ss.joints[i].q.z();
      //}
    }
    //isets.push_back({isetId, logitp});

    // score using weights for first verb
    const string v = vs[0];
    if (m_logitJointWeightsPerVerb.count(v) == 0) { continue; }
    const JointLogitWeights& w = getJointWeights(v);
    double logitp = w[Skeleton::kNumJoints];  // intercept
    for (int i = 0; i < Skeleton::kNumJoints; ++i) {
      logitp += w[i] * ss.joints[i].q.z();
    }
    isets.push_back({v, logitp});
  }
  sort(isets.begin(), isets.end(), [] (const ScoredISet& a, const ScoredISet& b) {
    return a.second < b.second;
  });
  return isets;
}

double SkeletonDatabase::predictInteractionSetLikelihood(const string& isetId, const Skeleton& s) const {
  if (m_isetIdToSkelDists.count(isetId) > 0) {
    const SkeletonDistribution& skelDist = m_isetIdToSkelDists.at(isetId);
    SkelState ss;
    skel2state(s, &ss);
    ss = makeHierarchical(ss);
    return skelDist.logprob(ss);
  } else {
    SG_LOG_WARN << "Cannot get iset likelihood for unknown isetId " << isetId;
    return math::constants::NEGINF;
  }
}

double SkeletonDatabase::predictInteractionSetLikelihood(const string& isetId, const SkelState& ss) const {
  if (m_isetIdToSkelDists.count(isetId) > 0) {
    const SkeletonDistribution& skelDist = m_isetIdToSkelDists.at(isetId);
    SkelState ssh = makeHierarchical(ss);
    return skelDist.logprob(ssh);
  } else {
    SG_LOG_WARN << "Cannot get iset likelihood for unknown isetId " << isetId;
    return math::constants::NEGINF;
  }
}

arr<float, Skeleton::kNumJoints> SkeletonDatabase::getNormalizedJointWeights(const string& verb) const {
  arr<float, Skeleton::kNumJoints> out;
  const auto& logOdds = getJointWeights(verb);
  const auto minmax = minmax_element(logOdds.begin(), logOdds.begin() + Skeleton::kNumJoints);
  const double norm = 1.0 / (*minmax.second - *minmax.first);
  for (int i = 0; i < Skeleton::kNumJoints; ++i) {
    out[i] = static_cast<float>((logOdds[i] - *minmax.first) * norm);
  }
  // copy wrist to hand joints (since we fixed these to zero)
  out[Skeleton::JointType_HandLeft] = out[Skeleton::JointType_WristLeft];
  out[Skeleton::JointType_HandTipLeft] = out[Skeleton::JointType_WristLeft];
  out[Skeleton::JointType_ThumbLeft] = out[Skeleton::JointType_WristLeft];
  out[Skeleton::JointType_HandRight] = out[Skeleton::JointType_WristRight];
  out[Skeleton::JointType_HandTipRight] = out[Skeleton::JointType_WristRight];
  out[Skeleton::JointType_ThumbRight] = out[Skeleton::JointType_WristRight];
  return out;
}

void SkeletonDatabase::loadFromRecordingsAndInteractions(const RecordingDatabase& recs,
                                                         InteractionDatabase& interactionDb) {
  for (const string recId : recs.getRecordingIds()) {
    const Recording& rec = recs.getRecording(recId, true, false);
    for (interaction::Interaction* pInter : rec.interactions) {
      interactionDb.registerAndPopulateISets(pInter);
      const string interactionId = pInter->id;
      const size_t interactionIdx = m_interactionIds.size();
      const auto& skels = pInter->skelRange;
      m_interactionIds.push_back(interactionId);
      for (const Skeleton& skel : skels) {
        const size_t skelIdx = m_skeletons.size();
        m_skeletons.push_back(skel);
        m_interIdToSkelIndices[interactionId].push_back(skelIdx);
        for (const interaction::InteractionSet* pISet : pInter->interactionSets) {
          const string isetId = pISet->id;
          m_isetIdToInteractionIndex[isetId].push_back(interactionIdx);
          m_isetIdToSkelIndices[isetId].push_back(skelIdx);
        }
      }
      size_t nISets = pInter->interactionSets.size();
      SG_LOG_INFO << recId << "," << pInter->isetId << "(" << nISets << "),"
        << interactionId << "nSkels=" << skels.size();
    }
  }
}

}  // namespace core
}  // namespace sg
