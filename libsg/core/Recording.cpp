#include "common.h"  // NOLINT

#include "core/Recording.h"

#include <ext-boost/serialization.h>

#include "core/SkelState.h"
#include "core/Scan.h"
#include "geo/geo.h"
#include "interaction/Interaction.h"
#include "io/json.h"
#include "math/OneEuroFilter.h"
#include "util/Params.h"

using sg::interaction::Interaction;

namespace sg {
namespace core {

const float Recording::kTimestampToSec = 1.0e-7f;

void Recording::finalizeLoad(const string& file) {
  // Transform skeleton joints and orientations to recording world space
  applyTransform(camera);
  for (Skeleton& s : skeletons) {
    s.rec = this;
    s.isKinectSkel = true;
  }

  id = io::basename(file);
  isLoaded = true;

  SG_LOG_INFO << "[Recording: " << ml::util::fileNameFromPath(file)
       << ", nSkels: " << skeletons.size() << ", time: " << durationInSec() << "s]";
}

void Recording::applyRotation(const ml::mat4f& xform) {
  const geo::Quatf Rq(geo::from(xform).rotation());
  for (Skeleton& s : skeletons) {
    for (int i = 0; i < Skeleton::kNumJoints; i++) {
      s.jointPositions[i] = geo::to<ml::vec3f>(Rq * geo::vec3f(s.jointPositions[i]));
      const ml::vec4f& v = s.jointOrientations[i];
      const geo::Quatf q = Rq * geo::Quatf(v.w, v.x, v.y, v.z);
      s.jointOrientations[i].x = q.x();
      s.jointOrientations[i].y = q.y();
      s.jointOrientations[i].z = q.z();
      s.jointOrientations[i].w = q.w();
    }
    s.computeGaze();
  }
}

void Recording::applyTransform(const ml::mat4f& xform) {
  const geo::Quatf Rq(geo::from(xform).rotation());
  for (Skeleton& s : skeletons) {
    for (int i = 0; i < Skeleton::kNumJoints; i++) {
      s.jointPositions[i] = xform * s.jointPositions[i];
      const ml::vec4f& v = s.jointOrientations[i];
      const geo::Quatf q = Rq * geo::Quatf(v.w, v.x, v.y, v.z);
      s.jointOrientations[i].x = q.x();
      s.jointOrientations[i].y = q.y();
      s.jointOrientations[i].z = q.z();
      s.jointOrientations[i].w = q.w();
    }
    s.computeGaze();
  }
}

bool Recording::loadFromArchive(const string& file, const ml::mat4f* cameraExtrinsics) {
  if (isLoaded || !ml::util::fileExists(file)) { return false; }

  ifstream ifs(file, std::ios::binary);
  { boost::archive::binary_iarchive(ifs) >> *this; }
  ifs.close();
  if (cameraExtrinsics) { camera = *cameraExtrinsics; }
  finalizeLoad(file);

  return true;
}

bool Recording::computeActiveSegments(const util::Params& params, const Scan& scene,
                                      bool forceRecompute /* = false */) {
  float maxDistToSegment = params.get<float>("Interaction.maxDistToSegment");
  float maxDistGaze = params.get<float>("Interaction.maxDistGaze");
  return computeActiveSegments(params, scene, maxDistToSegment, maxDistGaze, forceRecompute);
}

bool Recording::computeActiveSegments(const util::Params& params, const Scan& scene,
                                      float maxDistToSegment, float maxDistGaze,
                                      bool forceRecompute /* = false */) {
  if (hasActiveSegments && !forceRecompute) {
    return true;
  }

  cout << "[Recording] Computing actSegs: " << id << "...";
  const int kNearestSegsPerJoint = params.get<int>("Interaction.kNearestSegsPerJoint");
  const bool ignoreInferredJoints = params.get<bool>("Interaction.ignoreInferredJoints");
  const float maxSegmentSizeRatio = params.get<float>("Interaction.maxSegmentSizeRatio");

  for (Interaction* in : interactions) {
    const int numSkels = static_cast<int>(in->skelRange.size());
    in->jointSegments.resize(numSkels);
    in->activeSegments.clear();

#pragma omp parallel for
    for (int iSkel = 0; iSkel < numSkels; iSkel++) {
      const Skeleton& skel = in->skelRange[iSkel];
      scene.getActiveSegments(skel, ignoreInferredJoints, kNearestSegsPerJoint,
                              maxDistToSegment, maxDistGaze, maxSegmentSizeRatio,
                              &in->jointSegments[iSkel]);
    }

    // Also accumulate all segs in activeSegments
    for (int iSkel = 0; iSkel < numSkels; iSkel++) {
      const auto& segs = in->jointSegments[iSkel];
      for (const auto& jointSegs : segs) {
        in->activeSegments.insert(jointSegs.begin(), jointSegs.end());
      }
    }
  }

  cout << "done." << endl;
  hasActiveSegments = true;
  return true;
}

string Recording::interactionsFilename() const {
  return m_interactionsFilename;
}

bool Recording::loadInteractions(const util::Params& params) {  
  const string file = params.get<string>("dataDir") + params.get<string>("recDir") + id + ".interactions.txt";
  if (!ml::util::fileExists(file)) { return false; }

  // Initialize interactions filename
  m_interactionsFilename = file;

  cout << "[+interactions]";

  // Clear existing interactions if there are any
  for (Interaction* in : interactions) { if (in) delete in; }
  interactions.clear();

  const size_t skeletonSkip = params.get<size_t>("Interaction.skeletonSkip");
  for (const string line : ml::util::getFileLines(file)) {
    const vec<string> parts = ml::util::split(line, ',');
    if (parts.size() > 1) {
      Interaction& in = *(new Interaction());
      in.recording = this;
      in.recId = id;
      in.scanId = baseScanId();
      const vec<string> timeStrPair = ml::util::split(parts[0], '-');
      assert(timeStrPair.size() == 2);
      in.startTime = stof(timeStrPair[0]);
      in.endTime   = stof(timeStrPair[1]);
      for (const string& verbNounStr : ml::util::split(parts[1], '|')) {
        const vec<string> verbNounPair = ml::util::split(verbNounStr, ':');
        in.verbNouns.insert({verbNounPair[0], verbNounPair[1]});
      }
      in.isetId = interaction::VerbNoun::getVerbNounSetId(in.verbNouns);
      in.id = id + "_" + parts[0] + "_" + in.isetId;
      in.skelRange = getSkeletonRange(in.startTime, in.endTime, skeletonSkip);
      if (in.skelRange.size() == 0) {
        cerr << "WARNING: No skeletons found in interaction: " << line << endl;
      }
      interactions.push_back(&in);
    }
  }
  hasInteractions = true;
  // Existing active segments are invalid because we have recreated all Interactions
  hasActiveSegments = false;
  return true;
}

bool Recording::save(const string& file) const {
  ofstream ofs(file, std::ios::binary);
  { boost::archive::binary_oarchive(ofs) << *this; }
  ofs.close();
  cout << "Current working directory: " << ml::util::getWorkingDirectory() << endl;
  cout << "Saved recording " << file
    << " [nSkels=" << skeletons.size() << ", time=" << durationInSec() << "s]" << endl;
  return true;
}

bool Recording::getSkeletonAtTime(float tInSec, SkelIter& skelItOut) const {  // NOLINT(*)
  auto skelAfterNow = [&](const Skeleton & s) {
    return (s.timestamp - skeletons.front().timestamp) * kTimestampToSec >= tInSec;
  };
  skelItOut = std::find_if(skeletons.begin(), skeletons.end(), skelAfterNow);
  return skelItOut != skeletons.end();
}

const Skeleton& Recording::getSkeletonAtTime(float tInSec) const {
  SkelIter result;
  getSkeletonAtTime(tInSec, result);
  if (result == skeletons.end()) {
    return skeletons.back();
  } else {
    return (*result);
  }
}

SkelRange Recording::getSkeletonRange(float tStart, float tEnd, size_t stride) const {
  SkelRange::container pSkels;

  const auto tFirst = skeletons.front().timestamp;
  size_t iStart = 0, iEnd, iSkel = 0;
  bool foundStart = false;
  for (; iSkel < skeletons.size(); iSkel++) {
    const Skeleton& skel = skeletons[iSkel];
    if ((skel.timestamp - tFirst) * kTimestampToSec >= tStart) {
      iStart = iSkel;
      foundStart = true;
      break;
    }
  }
  if (!foundStart) {
    return SkelRange(pSkels);
  }
  for (; iSkel < skeletons.size(); iSkel++) {
    const Skeleton& skel = skeletons[iSkel];
    if ((skel.timestamp - tFirst) * kTimestampToSec > tEnd) {
      break;
    }
  }
  iEnd = iSkel;

  const size_t numPreStrided = iEnd - iStart;
  const size_t numPostStrided = numPreStrided / stride;

  pSkels.reserve(numPostStrided);
  for (size_t i = iStart; i < iEnd; i += stride) {
    pSkels.push_back(std::cref(skeletons[i]));
  }
  return SkelRange(pSkels);
}

string Recording::baseScanId() const {
  vec<string> parts = ml::util::split(id, '_');
  return parts[0];
}

ostream& Recording::toJSON(ostream& os, bool endlines) const {  // NOLINT(*)
  using io::toJSON;
  const std::function<void(void)> sep = io::put(os, ",", endlines);
  const auto key = [ ] (const string& id) { return "\"" + id + "\": "; };

  os << "{";                      if (endlines) { os << endl; }
  os << key("id")                 << "\"" + id + "\"";  sep();
  os << key("camera");            toJSON(os, camera);  sep();
  os << key("startTime")          << startTime;  sep();
  os << key("endTime")            << endTime;  sep();
  os << key("skeletons");         toJSON(os, skeletons);  sep();
  os << key("colorTimestamps");   toJSON(os, colorTimestamps);  sep();
  os << key("depthTimestamps");   toJSON(os, depthTimestamps);  if (endlines) { os << endl; }
  os << "}";                      if (endlines) { os << endl; }

  return os;
}

bool Recording::loadFromJSON(const string& file) {
  if (isLoaded) { return false; }
  if (!ml::util::fileExists(file)) {
    cerr << "[Recording] Failed loading from " << file << " (file does not exist)" << endl;
  }

  // Parse JSON document
  rapidjson::Document d;
  if (!io::parseRapidJSONDocument(file, &d)) {
    cerr << "Parse error reading " << file << endl
         << "Error code " << d.GetParseError() << " at " << d.GetErrorOffset() << endl;
    return false;
  }

  // Parse out elements
  id = d["id"].GetString();

  const auto& camArr = d["camera"];
  for (unsigned i = 0; i < camArr.Size(); i++) {
    camera[i] = static_cast<float>(camArr[i].GetDouble());
  }

  startTime = d["startTime"].GetUint64();
  endTime = d["endTime"].GetUint64();

  const auto& colorTarr = d["colorTimestamps"];
  const unsigned numColorTs = colorTarr.Size();
  colorTimestamps.resize(numColorTs);
  for (unsigned i = 0; i < numColorTs; i++) {
    colorTimestamps[i] = colorTarr[i].GetInt64();
  }

  const auto& depthTarr = d["depthTimestamps"];
  const unsigned numDepthTs = depthTarr.Size();
  depthTimestamps.resize(numDepthTs);
  for (unsigned i = 0; i < numDepthTs; i++) {
    depthTimestamps[i] = depthTarr[i].GetInt64();
  }

  const auto& skelArr = d["skeletons"];
  const unsigned numSkels = skelArr.Size();
  skeletons.resize(numSkels);

  // Skeleton loading helper function
  auto loadSkeleton = [&] (unsigned i) {
    const auto& sIn = skelArr[i];
    Skeleton& s = skeletons[i];

    s.trackingId = sIn["trackingId"].GetUint64();
    const auto& posArr = sIn["jointPositions"];
    const auto& confArr = sIn["jointConfidences"];
    const auto& oriArr = sIn["jointOrientations"];
    for (unsigned iJoint = 0; iJoint < Skeleton::kNumJoints; iJoint++) {
      s.jointConfidences[iJoint] = static_cast<float>(confArr[iJoint].GetDouble());
      for (unsigned iDim = 0; iDim < 3; iDim++) {
        s.jointPositions[iJoint][iDim] = static_cast<float>(posArr[iJoint][iDim].GetDouble());
        s.jointOrientations[iJoint][iDim] = static_cast<float>(oriArr[iJoint][iDim].GetDouble());
      }
      s.jointOrientations[iJoint][3] = static_cast<float>(oriArr[iJoint][3].GetDouble());
    }
    const auto& handState = sIn["handState"];
    s.handLeftState = static_cast<Skeleton::HandState>(handState[0].GetUint());
    s.handLeftConfidence = static_cast<Skeleton::TrackingConfidence>(handState[1].GetUint());
    s.handRightState = static_cast<Skeleton::HandState>(handState[2].GetUint());
    s.handRightConfidence = static_cast<Skeleton::TrackingConfidence>(handState[3].GetUint());
    const auto& activArr = sIn["activities"];
    for (unsigned iActivity = 0; iActivity < Skeleton::Activity_Count; iActivity++) {
      s.activities[iActivity] = static_cast<Skeleton::DetectionResult>(activArr[iActivity].GetUint());
    }
    const auto& leanState = sIn["leanState"];
    s.leanLeftRight = static_cast<float>(leanState[0].GetDouble());
    s.leanForwardBack = static_cast<float>(leanState[1].GetDouble());
    s.leanConfidence = static_cast<float>(leanState[2].GetDouble());
    s.clippedEdges = sIn["clippedEdges"].GetUint64();
    s.timestamp = sIn["timestamp"].GetInt64();
  };

  for (unsigned iSkel = 0; iSkel < numSkels; iSkel++) {
    loadSkeleton(iSkel);
  }

  finalizeLoad(file);
  return true;
}

bool Recording::loadFromSkelStates(const string& _id, const vec<SkelState>& skelStates, float fps,
                                   const ml::mat4f* pXform /*= nullptr */) {
  id = _id;
  startTime = 0;
  const auto numSkels = skelStates.size();
  const float deltaT = 1.f / fps;
  const float duration = numSkels * deltaT;
  const float timestampMultiplier = deltaT / kTimestampToSec;
  skeletons.clear();
  for (int i = 0; i < numSkels; ++i) {
    const auto& ss = skelStates[i];
    Skeleton skel;
    state2skel(ss, &skel);
    skel.rec = this;
    skel.timestamp = static_cast<uint64_t>(i * timestampMultiplier);
    skel.trackingId = 0;
    skeletons.push_back(skel);
  }
  endTime = static_cast<uint64_t>(duration * 1000000);  // in microseconds

  if (pXform) {
    applyRotation(*pXform);
  }

  isLoaded = true;

  SG_LOG_INFO << "[Recording: converted from ASF/AMC, id: " << id
       << ", nSkels: " << skeletons.size() << ", time: " << durationInSec() << "s]";

  return true;
}

void dejitterSkeletons(vec<Skeleton>& skels) {
  //SG_LOG_INFO << "Dejittering skeletons using One Euro filter...";
  //const float
  //  frequency = 30.f,
  //  mincutoff = .01f,
  //  beta = 0.f,
  //  dcutoff = .01f;
  arr<math::OneEuroFilterVec, Skeleton::kNumJoints> filtVec;
  arr<math::OneEuroFilterQuat, Skeleton::kNumJoints> filtQuat;

  int64_t prevTimestamp = skels[0].timestamp;
  for (Skeleton& s : skels) {
    float dt = (s.timestamp - prevTimestamp) * Recording::kTimestampToSec;
    prevTimestamp = s.timestamp;
    for (int iJoint = 0; iJoint < Skeleton::kNumJoints; ++iJoint) {
      ml::vec3f& p = s.jointPositions[iJoint];
      const ml::vec3f p2 = filtVec[iJoint].filter(p, dt);
      //cout << p2 - p << endl;
      p = p2;
      ml::vec4f& o = s.jointOrientations[iJoint];
      geo::Quatf q(o.w, o.x, o.y, o.z);
      q = filtQuat[iJoint].filter(q, dt);
      o.w = q.w();
      o.x = q.x();
      o.y = q.y();
      o.z = q.z();
    }
  }
  //SG_LOG_INFO << "Dejittering done.";
}

void Recording::unfloatSkeletons(const Scan& scan) {
  if (isUnfloated) { return; }
  for (Skeleton& s : skeletons) {
    s.unfloat(scan);
  }
  dejitterSkeletons(skeletons);
  isUnfloated = true;
}

}  // namespace core
}  // namespace sg
