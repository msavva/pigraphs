#include "common.h"  // NOLINT

#include "core/MotionDatabase.h"

// ReSharper disable once CppUnusedIncludeDirective
#include <ext-boost/serialization.h>

#include "core/AMCRecording.h"
#include "core/Recording.h"
#include "io/io.h"
#include "io/csv.h"
#include "util/Params.h"
#include "util/Timer.h"
#include "util/util.h"

using sg::core::ASFSkeleton;  using sg::core::AMCRecording;

namespace sg {
namespace core {

bool populateInfos(const string& indexFile, map<string, MotionInfo>* out) {
  using io::csv::CSVReader;
  if (!io::fileExists(indexFile)) { return false; }

  CSVReader<3, io::csv::trim_chars<' '>, io::csv::double_quote_escape<',','\"'> > csv(indexFile);
  csv.read_header(io::csv::ignore_extra_column, "id", "description", "subject");
  string id, description, subject;
  while(csv.read_row(id, description, subject)) {
    const auto tokens = util::tokenize(id, "_");
    const string subjectId = tokens[0];
    const string motionId = id;
    (*out)[id] = {motionId, subjectId, description, subject, false};
  }

  return true;
}

MotionDatabase::MotionDatabase(const util::Params& params) : m_params(params) {
  const string dir = params.get<string>(".cmuMocapDir");
  const string indexFile = dir + "index.csv";
  if (!populateInfos(indexFile, &m_infos)) {
    SG_LOG_ERROR << "[MotionDatabase] Error loading index from: " << indexFile;
  }
}

const Recording& MotionDatabase::getMotion(const string& id) const {
  const auto& m = m_infos.at(id);
  const_cast<MotionDatabase*>(this)->ensureSubjectMotionsLoaded(m.subjectId);  // it's ok to call non-const fun here!
  return m_motionsBySubject.at(m.subjectId).at(m.motionId);
}

vec<string> MotionDatabase::getMotionIds() const {
  vec<string> ids;
  util::mapToKeyVec(m_infos, ids);
  return ids;
}

vec<string> MotionDatabase::getMotionIdsByKeyword(const string& keyword) const {
  vec<string> ids;
  for (const auto& p : m_infos) {
    const MotionInfo& mi = p.second;
    if (ml::util::contains(mi.description, keyword) ||
        ml::util::contains(mi.subject, keyword)) {
      ids.push_back(mi.motionId);
    }
  }
  return ids;
}

void MotionDatabase::ensureSubjectMotionsLoaded(const string& subjectId) {
  auto& subjectMotions = m_motionsBySubject[subjectId];
  if (subjectMotions.size()) { return; }  // if already loaded

  util::Timer timer("[MotionDatabase] Load subject " + subjectId);
  const string cacheFile = m_params.get<string>("dataCacheDir") + "motions/" + subjectId + ".cached";
  if (io::fileExists(cacheFile)) {  // load cached subject
    ifstream ifs(cacheFile, std::ios::binary);
    boost::archive::binary_iarchive bia(ifs);
    bia >> subjectMotions;
    for (auto& p : m_infos) {
      if (p.second.subjectId == subjectId) {
        p.second.isLoaded = true;
      }
    }
  } else {  // load original and save to cache
    // get filtered list of MotionInfos
    vec<MotionInfo> mis;
    for (const auto& mi : m_infos) {
      if (mi.second.subjectId == subjectId) { mis.push_back(mi.second); }
    }

    const string
      baseDir = m_params.get<string>(".cmuMocapDir") + subjectId + "/",
      asfFile = baseDir + subjectId + ".asf";
    ASFSkeleton asf(asfFile, MOCAP_SCALE);

    // load subject motions
    for (int i = 0; i < mis.size(); ++i) {
      const MotionInfo& mi = mis[i];
      const string amcFile = baseDir + mi.motionId + ".amc";
      AMCRecording amc(amcFile, MOCAP_SCALE, &asf);
      const auto skelStates = amc.toSkelStates();
      const ml::mat4f asf2world = geo::to<ml::mat4f>(amc.getRecordingToWorldXform());
      subjectMotions[mi.motionId].loadFromSkelStates(mi.motionId, skelStates, 120.f, &asf2world);
      m_infos[mi.motionId].isLoaded = true;
    }

    // cache locally
    io::ensurePathToFileExists(cacheFile);
    ofstream ofs(cacheFile, std::ios::binary);
    boost::archive::binary_oarchive boa(ofs);
    boa << subjectMotions;
    SG_LOG_INFO << "[MotionDatabase] cached " << cacheFile;
  }
}

void MotionDatabase::ensureAllMotionsLoaded() {
  for (auto& p : m_infos) {
    ensureSubjectMotionsLoaded(p.second.subjectId);
    p.second.isLoaded = true;
  }
}

ostream& MotionDatabase::toJSON(ostream& os) const {
  float totalTime = 0;
  for (const auto& p : m_infos) {
    const MotionInfo& mi = p.second;
    const Recording& rec = getMotion(mi.motionId);
    mi.toJSON(os);
    const float t = rec.durationInSec();
    totalTime += t;
    os << ", {\"duration:\":" << t << "}" << endl;
  }
  os << "{\"duration\":" << totalTime << "}" << endl;
  return os;
}

float MotionDatabase::totalTime(const string& keyword) const {
  float totalTime = 0;
  for (const string id : getMotionIdsByKeyword(keyword)) {
    totalTime += getMotion(id).durationInSec();
  }
  return totalTime;
}

}  // namespace core
}  // namespace sg
