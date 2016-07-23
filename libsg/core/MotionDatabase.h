#pragma once

#include "libsg.h"  // NOLINT

namespace sg {
namespace core {

struct MotionInfo {
  string motionId;
  string subjectId;
  string description;
  string subject;
  bool isLoaded;
  template<class Ar>
  void serialize(Ar& ar, unsigned) {
    ar & motionId & subjectId & description & subject;
  }
  ostream& toJSON(ostream& os) const {
    return os << "{\"subjectId\":\"" << subjectId << "\", "
      << "\"motionId\":\"" << motionId << "\", "
      << "\"description\":\"" << description << "\", "
      << "\"subject\":\"" << subject << "\"}";
  }
};

class MotionDatabase {
 public:
  explicit MotionDatabase(const util::Params& params);
  const Recording& getMotion(const string& id) const;
  vec<string> getMotionIds() const;
  const MotionInfo& getMotionInfo(const string& id) const {
    return m_infos.at(id);
  }
  vec<string> getMotionIdsByKeyword(const string& keyword) const;
  // ensures motions with given subjectId are loaded
  void ensureSubjectMotionsLoaded(const string& subjectId);
  // ensures all motions are loaded
  void ensureAllMotionsLoaded();
  ostream& toJSON(ostream& os) const;
  // report total dataset time (optionally filtered by keyword in description)
  float totalTime(const string& keyword = "") const;

 private:
  const util::Params& m_params;
  map<string, map<string,Recording>> m_motionsBySubject;
  map<string, MotionInfo> m_infos;
};

}  // namespace core
}  // namespace sg
