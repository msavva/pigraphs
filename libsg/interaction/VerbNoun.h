#pragma once

#include "libsg.h"  // NOLINT
#include "util/util.h"

namespace sg {
namespace interaction {

//! A pair of verb+noun describing an interaction with an object
struct VerbNoun {
  string verb;
  string noun;
  bool operator<(const VerbNoun& o) const {
    if (verb == o.verb) {
      return noun < o.noun;
    } else {
      return verb < o.verb;
    }
  }
  const string toString() const {
    return verb + "_" + noun;
  }
  static string getVerbNounSetId(const set<VerbNoun>& vns) {
    string id;
    for (const VerbNoun& vn : vns) {
      id += vn.verb + "_" + vn.noun + "-";
    }
    id.pop_back();  // Remove last "-"
    return id;
  }
  static void getVerbNouns(const string& str, set<VerbNoun>* pVerbNouns) {
    size_t p = str.find('/');
    vec<string> fields;
    const string s = (p >= 0) ? str.substr(p + 1) : str;
    vec<string> pairs = util::tokenize(s, "-");
    for (const string& pair : pairs) {
      util::tokenize(pair, "_:", &fields);
      if (fields.size() > 1) {
        pVerbNouns->insert({fields[0], fields[1]});
      } else {
        pVerbNouns->insert({fields[0], ""});
      }
    }
  }
};

}  // namespace interaction
}  // namespace sg


