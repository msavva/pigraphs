#include "common.h"  // NOLINT

#include "util/Params.h"

#include "util/util.h"

#include <boost/bind.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ini_parser.hpp>

namespace sg {
namespace util {

vec<string> sg::util::Params::getVec(const string& name, const string& delim /* = "," */) const {
  const string& s = get<string>(name);
  return tokenize(s, delim);
}

ostream& sg::util::Params::toJSON(ostream& os) const {
  if (!m_pt.get_value<std::string>().empty()) {
    os << m_pt.get_value<std::string>();
  } else {
    boost::property_tree::json_parser::write_json(os, m_pt);
  }
  return os;
}

bool sg::util::Params::read(const string& file) {
  assert(boost::filesystem::exists(file));
  boost::property_tree::read_ini(file, m_pt);
  return true;
}

void sg::util::Params::update(const string& file) {
  if (!boost::filesystem::exists(file)) {
    cerr << "WARNING: Tried to update Params from non-existent file at: " << file << endl;
  }
  ptree pt;
  boost::property_tree::read_ini(file, pt);
  traverse(pt, boost::bind(&Params::merge, this, _1, _2, _3));
}

void sg::util::Params::update(const Params& other) {
  traverse(other.m_pt, boost::bind(&Params::merge, this, _1, _2, _3));
}

void sg::util::Params::updateIfExists(const string& file) {
  if (!boost::filesystem::exists(file)) {
    return;
  }
  ptree pt;
  boost::property_tree::read_ini(file, pt);
  traverse(pt, boost::bind(&Params::merge, this, _1, _2, _3));
}

void sg::util::Params::write(const string& file) const {
  boost::property_tree::write_ini(file, m_pt);
}

}  // namespace util
}  // namespace sg
