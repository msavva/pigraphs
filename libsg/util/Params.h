#pragma once

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/spirit/home/support/detail/hold_any.hpp>

#include "libsg.h"  // NOLINT

namespace sg {
namespace util {

using boost::property_tree::ptree;

//! Generic parameter parsing utility class
struct Params {
  //! Constructor: prefix indicates section in parameter file (e.g. [MyParams])
  //! to pre-pend variable names by default: e.g. MyParams.numFiles where MyParams
  //! is the prefix
  explicit Params(const string& _prefix = "") : m_prefix(_prefix) { }

  // Returns value of parameter given its name
  template <typename T>
  T get(const string& name) const {
    const string& prename = prefixedName(name);
    if (m_pt.get_child_optional(prename)) {  // Check exists
      try {
        return m_pt.get<T>(prename);
      } catch (std::exception e) {
        cerr << "[Params]: Reading param with name '" << name << "' failed (check type)" << endl;
        assert(false);
        return m_pt.get<T>(prename);
      }
    } else {
      cerr << "[Params]: Param '" << prename << "' not found! Add it to your parameters file."
                << endl;
    }
    return m_pt.get<T>(prename);  // This is here to ensure types check out to the compiler's happiness
  }

  // Returns value of parameter given its name and a default value
  template <typename T>
  T getWithDefault(const string& name, const T& defaultValue) const {
    const string& prename = prefixedName(name);
    if (m_pt.get_child_optional(prename)) {  // Check exists
      try {
        return m_pt.get<T>(prename);
      } catch (std::exception e) {
        cerr << "[Params]: Reading param with name '" << name << "' failed (check type)" << endl;
        assert(false);
        return defaultValue;
      }
    } else {
      return defaultValue;
    }
  }

  // Sets value of parameter with given name into passed pointer location
  template <typename T>
  void get(const string& name, T* val) const {
    *val = get<T>(name);
  }

  // Returns value of parameter as a vector of strings given its name
  vec<string> getVec(const string& name, const string& delim = ",") const;

  //! Set param name to val. Will overwrite current val if it exists
  template <typename T>
  void set(const string& name, const T& val) {
    m_pt.put(prefixedName(name), val);
  }

  //! Returns whether parameter with name exists in this Params
  bool exists(const string& name) const {
    return !!m_pt.get_child_optional(name);  // NOTE: Double negation to get bool
  }

  //! Returns whether parameter with name exists in this Params and is leaf 
  bool isLeaf(const string& name) const {
    return exists(name) && !m_pt.get_child(name).get_value<string>().empty();
  }

  //! Read parameter file
  bool read(const string& file);

  //! Update this Params by reading in file, adding and overwriting wherever overlap exists.
  //! If file does not exist emit warning.
  void update(const string& file);

  //! Update this Params by reading in file if it exists, otherwise do nothing.
  void updateIfExists(const string& file);

  //! Update this Params by adding and overwriting settings from other wherever overlap exists
  void update(const Params& other);

  //! Write to parameter file
  void write(const string& file) const;

  //! Extract a subset of the parameters that belong to the given section
  Params extract(const string& section) const {
    return Params(m_pt.get_child(section));
  }

  ostream& toJSON(ostream& os) const;  // NOLINT

  template <typename GV>
  void fromRapidJson(const GV& value) {
    fromRapidJson(value, &m_pt);
  }

 private:
  explicit Params(ptree _pt) : m_pt(_pt) {};

  string prefixedName(const string& name) const {
    if (name[0] == '.') { return name.substr(1); }
    if (m_prefix.empty()) {
      return name;
    } else {
      return m_prefix + "." + name;
    }
  }

  // Helper functions to recursively apply a method function on all nodes
  // http://stackoverflow.com/questions/8154107/how-do-i-merge-update-a-boostproperty-treeptree
  template<typename T>
  void traverse_recursive(const ptree& parent, const ptree::path_type& childPath,
                          const ptree& child, const T& method) {
    method(parent, childPath, child);
    for (ptree::const_iterator it = child.begin(); it != child.end(); ++it) {
      ptree::path_type curPath = childPath / ptree::path_type(it->first);
      traverse_recursive(parent, curPath, it->second, method);
    }
  }

  template<typename T>
  void traverse(const ptree& parent, const T& method) {
    traverse_recursive(parent, "", parent, method);
  }

  void merge(const ptree& parent, const ptree::path_type& childPath, const ptree& child) {
    m_pt.put(childPath, child.data());
  }

  template <typename GV>
  void fromRapidJson(const GV& value, ptree* pt) {
    pt->clear();
    for (auto it = value.MemberBegin(); it != value.MemberEnd(); ++it) {
      const std::string name = it->name.GetString();
      if (it->value.IsObject()) {
        pt->add_child(name, ptree());
        fromRapidJson(it->value, &pt->get_child(name));
      } else {
        pt->put(name, it->value.GetString());
      }
    }
  }

  //! Parameter group prefix (pre-pended on all params as prefix.param)
  string m_prefix;

  // Boost property tree storing key-value pairs for all contained parameters
  ptree m_pt;
};

inline ostream& toJSON(ostream& os, const Params& params) {  // NOLINT
  return params.toJSON(os);
}


}  // namespace util
}  // namespace sg


