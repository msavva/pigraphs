#pragma once

#include <boost/bimap.hpp>
#include <mLibCore.h>

#include "libsg.h"  // NOLINT

namespace sg {
namespace util {

// Convenient bidirectional map between objects of type T and indices: id <-> obj
template <typename T>
class Index {
 public:
  int indexOf(const T&x) const {
    if (m_bimap.right.count(x) > 0) {
      return static_cast<int>(m_bimap.right.at(x));
    } else {
      return -1;
    }
  }
  int indexOf(const T& x, bool add = false) {
    if (m_bimap.right.count(x) > 0) {
      return static_cast<int>(m_bimap.right.at(x));
    } else {
      if (add) {
        m_bimap.insert(bm_type::value_type(m_bimap.size(), x));
        return static_cast<int>(m_bimap.right.at(x));
      } else {
        return -1;
      }
    }
  }
  int add(const T& x) { return indexOf(x, true); }
  size_t size() const { return m_bimap.size(); }
  const T& operator[](size_t id) const { return m_bimap.left.at(id); }
  void clear() { m_bimap.clear(); }
  vec<T> labels() const {
    vec<T> res;
    for (size_t i = 0; i < m_bimap.size(); ++i) {
      res.push_back(m_bimap.left.at(i));
    }
    return res;
  }

 private:
  // boost serialization support
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {  // NOLINT
    ar & m_bimap;
  }

  typedef boost::bimap<size_t, T> bm_type;
  bm_type m_bimap;
};

template <typename T>
void saveIndexCsv(const string& filename, const util::Index<T>& index) {
  io::ensurePathToFileExists(filename);
  ofstream ofs(filename);
  for (int i = 0; i < index.size(); i++) {
    ofs << index[i] << "," << i << endl;
  }
  ofs.close();
}


}  // namespace util
}  // namespace sg


