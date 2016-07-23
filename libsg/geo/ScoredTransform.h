#pragma once

#include <mLibCore.h>

#include "libsg.h"  // NOLINT

namespace sg {
namespace geo {

//! Transform with a score, a weight, a label
//! and a index into some other data structure
struct ScoredTransform {
  int index;
  double weight;
  double score;
  string label;
  ml::mat4f xform;
private:
  friend class boost::serialization::access;
  //! boost serialization function
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {  // NOLINT
    ar & index;
    ar & weight;
    ar & score;
    ar & label;
    ar & xform;
  }
};

}  // namespace geo
}  // namespace sg



