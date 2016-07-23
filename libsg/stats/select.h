#pragma once

#include "libsg.h"  // NOLINT

#include "math/math.h"
#include "util/log.h"
#include "util/smartenum.h"

#include <boost/optional.hpp>

namespace sg {
namespace stats {

//! Generic strategy for selecting from a weighted list of things
//! kFirst = select first, kRandom = select random uniform, 
//! kAverage = select average, kSample = select weighted
#define SELECT_STRATEGY_TYPES(m,t)  \
        m(t, First)              \
        m(t, Random)             \
        m(t, Sample)             \
        m(t, Max)

SMARTENUMCLASS_DEFINE_ENUM(SelectStrategy, SELECT_STRATEGY_TYPES)
static SMARTENUMCLASS_DEFINE_NAMES(SelectStrategy, SELECT_STRATEGY_TYPES)
inline SMARTENUMCLASS_DEFINE_GET_VALUE_FROM_STRING(SelectStrategy, SelectStrategy::kCount)
OSTREAMABLE(SelectStrategy)

// Select an item from weighted candidates
template <typename Iter, typename T, typename num_t = double>
boost::optional<pair<T,num_t>> selectFrom(Iter candidates,
                                          const std::function<num_t(const T&)>& weightFunc,
                                          SelectStrategy selectStrategy, 
                                          sg::math::random<> rand = sg::math::DEF_RAND) {
  boost::optional<pair<T,num_t>> selected;
  num_t w;
  if (!candidates.empty()) {
    switch (selectStrategy) {
      case SelectStrategy::kFirst: {
          const T& f = candidates[0];
          w = weightFunc(f);
          selected.emplace(f,w);
        }
        break;
      case SelectStrategy::kRandom: {
          const T& r = *rand.select(candidates.begin(), candidates.end());
          w = weightFunc(r);
          selected.emplace(r,w);
        }
        break;
      case SelectStrategy::kSample:
        selected.emplace(); 
        rand.selectFromWeighted(candidates, weightFunc, &selected.get());
        break;
      case SelectStrategy::kMax:
        selected = selectMax(candidates, weightFunc);
        break;
      default:
        SG_LOG_WARN << "Unknown select strategy " << selectStrategy;
    }
  } else {
    SG_LOG_WARN << "No items to select from";
  }
  return selected;
}

template <typename Iter, typename T, typename num_t = double>
boost::optional<pair<T,num_t>> selectMax(Iter& iter, 
                                         const std::function<num_t(const T&)>& weightFunc) {
  boost::optional<pair<T,num_t>> selected;
  for (const T& v : iter) {
    num_t w = weightFunc(v);
    if (selected == boost::none || w > selected.value().second) {
      selected.emplace(v,w);
    }
  }
  return selected;
}

}  // namespace stats
}  // namespace sg
