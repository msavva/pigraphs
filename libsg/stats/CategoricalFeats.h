#pragma once

#include "stats/histogram.h"

namespace sg {
namespace stats {

//! Encapsulates a categorical distribution over feature space Feats described by FeatDesc
template<class Feats, class FeatDesc>
class CategoricalFeats {
 public:
  explicit CategoricalFeats(const FeatDesc& _featDesc) : m_featDesc(_featDesc) { }

  //! Return by reference features for given category id. Initializes new empty features if not already existing
  Feats& getFeatures(const string& id) {
    if (cat2feats.count(id) > 0) {
      return cat2feats[id];
    } else {
      Feats& feats = cat2feats[id];
      initFeatures(m_featDesc, &feats);
      return feats;
    }
  }

  map<string, Feats> cat2feats;  //! Map category -> features

 private:
  //! Initializes features in feats given the feature space descriptor featDesc
  void initFeatures(const FeatDesc& featDesc, Feats* feats) {
    feats->reserve(m_featDesc.numDims);
    for (int iDim = 0; iDim < m_featDesc.numDims; iDim++) {
      const auto& featDim = m_featDesc.dims[iDim];
      feats->push_back(Histogram<float>(featDim.range, featDim.numBins, featDim.id));
    }
  }

  FeatDesc m_featDesc;  //! Descriptor of feature space for this Distribution (used for initializing)
};

}  // namespace stats
}  // namespace sg


