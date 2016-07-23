#pragma once

#include "libsg.h"  // NOLINT

namespace sg {
namespace ann {

template <typename T, typename num_t = double>
class Distance {
 public:
  virtual ~Distance() { }
  virtual num_t distance(const T& t1, const T& t2) const = 0;
};

template <typename T, typename FeatureGenerator>
class FeatureDistance : public Distance<T,double> {
 public:
  double distance(const T& t1, const T& t2) const override {
    const vecd& features1 = m_featureGenerator.generate(t1);
    const vecd& features2 = m_featureGenerator.generate(t2);
    assert(features1.size() == features2.size());
    double sum = 0.0;
    for (int i = 0; i < features1.size(); i++) {
      double diff = features1[i]-features2[i];
      sum += diff*diff;
    }
    return sqrt(sum);
  }

 private:
  FeatureGenerator m_featureGenerator;
};

}  // namespace ann
}  // namespace sg


