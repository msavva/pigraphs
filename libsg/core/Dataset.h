#pragma once

#include "libsg.h"  // NOLINT

#include "math/math.h"

namespace sg {
namespace core {

//! Training dataset encapsulation
template <typename T>
struct Dataset {
  //! Instance labels (just 0 = negative and 1 = positive for now)
  enum Label {
    Label_NEG = 0,
    Label_POS = 1
  };

  typedef pair<T, Label> LabeledInstance;  //! An instance with a label
  typedef std::function<bool(const T&)> InstanceFilterFun;  //! Predicate function for filtering instances
  typedef std::function<bool(const LabeledInstance&)> LabeledInstanceFilterFun;  //! Predicate function for filtering LabeledInstances
  typedef std::function<LabeledInstance(const LabeledInstance&)> LabeledInstanceXformFun;  //! Instance transformation function

  //! Add all instances in iterator range filtering into positive/negative using posFilterFun
  template <typename Iter>
  void add(const Iter begin, const Iter end, const InstanceFilterFun& posFilterFun) {
    for (Iter it = begin; it != end, it++) {
      const T& x = *it;
      if (posFilterFun(x)) {
        m_instances.push_back(std::make_pair(x, Label_POS));
      } else {
        m_instances.push_back(std::make_pair(x, Label_NEG));
      }
    }
  }

  //! Add all *LabeledInstances* in iterator range directly into dataset
  template <typename Iter>
  void add(const Iter begin, const Iter end) {
    for (Iter it = begin; it != end; it++) {
      const LabeledInstance& x = *it;
      m_instances.push_back(x);
    }
  }

  //! Add all *LabeledInstances* in container directly into dataset
  template <typename Container>
  void add(const Container& c) { add(c.begin(), c.end()); }

  //! Add given LabeledInstance into dataset
  void add(const LabeledInstance& x) { m_instances.push_back(x); }
  void addPositive(const T& x) { m_instances.push_back(std::make_pair(x, Label::Label_POS)); }
  void addNegative(const T& x) { m_instances.push_back(std::make_pair(x, Label::Label_NEG)); }

  //! Delete all contained instances
  void clear() { m_instances.clear(); }

  //! Return instances of this dataset that satisify filterFun predicate
  vec<LabeledInstance> filteredInstances(const LabeledInstanceFilterFun& filterFun) const {
    vec<LabeledInstance> out;
    std::copy_if(m_instances.begin(), m_instances.end(), std::back_inserter(out), filterFun);
    return out;
  }

  const vec<LabeledInstance>& instances() const { return m_instances; }
  vec<LabeledInstance> positives() const {
    return filteredInstances([] (const LabeledInstance& x) { return x.second == Label_POS; });
  }
  vec<LabeledInstance> negatives() const {
    return filteredInstances([] (const LabeledInstance& x) { return x.second == Label_NEG; });
  }
  size_t numPositives() const { return filteredInstances(positivesFilter).size(); }  // TODO(ms): Is inefficient, but meh for now!
  size_t numNegatives() const { return filteredInstances(negativesFilter).size(); }
  size_t size() const { return m_instances.size(); }

 private:
  //! Underlying labeled instance container
  vec<LabeledInstance> m_instances;
};

//! Generator to create Datasets, split them into positive and negative instances, training and testing sets etc.
template <typename T>
struct DatasetGenerator {
  typedef Dataset<T> DatasetT;
  typedef typename DatasetT::InstanceFilterFun InstanceFilterFun;
  typedef vec<T> VecRawInstance;
  typedef vec<typename DatasetT::LabeledInstance> VecLabeledInstance;

  //! Create labeled Dataset by using pos/neg split function (returns true for pos instances)
  DatasetT createDataset(const VecRawInstance& rawInstances, const typename InstanceFilterFun& posFun) const {
    DatasetT out;
    for (const T& x : rawInstances) {
      if (posFun(x)) {
        out.add(std::make_pair(x, DatasetT::Label_POS));
      } else {
        out.add(std::make_pair(x, DatasetT::Label_NEG));
      }
    }
    return out;
  }

  //! Rebalance dataset by picking numPos positive instances and numNeg instances at random from original instances
  VecLabeledInstance rebalancedInstances(const DatasetT& dataset,
                                         size_t numPos, size_t numNeg) const {
    VecLabeledInstance out;
    out.reserve(numNeg + numPos);

    // Randomly select numNeg subset of negatives
    auto negatives = dataset.negatives();
    std::random_shuffle(negatives.begin(), negatives.end());
    if (negatives.size() > numNeg) {
      negatives.resize(numNeg);
    }
    out.insert(out.end(), negatives.begin(), negatives.end());

    // Randomly select numPos subset of positives
    auto positives = dataset.positives();
    std::random_shuffle(positives.begin(), positives.end());
    if (positives.size() > numPos) {
      positives.resize(numPos);
    }
    out.insert(out.end(), positives.begin(), positives.end());

    return out;
  }

  //! Return numPerturbedInstances set of randomly chosen instances perturbed using perturbFun
  VecLabeledInstance perturbedInstances(const VecLabeledInstance& instances, size_t numPerturbedInstances,
                                        const typename DatasetT::LabeledInstanceXformFun& perturbFun) const {
    VecLabeledInstance perturbed;
    for (size_t i = 0; i < numPerturbedInstances; i++) {
      const auto& inst = math::DEF_RAND(instances);
      perturbed.push_back(perturbFun(inst));
    }
    return perturbed;
  }
};

}  // namespace core
}  // namespace sg


