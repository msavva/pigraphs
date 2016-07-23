#pragma once

#include "libsg.h"  // NOLINT

#include "core/SkelState.h"
#include "stats/distributions.h"

namespace sg {
namespace core {

//! Distribution over joint parameters
class JointCoordsDistribution : public stats::Distribution<JointCoords> {
 public:
  JointCoordsDistribution() : qlat(), qlon(), qang(), px(), py(), pz(), blen() { }
  explicit JointCoordsDistribution(const JointCoords::Data& data);
  string type() const override { return "JointCoordsDistribution"; }
  JointCoords sample() const override;
  JointCoords mean() const override;
  double logprob(const JointCoords& x) const override;

  //! Latitude, longitude and rotation angle distributions
  stats::VonMisesDistribution qlat, qlon, qang;
  //! Child joint position distribution
  stats::GaussianDistribution px, py, pz;
  //! Child bone length distribution
  stats::GaussianDistribution blen;
};
ostream& operator<<(ostream& os, const JointCoordsDistribution& d);

//! A distribution over SkelStates
class SkeletonDistribution : public stats::Distribution<SkelState> {
 public:
  SkeletonDistribution() : body(), joints() { }
  explicit SkeletonDistribution(const SkelState::Data& data);
  string type() const override { return "SkeletonDistribution"; }
  SkelState sample() const override;
  pair<SkelState, double> sampleMax(int nSamples) const override;
  SkelState mean() const override;
  double logprob(const SkelState& x) const override;

  //! Sample each joint independently nSamples times and return max scoring SkelState
  pair<SkelState, double> sampleJointsMax(int nSamples) const;

  JointCoordsDistribution body;  //! body frame distribution
  arr<JointCoordsDistribution, SkelState::kNumJoints> joints;  //! joint distributions
  stats::VonMisesDistribution spineAngleDist;  //! angle of spine to horizontal
};
ostream& operator<<(ostream& os, const SkeletonDistribution& d);

}  // namespace core
}  // namespace sg
