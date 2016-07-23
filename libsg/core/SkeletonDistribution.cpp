#include "common.h"  // NOLINT

#include "core/SkeletonDistribution.h"

#include "math/math.h"

namespace sg {
namespace core {

using namespace stats;  using namespace math;  using namespace geo;

//! Transforms Y-up SkelState space to Z-up world space
const Quatf kSkelState2World(AngleAxisf(constants::PIf/2.f, Vec3f::UnitX()));

void setBodyFrame(float totalBoneLength, double spineAngle, SkelState* ss) {
  ss->bodyFrame.p.setZero();
  const Quatf spineTilt(AngleAxisf(static_cast<float>(spineAngle), Vec3f::UnitZ()));
  ss->bodyFrame.q = toAngleEncoding(kSkelState2World * spineTilt, ss->angleEncoding);
  ss->bodyFrame.l = totalBoneLength;
}

JointCoordsDistribution::JointCoordsDistribution(const JointCoords::Data& p) {
  Vec3f kq;
  const Vec3f mq(circularMean(p[3], &kq[0]),
                 circularMean(p[4], &kq[1]),
                 circularMean(p[5], &kq[2]));
  qlat = VonMisesDistribution(mq[0], kq[0]);
  qlon = VonMisesDistribution(mq[1], kq[1]);
  qang = VonMisesDistribution(mq[2], kq[2]);
  px   = GaussianDistribution(math::mean(p[0]), stdev(p[0]));
  py   = GaussianDistribution(math::mean(p[1]), stdev(p[1]));
  pz   = GaussianDistribution(math::mean(p[2]), stdev(p[2]));
  blen = GaussianDistribution(math::mean(p[6]), stdev(p[6]));
}

JointCoords JointCoordsDistribution::sample() const {
  JointCoords jc;
  jc.p[0] = static_cast<float>(px.mean());
  jc.p[1] = static_cast<float>(py.mean());
  jc.p[2] = static_cast<float>(pz.mean());
  jc.q[0] = static_cast<float>(qlat.mean());
  jc.q[1] = static_cast<float>(qlon.mean());
  jc.q[2] = static_cast<float>(qang.sample());
  jc.l    = static_cast<float>(blen.mean());
  return jc;
}

JointCoords JointCoordsDistribution::mean() const {
  JointCoords jc;
  jc.p[0] = static_cast<float>(px.mean());
  jc.p[1] = static_cast<float>(py.mean());
  jc.p[2] = static_cast<float>(pz.mean());
  jc.q[0] = static_cast<float>(qlat.mean());
  jc.q[1] = static_cast<float>(qlon.mean());
  jc.q[2] = static_cast<float>(qang.mean());
  jc.l    = static_cast<float>(blen.mean());
  return jc;
}

double JointCoordsDistribution::logprob(const JointCoords& jc) const {
  const double w = constants::PI / 180;  // integration width for pdf (~= 1 degrees)
  double lp = 0;
  lp += qang.logprob(jc.q[2], w);
  //lp += qlat.logprob(jc.q[0], w) + qlon.logprob(jc.q[1], w) + qang.logprob(jc.q[2], w);
  //lp += px.logprob(jc.p[0]) + px.logprob(jc.p[1]) + px.logprob(jc.p[2]);
  //lp += blen.logprob(jc.l);
  return lp;
}

SkeletonDistribution::SkeletonDistribution(const SkelState::Data& p) {
  for (int iJoint = 0; iJoint < SkelState::kNumJoints; ++iJoint) {
    joints[iJoint] = JointCoordsDistribution(p[iJoint]);
  }
  body = JointCoordsDistribution(p[SkelState::kNumJoints]);  // body params stored last
  const auto& spineAngles = p[SkelState::kNumJoints + 1][0];
  float spineAngleKappa;
  const float spineAngleMu = circularMean(spineAngles, &spineAngleKappa);
  spineAngleDist = VonMisesDistribution(spineAngleMu, spineAngleKappa);
}

SkelState SkeletonDistribution::sample() const {
  SkelState ss;
  for (int iJoint = 0; iJoint < SkelState::kNumJoints; ++iJoint) {
    ss.joints[iJoint] = joints[iJoint].sample();
  }
  setBodyFrame(body.mean().l, spineAngleDist.mean(), &ss);
  ss.isHierarchical = true;
  ss.angleEncoding = geo::AngleEncoding::LAT_LONG_ROT;
  const SkelState ssAbs = makeAbsolute(ss);
  return ssAbs;
}

pair<SkelState, double> SkeletonDistribution::sampleMax(int numSamples) const {
  SkelState maxSs;
  double maxLP = -std::numeric_limits<double>::infinity();
  for (int iSample = 0; iSample < numSamples; ++iSample) {
    SkelState ss;
    for (int iJoint = 0; iJoint < SkelState::kNumJoints; ++iJoint) {
      ss.joints[iJoint] = joints[iJoint].sample();
    }
    ss.isHierarchical = true;
    ss.angleEncoding = geo::AngleEncoding::LAT_LONG_ROT;
    double lp = logprob(ss);
    if (maxLP < lp) {
      maxLP = lp;
      maxSs = ss;
    }
  }
  setBodyFrame(body.mean().l, spineAngleDist.mean(), &maxSs);
  const SkelState ssAbs = makeAbsolute(maxSs);
  return {ssAbs, maxLP};
}

pair<SkelState, double> SkeletonDistribution::sampleJointsMax(int numSamples) const {
  SkelState ss;
  ss.isHierarchical = true;
  ss.angleEncoding = geo::AngleEncoding::LAT_LONG_ROT;
  arr<double, SkelState::kNumJoints> lps;
  lps.fill(-std::numeric_limits<double>::infinity());
  for (int iSample = 0; iSample < numSamples; ++iSample) {
    for (int iJoint = 0; iJoint < SkelState::kNumJoints; ++iJoint) {
      const auto jc = joints[iJoint].sample();
      const double jclp = joints[iJoint].logprob(jc);
      if (jclp > lps[iJoint]) {
        ss.joints[iJoint] = jc;
        lps[iJoint] = jclp;
      }
    }
  }
  setBodyFrame(body.mean().l, spineAngleDist.mean(), &ss);
  const SkelState ssAbs = makeAbsolute(ss);
  return {ssAbs, logprob(ss)};
}

SkelState SkeletonDistribution::mean() const {
  SkelState ss;
  for (int iJoint = 0; iJoint < SkelState::kNumJoints; ++iJoint) {
    ss.joints[iJoint] = joints[iJoint].mean();
  }
  setBodyFrame(body.mean().l, spineAngleDist.mean(), &ss);
  ss.isHierarchical = true;
  ss.angleEncoding = geo::AngleEncoding::LAT_LONG_ROT;
  SG_LOG_INFO << logprob(ss);
  const SkelState ssAbs = makeAbsolute(ss);
  return ssAbs;
}

//! joint probability weights -- for testing
const arr<double, SkelState::kNumJoints> kJointWeights = {
  0.0,  // SpineBase
  1.0,  // SpineMid
  0.1,  // Neck
  0.0,  // Head
  1.0,  // ShoulderLeft
  1.0,  // ElbowLeft
  1.0,  // WristLeft
  0.0,  // HandLeft
  1.0,  // ShoulderRight
  1.0,  // ElbowRight
  1.0,  // WristRight
  0.0,  // HandRight
  1.0,  // HipLeft
  1.0,  // KneeLeft
  1.0,  // AnkleLeft
  0.0,  // FootLeft
  1.0,  // HipRight
  1.0,  // KneeRight
  1.0,  // AnkleRight
  0.0,  // FootRight
  0.0,  // SpineShoulder
  0.0,  // HandTipLeft
  0.0,  // ThumbLeft
  0.0,  // HandTipRight
  0.0   // ThumbRight
};

double SkeletonDistribution::logprob(const SkelState& ss) const {
  double lp = 0;
  for (int iJoint = 0; iJoint < SkelState::kNumJoints; ++iJoint) {
    const double w = kJointWeights[iJoint];
    const double jlp = joints[iJoint].logprob(ss.joints[iJoint]);
    lp += w * jlp;
    //lp += jlp;
  }
  // NOTE: body is ignored since global body frame should not impact prob
  return lp;
}

ostream& operator<<(ostream& os, const JointCoordsDistribution& j) {
  os << "{ \"qlat\":"<<j.qlat << ", \"qlon\":"<<j.qlon << ", \"qang\":"<<j.qang << ", \"blen\":"<< j.blen<< " }";
  return os;
}

ostream& operator<<(ostream& os, const SkeletonDistribution& s) {
  os << "{ \"type\":\"" << s.type() << "\", \"joints\": [";
  for (int iJoint = 0; iJoint < SkelState::kNumJoints; ++iJoint) {
    const string& jointName = SkelState::kJointNames[iJoint];
    os << "{\"name\":\"" << jointName << "\", \"dist\":" << s.joints[iJoint] << " }";
    if (iJoint != SkelState::kNumJoints - 1) { os << ", "; }
  }
  os << "]}";
  return os;
}

}  // namespace core
}  // namespace sg
