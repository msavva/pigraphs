#include "common.h"  // NOLINT

#include "interaction/InteractionSet.h"

// ReSharper disable once CppUnusedIncludeDirective
#include <ext-boost/serialization.h>

#pragma warning(disable:4267)
#include <boost/graph/adj_list_serialize.hpp>
#pragma warning(default:4267)
// pragmas to disable type conversion warning spewed by adj_list_serialize header

#include "interaction/Interaction.h"
#include "interaction/ProtoInteractionGraph.h"
#include "interaction/similarity.h"
// ReSharper disable once CppUnusedIncludeDirective
#include "util/eigen_boost_serialization.h"

using sg::core::TransformedSkeleton;

namespace sg {
namespace interaction {

const arr<string, InteractionSet::kNumInteractionSetTypes> InteractionSet::kInteractionSetNames = {
  "composite",      // 0
  "verbNoun",       // 1
  "verb",           // 2
  "all"             // 3
};

map<string, InteractionSetType> createInteractionSetTypeLookupByName() {
  map<string, InteractionSetType> m;
  for (int i = 0; i < InteractionSet::kNumInteractionSetTypes; ++i) {
    m.insert({InteractionSet::kInteractionSetNames[i], static_cast<InteractionSetType>(i)});
  }
  return m;
}
const map<string, InteractionSetType> InteractionSet::kInteractionSetTypeLookupByName =
  createInteractionSetTypeLookupByName();

BOOST_BINARY_SERIALIZABLE_IMPL(InteractionSet)

// Stop compiler from complaining about default initialization of arrays
#pragma warning(disable:4351)
InteractionSet::InteractionSet()
  : type(ISetType_Composite)
  , id()
  , verbs()
  , interactions()
  , sampledSkeletons()
  , jointActivationProb()
  , jointGroupActivationProb()
  , jointGroupWeights()
  , segmentPoseClassifier(nullptr)
  , segPerJointGroupClassifier()
  , segJointPerJointGroupClassifier()
  , segmentJointsAggregatedClassifier(nullptr)
  , segmentCentroidActivationAggregatedClassifier(nullptr)
  , segmentJointsLinearWeightedAggregatedClassifier(nullptr)
  , igKNNSimilarityClassifier()
  , interactionGraphs()
  , protoInteractionGraph()
  , protoInteractionFrame() {
  for (int i = 0; i < Skeleton::kNumJointGroups; ++i) {
    segPerJointGroupClassifier[i] = nullptr;
    segJointPerJointGroupClassifier[i] = nullptr;
  }
}
#pragma warning(default:4351)

  void InteractionSet::sampleSkeletons(UINT sampleCount) {
  size_t count = 0;
  for (Interaction* i : interactions) {
    count += i->skelRange.size();
  }
  // Select sampleCount numbers
  size_t n = std::min(static_cast<size_t>(sampleCount), count);
  set<size_t> sampled;
  while (sampled.size() < n) {
    size_t sample = static_cast<size_t>(floor(math::DEF_RAND.uniform_float_01() * count));
    sampled.insert(sample);
  }
  // Get skeletons
  auto sampledIt = sampled.begin();
  sampledSkeletons.clear();
  sampledSkeletons.reserve(n);
  count = 0;
  for (Interaction* i : interactions) {
    for (const Skeleton& s : i->skelRange) {
      if (count == *sampledIt) {
        ml::mat4f xform = s.getNormalizingTransform();
        // Assumes that memory for s will stick around
        sampledSkeletons.emplace_back(s, xform);
        ++sampledIt;
        if (sampledIt == sampled.end()) break;
      }
      count++;
    }
    if (sampledIt == sampled.end()) break;
  }
}

void InteractionSet::clusterSkeletons(UINT clusterCount) {
  using namespace ml;

  struct SkeletonBodyPart {
    SkeletonBodyPart(Skeleton::JointType joint, float _mass) {
      joints.push_back(joint);
      mass = _mass;
    }
    SkeletonBodyPart(Skeleton::JointType joint0, Skeleton::JointType joint1, float _mass) {
      joints.push_back(joint0);
      joints.push_back(joint1);
      mass = _mass;
    }

    vec<Skeleton::JointType> joints;
    float mass;
  };

  const float mHead = 0.073f, mTrunk = 0.507f, mUpperArm = 0.026f, mForearm = 0.016f,
              mHand = 0.007f, mThigh = 0.103f, mCalf = 0.043f, mFoot = 0.015f;

  vec< SkeletonBodyPart > bodyParts;
  bodyParts.push_back(SkeletonBodyPart(Skeleton::JointType_SpineMid, mTrunk));
  bodyParts.push_back(SkeletonBodyPart(Skeleton::JointType_Head, mHead));
  bodyParts.push_back(SkeletonBodyPart(Skeleton::JointType_ShoulderLeft, Skeleton::JointType_ElbowLeft, mUpperArm));
  bodyParts.push_back(SkeletonBodyPart(Skeleton::JointType_ShoulderRight, Skeleton::JointType_ElbowRight, mUpperArm));
  bodyParts.push_back(SkeletonBodyPart(Skeleton::JointType_ElbowLeft, Skeleton::JointType_HandLeft, mForearm));
  bodyParts.push_back(SkeletonBodyPart(Skeleton::JointType_ElbowRight, Skeleton::JointType_HandRight, mForearm));
  bodyParts.push_back(SkeletonBodyPart(Skeleton::JointType_HandLeft, mHand));
  bodyParts.push_back(SkeletonBodyPart(Skeleton::JointType_HandRight, mHand));
  bodyParts.push_back(SkeletonBodyPart(Skeleton::JointType_HipLeft, Skeleton::JointType_KneeLeft, mThigh));
  bodyParts.push_back(SkeletonBodyPart(Skeleton::JointType_HipRight, Skeleton::JointType_KneeRight, mThigh));
  bodyParts.push_back(SkeletonBodyPart(Skeleton::JointType_KneeLeft, Skeleton::JointType_AnkleLeft, mCalf));
  bodyParts.push_back(SkeletonBodyPart(Skeleton::JointType_KneeRight, Skeleton::JointType_AnkleRight, mCalf));
  bodyParts.push_back(SkeletonBodyPart(Skeleton::JointType_FootLeft, mFoot));
  bodyParts.push_back(SkeletonBodyPart(Skeleton::JointType_FootRight, mFoot));

  auto makeSkeletonFeatures = [&](const TransformedSkeleton & s) {
    MathVector<float> features;

    //
    // accumulate feature vector from each joint, skipping the gaze feature
    //
    for (const SkeletonBodyPart& bodyPart : bodyParts) {
      vec3f bodyPartPosition = vec3f::origin;
      for (Skeleton::JointType joint : bodyPart.joints) {
        bodyPartPosition += s.transformedJoint(joint) - s.transformedJoint(Skeleton::JointType_SpineBase);
      }
      bodyPartPosition *= bodyPart.mass / bodyPart.joints.size();

      features.push_back(bodyPartPosition.x);
      features.push_back(bodyPartPosition.y);
      features.push_back(bodyPartPosition.z);
    }

    return features;
  };

  vec< TransformedSkeleton > normalizedSkeletons;
  vec< MathVector<float> > features;

  for (Interaction* i : interactions) {
    for (const Skeleton& s : i->skelRange) {
      mat4f xform = s.getNormalizingTransform();
      // Assumes that memory for s will stick around
      TransformedSkeleton normalizedSkeleton(s, xform);
      normalizedSkeletons.push_back(normalizedSkeleton);
      features.push_back(makeSkeletonFeatures(normalizedSkeleton));
    }
  }

  const UINT maxIterations = 100;

  KMeansClustering< MathVector<float>, MathVectorKMeansMetric<float> > clustering;
  clustering.cluster(features, clusterCount, maxIterations, false);

  sampledSkeletons.resize(clusterCount);

  for (UINT clusterIndex = 0; clusterIndex < clusterCount; clusterIndex++) {
    sampledSkeletons[clusterIndex] = normalizedSkeletons[clustering.findBestClusterRepresenative(features, clusterIndex)];
  }

}

void InteractionSet::computeJointActivationProbabilities() {
  size_t numObservations = 0;
  jointActivationProb.fill(0);
  jointGroupActivationProb.fill(0);
  arr<bool, Skeleton::kNumJointGroups> jointGroupSeen;
  for (Interaction* interaction : interactions) {
    for (size_t iSkel = 0; iSkel < interaction->jointSegments.size(); iSkel++) {
      numObservations++;
      const auto& jointSegments = interaction->jointSegments[iSkel];
      jointGroupSeen.fill(false);
      for (size_t iJoint = 0; iJoint < Skeleton::kNumJoints + 1; iJoint++) {
        if (!jointSegments[iJoint].empty()) {
          jointActivationProb[iJoint]++;
          const size_t iJointGroup = Skeleton::kJointToJointGroup[iJoint];
          if (!jointGroupSeen[iJointGroup]) {
            jointGroupActivationProb[iJointGroup]++;
            jointGroupSeen[iJointGroup] = true;
          }
        }
      }
    }
  }
  const double normFactor = 1 / static_cast<double>(numObservations);
  for (int i = 0; i < jointActivationProb.size(); i++) {
    jointActivationProb[i] *= normFactor;
  }
  for (int i = 0; i < jointGroupActivationProb.size(); i++) {
    jointGroupActivationProb[i] *= normFactor;
  }
}

//! Helper to compute similarity of each InteractionGraph in vSetIGs to given InteractionGraph ig
vecf computeSimilarities(const vec<std::shared_ptr<InteractionGraph>>& vSetIGs,
                         const InteractionGraph& ig) {
  const similarity::InteractionGraphSimilarity simIG;
  const int numIGs = static_cast<int>(vSetIGs.size());
  vecf similarities(numIGs);
  #pragma omp parallel for
  for (int i = 0; i < numIGs; ++i) {
    similarities[i] = simIG.sim(ig, *vSetIGs[i]);
  }
  return similarities;
}

float InteractionSet::similarity(const InteractionGraph& ig) const {
  const vecf similarities = computeSimilarities(interactionGraphs[ig.getPartType()], ig);
  const float sumSim = accumulate(similarities.begin(), similarities.end(), 0.f),
              avgSim = sumSim / similarities.size();
  return avgSim;
}

float InteractionSet::similarity(const InteractionGraph& ig, float nearestNeighborPercent,
                                 int maxNumNeighbors) const {
  if (interactionGraphs.empty()) {
    SG_LOG_WARN << "Cannot computed nearest neighbor without interactions graphs for iset " << id;
    return 0.0f;
  }
  vecf similarities = computeSimilarities(interactionGraphs[ig.getPartType()], ig);
  sort(similarities.begin(), similarities.end(), std::greater<float>());
  const size_t maxNNs = 50;  // TODO(MS): Parameterize
  const size_t numNNs = std::min(static_cast<size_t>(ceil(similarities.size() * nearestNeighborPercent)), maxNNs);
  //cout << "numNNs=" << numNNs << endl;
  float nnAvgSim = 0;
  for (size_t i = 0; i < numNNs; ++i) {
    nnAvgSim += similarities[i];
  }
  nnAvgSim /= numNNs;
  return nnAvgSim;
}

float InteractionSet::protoSimilarity(const InteractionGraph& ig, bool useJointWeights) const {
  similarity::IGSimParams params;
  params.useJointWeights = useJointWeights;
  params.pJointGroupWeights = &jointGroupWeights;
  const similarity::ProtoToInteractionGraphSimilarity simPIG(params);
  if (protoInteractionGraph == nullptr) { return 0.f; }
  return simPIG.sim(*protoInteractionGraph, ig);
}

void InteractionSet::populateVerbNounsFromId() {
  verbNouns.clear();
  nouns.clear();
  verbs.clear();

  VerbNoun::getVerbNouns(id, &verbNouns);
  for (const VerbNoun& vn : verbNouns) {
    nouns.insert(vn.noun);
    verbs.insert(vn.verb);
  }
}

std::shared_ptr<WeightedPIG> InteractionSet::getWeightedPig() const {
  if (m_weightedPig) {
    if (&m_weightedPig->pig == protoInteractionGraph.get()) {
      return m_weightedPig;
    } else {
      m_weightedPig = nullptr;
    }
  }
  if (protoInteractionGraph != nullptr) {
    m_weightedPig = getWeightedPig(protoInteractionGraph->getPartType(),
                                   protoInteractionGraph->getPigType());
    if (m_weightedPig == nullptr) {
      m_weightedPig = std::make_shared<WeightedPIG>(*protoInteractionGraph, jointGroupWeights);
    }
  } 
  return m_weightedPig;
}

std::shared_ptr<WeightedPIG> InteractionSet::getWeightedPig(const PartType& pt, const string& pigType) const {
  if (m_weightedPigs.empty()) {
    m_weightedPigs.resize(PartType::kPartTypeCount);
  }
  auto& map = m_weightedPigs.at(pt);
  if (map.count(pigType) == 0) {
    if (pt < pigs.size() && pigs[pt].count(pigType) > 0) {
      const std::shared_ptr<PIG> pig = pigs[pt].at(pigType);
      map[pigType] = std::make_shared<WeightedPIG>(*pig, jointGroupWeights);
    }
  }
  return map[pigType];
}

}  // namespace interaction
}  // namespace sg
