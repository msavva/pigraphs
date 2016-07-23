#include "common.h"  // NOLINT

#include "interaction/ProtoInteractionGraph.h"

#include <boost/graph/graphml.hpp>
#include <ext-boost/serialization.h>

#pragma warning(disable:4267)
#include <boost/graph/adj_list_serialize.hpp>
#pragma warning(default:4267)
// pragmas to disable type conversion warning spewed by adj_list_serialize header

#include "interaction/InteractionGraph.h"
#include "io/io.h"
#include "math/math.h"
#include "segmentation/MeshSegment.h"

namespace sg {
namespace interaction {

const int ProtoInteractionGraph::kComIdx = kSkelParams.kNumJoints;
const size_t ProtoInteractionGraph::kVertsSize = kSkelParams.kNumJoints + 1;

template <typename ProtoMemberT>
void initProtoMember(const FeatType type, bool setId, const string label, ProtoMemberT* out) {
  const FeatDesc& featDesc = kFeats.at(type);
  if (setId) {
    if (label.empty()) {
      out->id = featDesc.id;
    } else {
      out->id = featDesc.id + ":" + label;
    }
  }
  if (out->label.empty()) { out->label = label; }  // Hmm, this hacky fix should be handled by cleaner logic
  out->type = type;
  out->interactionCount = 0;
  out->count = 0;
  out->feats.resize(featDesc.numDims);
  for (int iDim = 0; iDim < featDesc.numDims; iDim++) {
    const FeatDim& featDim = featDesc.dims[iDim];
    out->feats[iDim] = stats::Histogram<float>(featDim.range, featDim.numBins, featDim.id, featDim.isAngle);
  }
}

BOOST_BINARY_SERIALIZABLE_IMPL(ProtoInteractionGraph)

ProtoInteractionGraph::ProtoInteractionGraph(const string& id) : m_skel(this) {
  auto& graphBundle = getGraphBundle();
  graphBundle.id = id;    // Set identifier
  graphBundle.count = 0;  // Initialize overall count

  // Last vert/edge is for com verts/edges
  m_verts.resize(PIG::kVertsSize);
  m_edges.resize(PIG::kVertsSize);
  for (int iJoint = 0; iJoint < kSkelParams.kNumJoints; ++iJoint) {
    auto& protoJoint = getVertexBundle(m_skel.jointVertex(iJoint));
    protoJoint.interactionCount = 0;
    protoJoint.count = 0;
  }

  for (int iBone = 0; iBone < kSkelParams.kNumBones; ++iBone) {
    auto& protoBone = getEdgeBundle(m_skel.boneEdge(iBone));
    protoBone.interactionCount = 0;
    protoBone.count = 0;
  }

  auto& comNode = getVertexBundle(m_skel.comVertex());
  comNode.interactionCount = 0;
  comNode.count = 0;
}

void ProtoInteractionGraph::writeGraphML(ostream& os) {  // NOLINT
  boost::dynamic_properties dp;
  using boost::get;
  dp.property("id",     get(&ProtoInteractionNode::id, m_G));
  dp.property("label",  get(&ProtoInteractionNode::label, m_G));
  dp.property("count",  get(&ProtoInteractionNode::count, m_G));
  dp.property("feats",  get(&ProtoInteractionNode::feats, m_G));
  dp.property("id",     get(&ProtoInteractionLink::id, m_G));
  dp.property("label",  get(&ProtoInteractionLink::label, m_G));
  dp.property("count",  get(&ProtoInteractionLink::count, m_G));
  dp.property("feats",  get(&ProtoInteractionLink::feats, m_G));
  write_graphml(os, m_G, dp);
}

void ProtoInteractionGraph::writeGraphML(const string& filename) {
  io::ensurePathToFileExists(filename);
  writeGraphML(ofstream(filename));
}

ProtoInteractionNode& ProtoInteractionGraph::getProtoCom() {
  auto& protoV = getVertexBundle(m_skel.comVertex());
  if (protoV.feats.size() == 0) {
    // ID is set when when m_skel is initialized
    initProtoMember(kCOM, false, "com", &protoV);
  }
  return protoV;
}
const ProtoInteractionNode& ProtoInteractionGraph::getProtoCom() const {
  return getVertexBundle(m_skel.comVertex());
}

ProtoInteractionNode& ProtoInteractionGraph::getProtoJoint(const int iJoint) {
  auto& protoJoint = getVertexBundle(m_skel.jointVertex(iJoint));
  if (protoJoint.feats.size() == 0) {
    // ID is set when when m_skel is initialized
    initProtoMember(kJoint, false, kSkelParams.jointName(iJoint), &protoJoint);
    protoJoint.jointId = iJoint;
  }
  return protoJoint;
}
const ProtoInteractionNode& ProtoInteractionGraph::getProtoJoint(const int iJoint) const {
  return getVertexBundle(m_skel.jointVertex(iJoint));
}

ProtoInteractionLink& ProtoInteractionGraph::getProtoBone(const int iBone) {
  const edge e = m_skel.boneEdge(iBone);
  auto& protoBone = getEdgeBundle(e);
  if (protoBone.feats.size() == 0) {
    // ID is set when when m_skel is initialized
    const auto j0 = kSkelParams.kBones[iBone][0], j1 = kSkelParams.kBones[iBone][1];
    const string boneLabel = kSkelParams.jointName(j0) + "-" + kSkelParams.jointName(j1);
    initProtoMember(kBone, false, boneLabel, &protoBone);
  }
  return protoBone;
}
const ProtoInteractionLink& ProtoInteractionGraph::getProtoBone(const int iBone) const {
  const edge e = m_skel.boneEdge(iBone);
  return getEdgeBundle(e);
}

ProtoInteractionGraph::ProtoInteractionPair ProtoInteractionGraph::getProtoPair(const int iJoint, const string& label) {
  assert(iJoint >=0 && iJoint < m_verts.size());
  assert(iJoint >=0 && iJoint < m_edges.size());

  vertex v; ProtoInteractionNode* pNode;
  const string bodyPartName = (iJoint == kComIdx)? "com": kSkelParams.jointName(iJoint);
  if (m_verts[iJoint].count(label) == 0) {  // initialize proto node
    v = addVertex();
    m_verts[iJoint][label] = v;
    pNode = &getVertexBundle(v);
    initProtoMember(kSeg, true, label, pNode);
    pNode->jointId = -1;  // Not a joint
    pNode->id = pNode->id + ":" + bodyPartName;
  } else {  // retrieve proto node
    v = m_verts[iJoint][label];
    pNode = &getVertexBundle(v);
  }

  ProtoInteractionLink* pLink;
  if (m_edges[iJoint].count(label) == 0) {  // initialize proto link
    FeatType ft = kContact;
    if (iJoint == kSkelParams.kGazeJoint) {
      ft = kGaze;
    } else if (iJoint == kComIdx) {
      ft = kCOMLink;
    }
    const vertex vSource = (iJoint == kComIdx)? m_skel.comVertex() : m_skel.jointVertex(iJoint);
    const edge e = addEdge(vSource, v);
    m_edges[iJoint][label] = e;
    pLink = &getEdgeBundle(e);
    const string linkLabel = bodyPartName + "-" + label;
    initProtoMember(ft, true, linkLabel, pLink);
  } else {  // retrieve proto link
    pLink = &getEdgeBundle(m_edges[iJoint][label]);
  }

  return {pLink, pNode};
}

ProtoInteractionGraph::ProtoInteractionPair ProtoInteractionGraph::getProtoPairForCom(const string& label) {
  return getProtoPair(kComIdx, label);
}

ProtoInteractionGraphAggregator::ProtoInteractionGraphAggregator(const string& aggStratId) {
  const auto it = find(kAggregationStrategyIds.begin(), kAggregationStrategyIds.end(), aggStratId);
  if (it == kAggregationStrategyIds.end()) {
    cerr << "[ProtoInteractionGraphAggregator] unknown aggregation strategy id: " << aggStratId << endl;
    exit(-1);
  }
  m_aggStrat = static_cast<AggregationStrategy>(distance(kAggregationStrategyIds.begin(), it));
}

// Helper function for aggregating links
void ProtoInteractionGraphAggregator::aggregateByTargetVerb(const IG& ig, 
                                                            int iJoint,
                                                            const vec<PIG::edge>& igEdges,
                                                            PIG* pig) const {
  string nodeLabel = kAnyLabelId;
  set<string> seenLabels;
  for (const IG::edge e : igEdges) {  // Accumulate all observed contacts for this joint
    const InteractionLink& igLink = ig.getEdgeBundle(e);
    const InteractionNode& igSeg = ig.getVertexBundle(e.m_target);

    // label segments for each verb that is targetedBy for this segment
    const vec<string> verbs = igSeg.getTargetedByVerbs();
    for (const string& verb : verbs) {
      nodeLabel = verb;
      PIG::ProtoInteractionPair protoPair = pig->getProtoPair(iJoint, nodeLabel);
      protoPair.link->count++;
      stats::util::addVecToVecHistogram(igLink.feats, &protoPair.link->feats);
      protoPair.node->count++;
      stats::util::addVecToVecHistogram(igSeg.feats, &protoPair.node->feats);

      if (seenLabels.count(nodeLabel) == 0) {
        protoPair.link->interactionCount++;
        protoPair.node->interactionCount++;
        seenLabels.insert(nodeLabel);
      }
    }
  }  
}

void ProtoInteractionGraphAggregator::aggregateByLabel(const IG& ig, 
                                                       int iJoint,
                                                       const vec<PIG::edge>& igEdges,
                                                       PIG* pig) const {
  string nodeLabel = kAnyLabelId;
  set<string> seenLabels;
  for (const IG::edge e : igEdges) {  // Accumulate all observed contacts for this joint
    const InteractionLink& igLink = ig.getEdgeBundle(e);
    const InteractionNode& igSeg = ig.getVertexBundle(e.m_target);

    if (m_aggStrat == kPerCategoryProtoNodes || m_aggStrat == kPerPartProtoNodes) {
      if (igSeg.label.size() > 0) {
        if (m_aggStrat == kPerCategoryProtoNodes) {
          nodeLabel = util::tokenize(igSeg.label, ":")[0];
        } else {
          nodeLabel = igSeg.label;
        }
      } else {
        nodeLabel = kAnyLabelId;
      }
    } else {
      nodeLabel = kAnyLabelId;
    }

    PIG::ProtoInteractionPair protoPair = pig->getProtoPair(iJoint, nodeLabel);
    protoPair.link->count++;
    stats::util::addVecToVecHistogram(igLink.feats, &protoPair.link->feats);
    protoPair.node->count++;
    stats::util::addVecToVecHistogram(igSeg.feats, &protoPair.node->feats);

    if (seenLabels.count(nodeLabel) == 0) {
      protoPair.link->interactionCount++;
      protoPair.node->interactionCount++;
      seenLabels.insert(nodeLabel);
    }
  }
}

void ProtoInteractionGraphAggregator::addInteractionGraph(const IG& ig, PIG* pig) const {
  // Aggregate Graph-level features
  auto& graphBundle = pig->getGraphBundle();
  graphBundle.count++;
  const geo::Vec3f& com = ig.getGraphBundle().COM;
  const geo::Vec3f& gazeDir = ig.getGraphBundle().gazeDir;
  graphBundle.zCOMHist.add(math::sigmoid<float>(com.z()));  // Accumulate center of mass
  map<string, set<ConstPartPtr>> segsByLabel;
  size_t totalLabeledSegs = 
    (m_aggStrat == kPerVerbTargetProtoNodes)? ig.getPartsByTargetedBy(&segsByLabel, true) : ig.getPartsByLabel(&segsByLabel);
  vecf comLinkFeats;
  const auto& comLinkFeatsIndex = interaction::kFeats.at(interaction::FeatType::kCOMLink).getFeatIndex();
  const int iR = comLinkFeatsIndex.indexOf("rDist");
  const int iZ = comLinkFeatsIndex.indexOf("vOffset");
  const int iAng = comLinkFeatsIndex.indexOf("angDxy");
  for (const auto it : segsByLabel) {
    // TODO: Rename noun to be label
    const string& noun = it.first;
    const set<ConstPartPtr>& segs = it.second;
    graphBundle.nouns.inc(noun);
    graphBundle.nounsCount[noun].inc(segs.size());

    const float pi = math::constants::PIf;
    graphBundle.rDistHist.insert(std::make_pair(noun, stats::Histogram<float>(r01, 10, "rDist")));
    graphBundle.vOffsetHist.insert(std::make_pair(noun, stats::Histogram<float>(rm11, 10, "vOffset")));
    graphBundle.angDxyHist.insert(std::make_pair(noun, stats::Histogram<float>(r02p, 36, "angDxy", true)));
    for (const ConstPartPtr& seg : segs) {
      const geo::Vec3f closestPt = seg->closestPoint(com);
      const geo::Vec3f segNormal = geo::vec3f(seg->dominantNormal());
      relFeats(com, gazeDir, closestPt, segNormal, &comLinkFeats);
      graphBundle.rDistHist.at(noun).add(comLinkFeats[iR]);
      graphBundle.vOffsetHist.at(noun).add(comLinkFeats[iZ]);
      graphBundle.angDxyHist.at(noun).add(comLinkFeats[iAng]);
    }
  }
  for (const auto& vn : ig.getGraphBundle().verbNouns) {
    graphBundle.verbs.inc(vn.verb);
  }

  // For each bone, aggregate joint-to-joint features
  for (int iBone = 0; iBone < kSkelParams.kNumBones; iBone++) {
    auto& protoBone = pig->getProtoBone(iBone);
    protoBone.count++;
    protoBone.interactionCount++;
    const auto& igBone = ig.getBoneEdgeBundle(iBone);
    stats::util::addVecToVecHistogram(igBone.feats, &protoBone.feats);
  }

  // For each joint, aggregate joint features
  for (int iJoint = 0; iJoint < kSkelParams.kNumJoints; ++iJoint) {
    auto& protoJoint = pig->getProtoJoint(iJoint);
    protoJoint.interactionCount++;

    // Skip fake gaze joint... we are only doing real joints
    if (iJoint == kSkelParams.kGazeJoint) { continue; }

    auto& igJoint = ig.getJointVertexBundle(iJoint);
    protoJoint.count++;
    stats::util::addVecToVecHistogram(igJoint.feats, &protoJoint.feats);
  }

  if (m_aggStrat == kPerVerbTargetProtoNodes) {
    // Retrieve each contact/gaze edge, aggregate edge and segment features into proto-contact/gaze link and node
    for (int iJoint = 0; iJoint < kSkelParams.kNumJoints; ++iJoint) {
      // Get either appropriate joint contact edges, or gaze edges for last index
      const auto& igEdges = (iJoint == kSkelParams.kGazeJoint) ? ig.getGazeEdges() : ig.getContactEdges(iJoint);
      aggregateByTargetVerb(ig, iJoint, igEdges, pig);
    }
  } else {
    // Retrieve each contact/gaze edge, aggregate edge and segment features into proto-contact/gaze link and node
    for (int iJoint = 0; iJoint < kSkelParams.kNumJoints; ++iJoint) {
      // Get either appropriate joint contact edges, or gaze edges for last index
      const auto& igEdges = (iJoint == kSkelParams.kGazeJoint) ? ig.getGazeEdges() : ig.getContactEdges(iJoint);
      aggregateByLabel(ig, iJoint, igEdges, pig);
    }
  }

  // Aggregate com features
  auto& protoCom = pig->getProtoCom();
  protoCom.interactionCount++;

  auto& igCom = ig.getComVertexBundle();
  protoCom.count++;
  stats::util::addVecToVecHistogram(igCom.feats, &protoCom.feats);

  const auto& comEdges = ig.getComEdges();
  if (m_aggStrat == kPerVerbTargetProtoNodes) {
    aggregateByTargetVerb(ig, PIG::kComIdx, comEdges, pig);
  } else {
    aggregateByLabel(ig, PIG::kComIdx, comEdges, pig);
  }


  // TODO(MS): Retrieve each spatial edge and aggregate features
}

void ProtoInteractionGraph::clear() {
  auto& graphBundle = getGraphBundle();
  graphBundle.clear();

  // Clear nodes/edges of the skeleton
  for (int iBone = 0; iBone < kSkelParams.kNumBones; iBone++) {
    auto& protoBone = getProtoBone(iBone);
    protoBone.clear();
  }

  for (int iJoint = 0; iJoint < kSkelParams.kNumJoints; ++iJoint) {
    auto& protoJoint = getProtoJoint(iJoint);
    protoJoint.clear();
  }

  // Clear com 
  auto& protoCom = getProtoCom();
  protoCom.clear();

  // Clear vertices and remove edges that are not part of the skeleton
  for (auto& m : m_verts) {
    for (auto& pair : m) {
      remove_vertex(pair.second, m_G);
    }
  }
  
  for (auto& edge : m_edges) {
    edge.clear();
  }
  for (auto& vert : m_verts) {
    vert.clear();
  }
}

template <typename T>
bool relabel(map<string,T>& h, const string& from, const string &to) {
  if (h.count(from) > 0) {
    h[to] = h.at(from);
    h.erase(from);
    return true;
  } else {
    return false;
  }
}

std::shared_ptr<ProtoInteractionGraph> ProtoInteractionGraph::retarget(const string& fromLabel, const string& toLabel) {
  const auto& graphBundle = getGraphBundle();
  std::shared_ptr<ProtoInteractionGraph> pPIG =
    std::make_shared<ProtoInteractionGraph>(graphBundle.id);

  // Copy pig
  vec<vec<double>> jweights;
  vec<std::reference_wrapper<const PIG>> pigs;
  pigs.push_back(*this);
  combine(pigs, jweights, kPIGCombineJointsMax, pPIG.get());

  // Retarget global features
  SG_LOG_INFO << "Retarget global features from " << fromLabel << " to " << toLabel;
  // Rename target fromLabel to toLabel
  // Global and link features
  auto& retargetedGraphBundle = pPIG->getGraphBundle();
  if (retargetedGraphBundle.nouns.count(fromLabel) > 0) {
    retargetedGraphBundle.nouns.set(toLabel, retargetedGraphBundle.nouns.count(fromLabel));
    retargetedGraphBundle.nouns.set(fromLabel, 0);
  }
  relabel(retargetedGraphBundle.nounsCount, fromLabel, toLabel);
  relabel(retargetedGraphBundle.rDistHist, fromLabel, toLabel);
  relabel(retargetedGraphBundle.angDxyHist, fromLabel, toLabel);
  relabel(retargetedGraphBundle.vOffsetHist, fromLabel, toLabel);

  // For each bone, copy joint-to-joint features
  // For each joint, copy joint features

  // Retrieve each contact/gaze edge, copy edge and segment features
  // into retargeted proto-contact/gaze link and node

  for (int iJoint = 0; iJoint < kVertsSize; ++iJoint) {
    // Get appropriate joint contact edges from pig (includes gaze)
    auto& pigEdges = pPIG->m_edges[iJoint];
    auto& pigVerts = pPIG->m_verts[iJoint];
    SG_LOG_INFO << "Retarget contact/gaze features for joint " 
      << iJoint << "/" << kSkelParams.kNumJoints << " with "
      << pigEdges.size() << " edges and " << pigVerts.size() << " verts";
    // Retarget gaze and contact edges
    if (relabel(pigEdges, fromLabel, toLabel)) {
      SG_LOG_INFO << "Relabeled edge from " << fromLabel << " to " << toLabel;
      auto& pigEdgeBundle = pPIG->getEdgeBundle(pigEdges.at(toLabel));
    }
    if (relabel(pigVerts, fromLabel, toLabel)) {
      SG_LOG_INFO << "Relabeled node from " << fromLabel << " to " << toLabel;
      auto& pigSegNodeBundle = pPIG->getVertexBundle(pigVerts.at(toLabel));
      if (pigSegNodeBundle.label == fromLabel) {
        pigSegNodeBundle.label = toLabel;
      } 
    }
  }
  return pPIG;
}

void ProtoInteractionGraph::combine(const vec<std::reference_wrapper<const ProtoInteractionGraph>> pigs,
                                    const vec<vec<double>>& jointGroupWeights,
                                    const PIGCombineStrategy pigCombineStrategy,
                                    ProtoInteractionGraph* pCombined) {
  // Clears the combined graph
  pCombined->clear();

  auto& combinedGraphBundle = pCombined->getGraphBundle();

  // Dummy weights (placeholder when useJointGroupWeights is false
  vec<double> dummyWeights;
  bool useJointGroupWeights = jointGroupWeights.size() > 0;
  if (useJointGroupWeights) {
    assert(pigs.size() == jointGroupWeights.size());
  }

  vec<int> bestPIGforJoint(kSkelParams.kNumJoints);
  if (useJointGroupWeights && pigCombineStrategy == kPIGCombineJointsMax) {
    for (int iJoint = 0; iJoint < kSkelParams.kNumJoints; ++iJoint) {
      bestPIGforJoint[iJoint] = 0;
    }
    for (size_t i = 1; i < jointGroupWeights.size(); ++i) {
      const vec<double>& weights = jointGroupWeights.at(i);
      assert(weights.size() == kSkelParams.kNumJoints);
      for (int iJoint = 0; iJoint < kSkelParams.kNumJoints; ++iJoint) {
        const double w = weights[iJoint];
        const int bestPIG = bestPIGforJoint[iJoint];
        const double bestw = jointGroupWeights[bestPIG][iJoint];
        if (w > bestw) {
          bestPIGforJoint[iJoint] = static_cast<int>(i);
        }
      } 
    }
  }

  const auto getJointWeightFn = [&](const vec<double>& weights, int iPIG, int iJoint) {
    double jointGroupWeight = 1.0;
    if (useJointGroupWeights) {
      if (pigCombineStrategy == kPIGCombineJointsMax) {
        jointGroupWeight = (iPIG == bestPIGforJoint[iJoint]) ? 1.0 : 0.00;
      } else {
        jointGroupWeight = weights.at(iJoint);
      }
    }
    return jointGroupWeight;
  };

  // Iterate over pigs
  for (int i = 0; i < pigs.size(); i++) {
    const ProtoInteractionGraph& pig = pigs.at(i);
    const vec<double>& weights = (useJointGroupWeights)? jointGroupWeights.at(i) : dummyWeights;
    if (useJointGroupWeights) {
      assert(weights.size() == kSkelParams.kNumJoints);
    }

    // Aggregate global features
    SG_LOG_INFO << "Aggregate global features";
    auto& graphBundle = pig.getGraphBundle();
    combinedGraphBundle.count += graphBundle.count;

    // Accumulate center of mass
    combinedGraphBundle.zCOMHist.addAll(graphBundle.zCOMHist);
    stats::util::addVecHistogramMapToVecHistogramMap(graphBundle.rDistHist, &combinedGraphBundle.rDistHist);
    stats::util::addVecHistogramMapToVecHistogramMap(graphBundle.angDxyHist, &combinedGraphBundle.angDxyHist);
    stats::util::addVecHistogramMapToVecHistogramMap(graphBundle.vOffsetHist, &combinedGraphBundle.vOffsetHist);

    combinedGraphBundle.nouns.add(graphBundle.nouns);
    for (const auto& pair : graphBundle.nounsCount) {
      combinedGraphBundle.nounsCount[pair.first].add(pair.second);
    }

    combinedGraphBundle.verbs.add(graphBundle.verbs);
    // Accumulate nodes and edge distributions

    // Aggregate com features
    bool aggregateComFeatures = true;
    if (aggregateComFeatures) {
      SG_LOG_INFO << "Aggregate com features";
      auto& combinedProtoJoint = pCombined->getProtoCom();
      auto& protoJoint = pig.getProtoCom();
      combinedProtoJoint.count += protoJoint.count;
      combinedProtoJoint.interactionCount += protoJoint.interactionCount;
      stats::util::addVecHistogramToVecHistogram(protoJoint.feats, &combinedProtoJoint.feats);

      // Get interacting segment from pig
      const auto& pigVerts = pig.m_verts.at(kComIdx);
      // Aggregate segment features
      for (const auto& pigVert : pigVerts) {
        const auto& pigNodeBundle = pig.getVertexBundle(pigVert.second);
        const auto& combinedProtoPair = pCombined->getProtoPair(kComIdx, pigNodeBundle.label);
        combinedProtoPair.node->count += pigNodeBundle.count;
        combinedProtoPair.node->interactionCount += pigNodeBundle.interactionCount;
        stats::util::addVecHistogramToVecHistogram(pigNodeBundle.feats, &combinedProtoPair.node->feats);
      }

      // Get interacting segment link from pig
      const auto& pigEdges = pig.m_edges.at(kComIdx);
      // Aggregate segment features
      for (const auto& pigEdge : pigEdges) {
        const auto& pigEdgeBundle = pig.getEdgeBundle(pigEdge.second);
        const auto& pigSegNodeBundle = pig.getVertexBundle(pigEdge.second.m_target);
        const auto& combinedProtoPair = pCombined->getProtoPair(kComIdx, pigSegNodeBundle.label);
        combinedProtoPair.link->count += pigEdgeBundle.count;
        combinedProtoPair.link->interactionCount += pigEdgeBundle.interactionCount;
        // TODO: Use joint weights
        stats::util::addVecHistogramToVecHistogram(pigEdgeBundle.feats, &combinedProtoPair.link->feats);
      }
    }

    // For each bone, aggregate joint-to-joint features
    SG_LOG_INFO << "Aggregate bone features";
    for (int iBone = 0; iBone < kSkelParams.kNumBones; iBone++) {
      auto& combinedProtoBone = pCombined->getProtoBone(iBone);
      auto& protoBone = pig.getProtoBone(iBone);
      combinedProtoBone.count += protoBone.count;
      combinedProtoBone.interactionCount += protoBone.interactionCount;
      stats::util::addVecHistogramToVecHistogram(protoBone.feats, &combinedProtoBone.feats);
    }

    // For each joint, aggregate joint features
    SG_LOG_INFO << "Aggregate joint features";
    for (int iJoint = 0; iJoint < kSkelParams.kNumJoints; ++iJoint) {
      double jointGroupWeight = getJointWeightFn(weights, i, iJoint);

      if (jointGroupWeight > 0) {
        auto& combinedProtoJoint = pCombined->getProtoJoint(iJoint);
        auto& protoJoint = pig.getProtoJoint(iJoint);
        combinedProtoJoint.count += protoJoint.count;
        combinedProtoJoint.interactionCount += protoJoint.interactionCount;

        // Skip fake gaze joint... we are only doing real joints
        if (iJoint == kSkelParams.kGazeJoint) { continue; }

        // TODO: Use joint weights
        stats::util::addVecHistogramToVecHistogram(protoJoint.feats, &combinedProtoJoint.feats);
      }
    }

    // Retrieve each contact/gaze edge, aggregate edge and segment features into proto-contact/gaze link and node
    SG_LOG_INFO << "Aggregate contact/gaze features";
    for (int iJoint = 0; iJoint < kSkelParams.kNumJoints; ++iJoint) {
      // SG_LOG_INFO << "Aggregate contact/gaze features for joint " << iJoint << "/" << kSkelParams.kNumJoints;
      double jointGroupWeight = getJointWeightFn(weights, i, iJoint);
      if (jointGroupWeight > 0) {
        // Get appropriate joint contact edges from pig (includes gaze)
        const auto& pigEdges = pig.m_edges.at(iJoint);
        // Aggregate gaze and contact edges
        for (const auto& pigEdge : pigEdges) {
          const auto& pigEdgeBundle = pig.getEdgeBundle(pigEdge.second);
          const auto& pigSegNodeBundle = pig.getVertexBundle(pigEdge.second.m_target);
          const auto& combinedProtoPair = pCombined->getProtoPair(iJoint, pigSegNodeBundle.label);
          combinedProtoPair.link->count += pigEdgeBundle.count;
          combinedProtoPair.link->interactionCount += pigEdgeBundle.interactionCount;
          // TODO: Use joint weights - TODO: Fix type issue
          stats::util::addVecHistogramToVecHistogram(
            pigEdgeBundle.feats, &combinedProtoPair.link->feats);
        }
      }
    }

    // Retrieve each segment node and aggregate features
    SG_LOG_INFO << "Aggregate segment features";
    for (int iJoint = 0; iJoint < kSkelParams.kNumJoints; ++iJoint) {
      double jointGroupWeight = getJointWeightFn(weights, i, iJoint);
      if (jointGroupWeight > 0.0) {
        // Get interacting segment from pig
        const auto& pigVerts = pig.m_verts.at(iJoint);
        // Aggregate segment features
        for (const auto& pigVert : pigVerts) {
          const auto& pigNodeBundle = pig.getVertexBundle(pigVert.second);
          const auto& combinedProtoPair = pCombined->getProtoPair(iJoint, pigNodeBundle.label);
          combinedProtoPair.node->count += pigNodeBundle.count;
          combinedProtoPair.node->interactionCount += pigNodeBundle.interactionCount;
          // TODO: Use joint weights - TODO: Fix type issue
          stats::util::addVecHistogramToVecHistogram(
            pigNodeBundle.feats, &combinedProtoPair.node->feats);
        }
      }
    }

    // TODO(MS): Retrieve each spatial edge and aggregate features
  }
}

template<typename numT>
void dumpHist(ofstream& ofs, const string& tag, const stats::Histogram<numT>& hist,
std::function<numT(numT)> convertFn = [](numT x) { return x; }) {
  numT mean = convertFn(hist.mean());
  for (int i = 0; i < hist.getNumBins(); ++i) {
    const auto& interval = hist.binInterval(i);
    numT a = convertFn(interval.min);
    numT b = convertFn(interval.max);
    ofs << tag << "," << hist.id() << "," << mean << "," << i << "," << a << "," << b << "," 
      << hist.binWeight(i) << "," << hist.count(i) << endl;
  }
}

template<typename numT>
void dumpHistSummary(ofstream& ofs, const string& tag, const stats::Histogram<numT>& hist,
std::function<numT(numT)> convertFn = [](numT x) { return x; }) {
  numT mean = convertFn(hist.mean());
  ofs << tag << "," << hist.id() << "," << mean << "," 
    << hist.getNumBins() << "," << hist.totalCount() << endl;
}

template<typename numT>
void dumpHists(ofstream& ofs, const string& tag, const map<string, stats::Histogram<numT>>& hists,
std::function<numT(numT)> convertFn = [] (numT x) { return x; }) {
  for (auto& it : hists) {
    dumpHist(ofs, tag + "-" + it.first, it.second, convertFn);
  }
}

typedef std::function<void(ofstream&, const string&,
  const stats::Histogram<float>&, std::function<float(float)> )> WriteHistFn;
typedef std::function<float(float)> ConvertFn;
WriteHistFn dumpHistFn = dumpHist<float>;
WriteHistFn dumpHistSummaryFn = dumpHistSummary<float>;
ConvertFn identityFn = [] (float x) { return x; };
ConvertFn invSigmoidFn = [](float f) { return static_cast<float>(math::invSigmoid(f)); };

void dumpHists(ofstream& ofs, const string& tag, const map<string, stats::Histogram<float>>& hists,
               WriteHistFn writeHistFn,
               ConvertFn convertFn = identityFn) {
  for (auto& it : hists) {
    writeHistFn(ofs, tag + "-" + it.first, it.second, convertFn);
  }
}

void dumpGraphBundleFeats(ofstream& ofs, const string& tag, const ProtoInteractionGraph::GraphBundleT& g,
                          WriteHistFn writeHistFn) {
  writeHistFn(ofs, tag + "-zCom", g.zCOMHist, identityFn);
  dumpHists(ofs, tag + "-ang", g.angDxyHist, writeHistFn);
  dumpHists(ofs, tag + "-rDistHist", g.rDistHist, writeHistFn);
  dumpHists(ofs, tag + "-vOffsetHist", g.vOffsetHist, writeHistFn);
}

void dumpEdgeBundleFeats(ofstream& ofs, const string& tag,
                         const ProtoInteractionGraph::EdgeBundleT& e,
                         WriteHistFn writeHistFn) {
  const auto& featDesc = kFeats.at(e.type);
  for (int i = 0; i < e.feats.size(); ++i) {
    const auto& feat = e.feats[i];
    const auto& featDimDesc = featDesc.dims[i];
    string label = e.id;
    if (label.empty()) {
      label = label;
    }
    if (featDimDesc.useSig) {
      // Invert sigmoid
      writeHistFn(ofs, tag + "-" + label, feat, invSigmoidFn);
    } else {
      writeHistFn(ofs, tag + "-" + label, feat, identityFn);
    }
  }
}

void dumpVertexBundleFeats(ofstream& ofs, const string& tag,
                           const ProtoInteractionGraph::VertexBundleT& e,
                           WriteHistFn writeHistFn) {
  const auto& featDesc = kFeats.at(e.type);
  for (int i = 0; i < e.feats.size(); ++i) {
    const auto& feat = e.feats[i];
    const auto& featDimDesc = featDesc.dims[i];
    string label = e.id;
    if (label.empty()) {
      label = label;
    }
    if (featDimDesc.useSig) {
      // Invert sigmoid
      writeHistFn(ofs, tag + "-" + label, feat, invSigmoidFn);
    } else {
      writeHistFn(ofs, tag + "-" + label, feat, identityFn);
    }
  }
}

void dumpEdgeFeats(ofstream& ofs, const string& tag,
                   const ProtoInteractionGraph& g,
                   const map<string, ProtoInteractionGraph::edge>& edges,
                   WriteHistFn writeHistFn) {
  for (auto& it : edges) {
    const auto& edgeBundle = g.getEdgeBundle(it.second);
    dumpEdgeBundleFeats(ofs, tag /* + "-" + it.first */, edgeBundle, writeHistFn);
  }
}

bool ProtoInteractionGraph::dumpFeats(const string& file) const {
  io::ensurePathToFileExists(file);
  ofstream ofs(file);
  ofs << "tag1,tag2,mean,binIndex,binMin,binMax,binWeight,binCount" << endl;
  // Go through whole graph and dump feats
  dumpGraphBundleFeats(ofs, "g", getGraphBundle(), dumpHistFn);
  // Go through edges
  for (const auto& edge : m_G.m_edges) {
    dumpEdgeBundleFeats(ofs, "e", edge.m_property, dumpHistFn);
  }
  // Go through nodes
  for (const auto& vert : m_G.m_vertices) {
    dumpVertexBundleFeats(ofs, "v", vert.m_property, dumpHistFn);
  }
  SG_LOG_INFO << "Saved PIG feats to " << file;
  ofs.close();
  return true;
}

bool ProtoInteractionGraph::dumpFeatsSummary(const string& file) const {
  io::ensurePathToFileExists(file);
  ofstream ofs(file);
  ofs << "tag1,tag2,mean,nBins,count" << endl;
  // Go through whole graph and dump feats
  dumpGraphBundleFeats(ofs, "g", getGraphBundle(), dumpHistSummaryFn);
  // Go through edges
  for (const auto& edge : m_G.m_edges) {
    dumpEdgeBundleFeats(ofs, "e", edge.m_property, dumpHistSummaryFn);
  }
  // Go through nodes
  for (const auto& vert : m_G.m_vertices) {
    dumpVertexBundleFeats(ofs, "v", vert.m_property, dumpHistSummaryFn);
  }
  SG_LOG_DEBUG << "Saved PIG feats summary to " << file;
  ofs.close();
  return true;
}

void ProtoInteractionNode::clear() {
  interactionCount = 0;
  count = 0;
  feats.clear();
}

void ProtoInteractionLink::clear() {
  interactionCount = 0;
  count = 0;
  feats.clear();
}

void ProtoInteractionAnnotation::getNounProbs(stats::CounterSd* pNounsProbs) const {
  pNounsProbs->copy(nouns);
  pNounsProbs->normalize();
}

void ProtoInteractionAnnotation::clear() {
  nouns.clear();
  nounsCount.clear();
  count = 0;
  zCOMHist.clear();
  rDistHist.clear();
  vOffsetHist.clear();
  angDxyHist.clear();
}

PigFeatIndices createPigFeatIndices() {
  // Assumes kContact/kGaze/kComLink all uses kRelFeats
  const auto& contactFeatDescs = interaction::kFeats.at(interaction::kContact);
  const auto& contactFeatIndex = contactFeatDescs.getFeatIndex();
  PigFeatIndices pfi;
  pfi.iRelHeight = contactFeatIndex.indexOf("height");
  pfi.iRelRDist = contactFeatIndex.indexOf("rDist");
  pfi.iRelVOffset = contactFeatIndex.indexOf("vOffset");
  pfi.iRelAngDxy = contactFeatIndex.indexOf("angDxy");
  pfi.iRelCosDS = contactFeatIndex.indexOf("cosDS");
  return pfi;
}
const PigFeatIndices ProtoInteractionGraph::kPigFeatIndices = createPigFeatIndices();

void getLinks(const std::function<string(const string&)> getCat,
              const ProtoInteractionGraph& pig,
              const map<string, ProtoInteractionGraph::edge>& pigEdges,
              CategoryPIGLinkMap* pLinksByCategory) {
  for (const auto it : pigEdges) {
    const auto& pigEdge = it.second;
    const ProtoInteractionLink& pigLink = pig.getEdgeBundle(pigEdge);
    const ProtoInteractionNode& pigSegNode = pig.getVertexBundle(pigEdge.m_target);
    const string label = pigSegNode.label;
    const string category = getCat(label);
    (*pLinksByCategory)[category].push_back(pigLink);
  }
}

void WeightedPIG::populateJointLinks(const std::function<string(const string&)> getCat) const {
  jointLinksByCategory.resize(PIG::kVertsSize);
  for (int i = 0; i < jointLinksByCategory.size(); ++i) {
    const auto& pigEdges = pig.m_edges[i];
    getLinks(getCat, pig, pigEdges, &jointLinksByCategory[i]);
  }
}

}  // namespace interaction
}  // namespace sg

//! Required for writing vectors to GraphML
namespace std {
template <typename T> ostream& operator<<(ostream& os, vector<T> v) {
  os << "[";
  for (size_t i = 0; i < v.size(); i++) { os << v[i] << (i == v.size() - 1 ? "" : ","); }
  return os << "]" << endl;
}
template <typename T> istream& operator>>(istream& is, vector<T>& v) {
  // TODO(MS): Implement reading when it becomes necessary
  cerr << "Unimplemented: InteractionGraph operator >> (istream, vec<T>)" << endl;
  return is;
}
}  // namespace std
