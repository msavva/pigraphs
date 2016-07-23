#include "common.h"  // NOLINT

#include "interaction/InteractionGraph.h"

#include <boost/graph/graphml.hpp>

#include "segmentation/Part.h"
#include "io/json.h"

namespace sg {
namespace interaction {

using vertex = InteractionGraph::vertex;
using edge = InteractionGraph::edge;

InteractionGraph::InteractionGraph(const PartType partType,
                                   const Skeleton& skel, 
                                   const Skeleton::PartsByJointPlusGaze& parts,
                                   const std::function<string(ConstPartPtr)>& getPartLabel,
                                   const std::function<string(ConstPartPtr)>& getTargetedBy,
                                   const set<VerbNoun>& verbNouns,
                                   const core::OccupancyGrid* pOccupancyGrid /* = nullptr */) 
                                   : m_skel(this) {
  init(partType, skel, parts, getPartLabel, getTargetedBy, verbNouns, nullptr);
}

void InteractionGraph::init(const PartType partType,
                            const Skeleton& skel, const Skeleton::PartsByJointPlusGaze& parts,
                            const std::function<string(ConstPartPtr)>& getPartLabel,
                            const std::function<string(ConstPartPtr)>& getTargetedBy,
                            const set<VerbNoun>& verbNouns,
                            const core::OccupancyGrid* pOccupancyGrid) {
  // Create aggregated joints
  vec<Joint> joints(kSkelParams.kNumJoints);
  for (int iJoint = 0; iJoint < kSkelParams.kNumJoints; iJoint++) {
    // Skip fake gaze joint... we are only doing actual contact now
    if (iJoint == kSkelParams.kGazeJoint) continue;

    createCombinedJoint(kSkelParams, iJoint, skel, &joints[iJoint]);
  }

  // Initialize global features
  auto& graphBundle = getGraphBundle();
  graphBundle.partType = partType;
  graphBundle.verbNouns = verbNouns;
  graphBundle.COM = geo::vec3f(skel.centerOfMass());
  graphBundle.gazeDir = geo::vec3f(skel.gazeDirection);

  // Create com node features
  bool createComFeats = true;
  if (createComFeats) {
    const auto& com = m_skel.comVertex();
    comFeats(skel, &getVertexBundle(com).feats);

    map<string,set<ConstPartPtr>> partsByLabel;
    for (size_t i = 0; i < Skeleton::kNumJoints + 1; ++i) {
      for (const auto& it : parts[i]) {
        const string label = getPartLabel(it);
        partsByLabel[label].insert(it);
      }
    }
    for (const auto it : partsByLabel) {
      for (const auto pPart : it.second ) {
        const vertex segV = addPartVertex(it.first, pPart);
        const edge comE = addComEdge(segV);
        comLinkFeats(skel, pPart, &getEdgeBundle(comE).feats);
      }
    }
  }

  // For each joint, create contacting segment nodes and hook them up through contact edges
  m_contactEdges.resize(kSkelParams.kNumJoints);
  for (int iJoint = 0; iJoint < kSkelParams.kNumJoints; iJoint++) {
    // Skip fake gaze joint... we are only doing actual contact now
    if (iJoint == kSkelParams.kGazeJoint) continue;

    const vertex jointV = m_skel.jointVertex(iJoint);
    const Joint& joint = joints.at(iJoint);

    jointFeats(skel, joint, &getVertexBundle(jointV).feats);
    if (pOccupancyGrid != nullptr) {
      jointVolumeFeats(*pOccupancyGrid, skel, joint, getVertexBundle(jointV));
    }

    // Make sure parts are not added multiple times for this joint
    uset<ConstPartPtr> addedParts;
    for (size_t iSkelJoint: joint.rawIndices) {
      for (const ConstPartPtr pPart : parts[iSkelJoint]) {
        if (addedParts.count(pPart) == 0) {
          const vertex segV = addPartVertex(getPartLabel(pPart), pPart);
          const edge contactE = addContactEdge(iJoint, segV);
          contactFeats(skel, joint, pPart, &getEdgeBundle(contactE).feats);
          addedParts.insert(pPart);
        }
      }
    }
  }

  // Compute joint-to-joint features for bone edges
  for (int iBone = 0; iBone < kSkelParams.kNumBones; iBone++) {
    InteractionLink& bone = getEdgeBundle(m_skel.boneEdge(iBone));
    const Joint& joint1 = joints.at(kSkelParams.kBones[iBone][0]);
    const Joint& joint2 = joints.at(kSkelParams.kBones[iBone][1]);
    boneFeats(skel, joint1, joint2, &bone.feats);
  }

  // Create gazed segment nodes and hook up
  for (const ConstPartPtr pPart : parts.back()) {
    const vertex segV = addPartVertex(getPartLabel(pPart), pPart);
    const edge gazeE = addGazeEdge(segV);
    gazeFeats(skel, pPart, &getEdgeBundle(gazeE).feats);
  }

  // Go through segments and make sure that their targetBy is populated
  for (const auto& pair : m_partPtrToVertex) {
    InteractionNode& vertexBundle = getVertexBundle(pair.second);
    vertexBundle.targetedBy = getTargetedBy(pair.first);
  }

  //// Create spatial edges for all seg-seg pairs and compute their features
  //vec<pair<PartPtr, vertex>> segVertPairs;
  //util::mapToPairVec(m_PartPtrToVertex, segVertPairs);
  //for (size_t i = 0; i < segVertPairs.size(); ++i) {
  //  const PartPtr& iSeg  = segVertPairs[i].first;
  //  const vertex& iVert = segVertPairs[i].second;
  //  for (size_t j = i + 1; j < segVertPairs.size(); ++j) {
  //    const PartPtr& jSeg  = segVertPairs[j].first;
  //    const vertex& jVert = segVertPairs[j].second;
  //    const edge spatialE = addSpatialEdge(iVert, jVert);
  //    interaction::segToSegFeats(iSeg, jSeg, &getEdgeBundle(spatialE).feats);
  //  }
  //}
}

void InteractionGraph::writeGraphML(ostream& os) {  // NOLINT
  boost::dynamic_properties dp;
  dp.property("id", boost::get(&InteractionNode::id, m_G));
  dp.property("label", boost::get(&InteractionNode::label, m_G));
  dp.property("features", boost::get(&InteractionNode::feats, m_G));
  dp.property("targetedBy", boost::get(&InteractionNode::targetedBy, m_G));
  dp.property("id", boost::get(&InteractionLink::id, m_G));
  dp.property("features", boost::get(&InteractionLink::feats, m_G));
  write_graphml(os, m_G, dp);
}

vertex InteractionGraph::addPartVertex(const string& label, const ConstPartPtr pPart) {
  // Check if Vertex already created for this segment
  if (m_partPtrToVertex.count(pPart) == 0) {
    m_partPtrToVertex[pPart] = addVertex();
  }
  const vertex segV = m_partPtrToVertex[pPart];

  // Populate segment node
  InteractionNode& segNode = getVertexBundle(segV);
  segNode.type = kSeg;
  segNode.id = "seg:" + to_string(pPart->id);
  segNode.jointId = -1;  // No joint associated with segments
  partFeats(pPart, &segNode.feats);
  segNode.label = label;
  return segV;
}

edge InteractionGraph::addContactEdge(const int iJoint, const vertex& segV) {
  const edge jointToSegE = addEdge(m_skel.jointVertex(iJoint), segV);
  InteractionLink& link = getEdgeBundle(jointToSegE);
  link.type = kContact;
  link.id = "contact";
  m_contactEdges[iJoint].push_back(jointToSegE);
  return jointToSegE;
}

edge InteractionGraph::addGazeEdge(const vertex& segV) {
  const vertex gazeV = m_skel.jointVertex(kSkelParams.kGazeJoint);
  const edge jointToSegE = addEdge(gazeV, segV);
  InteractionLink& link = getEdgeBundle(jointToSegE);
  link.type = kGaze;
  link.id = "gaze";
  m_gazeEdges.push_back(jointToSegE);
  return jointToSegE;
}

edge InteractionGraph::addSpatialEdge(const vertex& segU, const vertex& segV) {
  const edge segToSegE = addEdge(segU, segV);
  InteractionLink& link = getEdgeBundle(segToSegE);
  link.type = kSpatial;
  link.id = "spatial";
  m_spatialEdges.push_back(segToSegE);
  return segToSegE;
}

edge InteractionGraph::addComEdge(const vertex& segV) {
  const vertex comV = m_skel.comVertex();
  const edge jointToSegE = addEdge(comV, segV);
  InteractionLink& link = getEdgeBundle(jointToSegE);
  link.type = kCOMLink;
  link.id = "comLink";
  m_comEdges.push_back(jointToSegE);
  return jointToSegE;
}

size_t InteractionGraph::getPartsByLabel(map<string, set<ConstPartPtr>>* out, bool ignoreUnlabeled /* = false */) const {
  out->clear();
  for (const auto segIt: m_partPtrToVertex) {
    const auto PartPtr = segIt.first;
    const auto& vertexBundle = getVertexBundle(segIt.second);
    if (ignoreUnlabeled && vertexBundle.label.empty()) continue;
    (*out)[vertexBundle.label].insert( PartPtr );
  }
  size_t total = 0;
  for (const auto it: (*out) ) {
    total += it.second.size();
  }
  return total;
}

size_t InteractionGraph::getPartsByTargetedBy(map<string, set<ConstPartPtr>>* out, bool ignoreUntargeted /* = false */) const {
  out->clear();
  for (const auto segIt : m_partPtrToVertex) {
    const auto PartPtr = segIt.first;
    const auto& vertexBundle = getVertexBundle(segIt.second);
    if (ignoreUntargeted && vertexBundle.targetedBy.empty()) continue;
    
    const vec<string> verbs = vertexBundle.getTargetedByVerbs();
    for (const string& verb : verbs) {
      (*out)[verb].insert(PartPtr);
    }
  }
  size_t total = 0;
  for (const auto it : (*out)) {
    total += it.second.size();
  }
  return total;
}

}  // namespace interaction
}  // namespace sg

// These operators are required for writing vecf members to GraphML
namespace std {
ostream& operator << (ostream& os, const vector<float>& v) { return sg::io::toJSON(os, v); }
istream& operator >> (istream& is, vector<float>& v) {
  // TODO(MS): Implement reading when it becomes necessary
  cerr << "Unimplemented: InteractionGraph operator >> (istream, vec<T>)" << endl;
  return is;
}
}  // namespace std
