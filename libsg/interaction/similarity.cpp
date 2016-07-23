#include "common.h"  // NOLINT

#include "interaction/similarity.h"

namespace sg {
namespace interaction {
namespace similarity {

bool debugSimilarity = false;

float InteractionGraphSimilarity::sim(const InteractionLink& a, const InteractionLink& b) const {
  if (a.type == kBone) {
    return simBone(a, b);
  } else if (a.type == kContact) {
    return simContact(a, b);
  } else if (a.type == kGaze) {
    return simGaze(a, b);
  } else if (a.type == kSpatial) {
    return simSpatial(a, b);
  } else {
    return 0.f;
  }
}

float InteractionGraphSimilarity::sim(const InteractionNode& a, const InteractionNode& b) const {
  if (a.type == kSeg) {
    return simSegment(a, b);
  } else if (a.type == kJoint) {
    return simJoint(a, b);
  } else {
    return 0.f;
  }
}

float InteractionGraphSimilarity::sim(const InteractionGraph& a, const InteractionGraph& b) const {
  float contactSim = 0.f, jointSim = 0.f, boneSim = 0.f, gazeSim = 0.f, contactVolSim = 0.f;
  size_t numContactJoints = 0;
  // Iterate over joints in a and b, and accumulate maximum contact edge-edge and seg-seg similarity
  for (int iJoint = 0; iJoint < kSkelParams.kNumJoints; ++iJoint) {
    // Skip fake gaze joint... we are only doing actual contact now
    if (iJoint == kSkelParams.kGazeJoint) continue;

    if (m_params.contactSimWeight > 0) {
      contactSim += maxContactSegSimilarity(a, b, iJoint);
    }
    if (m_params.contactVolSimWeight > 0) {
      contactVolSim += simVolume(a.getJointVertexBundle(iJoint), b.getJointVertexBundle(iJoint));
    }
    if (m_params.jointSimWeight > 0) {
      jointSim += sim(a.getJointVertexBundle(iJoint), b.getJointVertexBundle(iJoint));
    }
    numContactJoints++;
  }
  for (int iBone = 0; iBone < kSkelParams.kNumBones; ++iBone) {
    if (m_params.boneSimWeight > 0) {
      boneSim += sim(a.getBoneEdgeBundle(iBone), b.getBoneEdgeBundle(iBone));
    }
  }
  if (m_params.gazeSimWeight > 0) {
    gazeSim = maxGazeSegSimilarity(a, b);
  }

  contactSim    *= m_params.contactSimWeight / numContactJoints;
  contactVolSim *= m_params.contactVolSimWeight / numContactJoints;
  jointSim      *= m_params.jointSimWeight / numContactJoints;
  boneSim       *= m_params.boneSimWeight / kSkelParams.kNumBones;
  gazeSim       *= m_params.gazeSimWeight;

  const float
    totalWeight = m_params.contactSimWeight + m_params.contactVolSimWeight  + m_params.jointSimWeight
                + m_params.boneSimWeight + m_params.gazeSimWeight,
    totalSim    = (contactVolSim + contactSim + jointSim + boneSim + gazeSim) / totalWeight;
  assert(totalSim >= 0 && totalSim <= 1);
  return totalSim;
}

float InteractionGraphSimilarity::maxContactSegSimilarity(const InteractionGraph& a, const InteractionGraph& b,
                                                          int iJoint) const {
  const auto aEdges = a.getContactEdges(iJoint); const size_t aNumContacts = aEdges.size();
  const auto bEdges = b.getContactEdges(iJoint); const size_t bNumContacts = bEdges.size();
  if (aNumContacts == 0 && bNumContacts == 0) {
    return 1.f;  // No edges present so return max similarity
  } else if (aNumContacts == 0 || bNumContacts == 0) {
    return 0.f;  // One of two sides has no edges so return max dissimilarity TODO(MS): Consider implications of this
  }
  float maxSim = 0.f;
  for (size_t i = 0; i < aNumContacts; ++i) {
    const InteractionGraph::edge aEdge = aEdges[i];
    const InteractionLink aLink = a.getEdgeBundle(aEdge);
    const InteractionNode aSeg = a.getVertexBundle(aEdge.m_target);
    for (size_t j = 0; j < bNumContacts; ++j) {
      const InteractionGraph::edge bEdge = bEdges[j];
      const InteractionLink bLink = b.getEdgeBundle(bEdge);
      const InteractionNode bSeg = b.getVertexBundle(bEdge.m_target);
      const float
        simLink = simContact(aLink, bLink),
        simSeg = simSegment(aSeg, bSeg),
        simCombined = simLink * simSeg;
      if (simCombined > maxSim) {
        maxSim = simCombined;
      }
    }
  }
  return maxSim;
}

float InteractionGraphSimilarity::maxGazeSegSimilarity(const InteractionGraph& a, const InteractionGraph& b) const {
  const auto aEdges = a.getGazeEdges(); const size_t aNumGazed = aEdges.size();
  const auto bEdges = b.getGazeEdges(); const size_t bNumGazed = bEdges.size();
  if (aNumGazed == 0 && bNumGazed == 0) {
    return 1.f;  // No edges present so return max similarity
  } else if (aNumGazed == 0 || bNumGazed == 0) {
    return 0.f;  // One of two sides has no edges so return max dissimilarity TODO(MS): Consider implications of this
  }
  float maxSim = 0.f;
  for (size_t i = 0; i < aNumGazed; ++i) {
    const InteractionGraph::edge aEdge = aEdges[i];
    const InteractionLink aLink = a.getEdgeBundle(aEdge);
    const InteractionNode aSeg = a.getVertexBundle(aEdge.m_target);
    for (size_t j = 0; j < bNumGazed; ++j) {
      const InteractionGraph::edge bEdge = bEdges[j];
      const InteractionLink bLink = b.getEdgeBundle(bEdge);
      const InteractionNode bSeg = b.getVertexBundle(bEdge.m_target);
      const float
        simLink = simGaze(aLink, bLink),
        simSeg = simSegment(aSeg, bSeg),
        simCombined = simLink * simSeg;
      if (simCombined > maxSim) {
        maxSim = simCombined;
      }
    }
  }
  return maxSim;
}

// COPY OF ABOVE FOR PIG TO IG SIMILARITY
// TODO: Reduce copying!!!
// MAIN CHANGES FOR PIG
//  getJointVertexBundle => getProtoJoint
//  getBoneEdgeBundle => getProtoBone

float ProtoToInteractionGraphSimilarity::sim(const ProtoInteractionLink& a, const InteractionLink& b) const {
  if (a.type == kBone) {
    return simBone(a, b);
  } else if (a.type == kContact) {
    return simContact(a, b);
  } else if (a.type == kGaze) {
    return simGaze(a, b);
  } else if (a.type == kSpatial) {
    return simSpatial(a, b);
  } else {
    return 0.f;
  }
}

float ProtoToInteractionGraphSimilarity::sim(const ProtoInteractionNode& a, const InteractionNode& b) const {
  if (a.type == kSeg) {
    return simSegment(a, b);
  } else if (a.type == kJoint) {
    return simJoint(a, b);
  } else {
    return 0.f;
  }
}

float ProtoToInteractionGraphSimilarity::sim(const ProtoInteractionGraph& a, const InteractionGraph& b) const {
  float contactSim = 0.f, jointSim = 0.f, boneSim = 0.f, gazeSim = 0.f, contactVolSim = 0.f;
  // Iterate over joints in a and b, and accumulate maximum contact edge-edge and seg-seg similarity
  size_t numContactJoints = 0;
  for (int iJoint = 0; iJoint < kSkelParams.kNumJoints; ++iJoint) {
    // Skip fake gaze joint... we are only doing actual contact now
    if (iJoint == kSkelParams.kGazeJoint) continue;

    const int iJointGroup = Skeleton::kJointGroupLRToJointGroup[iJoint];
    const float wJoint =
      (m_params.useJointWeights) ? static_cast<float>((*m_params.pJointGroupWeights)[iJointGroup]) : 1.f;

    if (m_params.contactSimWeight > 0) {
      contactSim += wJoint * maxContactSegSimilarity(a, b, iJoint);
    }

    if (m_params.jointSimWeight > 0) {
      jointSim += wJoint * sim(a.getProtoJoint(iJoint), b.getJointVertexBundle(iJoint));
    }
    numContactJoints++;
  }
  for (int iBone = 0; iBone < kSkelParams.kNumBones; ++iBone) {
    if (m_params.boneSimWeight > 0) {
      boneSim += sim(a.getProtoBone(iBone), b.getBoneEdgeBundle(iBone));
    }
  }
  if (m_params.gazeSimWeight > 0) {
    const float wJoint =
      (m_params.useJointWeights) ? static_cast<float>((*m_params.pJointGroupWeights)[Skeleton::JointGroup_Gaze]) : 1.f;
    gazeSim = wJoint * maxGazeSegSimilarity(a, b);
  }

  contactSim *= m_params.contactSimWeight/numContactJoints;
  //contactVolSim *= m_params.contactVolSimWeight/numContactJoints;
  jointSim *= m_params.jointSimWeight/numContactJoints;
  boneSim *= m_params.boneSimWeight/kSkelParams.kNumBones;
  gazeSim *= m_params.gazeSimWeight;

  if (debugSimilarity) {
    SG_LOG_INFO << "PIG: " << a.getGraphBundle().id
                << ", ContactSim: " << contactSim
                << ", JointSim: " << jointSim 
                << ", BoneSim: " << boneSim
                << ", GazeSim: " << gazeSim;
  }

  const float totalWeight = m_params.contactSimWeight + m_params.jointSimWeight + m_params.boneSimWeight + m_params.gazeSimWeight;
  const float totalSim = (contactSim + jointSim + boneSim + gazeSim) / totalWeight;
  //const float totalWeight = m_params.contactSimWeight + m_params.contactVolSimWeight 
  //  + m_params.jointSimWeight  + m_params.boneSimWeight + m_params.gazeSimWeight;
  //const float totalSim = (contactVolSim + contactSim + jointSim +  boneSim + gazeSim) / totalWeight;
  assert(totalSim >= 0 && totalSim <= 1);
  return totalSim;
}

float ProtoToInteractionGraphSimilarity::maxContactSegSimilarity(const ProtoInteractionGraph& a, const InteractionGraph& b,
                                                          int iJoint) const {
  const auto aEdges = a.getContactEdges(iJoint); const size_t aNumContacts = aEdges.size();
  const auto bEdges = b.getContactEdges(iJoint); const size_t bNumContacts = bEdges.size();
  if (aNumContacts == 0 && bNumContacts == 0) {
    return 1.f;  // No edges present so return max similarity
  } else if (aNumContacts == 0 || bNumContacts == 0) {
    return 0.f;  // One of two sides has no edges so return max dissimilarity TODO(MS): Consider implications of this
  }
  float maxSim = 0.f;
  for (const auto aEdgeIt:aEdges) {
    const ProtoInteractionGraph::edge aEdge = aEdgeIt.second;
    const ProtoInteractionLink aLink = a.getEdgeBundle(aEdge);
    const ProtoInteractionNode aJoint = a.getVertexBundle(aEdge.m_source);
    const ProtoInteractionNode aSeg = a.getVertexBundle(aEdge.m_target);
    const float edgeWeight = static_cast<float>(aLink.interactionCount)/aJoint.interactionCount;
    for (size_t j = 0; j < bNumContacts; ++j) {
      const InteractionGraph::edge bEdge = bEdges[j];
      const InteractionLink bLink = b.getEdgeBundle(bEdge);
      const InteractionNode bSeg = b.getVertexBundle(bEdge.m_target);
      const float
        simLink = simContact(aLink, bLink),
        simSeg = simSegment(aSeg, bSeg),
        simCombined = edgeWeight*(simLink + simSeg)/2.f;
      if (simCombined > maxSim) {
        maxSim = simCombined;
      }
    }
  }
  return maxSim;
}

float ProtoToInteractionGraphSimilarity::maxGazeSegSimilarity(const ProtoInteractionGraph& a, const InteractionGraph& b) const {
  const auto aEdges = a.getGazeEdges(); const size_t aNumGazed = aEdges.size();
  const auto bEdges = b.getGazeEdges(); const size_t bNumGazed = bEdges.size();
  if (aNumGazed == 0 && bNumGazed == 0) {
    return 1.f;  // No edges present so return max similarity
  } else if (aNumGazed == 0 || bNumGazed == 0) {
    return 0.f;  // One of two sides has no edges so return max dissimilarity TODO(MS): Consider implications of this
  }
  float maxSim = 0.f;
  for (const auto aEdgeIt:aEdges) {
    const ProtoInteractionGraph::edge aEdge = aEdgeIt.second;
    const ProtoInteractionLink aLink = a.getEdgeBundle(aEdge);
    const ProtoInteractionNode aJoint = a.getVertexBundle(aEdge.m_source);
    const ProtoInteractionNode aSeg = a.getVertexBundle(aEdge.m_target);
    const float edgeWeight = static_cast<float>(aLink.interactionCount)/aJoint.interactionCount;
    for (size_t j = 0; j < bNumGazed; ++j) {
      const InteractionGraph::edge bEdge = bEdges[j];
      const InteractionLink bLink = b.getEdgeBundle(bEdge);
      const InteractionNode bSeg = b.getVertexBundle(bEdge.m_target);
      const float
        simLink = simGaze(aLink, bLink),
        simSeg = simSegment(aSeg, bSeg),
        simCombined = edgeWeight * simLink * simSeg;
      if (simCombined > maxSim) {
        maxSim = simCombined;
      }
    }
  }
  return maxSim;
}

// HELPER SIM FUNCTIONS

float simVolume(const InteractionNode& a, const InteractionNode& b) {
  //sg::vecf v0(2), v1(2);
  //v0[0] = a.occupied;   v0[1] = a.free;
  //v1[0] = b.occupied;   v1[1] = b.free;
  //return math::angularSimilarity(v0, v1);
  float dist = 0.0f;
  //#pragma omp critical
  //    {
  //      std::cout << a.heightHistogramOccupied << std::endl;
  //      std::cout << a.heightHistogramTotal << std::endl;
  //      std::cout << std::endl;


  size_t count = 0;
  for (int i = 0; i < a.heightHistogramOccupied.getNumBins(); i++) {
    if (a.heightHistogramTotal.count(i) > 0 || b.heightHistogramTotal.count(i) > 0) {
      const float totalCountA = (float)a.heightHistogramTotal.count(i);
      const float totalCountB = (float)b.heightHistogramTotal.count(i);
      float relA = totalCountA > 0.0f ? (float)a.heightHistogramOccupied.count(i) / totalCountA : 0.0f;
      float relB = totalCountB > 0.0f ? (float)b.heightHistogramOccupied.count(i) / totalCountB : 0.0f;

      float cDist = relA - relB;
      dist += cDist*cDist;  //at most 1
      count++;

      //std::cout << cDist << std::endl;
    }
  }
  dist /= (float)count;

  //      std::cout << "dist: " << dist << std::endl;
  //    }
  return 1.0f - dist;
}

float simHistFeats(const vec<stats::Histogram<float>>& histFeats, const vecf feats) {
  size_t nFeats = feats.size();
  assert(nFeats == histFeats.size());
  float total = 0.0f;
  for (int i = 0; i < nFeats; ++i) {
    float feat = feats.at(i);
    float density = histFeats.at(i).density(feat);
    const auto& range = histFeats.at(i).getRange();
    float unifDensity = 1.0f/(range.max - range.min);
    float weight = 0.0f;
    if (density > unifDensity) {
      weight = 1.0f;
    }
    total += weight;
  }
  return total/nFeats;
}

float simHistFeat(const stats::Histogram<float>& hist, const float v) {
  float density = hist.density(v);
  const auto& range = hist.getRange();
  float unifDensity = 1.0f / (range.max - range.min);
  float weight = 0.0f;
  if (density > unifDensity) {
    weight = 1.0f;
  }
  return weight;
}


}  // namespace similarity
}  // namespace graph
}  // namespace sg
