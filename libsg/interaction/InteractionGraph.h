#pragma once

#include "libsg.h"  // NOLINT
#include "interaction/SkeletonInteraction.h"
#include "graph/Graph.h"

namespace sg {
namespace interaction {

using segmentation::PartType;
using segmentation::ConstPartPtr;
using core::Skeleton;

// Graph-based representation of an interaction event i.e., Skeleton in
// contact with, or gazing parts.
// Use the InteractionFactory to create interaction graphs
class InteractionGraph : public graph::Graph<InteractionNode, InteractionLink, InteractionAnnotation> {
 public:
   //! Create InteractionGraph from Skeleton skel and interacting parts.
   //! Joints and parts as nodes, and edges representing joint and part connectivity
   InteractionGraph(const PartType partType, 
                    const Skeleton& skel,
                    const Skeleton::PartsByJointPlusGaze& parts,
                    const std::function<string(ConstPartPtr)>& getPartLabel,
                    const std::function<string(ConstPartPtr)>& getTargetedBy,
                    const set<VerbNoun>& verbNouns,
                    const core::OccupancyGrid* pOccupancyGrid = nullptr);

  //! VIP: delete copy and assignment since vertex and edge descriptors in underlying graph are invalidated
  InteractionGraph(const InteractionGraph&) = delete;
  InteractionGraph& operator=(const InteractionGraph&) = delete;

  //! Write GraphML representation out into os
  void writeGraphML(ostream& os) override;  // NOLINT

  //! Returns the part type
  PartType getPartType() const {
    return getGraphBundle().partType;
  }

  //! Return all contact edges for given joint index
  const vec<edge>& getContactEdges(int iJoint) const {
    return m_contactEdges[iJoint];
  }

  //! Return all gaze edges
  const vec<edge>& getGazeEdges() const {
    return m_gazeEdges;
  }

  //! Return all com edges
  const vec<edge>& getComEdges() const {
    return m_comEdges;
  }

  //! Return all spatial edges
  const vec<edge>& getSpatialEdges() const {
    return m_spatialEdges;
  }

  //! Return bone edge
  const InteractionLink& getBoneEdgeBundle(int iBone) const {
    return getEdgeBundle(m_skel.boneEdge(iBone));
  }

  //! Return joint bundle
  const InteractionNode& getJointVertexBundle(int iJoint) const {
    return getVertexBundle(m_skel.jointVertex(iJoint));
  }

  //! Return com bundle
  const InteractionNode& getComVertexBundle() const {
    return getVertexBundle(m_skel.comVertex());
  }

  //! Return joint index of given joint vertex
  int jointIndex(vertex v) const {
    assert(getVertexBundle(v).type == FeatType::kJoint);
    return m_skel.vertexJointIndex(v);
  }

  //! Get map of label to set of part pointers over the entire interaction
  //! Return total number of parts
  size_t getPartsByLabel(map<string, set<ConstPartPtr>>* out, bool ignoreUnlabeled = false) const;

  //! Get map of targetedBy to set of part pointers over the entire interaction
  //! Return total number of parts
  size_t getPartsByTargetedBy(map<string, set<ConstPartPtr>>* out, bool ignoreUntargeted = false) const;

  //! Return map from PartPtr to IG vertex
  const map<ConstPartPtr, vertex>& getParts() const {
    return m_partPtrToVertex;
  }

 private:
  //! Shared initialization routine
  void init(const PartType partType,
            const Skeleton& skel, 
            const Skeleton::PartsByJointPlusGaze& parts,
            const std::function<string(ConstPartPtr)>& getPartLabel,
            const std::function<string(ConstPartPtr)>& getTargetedBy,
            const set<VerbNoun>& verbNouns,
            const core::OccupancyGrid* pOccupancyGrid);

  //! Create a vertex corresponding to a part
  vertex addPartVertex(const string& label, const ConstPartPtr pPart);

  //! Create edge representing contact between joint and part
  edge addContactEdge(int iJoint, const vertex& segV);

  //! Create edge representing gazing of part
  edge addGazeEdge(const vertex& segV);

  //! Create edge representing seg-seg spatial relation
  edge addSpatialEdge(const vertex& segU, const vertex& segV);

  //! Create edge representing linkage of COM to part
  edge addComEdge(const vertex& segV);

  // Part to vertex map to ensure uniqueness
  map<ConstPartPtr, vertex>       m_partPtrToVertex;
  // Storage for skeleton nodes and edges
  GraphMembers<InteractionGraph>  m_skel;
  // Storage for contact edges (from each joint to contacted segment)
  vec<vec<edge>>                  m_contactEdges;
  // Storage for gaze edges (from head joint to gazed segments)
  vec<edge>                       m_gazeEdges;
  // Storage for spatial (seg-seg) edges
  vec<edge>                       m_spatialEdges;
  // Storage for COM edges
  vec<edge>                       m_comEdges;
};

//! Convenience typedef
typedef InteractionGraph IG;

}  // namespace interaction
}  // namespace sg


