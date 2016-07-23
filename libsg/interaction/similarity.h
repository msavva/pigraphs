#pragma once

#include "libsg.h"  // NOLINT
#include "interaction/InteractionGraph.h"
#include "interaction/ProtoInteractionGraph.h"
#include "math/math.h"

namespace sg {
namespace interaction {

//! Namespace containing algorithms for computing a variety of graph member and graph similarities
namespace similarity {

//! Abstract similarity functor for given graph type
template <typename GraphT>
class Similarity {
 public:
  typedef typename GraphT::VertexBundleT  vertex;
  typedef typename GraphT::EdgeBundleT    edge;
  typedef GraphT                          graph;
  //! Return similarity between vertices a and b
  virtual float sim(const vertex& a, const vertex& b) const = 0;
  //! Return similarity between edges a and b
  virtual float sim(const edge& a, const edge& b) const = 0;
  //! Return similarity between graphs a and b
  virtual float sim(const graph& a, const graph& b) const = 0;
};

//! Abstract similarity functor for two different graph types
template <typename Graph1T, typename Graph2T>
class Similarity2 {
 public:
  typedef typename Graph1T::VertexBundleT  vertex1;
  typedef typename Graph1T::EdgeBundleT    edge1;
  typedef Graph1T                          graph1;

  typedef typename Graph2T::VertexBundleT  vertex2;
  typedef typename Graph2T::EdgeBundleT    edge2;
  typedef Graph2T                          graph2;
  //! Return similarity between vertices a and b
  virtual float sim(const vertex1& a, const vertex2& b) const = 0;
  //! Return similarity between edges a and b
  virtual float sim(const edge1& a, const edge2& b) const = 0;
  //! Return similarity between graphs a and b
  virtual float sim(const graph1& a, const graph2& b) const = 0;
};

//! Returns the similarity between a histogram of features and a set of features
float simHistFeats(const vec<stats::Histogram<float>>& histFeats, const vecf feats);

float simHistFeat(const stats::Histogram<float>& hist, const float v);

struct IGSimParams {
  IGSimParams()
    : boneSimWeight(0.0f)
    , jointSimWeight(0.0f)
    , contactSimWeight(1.0f)
    , contactVolSimWeight(0.0f)
    , gazeSimWeight(1.0f)
    , useJointWeights(false)
    , pJointGroupWeights(nullptr) { }

  // Weights on different types of similarity
  float boneSimWeight;
  float jointSimWeight;
  float contactSimWeight;
  float contactVolSimWeight;
  float gazeSimWeight;

  // Joint weight parameters
  bool useJointWeights;
  const arr<double, Skeleton::kNumJointGroups>* pJointGroupWeights;
};

//! Concrete class to provide similarity functions between InteractionGraphs and their member elements
class InteractionGraphSimilarity : Similarity<InteractionGraph> {
 public:
  InteractionGraphSimilarity() : m_params() {}
  InteractionGraphSimilarity(const IGSimParams& p) : m_params(p) {}

  //! Similarity between interaction graphs
  float sim(const InteractionLink& a, const InteractionLink& b) const override;
  float sim(const InteractionNode& a, const InteractionNode& b) const override;
  float sim(const InteractionGraph& a, const InteractionGraph& b) const override;

 private:
  //! Returns the maximum [0,1] similarity among all pairs of contact edges and segs attached to iJoint
  //! in InteractionGraphs a and b.  The similarity is the product of edge-edge and seg-seg similarities
  float maxContactSegSimilarity(const InteractionGraph& a, const InteractionGraph& b, int iJoint) const;

  //! Returns the maximum [0,1] similarity among all pairs of gaze edges and gazed segs
  //! in InteractionGraphs a and b.  The similarity is the product of edge-edge and seg-seg similarities
  float maxGazeSegSimilarity(const InteractionGraph& a, const InteractionGraph& b) const;

  //! Params for how this similarity should work
  const IGSimParams m_params;
};

//! Concrete class to provide similarity functions between ProtoInteractionGraph and InteractionGraphs
class ProtoToInteractionGraphSimilarity : Similarity2<ProtoInteractionGraph, InteractionGraph> {
 public:
  ProtoToInteractionGraphSimilarity() : m_params() {}
  ProtoToInteractionGraphSimilarity(const IGSimParams& p) : m_params(p) {}

  //! Similarity between prototypical interation graph and single interaction graph
  float sim(const ProtoInteractionLink& a, const InteractionLink& b) const override;
  float sim(const ProtoInteractionNode& a, const InteractionNode& b) const override;
  float sim(const ProtoInteractionGraph& a, const InteractionGraph& b) const override;

 private:
  //! Returns the maximum [0,1] similarity among all pairs of contact edges and segs attached to iJoint
  //! in InteractionGraphs a and b.  The similarity is the product of edge-edge and seg-seg similarities
  float maxContactSegSimilarity(const ProtoInteractionGraph& a, const InteractionGraph& b, int iJoint) const;

  //! Returns the maximum [0,1] similarity among all pairs of gaze edges and gazed segs
  //! in InteractionGraphs a and b.  The similarity is the product of edge-edge and seg-seg similarities
  float maxGazeSegSimilarity(const ProtoInteractionGraph& a, const InteractionGraph& b) const;

  //! Params for how this similarity should work
  const IGSimParams m_params;
};

// TODO(MS): Make similarity methods below do something less trivial

//! Returns [0,1] similarity between two bones a and b
inline float simBone(const InteractionLink& a, const InteractionLink& b) {
  return math::angularSimilarity(a.feats, b.feats);
}

//! Returns [0,1] similarity between two contact links a and b
inline float simContact(const InteractionLink& a, const InteractionLink& b) {
  return math::angularSimilarity(a.feats, b.feats);
}

//! Returns [0,1] similarity between two gaze links a and b
inline float simGaze(const InteractionLink& a, const InteractionLink& b) {
  return math::angularSimilarity(a.feats, b.feats);
}

//! Returns [0,1] similarity between two spatial links a and b
inline float simSpatial(const InteractionLink& a, const InteractionLink& b) {
  return math::angularSimilarity(a.feats, b.feats);
}

//! Returns [0,1] similarity between two joints a and b
inline float simJoint(const InteractionNode& a, const InteractionNode& b) {
  return math::angularSimilarity(a.feats, b.feats);
}

//! Returns [0,1] similarity between two segments a and b
inline float simSegment(const InteractionNode& a, const InteractionNode& b) {
  return math::angularSimilarity(a.feats, b.feats);
}

//! Returns [0,1] similarity between two bones a and b
inline float simBone(const ProtoInteractionLink& a, const InteractionLink& b) {
  return simHistFeats(a.feats, b.feats);
}

//! Returns [0,1] similarity between two contact links a and b
inline float simContact(const ProtoInteractionLink& a, const InteractionLink& b) {
  return simHistFeats(a.feats, b.feats);
}

//! Returns [0,1] similarity between two gaze links a and b
inline float simGaze(const ProtoInteractionLink& a, const InteractionLink& b) {
  return simHistFeats(a.feats, b.feats);
}

//! Returns [0,1] similarity between two spatial links a and b
inline float simSpatial(const ProtoInteractionLink& a, const InteractionLink& b) {
  return simHistFeats(a.feats, b.feats);
}

//! Returns [0,1] similarity between two joints a and b
inline float simJoint(const ProtoInteractionNode& a, const InteractionNode& b) {
  return simHistFeats(a.feats, b.feats);
}

//! Returns [0,1] similarity between two segments a and b
inline float simSegment(const ProtoInteractionNode& a, const InteractionNode& b) {
  return simHistFeats(a.feats, b.feats);
}

//! Returns [0,1] similarity between two joint nodes a and b by computing volumetric features around their positions
inline float simVolume(const InteractionNode& a, const InteractionNode& b);

}  // namespace similarity
}  // namespace interaction
}  // namespace sg


