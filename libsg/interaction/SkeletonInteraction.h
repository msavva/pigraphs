#pragma once

#include "libsg.h"  // NOLINT
#include "core/Skeleton.h"
#include "interaction/VerbNoun.h"
#include "stats/stats.h"
#include "stats/histogram.h"

namespace sg {
namespace interaction {

//! Feature type identifier
enum FeatType {
  kContact  = 0,  //! joint-segment contact (edge)
  kGaze     = 1,  //! segment gazing (edge)
  kCOM      = 2,  //! center of mass (vertex)
  kCOMLink  = 3,  //! center of mass linkage (edge)
  kSeg      = 4,  //! geometric segment (vertex)
  kSpatial  = 5,  //! segment-segment spatial relation (edge)
  kBone     = 6,  //! skeleton bone (edge)
  kJoint    = 7,  //! joint features (vertex)
  kCount    = 8
};

using in = stats::Interval<float>;

//! Description of a feature dimension
struct FeatDim {
  FeatDim(const string& _id, bool _useSig, const in& _range, const int _numBins = 20, bool _isAngle = false) 
    : id(_id), useSig(_useSig), range(_range), numBins(_numBins), isAngle(_isAngle) { }
  string id;      //! name of this feature dimension
  bool useSig;    //! does it use a sigmoid to convert to bin intervals?
  in range;       //! range of this feature dimension
  int numBins;    //! number of bins for discretization
  bool isAngle;   //! is this an angle?
};

//! Description of a feature set (consisting of multiple dimensions
struct FeatDesc {
  FeatDesc(string _id, const vec<FeatDim>& _dims)
    : id(_id)
    , numDims(static_cast<int>(_dims.size()))
    , dims(_dims) { }
  string id;          //! readable name
  int numDims;        //! number of dimensions
  vec<FeatDim> dims;  //! dimension descriptors

  const util::Index<string>& getFeatIndex() const {
    if (m_index.size() == 0) {
      // Populate
      for (int i = 0; i < numDims; i++) {
        m_index.add(dims[i].id);
      }
    }
    return m_index;
  }

 private:
  mutable util::Index<string> m_index;
};

const stats::Interval<float>
  r01   = in{ 0, 1},                      // [0, 1]
  rm11  = in{-1, 1},                      // [-1, 1]
  r02p  = in{0, math::constants::PI2f};   // [0, 2pi]

//! Relative features
const vec<FeatDim> kRelFeats {
  {"height",      true,  r01},   // height of target [0, max_h] -> [0, 1]
  {"rDist",       true,  r01},   // radial distance ref to target [0, max_d] -> [0, 1]
  {"vOffset",     true,  rm11},  // vertical offset ref to target [-max_h, max_h] -> [-1, 1]
  {"angDxy",      false, r02p, 36, true},  // angle in xy ref frame from ref dir to delta dir [0, 2pi]
  {"cosDS",       false, r01}    // abs dot product of delta dir with target normal dir [0, 1]
};

//! Segment features
const vec<FeatDim> kSegFeats {
  {"centroidZ",   true,  r01},  // centroidZ [0, max_h] ->[0, 1]
  {"diagXY",      true,  r01},  // diagXY [0,max_diag] -> [0,1]
  {"sqrtAreaXY",  true,  r01},  // sqrtAreaXY [0,max_axy] -> [0,1]
  {"heightZ",     true,  r01},  // heightZ [0,max_height] -> [0,1]
  {"normalDotZ",  false, r01}   // normalDotZ [0,1]
};

//! Center of mass features
const vec<FeatDim> kCOMFeats {
  {"centroidZ",   true,  r01},  // centroidZ [0, max_h] ->[0, 1]
};

//! Map from string (feature set type id) to vector of feature descriptors describing each feature dimension
const map<FeatType, FeatDesc> kFeats {
  {kContact,  {"contact", kRelFeats }},
  {kGaze,     {"gaze",    kRelFeats }},
  {kSeg,      {"seg",     kSegFeats }},
  {kCOM,      {"com",     kCOMFeats }},
  {kCOMLink,  {"comLink", kRelFeats }},
  {kSpatial,  {"spatial", {
    {"minDist",     true,  r01},  // Minimum distance between the segments [0,1]
    {"dotNormals",  false, r01},  // Absolute of dot product of normals [0,1]
    {"ratioOverlap",false, r01}   // Ratio of points in one segment contained by the OBB of the other [0,1]
  }}},
  {kBone, {"bone", {
    {"length",      true,  r01},  // Distance along bone [0,max_d] -> [0,1]
    {"vDist",       false, rm11}   // Vertical distance bw joint positions [-z,z] -> [-1,1]
  }}},
  {kJoint, {"joint", {
    {"height",      true,  r01}   // Height above ground [0,max_h] -> [0,1]
  }}}
};

//! Represents features of a volume in which an interaction is taking place
struct InteractionVolume {
  float occupied = 0.0;         //! percentage of occupied blocks around the joint pos
  float free = 0.0;             //! percentage of free blocks around the joint pos
  //! Histograms representing slices of an axis aligned cylinder, counting occupied and total sampled voxels
  stats::Histogram<float> heightHistogramOccupied, heightHistogramTotal;
};

//! Represent a skeleton joint, or a part interacting with a joint or being gazed
struct InteractionNode : public InteractionVolume {
  FeatType type;            //! feature type
  string id;                //! identifier
  string label;             //! label of this InteractionNode (mostly categorical nouns for identifiying objects)
  string targetedBy;        //! Comma-separated list of verbs targeting this node
  vecf feats;               //! data
  int jointId;              //! jointId  (-1 if not a joint)

  vec<string> getTargetedByVerbs() const;
};

using segmentation::ConstPartPtr;
using core::Skeleton;

//! Represents a Skeleton joint which is potentially an abstraction of several raw joints
struct Joint {
  int         index;        //! Identifies this joint
  veci        rawIndices;   //! Indices to use for indexing into raw skeleton joints
  ml::vec3f   position;     //! Position of the joint in world-space
};

template<typename JointT, size_t numJoints>
void createCombinedJoint(const core::SkeletonParamsT<JointT,numJoints>& skelParams, 
                         const int iJoint, const Skeleton& skel, Joint* pJoint) {
  const auto& skelJoints = skelParams.kJointInvMap[iJoint];
  assert(skelJoints.size() > 0);
  ml::vec3f jointPosition;
  for (size_t iSkelJoint : skelJoints) {
    jointPosition += skel.jointPositions[iSkelJoint];
  }
  float totalSkelJoints = static_cast<float>(skelJoints.size());
  jointPosition /= totalSkelJoints;

  pJoint->index = iJoint;
  pJoint->rawIndices = skelJoints;
  pJoint->position = jointPosition;
}

//! Sigmoidal function for mapping [-inf,inf] to [-1,1]
inline float sig(const float x) {
  return math::sigmoid(x);
}

//! Compute 5D relative features between p0 and p1 with normals n0 and n1, and store into feats
template <typename Vec3>
inline void relFeats(const Vec3& p0, const Vec3& n0, const Vec3& p1, const Vec3& n1, vecf* feats) {
  vecf& f = *feats;
  f.resize(5);

  // Positional features
  const Vec3 d = p1 - p0;  // displacement vector
  const float h = p1[2];   // target height
  const float r = sqrtf(d[0] * d[0] + d[1] * d[1]);  // radial distance
  const float z = d[2];  // vertical displacement
  f[0] = sig(std::max(h, 0.f));
  f[1] = sig(r);
  f[2] = sig(z);

  const float dLength = sqrtf(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
  const float dxyLength = sqrtf(d[0] * d[0] + d[1] * d[1]);
  const float n0xyLength = sqrtf(n0[0] * n0[0] + n0[1] * n0[1]);

  // Check for coincident points or normals with no xy component
  // and return since we can't compute angular features for them
  if (dLength == 0 || dxyLength == 0 || n0xyLength == 0) {
    //SG_LOG_WARN << "Cannot compute relative angular features for coincident points or normals not in xy plane";
    //SG_LOG_WARN << "dL=" << dLength << ",dxyL=" << dxyLength << ",n0xyL=" << n0xyLength;
    f[3] = 0.f;
    f[4] = 0.f;
    return;
  }

  // Angular features
  const float nDxyX = d[0] / dxyLength;
  const float nDxyY = d[1] / dxyLength;
  const float n0xyX = n0[0] / n0xyLength;
  const float n0xyY = n0[1] / n0xyLength;
  float angDxy = atan2f(nDxyY, nDxyX) - atan2f(n0xyY, n0xyX);  // signed angle from n0 to nD
  if (angDxy < 0) {
    angDxy += math::constants::PI2f;
  }
  f[3] = angDxy;

  const Vec3 nD = d / dLength;  // displacement normal vector
  const float cosD1 = nD[0] * n1[0] + nD[1] * n1[1] + nD[2] * n1[2];  // dot product
  f[4] = std::min(fabsf(cosD1), 1.f);
  assert(std::isfinite(f[0]));
  assert(std::isfinite(f[1]));
  assert(std::isfinite(f[2]));
  assert(std::isfinite(f[3]));
  assert(std::isfinite(f[4]));
}

//! Compute joint-part contact features and store in feats
void contactFeats(const Skeleton& skel, const Joint& joint, const ConstPartPtr part, vecf* feats);
void contactFeats(const Skeleton& skel, const Joint& joint, const geo::OBB* pOBB, vecf* feats);
//! Compute part gaze features and store in feats
void gazeFeats(const Skeleton& skel, const ConstPartPtr part, vecf* feats);
void gazeFeats(const Skeleton& skel, const geo::OBB* pOBB, vecf* feats);
//! Compute Skeleton center of mass features and store in feats
void comFeats(const Skeleton& skel, vecf* feats);
//! Compute com - part linkage features and store in feats
void comLinkFeats(const Skeleton& skel, const ConstPartPtr part, vecf* feats);
void comLinkFeats(const Skeleton& skel, const geo::OBB* pOBB, vecf* feats);
//! Compute part features and store in feats
void partFeats(const ConstPartPtr part, vecf* feats);
//! Compute part-to-part features and store in feats
void partToPartFeats(const ConstPartPtr a, const ConstPartPtr b, vecf* feats);
//! Compute joint-to-joint features and store in feats
void boneFeats(const Skeleton& skel, const Joint& joint1, const Joint& joint2, vecf* feats);
//! Compute joint features and store in feats
void jointFeats(const Skeleton& skel, const Joint& joint, vecf* feats);
//! Computes features of scan volume around joint, and stores in InteractionNode node
void jointVolumeFeats(const core::OccupancyGrid& occupancyGrid, const Skeleton& skel, const Joint& joint, InteractionNode& node);
//! Computes features of scan volume at point, and stores in InteractionNode node
void jointVolumeFeats(const core::OccupancyGrid& occupancyGrid, const ml::vec3f& point, InteractionNode& node);

//! Represent a bone, joint-part contact event, or part gaze event
struct InteractionLink {
  FeatType type;            //! feature type
  string id;                //! identifier
  vecf feats;               //! data
};

//! Represent a set of annotations applied to an observed interaction instance
struct InteractionAnnotation {
  segmentation::PartType partType;    //! Part type used for constructing this IG
  set<VerbNoun> verbNouns;  //! Verb-Noun pairs applying to this InteractionGraph
  geo::Vec3f COM;  //! center of mass of Skeleton
  geo::Vec3f gazeDir;  //! gaze direction of Skeleton
};

//! SkeletonParams to use for all interactions
static const auto& kSkelParams = core::kSkelParamsJointGroupLR;

// NOTE: Above is kind of weird since static enforces internal linkage (i.e., copies of this for each translation unit)
// Acceptable since this is only a reference.

//! Helper storage for vertices and nodes corresponding to Skeleton in a graph
template <typename GraphT>
struct GraphMembers {
  // Convenience typedefs
  typedef typename GraphT::vertex       VertT;
  typedef typename GraphT::edge         EdgeT;
  typedef pair<VertT, VertT>            VertPairT;

  //! Instantiate vertices and edges corresponding to joints and bones of Skeleton in a Graph object
  //! Graph must possess addVertex() and addEdge(v0, v1) functions that return vertex and edge identifiers
  explicit GraphMembers(GraphT* g) : m_G(*g) {
    m_joint2vert.resize(kSkelParams.kNumJoints);
    for (int iJoint = 0; iJoint < kSkelParams.kNumJoints; ++iJoint) {
      const VertT v = g->addVertex();
      auto& node = g->getVertexBundle(v);
      node.type = kJoint;
      node.id = "joint:" + kSkelParams.jointName(iJoint);
      m_joint2vert[iJoint] = v;
      node.jointId = static_cast<int>(iJoint);
    }
    m_bone2edge.resize(kSkelParams.kNumBones);
    for (int iBone = 0; iBone < kSkelParams.kNumBones; ++iBone) {
      const auto j0 = kSkelParams.kBones[iBone][0], j1 = kSkelParams.kBones[iBone][1];
      const EdgeT e = g->addEdge(m_joint2vert[j0], m_joint2vert[j1]);
      auto& bone = g->getEdgeBundle(e);
      bone.type = kBone;
      bone.id = "bone:" + kSkelParams.jointName(j0) + "-" + kSkelParams.jointName(j1);
      m_bone2edge[iBone] = e;
    }
    // create com node
    m_com = g->addVertex();
    auto& comNode = g->getVertexBundle(m_com);
    comNode.type = kCOM;
    comNode.id = "com";
    comNode.label = "com";
    comNode.jointId = -1;
  }
  //! Returns vertex corresponding to center of mass
  VertT comVertex() const {
    return m_com;
  }
  //! Returns vertex corresponding to iJoint
  VertT jointVertex(int iJoint) const {
    return m_joint2vert[iJoint];
  }
  //! Returns edge corresponding to iBone
  EdgeT boneEdge(int iBone) const {
    return m_bone2edge[iBone];
  }
  //! Return the joint index of a given vertex
  int vertexJointIndex(const VertT& v) const {
    return static_cast<int>(std::distance(m_joint2vert.begin(),
                                          std::find(m_joint2vert.begin(), m_joint2vert.end(), v)));
  }
  //! Return the bone index of a given edge
  int boneEdgeIndex(const EdgeT& e) const {
    return static_cast<int>(std::distance(m_bone2edge.begin(),
                                          std::find(m_bone2edge.begin(), m_bone2edge.end(), e)));
  }

 private:
  // boost serialization support
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {  // NOLINT
    boost::serialization::split_member(ar, *this, version);
  }
  template <typename Archive>
  void save(Archive& ar, const unsigned int version) const {  // NOLINT
    ar & m_joint2vert;
    const vec<VertPairT> vertPairs = m_G.edgesToVertPairs(m_bone2edge);
    ar & vertPairs;
    ar & m_com;
  }
  template <typename Archive>
  void load(Archive& ar, const unsigned int version) {  // NOLINT
    ar & m_joint2vert;
    vec<VertPairT> vertPairs;
    ar & vertPairs;
    m_bone2edge = m_G.vertPairsToEdges(vertPairs);
    ar & m_com;
  }

  vec<VertT> m_joint2vert;  // Joint to vertex identifier map
  vec<EdgeT> m_bone2edge;   // Bone to edge identifier map
  VertT      m_com;         // Center of mass vertex
  const GraphT& m_G;        // Pointer to underlying graph
};

}  // namespace interaction
}  // namespace sg


