#pragma once

#include <tuple>

#include <Eigen/StdVector>

#include "libsg.h"  // NOLINT
#include "geo/geo.h"
#include "geo/OBB.h"
#include "mesh/TriMesh.h"
#include "mesh/ClusterMesh.h"
#include "mesh/MeshSamples.h"
#include "mesh/ContactDeterminator.h"
#include "symmetry/SymmetryDetector.h"
#include "util/util.h"

namespace sg {
namespace hierarchy {

// Class representing a top-down constructed hierarchy of ClusteredMesh parts
class Hierarchy {
 public:
  // Convenience typedefs
  typedef mesh::ClusterMesh ClusterMesh;
  typedef mesh::TriMesh Mesh;
  typedef ClusterMesh::Cluster Cluster;
  typedef geo::Vec3f Vec3f;
  typedef geo::Transform Transform;

  // Convenience typedefs
  typedef ClusterMesh::Part Part;
  typedef size_t PartHandle;
  typedef vec<PartHandle> VecPartHandle;
  typedef vec<VecPartHandle> VecVecPartHandle;
  struct PartSet { VecPartHandle parts; size_t parent; size_t child; };
  typedef vec<PartSet> PartPool;
  typedef vec<ClusterMesh::Part*> VecPartPointer;

  // Sorting axis for children
  enum SplitType { sym = 0, x = 1, y = 2, z = 3, none = 4 };

  // Represents a node in the part hierarchy
  struct Node {
    typedef size_t Handle;

    Node(const Hierarchy* _hierarchy, size_t _k)
      : hierarchy(_hierarchy)
      , k(_k)
      , partIndices_()
      , largestPartIndex_(0)
      , obb()
      , symmetry(symmetry::SymmetryDetector::SYMMETRY_NONE)
      , children_()
      //, visMesh(nullptr)
      //, visSymmetry(nullptr)
    { }

    Node(const Hierarchy* _hierarchy, size_t _k,
         const VecPartHandle& _partIdxs)
      : hierarchy(_hierarchy)
      , k(_k)
      , symmetry(symmetry::SymmetryDetector::SYMMETRY_NONE)
      , children_()
      //, visMesh(nullptr)
      //, visSymmetry(nullptr)
    { setParts(_partIdxs); }

    bool hasSymmetry() const {
      return symmetry.type != symmetry::SymmetryDetector::SymmetryType::None;
    }

    const VecPartHandle& getParts() const { return partIndices_; }
    void setParts(const VecPartHandle& _partIdxs) {
      partIndices_ = _partIdxs;
      largestPartIndex_ = hierarchy->largestPart(partIndices_);
      obb = hierarchy->obbFromPartIndices(partIndices_);
    }
    size_t nParts() const { return partIndices_.size(); }
    PartHandle largestPart() const { return largestPartIndex_; }
    const set<Handle>& getChildren() const { return children_; }
    void setChildren(const set<Handle>& _c) { children_ = _c; }
    void addChild(const Handle _c) { children_.insert(_c); }
    void dropAllChildren() { children_.clear(); }
    // Removes children with indices in _c and decrements all children handles
    // greater than removed children to preserve links
    void dropChildren(const set<Handle>& toDrop) {
      set<Handle> toReinsert;
      for (const Handle dropped : toDrop) {
        for (auto it = begin(children_); it != end(children_); ) {
          const Handle c = *it;
          if (c == dropped) { children_.erase(it++); }
          else if (c > dropped && toDrop.count(c) == 0) {
            // Remove and decrement by smaller index nodes dropped so far
            children_.erase(it++);
            size_t nDroppedSmallerThanMe = 0;
            for (auto d : toDrop) if (d < c) { nDroppedSmallerThanMe++; }
            toReinsert.insert(c - nDroppedSmallerThanMe);
          } else { it++; }
        }
      }
      children_.insert(begin(toReinsert), end(toReinsert));
    }
    template <typename Iter> void addChildren(const Iter beg, const Iter end) {
      children_.insert(beg, end);
    }

    const Hierarchy* hierarchy;
    const size_t k;
    geo::OBB obb;
    symmetry::SymmetryDetector::Symmetry symmetry;
    //TriMeshObject* visMesh;
    //BaseObject* visSymmetry;

   private:
    // Handles of parts contained in this node
    VecPartHandle partIndices_;
    // Index of largest contained part
    PartHandle largestPartIndex_;
    // Handles of all children nodes
    set<Handle> children_;
  };

  // cmesh: ClusterMesh with Clusters which will be sampled to create base parts
  //   of hierarchy
  // nSamples: total number of point samples across all Hierarchy Clusters
  // contactThreshDiagPerc: threshold distance for part contact expressed as
  //   percentage of part's OBB diagonal length
  Hierarchy(ClusterMesh* cmesh, size_t nSamples,
            float contactThreshDiagPerc = 0.001);

  ~Hierarchy();

  // ID used for storage in OpenFlipper objects
  static const char* ID;

  // Return root node of this hierarchy
  Node* root() const { return nodes_[0][0]; };

  // Return all nodes in this hierarchy, at the level with index iLevel
  const vec<Node*>& getNodeLevel(size_t iLevel) {
    return nodes_[iLevel];
  }

  // Create a new child node for referenced parent
  Node::Handle newChildNode(size_t parentLevel, const Node::Handle iParent);

  // Delete node at given hierarchy level and with given handle
  void deleteNode(size_t iLevel, const Node::Handle iNode);

  // Destroy all nodes
  void clearNodes();

  // TODO(ms): These break encapsulation! Remove and adjust structure.
  // Set nodes
  void setNodes(const vec<vec<Node*>>& nodes) {
    // TODO(ms): Fix this memory leak issue
    //clearNodes();
    nodes_ = nodes;
  }
  const vec<vec<Node*>>& getNodes() const { return nodes_; }

  // Return node at given hierarchy level and with given handle
  Node* getNode(const size_t iLevel, const Node::Handle iNode) const {
    return nodes_[iLevel][iNode];
  }

  const ClusterMesh* cmesh() const { return cmesh_; }

  // Start hierarchy construction
  // findSymmetries: whether to detect dominant symmetries to guide splitting
  // doContactCorrections: whether to enforce all parts within a hierarchy node
  //  are in contact with each other
  // middleSplitThreshDiagPerc: threshold distance for splitting into middle
  //   hierarchy node expressed as percentage of parent node's OBB diagonal
  // mergeAreaThreshPerc: threshold for preventing merging of approximately
  //   equal sized parts, expressed as % of area of smallest child node
  void construct(bool findSymmetries, bool doContactCorrections,
                 float middleSplitThreshDiagPerc = 0.01,
                 float mergeAreaThreshPerc = 0.95);

  //// Show node meshes at k-th level
  //void showKthLevelNodes(const size_t k = 0) {
  //  visitNodes(
  //  [k](const Node * n) { if (n->k == k && n->visMesh) { n->visMesh->hide(); }},
  //  root());
  //}

  //// Hide all nodes under tree at node (or root by default)
  //void hideNodeTree(Node* node = nullptr) {
  //  if (node == nullptr) { node = root(); }
  //  visitNodes([](const Node * n) { if (n->visMesh) { n->visMesh->hide(); }}, node);
  //}

  // Return pointer to part corresponding to given part handle
  const Part* getPart(const PartHandle ph) const { return parts_[ph]; }

  // Get all parts in this hierarchy as part pointer vector
  const VecPartPointer& getParts() const { return parts_; }

  // Get vector of part pointers corresponding to range of vector handles
  // defined by pIsBegin and pIsEnd
  VecPartPointer getParts(VecPartHandle::const_iterator phsBegin,
                          VecPartHandle::const_iterator phsEnd) const {
    VecPartPointer partSubset(phsEnd - phsBegin);
    std::transform(phsBegin, phsEnd, begin(partSubset),
    [this] (const PartHandle & ph) { return parts_[ph]; });
    return partSubset;
  }

  // Get vector of part pointers corresponding to vector of handles passed in
  VecPartPointer getParts(const VecPartHandle& phs) const {
    return getParts(begin(phs), end(phs));
  }

  // Return a VecVec3f containing world-space centroids of all parts
  // (indexed by part index)
  geo::VecVec3f getPartWorldCentroids() const;

  // Return root node local-space centroids of all parts (indexed by part index)
  geo::VecVec3f getPartLocalCentroids() const { return partLocalCs_; }

  // Return the graph of contacts between parts of this hierarchy
  const mesh::ContactDeterminator::Graph getPartContactGraph() const {
    return contactGraph_;
  }

  // Return nearest part to given queryPt within hierarchy. NOTE: queryPt is
  // expresed with coordinates in root node (global) coordinate frame for this
  // hierarchy
  PartHandle nearestPart(const Vec3f& queryPt) const;

  // Find handle of largest part (by area) within vector of parts
  PartHandle largestPart(const VecPartHandle& phs) const;

  // Visit all nodes in hierarchy (DFS order) and call passed visitor function
  // on each node
  void visitNodes(const std::function<void(const Node*)>& visitor,
                  const Node* parent);

  size_t numNodeLevels() const { return nodes_.size(); }

  // Return a random pair's center point (of one part) and offset to other part
  static pair<const Vec3f, const Vec3f>
  getRandomPartPairBaseAndOffset(const VecPartPointer& parts);

  // Separate and merge parts within nodes of a given hierarchy level so that
  // parts within a node are connected to each other in the underlying mesh
  static PartPool enforcePartConnectivity(const PartPool& children,
                                          float mergeAreaThreshPerc, const Hierarchy& H,
                                          bool postMatchConnectivity);

  // Merge separated children parts into nTarget nodes
  static PartPool merge(const PartPool& separated, size_t nTarget,
                        float mergeAreaThreshPerc, const Hierarchy& H,
                        bool postMatchConnectivity);

  // Given a vector of part splits (result of splitting operations),
  // create a PartPool that flattens out the part sets and parent indices
  // Return the total number of children created (= length of pool vectors)
  static size_t createPartPool(const vec<VecVecPartHandle>& childParts,
                               PartPool* pool);

  // Sort a PartPool by the area of each part set, also saving the total set
  // area in areas, indexed by the same sorted order as the PartPool itself
  static PartPool sortPartPoolByArea(const PartPool& pool, const Hierarchy& H,
                                     vecf* areas = nullptr);

  // Return PartPool corresponding to children nodes in children. Parents nodes
  // looked up to also establish parent-child connections in PartPool.
  // Return number of children nodes.
  static size_t getChildPartPoolFromNodeLevels(const vec<Node*>& parents,
                                               const vec<Node*>& children,
                                               PartPool* pool);

 private:
  Hierarchy(const Hierarchy&);
  Hierarchy& operator=(const Hierarchy&);

  // Split parent parts with priority order: symmetry, x, y, z
  // Two children, optional third child if part centroid close to split plane
  SplitType split(const Node* parent, VecVecPartHandle* bins,
                  float midSplitThreshDiagPerc);

  // Iteratively split a root top-bottom to construct a hierarchy
  // Terminate when no more splits can happen at a given child level
  void topDownSplitting(size_t k,
                        bool findSymmetry,
                        bool doContactCorrections = true,
                        float middleSplitThreshDiagPerc = 0.01,
                        float mergeAreaThreshPerc = 0.95);

  // Create a splitting pane for a given parent node and split type
  geo::Plane splittingPlane(const Node& parent, const SplitType split) const;

  // Return OBB containing all parts referenced by partIndices
  geo::OBB obbFromPartIndices(const VecPartHandle& partIndices) const;

  // MEMBER VARIABLES

  // Pointer to parent clustered mesh
  const ClusterMesh* cmesh_;

  // Vector of parts on which hierarchy is constructed
  const VecPartPointer parts_;

  // Part centroid positions in root node's OBB local space
  geo::VecVec3f partLocalCs_;

  // KD Tree of part centroids (in root local space)
  // Used for retrieval of nearest parts when matching within hierarchy
  //const sg::ann::ANN<sg::geo::VecVec3f, float>* partLocalCsKdtree_;

  // Part-Part contact graph with edges for each pair of parts in contact
  const mesh::ContactDeterminator::Graph contactGraph_;

  // Hierarchy levels: i-th vector contains all nodes at i-th level
  vec<vec<Node*>> nodes_;
};

}  // namespace hierarchy
}  // namespace sg


