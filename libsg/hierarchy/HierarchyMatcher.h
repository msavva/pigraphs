#pragma once

#include "libsg.h"  // NOLINT
#include "hierarchy/Hierarchy.h"

namespace sg {
namespace hierarchy {
namespace HierarchyMatcher {

// Convenience typedefs
typedef Hierarchy::Node Node;
typedef vec<vec<Node*>> NodeTree;

// Given two hierarchies S and T, compute a transformed node tree for each that
// establishes a one-to-one correspondence between their respective nodes. The
// NodeTrees have exactly the same number of nodes and structure but parts from
// each hierarchy are assigned to the nodes with a nearest neighbor approach
pair<NodeTree, NodeTree>
matchHierarchies(const Hierarchy& S, const Hierarchy& T);

// Match parent parts to child nodes by nearest world space centroid search
// Return set of handles to nodes which were not matched with any parts
set<Node::Handle>
matchPartsToChildNodes(const geo::VecVec3f& tgtPartWorldCs,
                       const vec<Node*>& srcParents,
                       const vec<Node*>& tgtParents,
                       const vec<Node*>& srcChildren,
                       const vec<Node*>& tgtChildren);


// Merge children nodes with handles given in emptyNodes with the closest
// non-empty children nodes of the same parent node
void mergeWithClosestNode(const vec<Node*>& parents,
                          const vec<Node*>& children,
                          const set<Node::Handle>& emptyNodes);

void enforceConnectivityInChildLevel(const vec<Node*>& parents,
                                     vec<Node*>& children,
                                     const Hierarchy& H);

void assertEquivalentStructure(const vec<Node*>& S_k,
                               const vec<Node*>& S_k1,
                               const vec<Node*>& T_k,
                               const vec<Node*>& T_k1);

// Check that each child node is only referred to by one parent (tree structure)
// and that all children nodes inherit parts only from their parent nodes
void assertProperParentChildInheritance(const vec<Node*>& parents,
                                        const vec<Node*>& children);

inline void createPartSamplesKdTree(const Hierarchy& H) {
  const Hierarchy::VecPartPointer& parts = H.getParts();
  for (size_t iPart = 0; iPart < parts.size(); iPart++) {
  }
}

}  // namespace HierarchyMatcher
}  // namespace hierarchy
}  // namespace sg


