#include "common.h"  // NOLINT

#include "hierarchy/HierarchyMatcher.h"

#include <limits>

namespace sg {
namespace hierarchy {

pair<HierarchyMatcher::NodeTree, HierarchyMatcher::NodeTree>
HierarchyMatcher::matchHierarchies(const Hierarchy& S, const Hierarchy& T) {
  // Get T parts and store world centroids for matching
  const geo::VecVec3f& TpartWorldCs = T.getPartWorldCentroids();

  NodeTree Snodes = S.getNodes();
  NodeTree Tnodes(1);
  Tnodes[0].push_back(T.root());
  Tnodes.resize(Snodes.size());  // Create empty node levels in T

  // For all S hierarchy levels starting at k = 0
  for (size_t k = 0; k < Snodes.size() - 1; k++) {
    const vec<Node*>& S_k = Snodes[k];
    const vec<Node*>& T_k = Tnodes[k];
    vec<Node*>& S_k1 = Snodes[k + 1];
    vec<Node*>& T_k1 = Tnodes[k + 1];
    assert(T_k.size() == S_k.size());

    // Create T children nodes
    const size_t nk1Children = S_k1.size();
    for (size_t iChild = 0; iChild < nk1Children; iChild++) {
      T_k1.push_back(new Node(&T, k + 1));
    }

    // Copy child pointers of parent S level into T parent level
    const size_t nParents = S_k.size();
    for (size_t iParent = 0; iParent < nParents; iParent++) {
      T_k[iParent]->setChildren(S_k[iParent]->getChildren());
    }

    ////
    // Assign T parts to T nodes by nearest neighbor matching (compare S node
    // positions to T part positions in coord system of parent node)
    ////
    set<Node::Handle> empties =
      matchPartsToChildNodes(TpartWorldCs, S_k, T_k, S_k1, T_k1);

    if (empties.size() > 0) {
      ////
      // Merge stage for fixing up nodes that are empty in T by deleting and
      // merging corresponding S nodes to closest non-empty S node
      ////
      mergeWithClosestNode(S_k, S_k1, empties);

      // Adjust parent indices to skip over empty node in both S and T
      for (size_t iParent = 0; iParent < S_k.size(); iParent++) {
        S_k[iParent]->dropChildren(empties);
        T_k[iParent]->setChildren(S_k[iParent]->getChildren());
      }

      // Now it is safe to remove all empty nodes from T and S
      for (const size_t iEmpty : empties) {
        if (T_k1[iEmpty] != nullptr) { delete T_k1[iEmpty]; }
        if (S_k1[iEmpty] != nullptr) { delete S_k1[iEmpty]; }
        T_k1[iEmpty] = nullptr;
        S_k1[iEmpty] = nullptr;
      }

      std::function<bool(const Node*)> remover = [] (const Node * n) {
        return n == nullptr;
      };
      auto endIt = remove_if(begin(T_k1), end(T_k1), remover);
      T_k1.resize(endIt - begin(T_k1));
      endIt = remove_if(begin(S_k1), end(S_k1), remover);
      S_k1.resize(endIt - begin(S_k1));
    }

    ////
    // Enforce no disconnected parts in T node hierarchy
    ////
    assertEquivalentStructure(S_k, S_k1, T_k, T_k1);
    enforceConnectivityInChildLevel(T_k, T_k1, T);
    //enforceConnectivityInChildLevel(S_k, S_k1, S);

    // Check that the structures of both child and parent levels are now
    // equivalent between S and T
    assertEquivalentStructure(S_k, S_k1, T_k, T_k1);
  }

  return make_pair(Snodes, Tnodes);
}

void HierarchyMatcher::enforceConnectivityInChildLevel
(const vec<Node*>& parents, vec<Node*>& children,
 const Hierarchy& H) {
  // Get adjusted child node part sets
  const float mergeAreaThresh = 0.95f;

  Hierarchy::PartPool childParts;
  const size_t numChildren =
    Hierarchy::getChildPartPoolFromNodeLevels(parents, children, &childParts);
  assert(childParts.size() == numChildren);

  const Hierarchy::PartPool& childParts2 =
    Hierarchy::enforcePartConnectivity(childParts, mergeAreaThresh, H, true);

  // Number of children should not have changed
  assert(childParts2.size() == childParts.size());

  // Delete existing child nodes and remove references from parents
  for (size_t iChild = 0; iChild < children.size(); iChild++) {
    if (children[iChild] != nullptr) { delete children[iChild]; }
  }
  children.clear();
  for (size_t iParent = 0; iParent < parents.size(); iParent++) {
    parents[iParent]->dropAllChildren();
  }

  // Recreate child nodes and update child pointers for parents
  const size_t k1 = parents[0]->k + 1;
  for (size_t iChild = 0; iChild < numChildren; iChild++) {
    children.push_back(new Node(&H, k1));
  }
  for (size_t i = 0; i < numChildren; i++) {
    const Hierarchy::VecPartHandle& child = childParts2[i].parts;
    const size_t iChild = childParts2[i].child;
    children[iChild]->setParts(child);
    const size_t iParent = childParts2[i].parent;
    parents[iParent]->addChild(iChild);
  }

  assertProperParentChildInheritance(parents, children);
}

set<Hierarchy::Node::Handle>
HierarchyMatcher::matchPartsToChildNodes(const geo::VecVec3f& tgtPartWorldCs,
                                         const vec<Node*>& srcParents,
                                         const vec<Node*>& tgtParents,
                                         const vec<Node*>& srcChildren,
                                         const vec<Node*>& tgtChildren) {
  typedef Node::Handle NodeHandle;
  typedef Hierarchy::VecPartHandle VecPartHandle;
  const size_t nParents = srcParents.size();
  assert(nParents == tgtParents.size());
  const size_t nChildren = tgtChildren.size();
  assert(nChildren == srcChildren.size());

  // Precompute centroids of target child nodes in parent coords assuming that
  // they map to the equivalent position of source child nodes in their parent
  // node's local coordinate systems
  geo::VecVec3f tgtChildLocalCs(nChildren);
  for (size_t iParent = 0; iParent < nParents; iParent++) {
    const Node* srcParent = srcParents[iParent];
    const Node* tgtParent = tgtParents[iParent];
    assert(srcParent->getChildren() == tgtParent->getChildren());
    for (const NodeHandle ch : tgtParent->getChildren()) {
      const Node* srcChild = srcChildren[ch];
      const Node* tgtChild = tgtChildren[ch];
      tgtChildLocalCs[ch] =
        srcParent->obb.worldToLocal() * srcChild->obb.centroid();
    }
  }

  map<NodeHandle, VecPartHandle> childPartVectors;
  for (const Hierarchy::Node* parent : tgtParents) {
    const Hierarchy::VecPartHandle& parentParts = parent->getParts();
    for (Hierarchy::PartHandle iPart : parentParts) {
      const geo::Vec3f partLocalC =
        parent->obb.worldToLocal() * tgtPartWorldCs[iPart];
      NodeHandle nearestChild;
      float minDist = std::numeric_limits<float>::max();
      for (const NodeHandle child : parent->getChildren()) {
        const geo::Vec3f& childLocalC = tgtChildLocalCs[child];
        const float dist = (partLocalC - childLocalC).squaredNorm();
        if (dist < minDist) { minDist = dist; nearestChild = child; }
      }
      childPartVectors[nearestChild].push_back(iPart);
    }

    // Make sure that children nodes can only contain parts from parent nodes
    for (const NodeHandle child : parent->getChildren()) {
      for (auto part : childPartVectors[child]) {
        assert(std::find(begin(parentParts), end(parentParts), part)
               != end(parentParts));
      }
    }
  }

  // Assign parts if matching parts, otherwise mark empty nodes for merging
  set<NodeHandle> emptyHandles;
  for (size_t iChild = 0; iChild < nChildren; iChild++) {
    if (childPartVectors[iChild].size() > 0) {
      tgtChildren[iChild]->setParts(childPartVectors[iChild]);
    } else { emptyHandles.insert(iChild); }
  }

  cout << "nEmpty=" << emptyHandles.size() << endl;

  return emptyHandles;
}

void HierarchyMatcher::mergeWithClosestNode(const vec<Node*>& parents,
                                            const vec<Node*>& children,
                                            const set<Node::Handle>& emptyNodes) {
  const size_t nParents = parents.size();
  const size_t nChildren = children.size();

  assertProperParentChildInheritance(parents, children);

  for (const Node::Handle iNode : emptyNodes) {
    // Find empty's parent (tree, so only one parent node can refer to it)
    size_t iParent = nParents + 1;  // Guard value outside parent index range
    for (size_t i = 0; i < nParents; i++) {
      const set<size_t>& children = parents[i]->getChildren();
      auto it = find(begin(children), end(children), iNode);
      if (it != end(children)) { iParent = i;  break; }
    }
    assert(iParent < nParents);  // Parent must exist

    cout << "Merging node i=" << iNode << ", parentI=" << iParent
              << " with closest non-empty node..." << endl;

    // Find closest child node of parent in S which has parts assigned in T
    Node* parent = parents[iParent];
    float minDist = std::numeric_limits<float>::max();
    Node::Handle iNearest = nChildren + 1;
    const geo::Vec3f& emptyC = children[iNode]->obb.centroid();
    for (size_t iOther : parent->getChildren()) {
      if (emptyNodes.count(iOther) > 0) { continue; }  // Found another empty
      const geo::Vec3f& otherC = children[iOther]->obb.centroid();
      float dist = (emptyC - otherC).squaredNorm();
      if (dist < minDist) {
        minDist = dist;
        iNearest = iOther;
      }
    }
    assert(iNearest < nChildren);

    // Merge S counterpart with closest other S node that has parts in T
    Node* nearestNode = children[iNearest];
    const Hierarchy::VecPartHandle& Sk1Parts = children[iNode]->getParts();
    const set<Node::Handle>& Sk1children = children[iNode]->getChildren();
    Hierarchy::VecPartHandle mergedParts(nearestNode->getParts());
    mergedParts.insert(mergedParts.end(), begin(Sk1Parts), end(Sk1Parts));
    nearestNode->setParts(mergedParts);
    nearestNode->addChildren(begin(Sk1children), end(Sk1children));

    // Sanity check: we didn't merge in parts that parent node did not contain
    const Hierarchy::VecPartHandle& parentPs = parent->getParts();
    for (auto part : nearestNode->getParts()) {
      assert(std::find(begin(parentPs), end(parentPs), part) != end(parentPs));
    }
  }

  assertProperParentChildInheritance(parents, children);
}

void HierarchyMatcher::assertProperParentChildInheritance
(const vec<Node*>& parents, const vec<Node*>& children) {
  const size_t nParents = parents.size();
  const size_t nChildren = children.size();
  // All children have parts and are owned by exactly one parent node
  for (size_t iChild = 0; iChild < nChildren; iChild++) {
    assert(children[iChild]->nParts() > 0);
    size_t count = 0;
    for (const Node* parent : parents) {
      count += parent->getChildren().count(iChild);
    }
    assert(count == 1);
  }

  // All children parts contained by their parent nodes
  for (size_t iParent = 0; iParent < nParents; iParent++) {
    const Hierarchy::VecPartHandle& pParts = parents[iParent]->getParts();
    for (const Node::Handle iChild : parents[iParent]->getChildren()) {
      const Hierarchy::VecPartHandle& cParts = children[iChild]->getParts();
      for (Hierarchy::PartHandle ph : cParts) {
        assert(std::find(begin(pParts), end(pParts), ph) != end(pParts));
      }
    }
  }
}

void HierarchyMatcher::assertEquivalentStructure(const vec<Node*>& S_k,
                                                 const vec<Node*>& S_k1,
                                                 const vec<Node*>& T_k,
                                                 const vec<Node*>& T_k1) {
  assert(S_k1.size() == T_k1.size());
  assert(S_k.size() == T_k.size());

  // k-th levels should have identical and valid references to k+1-th levels
  for (size_t iParent = 0; iParent < S_k.size(); iParent++) {
    const set<size_t> Schildren = S_k[iParent]->getChildren();
    const set<size_t> Tchildren = T_k[iParent]->getChildren();
    assert(Schildren == Tchildren);
    if (Schildren.size() == 0) { continue; }
    const size_t SmaxChild = *max_element(begin(Schildren), end(Schildren));
    assert(SmaxChild < S_k1.size());
    const size_t TmaxChild = *max_element(begin(Tchildren), end(Tchildren));
    assert(TmaxChild < T_k1.size());
  }
}

}  // namespace hierarchy
}  // namespace sg
