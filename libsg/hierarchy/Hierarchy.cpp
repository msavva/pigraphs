#include "common.h"  // NOLINT

#include "hierarchy/Hierarchy.h"

#include <functional>
#include <limits>

#include <boost/range/adaptor/map.hpp>

#include "math/math.h"

namespace sg {
namespace hierarchy {

const char* Hierarchy::ID = "SegHierarchy";

Hierarchy::Hierarchy(ClusterMesh* cmesh, size_t nSamples,
                     float contactThreshDiagPerc /* = 0.001 */)
  : cmesh_(cmesh)
  , parts_(cmesh->partsFromClusters(nSamples))
  , partLocalCs_(parts_.size())
  , contactGraph_(mesh::ContactDeterminator::contactGraph(parts_,
                                                    contactThreshDiagPerc))
  , nodes_(1) {
  // Construct root node with all parts
  const size_t nParts = parts_.size();
  VecPartHandle all(nParts);
  for (size_t i = 0; i < nParts; i++) { all[i] = i; }
  nodes_[0].push_back(new Node(this, 0, all));

  // Transform part centroids to root's OBB coordinates and create K-D tree
  const geo::Transform& rootT = root()->obb.worldToLocal();
  for (size_t i = 0; i < nParts; i++) {
    partLocalCs_[i] = rootT * parts_[i]->obb.centroid();
  }
  //partLocalCsKdtree_ = new sg::ann::ANN<sg::geo::VecVec3f, float>(partLocalCs_, 5);

  // Detect top level symmetry
  symmetry::SymmetryDetector::setDominantSymmetry(*this, root());
}

Hierarchy::~Hierarchy() {
  clearNodes();
  for (auto p : parts_) if (p) { delete p; }
  //if (partLocalCsKdtree_) { delete partLocalCsKdtree_; }
}

Hierarchy::Node::Handle
Hierarchy::newChildNode(size_t parentLevel, const Node::Handle iParent) {
  Node* parent = getNode(parentLevel, iParent);
  if (nodes_.size() < parentLevel + 1) { nodes_.resize(parentLevel + 1); }
  vec<Node*>& childLevel = nodes_[parentLevel + 1];
  childLevel.push_back(new Node(this, parentLevel + 1));
  const Node::Handle nh = childLevel.size();
  parent->addChild(nh);
  return nh;
}

void Hierarchy::deleteNode(size_t iLevel, const Node::Handle iNode) {
  assert(iLevel < nodes_.size() && nodes_[iLevel].size() < iNode);
  vec<Node*>& level = nodes_[iLevel];
  const Node* node = level[iNode];
  assert(node != nullptr);

  // Remove all children nodes
  for (Node::Handle iChild : node->getChildren()) {
    deleteNode(iLevel + 1, iChild);
  }

  // Destroy node storage
  if (level[iNode] != nullptr) { delete level[iNode]; }
  level.erase(begin(level) + iNode);

  // Remove pointers to this node from parent layer
  if (iLevel == 0) { return; }
}

void Hierarchy::clearNodes() {
  for (size_t iLevel = 0; iLevel < nodes_.size(); iLevel++) {
    vec<Node*>& level = nodes_[iLevel];
    for (size_t iNode = 0; iNode < level.size(); iNode++) {
      const Node* n = level[iNode];
      if (n != nullptr) { delete(n); }
      level[iNode] = nullptr;
    }
  }
  nodes_.clear();
}

void Hierarchy::visitNodes(const std::function<void(const Node*)>& visitor,
                           const Node* parent) {
  visitor(parent);
  for (auto ci : parent->getChildren()) {
    visitNodes(visitor, nodes_[parent->k + 1][ci]);
  }
}

pair<const geo::Vec3f, const geo::Vec3f>
Hierarchy::getRandomPartPairBaseAndOffset(const VecPartPointer& parts) {
  const VecPartPointer& pair = math::DEF_RAND.randomSubset(parts, 2);
  const geo::OBB& obbA = pair[0]->obb;
  const geo::OBB& obbB = pair[1]->obb;
  const Vec3f& ca = obbA.centroid();
  const Vec3f& d = obbB.centroid() - ca;
  return std::make_pair(ca, d);
}

Hierarchy::SplitType Hierarchy::split(const Node* parent,
                                      VecVecPartHandle* bins,
                                      float midSplitThreshDiagPerc) {
  // Helper function
  auto doSplit =
  [this, bins, parent, midSplitThreshDiagPerc] (const SplitType split) {
    bins->clear();
    bins->resize(3);  // low, mid, high sides of split plane
    const geo::Plane& plane = splittingPlane(*parent, split);
    const float midThresh =
      midSplitThreshDiagPerc * parent->obb.diagonalLength();

    // Iterate over children parts and determine side they fall on
    for (const PartHandle ph : parent->getParts()) {
      const Part* part = getPart(ph);
      const Vec3f& partWorldC = part->obb.centroid();
      float dist = plane.signedDistance(partWorldC);
      size_t index = 1;  // Middle bin as default for dist in [-mid, mid]
      if (dist < -midThresh) {
        index = 0;
      } else if (dist > midThresh) {
        index = 2;
      }
      (*bins)[index].push_back(ph);
    }

    // Successful split if at least two bins have parts
    size_t empties = 0;
    for (auto bin : *bins) if (bin.empty()) { empties++; }
    return empties < 2;
  };

  // Early check for single part base case. Just return in center bin
  if (parent->nParts() == 1) {
    bins->clear();  bins->resize(3);
    (*bins)[1].push_back(parent->getParts()[0]);
    return none;
  }

  // Symmetry
  if (parent->hasSymmetry()) {
    if (doSplit(sym)) { return sym; }
  }

  // X = 0
  if (doSplit(x)) { return x; }
  // Y = 0
  if (doSplit(y)) { return y; }
  // Z = 0
  if (doSplit(z)) { return z; }

  // Else, give up and return failure in splitting (only one bin with parts)
  return none;
}

Hierarchy::PartPool Hierarchy::merge(const PartPool& separated,
                                     size_t nTarget,
                                     float mergeAreaThreshPerc,
                                     const Hierarchy& H,
                                     bool postMatchConnectivity) {
  // Should not call merge if nTarget > nPartSets. Make parts out of thin air?
  const size_t nPartSets = separated.size();
  assert(nTarget <= nPartSets);

  // Get parts and create storage for partsets and areas
  const VecPartPointer& parts = H.getParts();
  vecf areas(nPartSets, 0.0f);
  PartPool partSets;

  // If we're doing this post-match the part sets are already sorted
  if (postMatchConnectivity) {
    partSets = separated;
    for (size_t i = 0; i < nPartSets; i++) {
      const VecPartHandle& node = partSets[i].parts;
      for (size_t iPart = 0; iPart < node.size(); iPart++) {
        areas[i] += parts[node[iPart]]->samples.area();
      }
    }
  } else {  // Else sort part pool by area
    partSets = sortPartPoolByArea(separated, H, &areas);
  }

  // Keep n largest nodes as merge targets
  PartPool kept;
  for (size_t i = 0; i < nTarget; i++) { kept.push_back(partSets[i]); }

  // Merging exception: nodes of area similar to smallest kept node
  // are kept independent as well
  size_t nKept = nTarget;
  if (!postMatchConnectivity) {
    // Assumption: the areas array is sorted in descending order. Important
    // for ordering of merges later on as well.
    const float areaThresh = mergeAreaThreshPerc * areas[nTarget - 1];
    for (size_t i = nTarget; i < nPartSets; i++) {
      if (areas[i] > areaThresh) { kept.push_back(partSets[i]);  nKept++; }
      else { break; }
    }
  }

  // Merge remaining nodes into kept nodes adjacent to them
  const size_t nRemaining = nPartSets - nKept;
  vec<bool> hasMerged(nRemaining, false);
  bool moreMerging = true;
  size_t nMerged = 0;
  while (moreMerging) {
    moreMerging = false;
    for (size_t i = nKept; i < nPartSets; i++) {
      const size_t hasMergedIdx = i - nKept;
      if (hasMerged[hasMergedIdx]) { continue; }

      const VecPartHandle& sepParts = partSets[i].parts;

      // Starting from smallest kept node first (prefer merging into smaller)
      for (size_t iKept = kept.size() - 1; iKept >= 0; iKept--) {
        const VecPartHandle& keptParts = kept[iKept].parts;

        // Are any of kept node parts contacting any currNode parts?
        if (contactPairExists(keptParts, sepParts,
                                                   H.getPartContactGraph())) {
          copy(begin(sepParts), end(sepParts),
                    back_inserter(kept[iKept].parts));
          nMerged++;
          moreMerging = true;
          hasMerged[hasMergedIdx] = true;
          break;
        }
      }
    }
  }

  // If remaining nodes (more than one part connected components)
  // then print error and continue ignoring those parts
  if (nKept + nMerged != nPartSets) {
    const size_t leftCount = nKept + nMerged - nPartSets;
    cout << "[ERROR] Failure enforcing connectivity:" << leftCount
              << " parts left disconnected...";
    if (!postMatchConnectivity) {  // Push on as independent parts
      for (size_t iLeft = nKept + nMerged; iLeft < nPartSets; iLeft++) {
        kept.push_back(partSets[iLeft]);
      }
      cout << " adding as independent parts.";
    } else {
      cout << " dropping to preserve hierarchy matching.";
    }
    cout << "Check for disconnected mesh components." << endl;
  }

  cout << "Merged from " << nPartSets << " down to " << kept.size()
            << " children nodes. " << nKept - nTarget << " area exceptions"
            << endl;

  return kept;
}

void Hierarchy::construct(bool findSymmetries,
                          bool doContactCorrections,
                          float middleSplitThreshDiagPerc, /* = 0.01 */
                          float mergeAreaThreshPerc        /* = 0.95 */) {
  // Recursively split
  topDownSplitting(0, findSymmetries, doContactCorrections,
                   middleSplitThreshDiagPerc, mergeAreaThreshPerc);
}

void Hierarchy::topDownSplitting(size_t k,
                                 bool findSymmetries,
                                 bool doContactCorrections,
                                 float midSplitThreshDiagPerc,
                                 float mergeAreaThreshPerc) {
  // For current level k, try to split each node into children at level k + 1
  assert(nodes_.size() == k + 1);
  const vec<Node*> currLevel = nodes_[k];
  const size_t numParents = currLevel.size();
  vec<VecVecPartHandle> children(numParents);
  vec<SplitType> splitTypes(numParents);
  bool atLeastOneChildSplit = false;
  for (size_t iParent = 0; iParent < numParents; iParent++) {
    Node* parent = currLevel[iParent];
    splitTypes[iParent] = split(parent, &children[iParent], midSplitThreshDiagPerc);
    atLeastOneChildSplit |= (splitTypes[iParent] != none);
  }

  // If all splits failed, terminate
  if (!atLeastOneChildSplit) { return; }

  // Push node part assignments onto part pool
  PartPool childParts;
  size_t numChildren = createPartPool(children, &childParts);

  // Do contact correction (separate-merge stage)
  if (doContactCorrections) {
    childParts = enforcePartConnectivity(childParts, mergeAreaThreshPerc,
                                         *this, false /*postMatch*/);
    numChildren = childParts.size();
  }

  // Now create resulting children nodes
  const size_t k1 = k + 1;
  nodes_.resize(k1 + 1);
  vec<Node*>& nextLevel = nodes_[k1];
  nextLevel.resize(numChildren);
  for (size_t iChild = 0; iChild < numChildren; iChild++) {
    Node* parent = currLevel[childParts[iChild].parent];
    Node* child = new Node(this, k1, childParts[iChild].parts);
    if (findSymmetries) { symmetry::SymmetryDetector::setDominantSymmetry(*this, child); }
    nextLevel[iChild] = child;
    parent->addChild(iChild);
  }

  // Iterate to next level
  topDownSplitting(k1, findSymmetries, doContactCorrections,
                   midSplitThreshDiagPerc, mergeAreaThreshPerc);
}

Hierarchy::PartPool
Hierarchy::enforcePartConnectivity(const PartPool& children,
                                   float mergeAreaThreshPerc,
                                   const Hierarchy& H,
                                   bool postMatchConnectivity) {
  // Separate all children to prevent disconnected parts in a child.
  PartPool separated;
  const size_t numChildren = children.size();
  for (size_t iChild = 0; iChild < numChildren; iChild++) {
    const VecPartHandle& child = children[iChild].parts;
    const size_t iParent = children[iChild].parent;
    const map<int, VecPartHandle>& sep =
      groupByConnectedComponent(child, H.getPartContactGraph());
    for (const VecPartHandle s : sep | boost::adaptors::map_values) {
      PartSet partSet = { s, iParent, children[iChild].child };
      separated.push_back(partSet);
    }
  }

  // In order to keep existing mapping between parents and children undisturbed
  // by the connectivity enforcement, take the numChildren largest parts coming
  // from unique children and enforce connectivity with them as targets for
  // merging.  This ensures that we can re-create children with same index
  // from the merged part sets
  if (postMatchConnectivity) {
    const size_t nSeparatedBefore = separated.size();
    const PartPool& sorted = sortPartPoolByArea(separated, H);
    set<size_t> seenChildren;
    set<size_t> partSetsPutIn;
    separated.clear();

    // First put in numChildrent larget partsets from unique children
    for (size_t i = 0; i < sorted.size(); i++) {
      const size_t iChild = sorted[i].child;
      if (seenChildren.count(iChild) == 0) {
        separated.push_back(sorted[i]);
        seenChildren.insert(iChild);
        partSetsPutIn.insert(i);
      }
      if (separated.size() == numChildren) { break; }
    }

    // Now put in the remaining partsets
    for (size_t i = 0; i < sorted.size(); i++) {
      if (partSetsPutIn.count(i) == 0) { separated.push_back(sorted[i]); }
    }

    assert(separated.size() == nSeparatedBefore);
  }

  // Now merge back separated parts while preserving connectivity and store
  // resulting nodes into parent nodes and into level vector
  const PartPool& merged = merge(separated, numChildren, mergeAreaThreshPerc, H,
                                 postMatchConnectivity);
  return merged;
}

geo::Plane Hierarchy::splittingPlane(const Node& parent,
                                     const SplitType splitType) const {
  const symmetry::SymmetryDetector::Symmetry& sym = parent.symmetry;
  const symmetry::SymmetryDetector::SymmetryType symType = sym.type;
  Vec3f n;  // Splitting plane normal
  geo::Plane plane;
  if (splitType == SplitType::sym) {
    assert(symType != sg::symmetry::SymmetryDetector::SymmetryType::None);
    plane = sym.plane;
  } else {  // Use axis splitting
    const Vec3f& parentWorldC = parent.obb.centroid();
    const Eigen::Matrix3f& R = parent.obb.normalizedAxes();
    if (splitType == x) {
      n = R.col(0);
    } else if (splitType == y) {
      n = R.col(1);
    } else {
      assert(splitType == SplitType::z);
      n = R.col(2);
    }
    plane = geo::Plane(parentWorldC, n);
  }
  return plane;
}

geo::OBB Hierarchy::obbFromPartIndices(const VecPartHandle& partIndices) const {
  const auto parts = getParts(partIndices);
  geo::VecVec3f points;
  for (auto p : parts) {
    copy(p->samples.pBegin(), p->samples.pEnd(),
              back_inserter(points));
  }
  //// TODO(ms): Push up options passed into OBB
  //const bool doConvexHull = true;
  //// TODO(ms): Fix this to use convexhull
  //if (doConvexHull) {
  //  points = geo::convexHull<Vec3f>(std::begin(points), std::end(points));
  //}

  return geo::OBB(points, geo::OBB::MIN_PCA | geo::OBB::AABB_FALLBACK);
}

geo::VecVec3f Hierarchy::getPartWorldCentroids() const {
  const size_t nParts = parts_.size();
  geo::VecVec3f cs(nParts);
  for (size_t i = 0; i < nParts; i++) { cs[i] = parts_[i]->obb.centroid(); }
  return cs;
}

Hierarchy::PartHandle Hierarchy::nearestPart(const geo::Vec3f& queryPt) const {
  vec<size_t> indices;
  vecf dists;
  // TODO(ms): Fix this by re-introducing ANN
  //partLocalCsKdtree_->queryKNN(queryPt, 1, &indices, &dists);
  return indices[0];
}

Hierarchy::PartHandle Hierarchy::largestPart(const VecPartHandle& pIs) const {
  float maxArea = 0;
  size_t maxI = 0;
  for (size_t pi : pIs) {
    float area = parts_[pi]->samples.area();
    if (area > maxArea) {
      maxArea = area;
      maxI = pi;
    }
  }
  return maxI;
}

size_t Hierarchy::createPartPool(const vec<VecVecPartHandle>& childParts,
                               PartPool* pool) {
  pool->clear();
  size_t numChildren = 0;
  for (size_t iParent = 0; iParent < childParts.size(); iParent++) {
    for (auto child : childParts[iParent]) {
      if (!child.empty()) {
        numChildren++;
        PartSet partSet = {child, iParent, -1};
        pool->push_back(partSet);
      }
    }
  }
  return numChildren;
}

size_t Hierarchy::getChildPartPoolFromNodeLevels(const vec<Node*>& parents,
                                               const vec<Node*>& children,
                                               PartPool* pool) {
  pool->clear();
  size_t numChildren = 0;
  for (size_t iParent = 0; iParent < parents.size(); iParent++) {
    for (size_t iChild : parents[iParent]->getChildren()) {
      numChildren++;
      const VecPartHandle& childParts = children[iChild]->getParts();
      PartSet partSet = {childParts, iParent, iChild};
      pool->push_back(partSet);
    }
  }
  return numChildren;
}

Hierarchy::PartPool Hierarchy::sortPartPoolByArea(const PartPool& pool,
                                                  const Hierarchy& H,
                                                  vecf* areas) {
  const VecPartPointer& parts = H.getParts();
  const size_t nPartSets = pool.size();
  vecf A(nPartSets);
  vec<size_t> sortedI;
  for (size_t iNode = 0; iNode < nPartSets; iNode++) {
    const VecPartHandle& node = pool[iNode].parts;
    for (size_t iPart = 0; iPart < node.size(); iPart++) {
      A[iNode] += parts[node[iPart]]->samples.area();
    }
  }
  util::sortIndices(begin(A), end(A), std::greater<float>(), sortedI);

  PartPool sorted;
  if (areas != nullptr) { areas->resize(nPartSets, 0.0f); }
  for (size_t i = 0; i < nPartSets; i++) {
    const PartSet& partSet = pool[sortedI[i]];
    sorted.push_back(partSet);
    if (areas != nullptr) { (*areas)[i] = A[sortedI[i]]; }
  }
  return sorted;
}

}  // namespace hierarchy
}  // namespace sg
