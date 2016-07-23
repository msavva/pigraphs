#pragma once

#include "libsg.h"  // NOLINT
#include "geo/geo.h"
#include "util/util.h"

namespace sg {
namespace symmetry {

// Here live the symmetry detection algorithms for mesh hierarchy nodes
namespace SymmetryDetector {

// Convenience typedefs
typedef sg::geo::Transform Transform;
typedef sg::geo::Plane Plane;
typedef sg::geo::Vec3f Vec3f;
typedef sg::geo::VecVec3f VecVec3f;

// Representation of different symmetry types
enum SymmetryType { Reflection = 0, Translation, Rotation, None };
struct Symmetry { SymmetryType type; Transform T; Plane plane; float support; };
const Symmetry SYMMETRY_NONE = { SymmetryType::None, Transform(), Plane(), -1.0f };

// Minimum number of trials to check for symmetries at a given node
const size_t MIN_SYMMETRY_TRIALS = 20;

// Minimum percentage support for dominant symmetry selection
const size_t MIN_SYMMETRY_SUPPORT_PERC = 80;

// Similarity function for two parts, currently just average normalized
// difference over each OBB axis size.  Axes are evaluated in world space with
// Transform Tb being applied to Part b first
template <typename Part>
float partSimilarity(const Part& a, const Part& b, const Transform& Tb) {
  const Vec3f& aS = a.obb.worldAxes().rowwise().norm();
  const Vec3f& bS = (Tb.linear() * b.obb.worldAxes()).rowwise().norm();
  float avgNormDiff = 0;
  for (size_t i = 0; i < 3; i++) {
    avgNormDiff += abs(aS[i] - bS[i]) / std::max(aS[i], bS[i]);
  }
  avgNormDiff /= 3;
  const float sim = 1 - avgNormDiff;
  assert(sim >= 0.f && sim <= 1.0f);
  return sim;
}

template <typename Hierarchy>
float supportEval(const Hierarchy& H,
                  const typename Hierarchy::VecPartHandle& partHs,
                  const Transform& xform) {
  const Transform& loc2world = H.root()->obb.localToWorld();
  const Transform& world2local = H.root()->obb.worldToLocal();
  const Transform localXform = world2local * xform;
  const float thresh = 0.25;
  typedef typename Hierarchy::PartHandle PartHandle;
  typedef typename Hierarchy::Part Part;
  float support = 0;
  const VecVec3f& partLocalCentroids = H.getPartLocalCentroids();
  for (typename PartHandle ph : partHs) {
    // Get nearest part to transformed centroid position in local space
    const Part* p = H.getPart(ph);
    const Vec3f& cLocalPrime = localXform * H.getPart(ph)->obb.centroid();
    const PartHandle nearestHandle = H.nearestPart(cLocalPrime);
    const Vec3f& nearestLocalC = partLocalCentroids[nearestHandle];

    // Ignore nearest if dist > quarter of part bounding box diagonal.
    // Distance is between centroids in world space. Transform does not use
    // translation component since this is a displacement
    const float dist =
      (loc2world.linear() * (nearestLocalC - cLocalPrime)).norm();
    if (dist > thresh * p->obb.diagonalLength()) { continue; }

    // Get nearest part and compute similarity to current part, add into support
    const Part* nearest = H.getPart(nearestHandle);
    const float sim = partSimilarity(*nearest, *p, xform);
    support += sim * p->samples.area();
  }

  return support;
}

// Generate nTrials candidate symmetries and evaluate their support in the
// given parts.  Returns pair of Transform with max support and the support
template <typename Hierarchy>
std::tuple<Transform, Plane, float>
dominantSymmetry(const Hierarchy& H, const typename Hierarchy::Node& node,
                 const SymmetryType symType, const size_t nTrials) {
  Transform maxXform;
  Plane maxPlane;
  float maxSupport = -1.0f;
  vecf support(nTrials);
  const typename Hierarchy::VecPartHandle& partHandles = node.getParts();
  const typename Hierarchy::VecPartPointer parts = H.getParts(partHandles);
  const Vec3f parentWorldC = node.obb.centroid();
  for (size_t iTrial = 0; iTrial < nTrials; iTrial++) {
    // Candidate symmetry: pick two random parts, find symmetry plane between
    const pair<const Vec3f, const Vec3f>& cd =
      H.getRandomPartPairBaseAndOffset(parts);
    Transform xform;
    Plane plane;

    if (symType == SymmetryType::Reflection) {
      const Vec3f& p = cd.first + 0.5 * cd.second;  // centr_a + 0.5 * dir_to_b
      plane = Plane(p, cd.second.normalized());
      xform = plane.reflection();
    } else {
      // Assume Translation for now since we don't use Rotational sym
      assert(symType == SymmetryType::Translation);
      plane = Plane(parentWorldC, cd.second.normalized());
      xform = Eigen::Translation<float, 3>(cd.second);
    }

    float support = xform.matrix().allFinite() ?
                    supportEval(H, partHandles, xform) : 0;
    if (support > maxSupport) {
      maxXform = xform;
      maxPlane = plane;
      maxSupport = support;
    }
  }
  return std::make_tuple(maxXform, maxPlane, maxSupport);
}

// Test for dominant reflection, then for dominant translation and set in Node
// If support is less than threshold, null symmetry with negative support set
template <typename Hierarchy>
void setDominantSymmetry(const Hierarchy& H, typename Hierarchy::Node* node) {
  // TODO(ms): Allow for detecting self-symmetries by removing this return
  if (node->nParts() < 2) { return; }

  const size_t nTrials = std::max(MIN_SYMMETRY_TRIALS, node->nParts());
  const float threshSupport = MIN_SYMMETRY_SUPPORT_PERC / 100.0f;

  typedef const std::tuple<Transform, geo::Plane, float> TransSup;
  const typename Hierarchy::VecPartPointer parts = H.getParts(node->getParts());
  // TODO(ms): No area weight?
  float totalArea = 0;
  for (auto p : parts) { totalArea += p->samples.area(); }
  const size_t nParts = parts.size();
  const float supportNorm = totalArea;
  Symmetry& sym = node->symmetry;
  // util::println("Finding dominant reflection...");
  TransSup& maxRe =
    dominantSymmetry(H, *node, SymmetryType::Reflection, nTrials);
  const float ReSupport = std::get<2>(maxRe) / supportNorm;
  if (ReSupport > threshSupport) {
    util::print("Reflection with support=");  util::println(ReSupport);
    sym.type = SymmetryType::Reflection;
    sym.T = std::get<0>(maxRe);
    sym.plane = std::get<1>(maxRe);
    sym.support = ReSupport;
    return;
  }

  // util::println("Finding dominant translation...");
  TransSup& maxT =
    dominantSymmetry(H, *node, SymmetryType::Translation, nTrials);
  const float TSupport = std::get<2>(maxT) / supportNorm;
  if (TSupport > threshSupport) {
    util::print("Translation with support=");  util::println(TSupport);
    sym.type = SymmetryType::Translation;
    sym.T = std::get<0>(maxT);
    sym.plane = std::get<1>(maxT);
    sym.support = TSupport;
    return;
  }

  util::println("No dominant symmetry found!");
  sym.type = SymmetryType::None;
  sym.T = Transform();
  sym.plane = geo::Plane();
  sym.support = -1.0f;
  return;
}

}  // namespace SymmetryDetector
}  // namespace symmetry
}  // namespace sg


