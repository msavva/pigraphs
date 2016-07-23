#include "common.h"  // NOLINT

#include "mesh/mesh_nn.h"

#include "ann/KdTreeFlann.h"
#include "core/Skeleton.h"
#include "geo/geo.h"
#include "math/math.h"
#include "segmentation/MeshSegment.h"

namespace sg {
namespace mesh {

using core::Skeleton;

MeshActivationRecord::MeshActivationRecord()
  : numIds(0)
  , numRawIds(0)
  , numVerts(0)
  , maxNNpoints(0) { }

MeshActivationRecord::MeshActivationRecord(int _numIds, int _numRawIds, int _numVerts, int _maxNNpoints) {
  init(_numIds, _numRawIds, _numVerts, _maxNNpoints);
}

void MeshActivationRecord::init(int _numIds, int _numRawIds, int _numVerts, int _maxNNpoints) {
  numIds = _numIds; numRawIds = _numRawIds; numVerts = _numVerts; maxNNpoints = _maxNNpoints;
  for (int i = 0; i < numIds; ++i) {
    actMap[i].resize(numVerts);
    actVertSets[i].reserve(maxNNpoints * 4);
  }
  m_nnDists.resize(maxNNpoints * numRawIds);
  m_nnIndices.resize(maxNNpoints * numRawIds);
  reset();  // clears memory
}

void MeshActivationRecord::reset() {
  for (auto& p : actMap) {
    std::fill(p.second.begin(), p.second.end(), VertWeight{0.f, 0});
  }
  for (auto& p : actVertSets) {
    p.second.clear();
  }
}

void MeshActivationRecord::colorizeVertices(int id, const ml::vec4f& idColor,
                                            vec<ml::vec4f>* pColors) const {
  assert(pColors->size() == numVerts);
  const vec<VertWeight>& weights = actMap.at(id);
  const uset<int>& activeVerts = actVertSets.at(id);
  vec<ml::vec4f>& colors = *pColors;
  bool foundWeightOutOfBounds = false;
  for (int iV : activeVerts) {
    const float w = weights[iV].weight / weights[iV].count;
    if(w > 0.f && w <= 1.f && !foundWeightOutOfBounds) {
      SG_LOG_WARN << "[MeshActivationRecord::colorizeVertices] Weight out of bounds w=" << w;
      foundWeightOutOfBounds = true;
    }
    auto& c = colors[iV];
    for (int i = 0; i < 3; ++i) {
      c[i] = math::lerp(0.f, 1.f, c[i], idColor[i], w);
    }
  }
}

void MeshActivationRecord::colorizeVerticesAllIds(const std::function<ml::vec4f(int)>& colorMap,
                                                  vec<ml::vec4f>* pColors) const {
  for (const auto& pair : actMap) {
    const int id = pair.first;
    assert(id < numIds);
    colorizeVertices(id, colorMap(id), pColors);
  }
}

MeshHelper::MeshHelper() : m_kdtree(nullptr), m_bvh(nullptr), m_mesh(nullptr) { }

MeshHelper::~MeshHelper() {
  if (m_kdtree != nullptr) { delete m_kdtree; }
  if (m_bvh != nullptr) { delete m_bvh; }
}

MeshHelper::MeshHelper(const ml::TriMeshf& mesh): m_kdtree(nullptr), m_bvh(nullptr), m_mesh(&mesh) {
  init(mesh);
}

void MeshHelper::init(const ml::TriMeshf& mesh) {
  if (m_kdtree != nullptr) { delete m_kdtree; }
  if (m_bvh != nullptr) { delete m_bvh; }
  const auto& V = mesh.getVertices();
  const size_t numVerts = V.size();
  const auto getVertPtrFun = [&] (const size_t i) { return &(V[i].position.array[0]); };
  m_kdtree = new ann::KdTree3f_FLANN(numVerts, getVertPtrFun);
  m_bvh = new ml::TriMeshAcceleratorBVHf;
  m_bvh->build(mesh);
  m_mesh = &mesh;
}

void MeshHelper::accumulateContactActivation(const float* pPoints, const int numPoints, const float maxRadius,
                                         const std::function<int(int)>& pIdxToIdFun, MeshActivationRecord* pRec) const {
  const int maxNNpoints = pRec->maxNNpoints;
  const float maxRadiusInv = 1.f / maxRadius;
  m_kdtree->queryRadius(numPoints, pPoints, maxRadius, &pRec->m_nnIndices, &pRec->m_nnDists, maxNNpoints);
  for (int iPoint = 0; iPoint < numPoints; iPoint++) {
    const int id = pIdxToIdFun(iPoint);
    assert(id < pRec->numIds);
    vec<MeshActivationRecord::VertWeight>& actMap = pRec->actMap[id];
    uset<int>& actVerts = pRec->actVertSets[id];
    const int baseIdx = iPoint * maxNNpoints;
    for (int iNeigh = 0; iNeigh < maxNNpoints; iNeigh++) {
      const int i = baseIdx + iNeigh;
      const int neighIdx = static_cast<int>(pRec->m_nnIndices[i]);
      if (neighIdx < 0) {
        break;
      }
      auto& v = actMap[neighIdx];
      v.weight += 1.f - (pRec->m_nnDists[i] * maxRadiusInv);
      v.count++;
      actVerts.insert(neighIdx);
    }
  }
};

void MeshHelper::accumulateActivation(const core::SkelRange& skelRange, float contactDistThresh, float gazeDistThresh,
                                      MeshActivationRecord* pRec) const {
  pRec->reset();
  const auto iJoint2JointGroup = [ ] (int iJoint) { return Skeleton::kJointToJointGroup[iJoint]; };
  for (const Skeleton& skel : skelRange) {
    accumulateContactActivation(&skel.jointPositions[0][0], Skeleton::kNumJoints, contactDistThresh, iJoint2JointGroup,
                                pRec);
    accumulateGazeActivation(skel, gazeDistThresh, Skeleton::JointGroup_Gaze, pRec);  // TODO: Clean up index hack
  }
}

bool MeshHelper::intersect(const ml::Rayf& r, IntersectionRecord* rec, float tmin /*= 0.f*/,
                       float tmax /*= std::numeric_limits<float>::max()*/,
                       bool intersectOnlyFrontFaces /*= false*/) const {
  rec->tri = m_bvh->intersect(r, rec->t, rec->u, rec->v, tmin, tmax, intersectOnlyFrontFaces);
  if (rec->tri) {
    //TODO(MS): This is silly -- should switch segmentation to be per-face
    rec->segIdx = static_cast<int>(rec->tri->getV0().texCoord[0]);
  }
  return (rec->tri != nullptr);
}

int MeshHelper::getGazed(const Skeleton& skel, Intersections* isecs) const {
  // Reset output
  assert(isecs != nullptr);
  isecs->clear();

  // Gaze cone sampling parameters
  const float
    cosTheta = cosf(1.0f * math::constants::PI2f / 12.f),
    minDist  = isecs->minDist,
    maxDist  = isecs->maxDist;
  const MeshElementType elementType = isecs->elementType;
  const ml::mat3f frame = ml::mat3f::frame(skel.gazeDirection);
  ml::RNG rng;

  // Shoot out sampling rays and store intersections
  IntersectionRecord rec;
  for (int iElement = 0; iElement < isecs->numRays; ++iElement) {
    rec.tri = nullptr;  // Note reseting of rec.tri for each iteration
    const ml::vec3f dir = frame * ml::Samplef::squareToUniformCone(rng.uniform2D(), cosTheta);
    const ml::Rayf r(skel.gazePosition, dir);
    if (intersect(r, &rec, minDist, maxDist) && rec.segIdx >= 0) {  // 2nd rejects filtered segs VIP: This only works for non-segment aggregation because texCoords are zero by default
      // Switch on aggregation type to get appropriate id
      int id;
      switch (elementType) {
      case MeshElementType_Triangle:
        id = static_cast<int>(rec.tri->getIndex());
        break;
      case MeshElementType_Segment:
        id = rec.segIdx;
        break;
      default:
        throw ml::MLibException("[MeshHelper::getGazed] Unsupported MeshElementType for gaze aggregation: "
                                + to_string(isecs->elementType));
        break;
      }
      auto& countDistPair = isecs->id2counts[id];
      countDistPair.count++;
      countDistPair.sumDist += rec.t;
      isecs->numIntersections++;
      if (rec.t < isecs->minDistActual) {
        isecs->minDistActual = rec.t;
      }
      if (rec.t > isecs->maxDistActual) {
        isecs->maxDistActual = rec.t;
      }
    }
  }

  return isecs->numIntersections;
}

void MeshHelper::getGazedSegments(const Skeleton& skel, float maxDistGaze, const VecSegPtr& segs,
                                  VecSegPtr* gazedSegs, bool doAccumulate /* = false */) const {
  if (!doAccumulate) { gazedSegs->clear(); }  // if not accumulating, clear

  // Aggregate intersections
  const int
    numSamples  = 200,
    maxNumGazed = 3;
  const float minDistGaze = 0.1f * maxDistGaze;
  Intersections isecs(MeshElementType_Segment, numSamples, minDistGaze, maxDistGaze);
  if (getGazed(skel, &isecs) == 0) { return; }  // No intersections so early out

  // Now sort isecs in descending order using a simple normalized distance metric, retaining maxNum first
  const float
    minPercent = 0.2f,
    countNorm = std::max(minPercent, static_cast<float>(isecs.numIntersections) / isecs.numRays);

  auto& sorted = isecs.sortedElements;
  for (const auto& iter : isecs.id2counts) {
    const int
      id = iter.first,
      count = iter.second.count;
    const float
      distToSeg = iter.second.sumDist / count,  // average ray t
      distNorm = ml::math::linearMap(isecs.minDistActual, isecs.maxDistActual, 1.f, 0.f, distToSeg);
    sorted.push_back({id, count, distToSeg, count * countNorm * distNorm});
  }
  std::sort(sorted.begin(), sorted.end(), [ ] (const Intersections::SortedMeshElement& l,
                                               const Intersections::SortedMeshElement& r) {
    return l.weight > r.weight;
  });

  // Retain up to maxNumGazed elements above gaze threshold percentage
  int iElement = 0;
  for (; iElement < std::min(static_cast<int>(sorted.size()), maxNumGazed); iElement++) {
    const auto& sc = sorted[iElement];
    if (sc.weight < minPercent) { break; }
  }
  sorted.resize(iElement);
  isecs.numIntersections = iElement;

  // Pick closest candidate segment
  auto& top = isecs.sortedElements;
  std::sort(top.begin(), top.end(), [ ] (const Intersections::SortedMeshElement& l,
                                         const Intersections::SortedMeshElement& r) {
    return l.avgDist < r.avgDist;
  });
  if (!top.empty()) {
    gazedSegs->push_back(segs[top[0].id]);
  }
}

void MeshHelper::accumulateGazeActivation(const Skeleton& skel, float maxDistGaze, int gazeJointId,
                                          MeshActivationRecord* pRec) const {
  // Aggregate intersections
  const int numSamples = 10000;
  const float minDistGaze = 0.1f * maxDistGaze;
  Intersections isecs(MeshElementType_Triangle, numSamples, minDistGaze, maxDistGaze);
  if (getGazed(skel, &isecs) == 0) { return; }  // No intersections so early out

  // Sort triangles by normalized distance
  const float
    minPercent = 0.2f,
    countNorm = std::max(minPercent, static_cast<float>(isecs.numIntersections) / isecs.numRays);
  auto& sorted = isecs.sortedElements;
  for (const auto& iter : isecs.id2counts) {
    const int
      id = iter.first,
      count = iter.second.count;
    const float
      distToTri = iter.second.sumDist / count,  // average ray t
      distNorm = ml::math::linearMap(isecs.minDistActual, isecs.maxDistActual, 1.f, 0.f, distToTri);
    sorted.push_back({id, count, distToTri, count * countNorm * distNorm});
  }
  std::sort(sorted.begin(), sorted.end(), [ ] (const Intersections::SortedMeshElement& l,
    const Intersections::SortedMeshElement& r) {
    return l.weight > r.weight;
  });

  const size_t
    numSorted = sorted.size(),
    p25Idx    = static_cast<size_t>(floor(0.25 * numSorted)),
    p75Idx    = static_cast<size_t>(floor(0.75 * numSorted));

  if (p25Idx == p75Idx) { return; }  // don't add any weights since we have a degenerate range

  const float
    p25dist   = sorted[p25Idx].weight,
    p75dist   = sorted[p75Idx].weight;

  // Accumulate gaze weights
  assert(gazeJointId < pRec->numIds);
  vec<MeshActivationRecord::VertWeight>& actMap = pRec->actMap[gazeJointId];
  uset<int>& actVerts = pRec->actVertSets[gazeJointId];
  const auto& T = m_mesh->getIndices();
  const auto& V = m_mesh->getVertices();
  for (const auto& e : sorted) {
    const ml::vec3ui& triIs = T[e.id];
    for (int iDim = 0; iDim < 3; ++iDim) {
      const unsigned iVert = triIs[iDim];
      auto& v = actMap[iVert];
      float w = ml::math::linearMap(p25dist, p75dist, 1.f, 0.f, e.weight);
      w = ml::math::clamp(w, 0.f, 1.f);
      v.weight += w;
      v.count++;
      actVerts.insert(iVert);
    }
  }
}

void MeshHelper::getContactSegments(const VecSegPtr& segs, const VecSegIdx& segIndices, const Skeleton& skel, 
                                    bool ignoreInferredJoints, unsigned int kNearestSegsPerJoint, 
                                    float maxContactDist, Skeleton::SegmentsByJointPlusGaze* activeSegs, 
                                    bool doAccumulate /*= false*/) const {
  // NOTE: If contact segment selection ends up being too noisy, consider having two-tiered
  // distance threshold: under first threshold assign segments directly, under second but
  // above first, assign only to closest joint
  for (size_t iJoint = 0; iJoint < Skeleton::kNumJoints;
       iJoint++) {  // NOTE: This goes over kNumJoints (excludes gaze)
    if (!doAccumulate) { (*activeSegs)[iJoint].clear(); }  // clear if not accumulating
    if (ignoreInferredJoints && skel.jointConfidences[iJoint] == 0) { continue; }
    const vec<size_t>& segIds
      = kNearestSegments(segs, segIndices, skel.jointPositions[iJoint], kNearestSegsPerJoint, maxContactDist);
    if (segIds.empty()) { continue; }
    for (size_t iSeg : segIds) {
      const auto seg = segs[iSeg];
      (*activeSegs)[iJoint].push_back(seg);
    }
  }
}

void MeshHelper::getActiveSegments(const VecSegPtr& segs, const Skeleton& skel, bool ignoreInferredJoints,
                                   unsigned int kNearestSegsPerJoint, float maxContactDist,
                                   float maxDistGaze, float maxSegmentSize,
                                   Skeleton::SegmentsByJointPlusGaze* activeSegs,
                                   bool doAccumulate /* = false */) const {

  // TODO: Have maxDist be a input parameter
  // Filter down to scan segments within a 2m radius of the skeleton's base
  const float maxDist = 2.f;
  const auto& skelHip = skel.jointPositions[Skeleton::JointType_SpineBase];
  VecSegIdx segIndices;
  for (unsigned int segIdx = 0; segIdx < segs.size(); segIdx++) {
    const auto& seg = segs[segIdx];
    // disregard segments that are too large...
    if (maxSegmentSize > 0 && seg->diagonalLength() > maxSegmentSize) { 
      continue; 
    }
    const float distToSegOBB = seg->distance(skelHip);
    if (distToSegOBB < maxDist) {
      segIndices.push_back(segIdx);
    }
  }
  // Retrieve contact segments from filtered set
  getContactSegments(segs, segIndices, skel,
                     ignoreInferredJoints, kNearestSegsPerJoint, maxContactDist, activeSegs,
                     doAccumulate);

  // Retrieve gazed segments within the scene
  if (skel.gazeConfidence > 0.0f) {
    getGazedSegments(skel, maxDistGaze, segs, &((*activeSegs)[Skeleton::kNumJoints]),
                     doAccumulate);
  }
}

vec<size_t> MeshHelper::kNearestSegments(const VecSegPtr segs, const VecSegIdx& segIndices, const ml::vec3f& pos, unsigned int k, float maxDist) const {
  ml::KNearestNeighborQueue<float> queue(k, std::numeric_limits<float>::max());
  for (unsigned int i = 0; i < segIndices.size(); i++) {
    const unsigned int segIdx = segIndices[i];
    const auto& seg = segs[segIdx];
    const float distToSegOBB = seg->distance(pos);
    // TODO(MS) + WARNING: This uses naive distance to OBB Unhack this so that true distance to closest point in segment
    if (distToSegOBB < maxDist) { queue.insert(segIdx, distToSegOBB); }
  }
  vec<size_t> result;
  for (const auto& e : queue.queue()) {
    if (e.index != -1 && e.dist < maxDist) { result.push_back(e.index); }
  }
  return result;
}

}  // namespace mesh
}  // namespace sg
