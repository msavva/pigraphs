#include "common.h"  // NOLINT

#include "segmentation/SegmentGroup.h"
#include "segmentation/MeshSegment.h"
#include "mesh/sampling.h"

#include <memory>
#include <unordered_map>

#include "geo/OBB.h"
#include "io/json.h"
#include "util/eigenutil.h"
#include "util/index.h"
#include "util/util.h"

namespace sg {
namespace segmentation {

SegmentGroup::SegmentGroup() 
  : Part(kPartSegmentGroup,-1,"")
  , objectId(-1)
  , m_segments() {}

SegmentGroup::SegmentGroup(const MeshSegment& segment, int _id, const string& _label, const int _objId)
  : Part(kPartSegmentGroup, _id, _label)
  , objectId(_objId)
  , m_segments() {
  m_segments.push_back(segment.id);
  m_obb = std::make_shared<OBB>(*segment.obb());
  computeDominantNormal();
}

ostream& SegmentGroup::toJSON(ostream& os, bool endlines) const {
  using sg::io::toJSON;
  const std::function<void(void)> sep = sg::io::put(os, ",", endlines);
  const auto key = [] (const string & id) { return "\"" + id + "\": "; };

  os << "{";
  if (endlines) { os << endl; }
  os << key("id") << id; sep();
  os << key("label"); toJSON(os, label); sep();
  os << key("objectId") << objectId; sep();
  os << key("obb"); m_obb->toJSON(os, endlines); sep();
  os << key("dominantNormal"); toJSON(os, m_dominantNormal); sep();
  os << key("segments"); toJSON(os, m_segments);
  os << "}";
  if (endlines) { os << endl; }

  return os;
}

bool SegmentGroup::contains(const MeshSegment& segment) const {
  return std::find(m_segments.begin(), m_segments.end(), segment.id) != m_segments.end();
}

bool SegmentGroup::contains(const int segId) const {
  return std::find(m_segments.begin(), m_segments.end(), segId) != m_segments.end();
}

int SegmentGroup::add(const int segId) {
  auto it = find(m_segments.begin(), m_segments.end(), segId);
  if (it == m_segments.end()) {
    // Not found - Add
    int pos = static_cast<int>(m_segments.size());
    m_segments.push_back(segId);
    return pos;
  } else {
    return std::distance(m_segments.begin(), it);
  }
}

bool SegmentGroup::remove(const int segId) {
  return util::findAndRemove(m_segments, segId);
}


bool SegmentGroup::isAbsorbable(const MeshSegment& segment, float absorbThreshold,
                                vec<ml::vec3f>* sampled) const {
  // Check if the OBB of the segment is close to that of this segment group
  const std::shared_ptr<sg::geo::OBB> segObb = segment.obb();
  const float dist = (m_obb->centroid() - segObb->centroid()).norm();
  const float segGroupR = m_obb->diagonalLength()/2;
  const float segR = segObb->diagonalLength()/2;
  if (dist <= segR + segGroupR) {
    // Segment is potentially within the segment group OBB
    // Do sampling of segment and see what percent of the points fall within the OBB
    const int nSamples = 100;
    vec<ml::TriMeshf::Vertex> tmpSamples;
    vec<ml::TriMeshf::Vertex>* samplesPtr = &tmpSamples;
    sg::mesh::sampling::sampleVertices(*segment.mesh, segment.elements, nSamples, samplesPtr);
    int samplesInOBB = 0;
    for (int i = 0; i < samplesPtr->size(); ++i) {
      const auto point = sg::geo::vec3f((*samplesPtr)[i].position);
      if (m_obb->contains(point)) {
        samplesInOBB++;
      }
    }
    if (sampled != nullptr) {
      sampled->resize(nSamples);
      for (size_t i = 0; i < nSamples; ++i) {
        (*sampled)[i] = (*samplesPtr)[i].position;
      }
    }
    float percentInOBB = static_cast<float>(samplesInOBB) / samplesPtr->size();
    return percentInOBB > absorbThreshold;
  } else {
    // Segment is completed outside of the segment group OBB
    return false;
  }
}

// Private helper functions
int SegmentGroup::absorb(const VecSegPtr& candidates,
                         const std::function<bool(const SegPtr)>& filter, 
                         float absorbThreshold,
                         vec<vec<ml::vec3f>>* sampled) {
  int added = 0;
  size_t numCandidates = candidates.size();
  if (sampled != nullptr) {
    sampled->resize(numCandidates);
  }
  for (size_t i = 0; i < numCandidates; ++i) {
    auto& segPtr = candidates[i];
    vec<ml::vec3f>* sampledPtr = (sampled != nullptr) ? &((*sampled)[i]) : nullptr;
    if (filter(segPtr) && !contains(*segPtr) && isAbsorbable(*segPtr, absorbThreshold, sampledPtr)) {
      m_segments.push_back(segPtr->id);
      added++;
    }
  }
  return added;
}

void SegmentGroup::computeOBB(const map<int, SegPtr>& segMap, bool constrainZup) {
  vec<ml::vec3f> points;
  size_t totalPoints = 0;
  for (int segId : m_segments) {
    const auto& segPtr = segMap.at(segId);
    totalPoints += segPtr->elements.size();
  }
  points.resize(totalPoints);
  int i = 0; 
  for (int segId : m_segments) {
    const auto& segPtr = segMap.at(segId);
    const auto& V = segPtr->mesh->getVertices();
    for (int vi : segPtr->elements) {
      points[i] = V[vi].position;
      i++;
    }
  }
  assert(i == totalPoints);
  if (totalPoints > 2) {
    m_obb = std::make_shared<OBB>(points, (constrainZup) ? OBB::FitOpts::CONSTRAIN_Z : OBB::FitOpts::MIN_PCA);
    computeDominantNormal();
  } else {
    // Don't do anything with the m_obb or the dominant normal if not enough points...
  }
}

// Segment Group Object
SegmentGroupObject::SegmentGroupObject(const int _id, const string& _label)
  : Part(kPartObject, _id, _label) { }

void SegmentGroupObject::computeOBB(const map<int, SegPtr>& segMap, bool constrainZup) {
  vec<ml::vec3f> points;
  size_t totalPoints = 0;
  for (const SegmentGroup& segGroup : m_segGroups) {
    for (int segId : segGroup.segments()) {
      const auto& segPtr = segMap.at(segId);
      totalPoints += segPtr->elements.size();
    }
  }
  points.resize(totalPoints);
  int i = 0;
  for (const SegmentGroup& segGroup : m_segGroups) {
    for (int segId : segGroup.segments()) {
      const auto& segPtr = segMap.at(segId);
      const auto& V = segPtr->mesh->getVertices();
      for (int vi : segPtr->elements) {
        points[i] = V[vi].position;
        i++;
      }
    }
  }
  assert(i == totalPoints);
  if (totalPoints > 2) {
    m_obb = std::make_shared<OBB>(points, (constrainZup) ? OBB::FitOpts::CONSTRAIN_Z : OBB::FitOpts::MIN_PCA);
    computeDominantNormal();
  } else {
    // Don't do anything with the m_obb or the dominant normal if not enough points...
  }
}

void SegmentGroupObject::computeOBB(bool constrainZup) {
  size_t nSamplesPerSegGroup = 50;
  vec<ml::vec3f> points;
  size_t totalPoints = m_segGroups.size() * nSamplesPerSegGroup;
  points.resize(totalPoints);
  int i = 0;
  for (const SegmentGroup& segGroup : m_segGroups) {
    const auto pObb = segGroup.obb();
    for (int j = 0; j < nSamplesPerSegGroup; j++) {
      points[i] = geo::to<ml::vec3f>(pObb->sample());
      i++;
    }
  }
  assert(i == totalPoints);
  if (totalPoints > 2) {
    m_obb = std::make_shared<OBB>(points, (constrainZup) ? OBB::FitOpts::CONSTRAIN_Z : OBB::FitOpts::MIN_PCA);
    computeDominantNormal();
  } else {
    // Don't do anything with the m_obb or the dominant normal if not enough points...
  }
}

// Segment Groups
SegmentGroups::SegmentGroups()
  : params()
  , absorbThreshold(0.5)
  , constrainZup(false)
  , m_segGroups()
  , m_segToSegGroupMap()
  , m_segMap()
  , m_segGroupIndicesMap()
  , m_nextSegId(0)
  , m_nextObjectId(0)
  , m_objMap()
  , m_objMapDirty(false) { }


string SegmentGroups::getSegmentLabel(const MeshSegment& segment) const {
  if (m_segToSegGroupMap.count(segment.id) > 0) {
    const auto segGroupId = m_segToSegGroupMap.at(segment.id);
    return get(segGroupId)->label;
  } else {
    return "";
  }
}

SegmentGroups::SegmentGroupHandle SegmentGroups::find(const MeshSegment& segment) const {
  if (m_segToSegGroupMap.count(segment.id) > 0) {
    return m_segToSegGroupMap.at(segment.id);
  } else {
    return -1;
  }
}

vec<SegmentGroups::SegmentGroupHandle> SegmentGroups::findAll(const MeshSegment& segment) const {
  vec<SegmentGroupHandle> matched;
  for (int i = 0; i < m_segGroups.size(); i++) {
    if (m_segGroups[i].contains(segment)) {
      matched.push_back(m_segGroups[i].id);
    }
  }
  return matched;
}

vec<SegmentGroups::SegmentGroupHandle> SegmentGroups::findAbsorbing(
  const MeshSegment& segment) const {
  vec<SegmentGroupHandle> matched;
  for (int i = 0; i < m_segGroups.size(); i++) {
    if (m_segGroups[i].isAbsorbable(segment, absorbThreshold)) {
      matched.push_back(m_segGroups[i].id);
    }
  }
  return matched;
}

vec<size_t> SegmentGroups::getSortedIndices() const {
  vec<size_t> sorted;
  sorted.resize(m_segGroups.size());
  for (int i = 0; i < m_segGroups.size(); i++) {
    sorted[i] = i;
  }
  std::sort(sorted.begin(), sorted.end(), [&](const size_t i1, const size_t i2){
    const SegmentGroup& sg1 = m_segGroups.at(i1);
    const SegmentGroup& sg2 = m_segGroups.at(i2);
    return (sg1.obb()->diagonalLength() > sg2.obb()->diagonalLength());
  });
  return sorted;
}

size_t SegmentGroups::findIndexOfLargestSegment(const vec<size_t>& segIndices) const {
  assert(!segIndices.empty());
  size_t selectedIdx = 0;
  float selectedSize = 0.0f;
  for (size_t idx : segIndices) {
    const SegmentGroup& sg = m_segGroups.at(idx);
    float size = sg.obb()->diagonalLength();
    if (size > selectedSize) {
      selectedIdx = idx;
      selectedSize = size;
    }
  }
  return selectedIdx;
}


//! Takes current segment groups, clears segments and regroups this set of candidates into
//!  the bounding boxes of the segment groups
//! The number and ids of the segment groups remain the same
void SegmentGroups::regroup(const VecSegPtr& candidates) {
  // Clear current segments from groups, but keep OBBs
  for (auto& segGroup : m_segGroups) {
    segGroup.m_segments.clear();
  }
  m_segToSegGroupMap.clear();
  // Regroup candidate segments into group based on the current segment group OBBs
  for (auto& segPtr : candidates) {
    // Find best segGroup that matches this candidate
    
    // TODO: Improve this
    // Pick the segment group that is the smallest 
    // that the segment can be absorbed into
    SegmentGroup* selectedGroup = nullptr;    
    for (auto& segGroup : m_segGroups) {
      bool considerSegGroup;
      if (selectedGroup != nullptr) {
        considerSegGroup = segGroup.obb()->diagonalLength() < selectedGroup->obb()->diagonalLength();
      } else {
        considerSegGroup = true;
      }
      if (considerSegGroup && segGroup.isAbsorbable(*segPtr, absorbThreshold)) {
        selectedGroup = &segGroup;
      }
    }
    if (selectedGroup != nullptr) {
      selectedGroup->m_segments.push_back(segPtr->id);
      m_segToSegGroupMap[segPtr->id] = selectedGroup->id;
    }
  }
  m_objMapDirty = true;
}

SegmentGroups::SegmentGroupHandle SegmentGroups::add(const MeshSegment& segment, const string& label) {
  // Check if the segment is already part of an existing segment group
  if (m_segToSegGroupMap.count(segment.id)) {
    // Indicate that we can't add this
    return -1;
  } else {
    // Not already part of an existing segment group - okay to be added
    int sgId = m_nextSegId;
    ++m_nextSegId;
    int idx = static_cast<int>(m_segGroups.size());
    SegmentGroup segGroup(segment, sgId, label, -1);
    m_segGroups.push_back(segGroup);
    m_segToSegGroupMap[segment.id] = sgId;
    m_segGroupIndicesMap[sgId] = idx;
    m_objMapDirty = true;
    return sgId;
  }
}

bool SegmentGroups::relabel(SegmentGroupHandle id, const string& label) {
  int idx = lookup(id);
  if (idx >= 0 && idx < m_segGroups.size()) {
    auto& segGroup = m_segGroups.at(idx);
    segGroup.label = label;
    return true;
  } else {
    return false;
  }
}

bool SegmentGroups::setObjectId(SegmentGroupHandle id, int objectId) {
  int idx = lookup(id);
  if (idx >= 0 && idx < m_segGroups.size()) {
    auto& segGroup = m_segGroups.at(idx);
    segGroup.objectId = objectId;
    m_objMapDirty = true;
    return true;
  } else {
    return false;
  }
}

bool SegmentGroups::remove(SegmentGroups::SegmentGroupHandle id) {
  int idx = lookup(id);
  if (idx >= 0 && idx < m_segGroups.size()) {
    // Clear from map of segment indices to groups
    for (int segId : m_segGroups[idx].m_segments) {
      m_segToSegGroupMap.erase(segId);
    }
    // Clear from list of segment groups
    m_segGroups.erase(m_segGroups.begin() + idx);

    // Update map of segment group to idx
    m_segGroupIndicesMap.clear();
    for (int i = 0; i < m_segGroups.size(); ++i) {
      m_segGroupIndicesMap[m_segGroups.at(i).id] = i;
    }
    m_objMapDirty = true;
    return true;
  } else {
    return false;
  }
}

//! If the existing segment is not part of the specified segment group, add it.
//! Otherwise, remove it
//! The number and ids of the segment groups remain the same
bool SegmentGroups::toggle(SegmentGroups::SegmentGroupHandle id, const MeshSegment& segment) {
  int idx = lookup(id);
  if (idx >= 0 && idx < m_segGroups.size()) {
    int curSegGroupId = -1;
    if (m_segToSegGroupMap.count(segment.id) > 0) {
      curSegGroupId = m_segToSegGroupMap[segment.id];
    }
    if (curSegGroupId == id) {
      // Currently part of the specified segment group
      // Remove it
      if (m_segGroups[idx].m_segments.size() > 1) {
        m_segGroups[idx].remove(segment.id);
        m_segToSegGroupMap.erase(segment.id);
        m_segGroups[idx].computeOBB(m_segMap, constrainZup);
        m_objMapDirty = true;
        return true;
      } else {
        // Removing this segment will cause the segment group to become empty
        // Do not allow this segment to be removed
        cout << "Cannot remove segment " << segment.id << " from group " << id
          << " because group will become empty." << endl;
        return false;
      }
    } else {
      // Currently not part of the specified segment group
      int curSegGroupIdx = lookup(curSegGroupId);
      if (curSegGroupIdx >= 0) {
        // The segment already belongs to another segment group
        // Do not allow the segment to switch groups
        cout << "Cannot add segment " << segment.id << " to group " << id
          << " because segment already belongs to group " << curSegGroupId << endl;
        return false;
      }
      // Add it
      m_segGroups[idx].add(segment.id);
      m_segToSegGroupMap[segment.id] = id;
      m_segGroups[idx].computeOBB(m_segMap, constrainZup);
      m_objMapDirty = true;
      return true;
    }
  } else {
    cout << "Cannot add segment " << segment.id << " to unknown group " << id << endl;
    return false;
  }
}

//! Absorbs segments from candidates into the specified segment group
//! The number and ids of the segment groups remain the same
int SegmentGroups::absorb(SegmentGroups::SegmentGroupHandle id, const VecSegPtr& candidates,
                          vec<vec<ml::vec3f>>* sampled /*= nullptr*/) {
  int idx = lookup(id);
  if (idx >= 0 && idx < m_segGroups.size()) {
    // Filter candidates so that we don't consider segments 
    // that are already have a segment group assigned
    auto filter = [=] (const SegPtr& segPtr) {
      return m_segToSegGroupMap.count(segPtr->id) == 0;
    };
    int absorbed = m_segGroups[idx].absorb(candidates, filter, absorbThreshold, sampled);
    // TODO: Only recompute OBB if absorbed > 0
    m_segGroups[idx].computeOBB(m_segMap, constrainZup);
    for (int segId : m_segGroups[idx].m_segments) {
      m_segToSegGroupMap[segId] = id;
    }
    m_objMapDirty = true;
    return absorbed;
  } else {
    return -1;
  }
}

//! Guess object ids for segment groups with unknown object Ids (-1)
void SegmentGroups::guessObjectIds() {
  set<string> unmergeableCategories = {"wall"};
  using mlOBB = ml::OrientedBoundingBox3f;
  const auto addToCluster = [&] (int clusterId, vec<size_t>& clusterSegGroupIndices, vec<size_t>& remainingIndices) {
    std::queue<size_t> todo;
    for (size_t sgIdx : clusterSegGroupIndices) {
      todo.push(sgIdx);
    }
    while (!todo.empty()) {
      size_t clusteredIdx = todo.front();
      todo.pop();
      const SegmentGroup& clusterSegGroup = m_segGroups.at(clusteredIdx);
      const mlOBB& clusterSegGroupOBB = geo::to<mlOBB, ml::vec3f>(*clusterSegGroup.obb());
      for (size_t idx : remainingIndices) {
        SegmentGroup& segGroup = m_segGroups.at(idx);
        // If obb of seg group is touches/overlaps cluster, put it in cluster
        const mlOBB& segGroupOBB = geo::to<mlOBB, ml::vec3f>(*segGroup.obb());
        if (clusterSegGroupOBB.intersects(segGroupOBB)) {
          SG_LOG_INFO << "Assign segGroup " << segGroup.id << " to cluster " << clusterId;
          // Assign segGroup to cluster
          segGroup.objectId = clusterId;
          clusterSegGroupIndices.push_back(idx);
          util::findAndRemove(remainingIndices, idx);
          // Add segGroup to todo queue
          todo.push(idx);
        }
      }
    }
  };

  // Create a map of noun to segment groups
  map<string, vec<size_t>> nounToSegGroupIdx;
  for (size_t idx = 0; idx < m_segGroups.size(); ++idx) {
    const SegmentGroup& segGroup = m_segGroups.at(idx);
    vec<string> parts = ml::util::split(segGroup.label, ":");
    const string& noun = parts[0];    
    nounToSegGroupIdx[noun].push_back(idx);
  }

  // Go over each noun category and group together the segment groups that are close together
  for (const auto& it : nounToSegGroupIdx) {
    const string& noun = it.first;
    const vec<size_t>& segGroupIndices = it.second;
    // Partition into clusters
    vec<size_t> unknownSegGroupIndices;
    // Segment groups clustered into objects
    map<int, vec<size_t>> clusters;
    for (size_t idx : segGroupIndices) {
      const SegmentGroup& segGroup = m_segGroups.at(idx);
      if (segGroup.objectId >= 0) {
        clusters[segGroup.objectId].push_back(idx);
      } else {
        unknownSegGroupIndices.push_back(idx);
      }
    }

    // Create independent clusters for unmergeable categories
    if (unmergeableCategories.count(noun) > 0) {
      for (const size_t selectedIdx : unknownSegGroupIndices) {
        SegmentGroup& selectedSegGroup = m_segGroups.at(selectedIdx);
        selectedSegGroup.objectId = m_nextObjectId;
        clusters[selectedSegGroup.objectId].push_back(selectedIdx);
        ++m_nextObjectId;
      }
    } else {
      // Try to group unknown seg groups into known clusters
      for (auto& clusterPair : clusters) {
        addToCluster(clusterPair.first, clusterPair.second, unknownSegGroupIndices);
      }

      // Go over unknown object ids and try to group them
      while (!unknownSegGroupIndices.empty()) {
        // Select largest unknown and put it in a new cluster
        size_t selectedIdx = findIndexOfLargestSegment(unknownSegGroupIndices);
        SegmentGroup& selectedSegGroup = m_segGroups.at(selectedIdx);
        selectedSegGroup.objectId = m_nextObjectId;
        clusters[selectedSegGroup.objectId].push_back(selectedIdx);
        util::findAndRemove(unknownSegGroupIndices, selectedIdx);
        ++m_nextObjectId;

        // Grab other segments that overlaps with this cluster
        addToCluster(selectedSegGroup.objectId, clusters[selectedSegGroup.objectId], unknownSegGroupIndices);
      }
    }
  }
  m_objMapDirty = true;
}

void SegmentGroups::recomputeObjectMap() const {
  m_objMap.clear();
  for (const auto& segGroup : m_segGroups) {
    int objId = segGroup.objectId;
    if (objId < 0) continue;  // Skip segment groups without objId
    if (m_objMap.count(objId) == 0) {
      vec<string> parts = ml::util::split(segGroup.label, ":");
      const string& noun = parts[0];
      m_objMap[objId] = std::make_shared<SegmentGroupObject>(objId, noun);
    }
    m_objMap[objId]->m_segGroups.push_back(segGroup);
  }
  bool hasSegMap = (m_segMap.size() > 0);
  if (hasSegMap) {
    SG_LOG_INFO << "Recomputing object maps and bounding boxes from segments";
    for (auto& pair : m_objMap) {
      pair.second->computeOBB(m_segMap, constrainZup);
    }
  } else {
    SG_LOG_INFO << "Recomputing object maps and bounding boxes from segment group bounding boxes";
    for (auto& pair : m_objMap) {
      pair.second->computeOBB(constrainZup);
    }
  }
  m_objMapDirty = false;
}

void SegmentGroups::computeOBBs() {
  for (auto& segGroup : m_segGroups) {
    segGroup.computeOBB(m_segMap, constrainZup);
  }
}

//! Updates which segment belong to which segment groups
//! The number and ids of the segment groups remain the same
void SegmentGroups::updateSegmentation(const VecSegPtr& segments,
                                       const VecSegPtr& rejectedSegments, 
                                       bool regroupSegments) {
  m_segMap.clear();
  for (auto& seg : segments) {
    m_segMap[seg->id] = seg;
  }
  for (auto& seg : rejectedSegments) {
    m_segMap[seg->id] = seg;
  }

  SG_LOG_INFO << "updated segmentation: "
    << segments.size() << ", " << rejectedSegments.size();

  if (regroupSegments) {
    cout << "Old segment groups:" << endl;
    dump(cout);
    regroup(segments);
    cout << "New segment groups:" << endl;
    dump(cout);
  }
  m_objMapDirty = true;
  ensureLabels(segments, rejectedSegments);
}

void SegmentGroups::ensureLabels(const VecSegPtr& segments, const VecSegPtr& rejectedSegments) const
{
  for (auto& seg : segments) {
    seg->label = getSegmentLabel(*seg);
  }
  for (auto& seg : rejectedSegments) {
    seg->label = getSegmentLabel(*seg);
  }
}

void SegmentGroups::dump(ostream& os) const {
  for (auto& segGroup : m_segGroups) {
    os << segGroup.id << ": ";
    io::toJSON(os, segGroup.m_segments);
    os << ", obbDiag=" << segGroup.m_obb->diagonalLength();
    os << ", domNorm=" << segGroup.m_dominantNormal << endl;
  }
}

ostream& SegmentGroups::toJSON(ostream& os, bool endlines) const {
  return io::toJSON(os, m_segGroups);
}

// SegmentGroupsRecord

SegmentGroupsRecord::SegmentGroupsRecord(const string& _scanId,
                                         segmentation::SegmentGroups* _segmentGroups) 
                                         : params(_segmentGroups->params)
                                         , scanId(_scanId)
                                         , segmentGroups(_segmentGroups) { }

ostream& SegmentGroupsRecord::toJSON(ostream& os, bool endlines) const {
  using io::toJSON;
  const std::function<void(void)> sep = io::put(os, ",", endlines);
  const auto key = [] (const string & id) { return "\"" + id + "\": "; };

  os << "{";
  if (endlines) { os << endl; }
  os << key("params"); toJSON(os, params); sep();
  os << key("sceneId"); toJSON(os, scanId); sep();
  os << key("segGroups"); segmentGroups->toJSON(os, endlines); sep();
  os << key("segIndices"); toJSON(os, segIndices);
  os << "}";
  if (endlines) { os << endl; }

  return os;
}

void SegmentGroupsRecord::save(const string& filename) const {
  ofstream ofs(filename);
  toJSON(ofs, true);
  ofs.close();
}

bool SegmentGroupsRecord::load(const string& filename) {
  if (!ml::util::fileExists(filename)) { return false; }

  // Parse JSON document
  rapidjson::Document d;
  if (!sg::io::parseRapidJSONDocument(filename, &d)) {
    cerr << "Parse error reading " << filename << endl
      << "Error code " << d.GetParseError() << " at " << d.GetErrorOffset() << endl;
    return false;
  }

  // Parse out elements

  // Parse params
  params.fromRapidJson(d["params"]);

  // Parse scan id
  scanId = d["sceneId"].GetString();

  // Parse segment groups
  const auto& segGroupsArr = d["segGroups"];
  const unsigned numSegGroups = segGroupsArr.Size();
  segmentGroups->clear();
  segmentGroups->m_segGroups.resize(numSegGroups);
  int maxSegId = -1;
  int maxObjectId = -1;
  for (unsigned i = 0; i < numSegGroups; i++) {
    // Parse segment group
    const auto& sIn = segGroupsArr[i];
    auto& segGroup = segmentGroups->m_segGroups[i];

    segGroup.id = sIn["id"].GetInt();
    segGroup.label = sIn["label"].GetString();
    if (sIn.HasMember("objectId")) {
      segGroup.objectId = sIn["objectId"].GetInt();
    } else {
      segGroup.objectId = -1;
    }
    
    // Read OBB
    segGroup.parseJsonOBB(sIn);

    // Read segments
    sg::io::toIntVector(sIn["segments"], &segGroup.m_segments);

    // Populate map
    for (int segId : segGroup.m_segments) {
      segmentGroups->m_segToSegGroupMap[segId] = segGroup.id;
    }

    // Populate segment group indices
    segmentGroups->m_segGroupIndicesMap[segGroup.id] = i;
    // Track maximum segment id so we know what the next id should be
    if (segGroup.id > maxSegId) {
      maxSegId = segGroup.id;
    }
    // Track maximum object id so we know what the next id should be
    if (segGroup.objectId > maxObjectId) {
      maxObjectId = segGroup.objectId;
    }
  }
  segmentGroups->m_nextSegId = maxSegId + 1;
  segmentGroups->m_nextObjectId = maxObjectId + 1;

  // Parse object obbs
  if (d.HasMember("objects")) {
    // TODO: Output object obbs and test
    // Populate basic objects
    const auto& segObjectsArr = d["objects"];
    for (unsigned i = 0; i < segObjectsArr.Size(); i++) {
      const auto& sIn = segObjectsArr[i];
      int id = sIn["id"].GetInt();
      string label = sIn["label"].GetString();
      std::shared_ptr<SegmentGroupObject> pObj = std::make_shared<SegmentGroupObject>(id, label);
      pObj->parseJsonOBB(sIn);
      segmentGroups->m_objMap[id] = pObj;
    }
    // Populate segment groups
    for (const auto& segGroup : segmentGroups->m_segGroups) {
      int objId = segGroup.objectId;
      if (objId < 0) continue;  // Skip segment groups without objId
      segmentGroups->m_objMap[objId]->m_segGroups.push_back(segGroup);
    }
    segmentGroups->m_objMapDirty = false;
  } else {
    segmentGroups->recomputeObjectMap();
  }

  // Parse segment indices
  sg::io::toIntVector(d["segIndices"], &segIndices);

  return true;
}

}  // namespace segmentation
}  // namespace sg
