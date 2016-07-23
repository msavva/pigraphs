#pragma once

#include <mLibCore.h>

#include "libsg.h"  // NOLINT

#include "segmentation/Segmentator.h"
#include "segmentation/Part.h"
#include "util/Params.h"
#include "util/util.h"

namespace sg {
namespace segmentation {

//! Group of segments that form an object part
class SegmentGroup : public Part {
  friend class SegmentGroups;
  friend class SegmentGroupsRecord;
 public:
  SegmentGroup();
  SegmentGroup(const MeshSegment& segment, int _id, const string& _label, const int _objId);

  //! Ids of the segments that make up this segment group
  const vec<int>& segments() const { return m_segments; }

  //! Id of the object this segment group belows to
  int objectId;

  //! Check if the given segment is one of the segments in this group
  bool contains(const MeshSegment& segment) const;
  bool contains(const int segId) const;
  //! Check if the given segment can be absorbed into the segment group
  bool isAbsorbable(const MeshSegment& segment, float absorbThreshold,
                    vec<ml::vec3f>* sampled = nullptr) const;

  //! Info string about this segment group
  string info() const { 
    return "seg" + ((m_segments.size() > 0)? to_string(m_segments[0]):"") 
      + "-sz" + to_string(m_segments.size());
  }

  //! Outputs this SegmentGroup to passed ostream in JSON format and returns the ostream
  ostream& toJSON(ostream& os, bool endlines) const;

 private:
  //! Absorb matching segments into this segment group
  int absorb(const VecSegPtr& candidates,
             const std::function<bool(const SegPtr)>& filter,
             float absorbThreshold,
             vec<vec<ml::vec3f>>* sampled = nullptr);

  //! Add segment
  int add(const int segId);

  //! Remove segment
  bool remove(const int segId);

  //! Compute the obb for this segment group based on the segments that are currently in it
  void computeOBB(const map<int, SegPtr>& segMap, bool constrainZup);

  //! Ids of the segments that make up this segment group
  vec<int> m_segments;
};

inline ostream& toJSON(ostream& os, const SegmentGroup& sg) {
  sg.toJSON(os, true);
  return os;
}

//! Object made up of segment groups
class SegmentGroupObject : public Part {
  friend class SegmentGroups;
  friend class SegmentGroupsRecord;
 public:
   SegmentGroupObject(const int _id, const string& _label);

 private:
   //! Compute the obb for this segment group object based on the segments that are currently in it
   void computeOBB(const map<int, SegPtr>& segMap, bool constrainZup);

   //! Compute the obb for this segment group object based on the bounding box of the segment groups
   void computeOBB(bool constrainZup);

   vec<std::reference_wrapper<const SegmentGroup>> m_segGroups;
};

//! A set of segment group in a scene
class SegmentGroups {
  friend class SegmentGroupsRecord;
 public:
   //! SegmentGroupHandle is the "id" of the segment group (not the index)
   typedef int SegmentGroupHandle;
   SegmentGroups();

   //! TODO: Cleanup this!!!  
   //! Would params and absorbThreshold/constrainZup ever be out of sync?

   //! Copy of params with which these segment groups were created with
   //! These should be the parameters under the "Segmentation" section
   util::Params params;
   //! Absorb threshold
   float absorbThreshold;
   //! ConstrainZup
   bool constrainZup;

  //! Returns label of this segment
  string getSegmentLabel(const MeshSegment& segment) const;

  //! Find the segment group that contains this segment
  SegmentGroupHandle find(const MeshSegment& segment) const;

  //! Find all segment groups that contains this segment
  vec<SegmentGroupHandle> findAll(const MeshSegment& segment) const;

  //! Find the segment groups that can absorb this segment
  vec<SegmentGroupHandle> findAbsorbing(const MeshSegment& segment) const;

  //! Get segment group indices sorted from largest to smallest
  vec<size_t> getSortedIndices() const;

  //! Add a new segment group to the list of segment groups
  //! Returns a segment group handle to the new segment group
  SegmentGroupHandle add(const MeshSegment& segment, const string& label);

  //! Relabels a segment group
  bool relabel(SegmentGroupHandle id, const string& label);

  //! Set object id for a segment group
  bool setObjectId(SegmentGroupHandle id, int objectId);

  //! Remove a segment from the list of segment groups
  bool remove(SegmentGroupHandle id);

  //! Returns pointer to segment group with id
  const SegmentGroup* get(SegmentGroupHandle id) const {
    int idx = lookup(id);
    return at(idx);
  }

  //! Returns pointer to segment group as a part ptr
  ConstPartPtr getSegGroupPartPtr(SegmentGroupHandle id) const {
    // Use special aliasing constructor so memory is handled by us
    const SegmentGroup* pSegGroup = get(id);
    return sg::util::ptr_to_shared<const Part, const SegmentGroup>(pSegGroup);
  }

  //! Returns pointer to segment group at index
  const SegmentGroup* at(SegmentGroupHandle idx) const {
    if (idx >= 0 && idx < m_segGroups.size()) {
      return &m_segGroups[idx];
    } else {
      return nullptr;
    }
  }

  //! Returns objects
  map<int, std::shared_ptr<SegmentGroupObject>> getObjectMap() const {
    if (m_objMapDirty) {
      recomputeObjectMap();
    }
    return m_objMap;
  }

  //! Return shared pointer to object
  std::shared_ptr<const SegmentGroupObject> getObject(int objId) const {
    if (m_objMapDirty) {
      recomputeObjectMap();
    }
    if (m_objMap.count(objId) > 0) {
      return m_objMap.at(objId);
    } else {
      return nullptr;
    }
  }

  //! Returns pointer to object as a part ptr
  ConstPartPtr getObjectPartPtr(int objId) const {
    const std::shared_ptr<const SegmentGroupObject> pObj = getObject(objId);
    return std::static_pointer_cast<const Part>(pObj);
  }

  //! Toggle the state of the segment in the specified segment group
  //! The segment state is unchanged if the segment already belongs
  //!   to a different segment group, or removing it will cause the
  //!   specified segment group to become empty
  //! Returns whether the segment state was changed or not
  bool toggle(SegmentGroupHandle id, const MeshSegment& segment);

  //! Absorb matching segments into the segment group with the given id
  //! Returns the number of segments absorbed
  int absorb(SegmentGroupHandle id, const VecSegPtr& candidates,
             vec<vec<ml::vec3f>>* sampled = nullptr);

  //! Guess object ids for segment groups with unknown object Ids (-1)
  void guessObjectIds();

  //! Clears the segment groups
  void clear() {
    m_segGroups.clear();
    m_segToSegGroupMap.clear();
    m_segGroupIndicesMap.clear();
    m_nextSegId = 0;
    m_nextObjectId = 0;
    m_objMap.clear();
  }

  //! Compute the OBBs for all segment groups
  void computeOBBs();

  //! Regroup candidate segments into group based on the current segment group OBBs
  void regroup(const VecSegPtr& candidates);

  //! Use given segments
  void updateSegmentation(const VecSegPtr& segments,
                          const VecSegPtr& rejectedSegments,
                          bool regroupSegments = true);

  //! Ensure segment labels are populated
  void ensureLabels(const VecSegPtr& segments,
                    const VecSegPtr& rejectedSegments) const;

  //! Number of segment groups we have
  size_t size() const { return m_segGroups.size(); }

  //! Iterators
  vec<SegmentGroup>::iterator begin() { return m_segGroups.begin(); }
  vec<SegmentGroup>::iterator end() { return m_segGroups.end(); }
  vec<SegmentGroup>::const_iterator begin() const { return m_segGroups.begin(); }
  vec<SegmentGroup>::const_iterator end() const { return m_segGroups.end(); }

  //! Dumps debug summary of segment groups to ostream
  void dump(ostream& os) const;

  //! Outputs these SegmentGroups to passed ostream in JSON format and returns the ostream
  ostream& toJSON(ostream& os, bool endlines) const;

 private:
  //! Lookup index from id
  int lookup(SegmentGroupHandle id) const {
    if (m_segGroupIndicesMap.count(id) > 0) {
      return m_segGroupIndicesMap.at(id);
    } else {
      return -1;
    }
  }

  size_t findIndexOfLargestSegment(const vec<size_t>& segIndices) const;

  void recomputeObjectMap() const;

  //! Vector of segment groups
  vec<SegmentGroup> m_segGroups;

  //! Map of segment group id to segment group index
  //! Should be kept in sync with m_segGroups
  map<int, int> m_segGroupIndicesMap;

  //! Next segment group id
  int m_nextSegId;

  //! Next object id
  int m_nextObjectId;

  //! Map of segment id to segment group
  //! We enforce that each segment should belong to just one segment group
  //! Update if segmentation or segment groups have changed
  map<int, int> m_segToSegGroupMap;

  //! Map of segment id to segment (update if segmentation has changed)
  map<int, SegPtr> m_segMap;

  //! Map of object ids to objects
  mutable map<int, std::shared_ptr<SegmentGroupObject>> m_objMap;
  //! If the object map is dirty and need to be updated
  mutable bool m_objMapDirty;
};

//! TODO: Most of this information is now in SegmentGroups
//!       (like the params)
//!       this class may not be needed anymore
//! Storage for segment groups along with the segmentation
class SegmentGroupsRecord {
public:
  typedef VecSegPtr VecSegPtr;
  SegmentGroupsRecord(const string& _sceneId, SegmentGroups* _segmentGroups);

  //! Copy of params with which these segment groups were created with
  //! These should be the parameters under the "Segmentation" section
  util::Params params;
  //! Identifies the scan for which these segment groups are for
  string scanId;
  //! Segment groups
  SegmentGroups* segmentGroups;
  //! Segmentation in creating the segment groups
  SegIndices segIndices;

  //! Outputs the SegmentGroupRecord to passed ostream in JSON format and returns the ostream
  ostream& toJSON(ostream&, bool endlines) const;

  //! Load the segment groups from a file
  bool load(const string& filename);

  //! Save the segment groups to a file
  void save(const string& filename) const;
};


}  // namespace segmentation
}  // namespace sg


