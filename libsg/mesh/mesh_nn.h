#pragma once

#include <unordered_set>

#include <mLibCore.h>

#include "libsg.h"  // NOLINT
#include "core/Skeleton.h"
#include "ann/KdTree.h"
#include "math/math.h"

namespace sg {
namespace mesh {

//! Encapsulates activation of a given mesh
struct MeshActivationRecord {
  MeshActivationRecord();

  //! Initializes record for given numIds, numRawIds, numVerts and maxNNpoints
  MeshActivationRecord(int _numIds, int _numRawIds, int _numVerts, int _maxNNpoints);

  //! Initializes record for given numIds, numRawIds, numVerts and maxNNpoints
  void init(int _numIds, int _numRawIds, int _numVerts, int _maxNNpoints);

  //! Clears all members
  void reset();

  //! Adds color for activated vertices of given id into pColors array.
  //! Scaling of baseColor is by normalized vertex weight
  void colorizeVertices(int id, const ml::vec4f& baseColor, vec<ml::vec4f>* pColors) const;

  //! Adds color for activated vertices of all ids into pColors array,
  //! mapping ids through colorMap
  void colorizeVerticesAllIds(const std::function<ml::vec4f(int)>& colorMap,
                              vec<ml::vec4f>* pColors) const;

  struct VertWeight { float weight; int count; };   //! (weight,count) per vertex
  typedef map<int, uset<int>>     VertSetMap;   //! id -> set of activated vertex indices
  typedef map<int, vec<VertWeight>>   WeightsMap;   //! id -> per-vertex weights and counts

  int numIds, numRawIds, numVerts, maxNNpoints;
  VertSetMap actVertSets;
  WeightsMap actMap;

 private:
  friend class MeshHelper;
  // preallocated temporary storage for KNN search
  vec<size_t> m_nnIndices;  //! indices of activated vertices
  vec<float> m_nnDists;  //! distances from joint to each activated vertex
};

//! Storage for ray intersections
struct IntersectionRecord {
  const ml::TriMeshf::Triangle* tri;
  float t, u, v;
  int segIdx;
};

//! Identifies a mesh element
enum MeshElementType {
  MeshElementType_Vertex          = 0,
  MeshElementType_Triangle        = 1,
  MeshElementType_Segment         = 2,
  MeshElementType_SegmentGroup    = 3,
  MeshElementType_Mesh            = 4,
  MeshElementType_Count           = 5
};

//! Aggregates intersection counts and distances over elements of type elementType, with integer ids
struct Intersections {
  const MeshElementType elementType;
  const int numRays;
  const float minDist, maxDist;

  struct AggCountDist { int count; float sumDist; };
  std::unordered_map<int, AggCountDist> id2counts;
  int numIntersections;
  float minDistActual, maxDistActual;

  // Struct for sorting elements by user-defined weight parameter
  struct SortedMeshElement { int id; int sampleCount; float avgDist; float weight; };
  vec<SortedMeshElement> sortedElements;  //! sorted elements in descending weight order

  Intersections(MeshElementType _elementType, int _numRays, float _minDist = 0.f,
                float _maxDist = math::constants::POSINFf)
    : elementType(_elementType)
    , numRays(_numRays)
    , minDist(_minDist)
    , maxDist(_maxDist)
    , id2counts()
    , numIntersections(0)
    , minDistActual(math::constants::POSINFf)
    , maxDistActual(math::constants::NEGINFf)
    , sortedElements() { }

  void clear() {
    id2counts.clear();
    numIntersections = 0;
    minDistActual = math::constants::POSINFf;
    maxDistActual = math::constants::NEGINFf;
    sortedElements.clear();
  }
};

//! Utility class for performing nearest neighbor lookups against a given mesh
//! TODO(MS): Consider renaming since this now performs intersections as well
class MeshHelper {
 public:
  MeshHelper();
  ~MeshHelper();
  explicit MeshHelper(const ml::TriMeshf& mesh);

  // Disallow copy and assignment
  // If we want to allow copy and assignment, need to ensure the m_kdtree is recreated for the copy
  MeshHelper(const MeshHelper&) = delete;
  MeshHelper& operator=(const MeshHelper&) = delete;

  //! Create a MeshNN for the given mesh
  void init(const ml::TriMeshf& mesh);

  //! Accumulate activation due to SkelRange skels contacting or gazing this MeshNN's mesh.
  //! Saves activation into pRec
  void accumulateActivation(const core::SkelRange& skelRange, const float contactDistThresh, const float gazeDistThresh,
                            MeshActivationRecord* pRec) const;

  //! Accumulate any activation due to numPoints 3D points continguously stored starting at pPoints. Points within
  //! maxRadius of contained mesh points are added to pRec using ptIdxToIdFun to get an appropriate int id
  void accumulateContactActivation(const float* pPoints, const int numPoints, const float maxRadius,
                                   const std::function<int(int)>& ptIdxToIdFun, MeshActivationRecord* pRec) const;

  //! Accumulate gaze activation due to skel gazing within maxDistGaze
  void accumulateGazeActivation(const core::Skeleton& skel, float maxDistGaze, int gazeJointId,
                                MeshActivationRecord* pRec) const;

  //! Intersect Ray r (with optional interval range = [tmin,tmax]) against mesh triangles. Returns whether
  //! intersection occurred and sets values for t, u, v, intersected tri and segIdx in the IntersectionRecord rec.
  bool intersect(const ml::Rayf& r, IntersectionRecord* rec, float tmin = 0.f,
                 float tmax = std::numeric_limits<float>::max(), bool intersectOnlyFrontFaces = false) const;

  //! Aggregate elements gazed by skel into isecs record. Returns number of intersections found.
  int getGazed(const core::Skeleton& skel, Intersections* isecs) const;

  //! Get most likely segment in segs gazed by skel
  typedef segmentation::VecSegPtr VecSegPtr;
  typedef vec<unsigned int> VecSegIdx;
  void getGazedSegments(const core::Skeleton& skel, float maxDistGaze, const VecSegPtr& segs,
                        VecSegPtr* gazedSegs, bool doAccumulate = false) const;

  //! Get segments in contact with the skel
  void getContactSegments(const VecSegPtr& segs, const VecSegIdx& segIndices, 
                          const core::Skeleton& skel, bool ignoreInferredJoints,
                          unsigned int kNearestSegsPerJoint, float maxContactDist,
                          core::Skeleton::SegmentsByJointPlusGaze* activeSegs, bool doAccumulate = false) const;

  //! Get active segments in contact with the skel or gazed by the skeleton
  void getActiveSegments(const VecSegPtr& segs, 
                         const core::Skeleton& skel, bool ignoreInferredJoints,
                         unsigned int kNearestSegsPerJoint, float maxContactDist,
                         float maxDistGaze, float maxSegmentSize,
                         core::Skeleton::SegmentsByJointPlusGaze* activeSegs,
                         bool doAccumulate = false) const;

  //! Get the kNearestSegments
  vec<size_t> kNearestSegments(const VecSegPtr segs, const VecSegIdx& segIndices, 
                               const ml::vec3f& pos,
                               unsigned int k, float maxDist) const;

 private:
  ann::KdTree3f* m_kdtree;
  ml::TriMeshAcceleratorBVHf* m_bvh;
  const ml::TriMeshf* m_mesh;
};

}  // namespace mesh
}  // namespace sg


