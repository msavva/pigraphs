#pragma once

#include <mLibCore.h>

#include "libsg.h"  // NOLINT
#include "core/Skeleton.h"
#include "mesh/projection.h"
#include "segmentation/SegmentGroup.h"

namespace sg {
namespace core {

class Scan {
 public:
  typedef segmentation::SegPtr SegPtr;
  typedef segmentation::VecSegPtr VecSegPtr;
  typedef vec<unsigned int> VecSegIdx;

  //! Extension to use for cached binary serialization of Scenes
  static const string kCachedExt;

  //! Extension for voxelized OccupancyGrid Scene files (parallel to base scan id)
  static const string kGridFileExt;

  Scan();
  ~Scan();

  //! Load scan mesh from file, applies segmentation params
  void load(const util::Params& params);

  //! Load scan mesh from Cornell ASCII format PLY with embedded segmentation and labels
  void loadFromCornellPLY(const util::Params& params, const string& plyfile);

  //! Load a cached binary version of the Scene from the given filename. Returns whether succeeded.
  bool loadCachedBinary(const string& file, const util::Params& params);

  //! Save a cached binary version of the Scene to the given filename
  void saveCachedBinary(const string& filename);

  //! Helper for performing post-load operations
  void finalizeLoad(const util::Params& params);

  //! Update scan segmentation with given parameters
  void updateSegmentation(const segmentation::SegmentationParams& segParams);

  //! Load a saved scan segmentation from file
  void loadSegmentation(const string& filename, bool constrainZup);

  //! Save the segment group annotations to file
  void saveSegmentGroups(const string& filename);

  //! Load the segment group annotations from file
  bool loadSegmentGroups(const string& filename,
                         float absorbThreshold, bool constrainZUp,
                         bool loadSegments, bool recomputeOBBs,
                         const std::function<void()>* segmentatorFun = nullptr);

  //! Retrieve "active segments" (contacted by joints and gazed by Skeleton)
  void getActiveSegments(const Skeleton& skel, bool ignoreInferredJoints,
                         unsigned int kNearestSegsPerJoint, float maxContactDist,
                         float maxDistGaze, float maxSegmentSizeRatio,
                         Skeleton::SegmentsByJointPlusGaze* segsByJoint,
                         bool doAccumulate = false) const;

  //! Retrieve "active segments" (contacted by joints and gazed by Skeletons in skelRange)
  void getActiveSegments(const SkelRange& skelRange, bool ignoreInferredJoints,
                         unsigned int kNearestSegsPerJoint, float maxContactDist,
                         float maxDistGaze, float maxSegmentSizeRatio,
                         Skeleton::SegmentsByJointPlusGaze* segsByJoint) const;

  //! Intersect against scan constructing ray from given camera and normalized image plane coordinates
  bool intersect(const ml::Cameraf& cam, const ml::vec2f& imagePlaneCoords, mesh::IntersectionRecord* rec) const;

  //! Returns segment corresponding to the segment id
  SegPtr getSegment(int segId) const {
    // For rejected segments, id starts at -1 (correspond to index 0)
    if (segId < 0) {
      return (*rejectedSegments)[-segId - 1];
    } else {
      return (*segments)[segId];
    }
  }

  //! Returns the MeshNN helper for this Scene
  const mesh::MeshHelper& getMeshHelper() const { return *m_meshHelper; }

  //! Returns this Scene's voxelized OccupancyGrid. NOTE: loads from file if not already loaded
  const OccupancyGrid& getOccupancyGrid() const;

  //! Returns this Scene's VertexStackGrid (XY binned, height-ordered mesh vertices). NOTE: Computes if unavailable
  const mesh::projection::VertexStackGrid& getVertexStackGrid() const;

  //! Returns whether this Scene is loaded into memory
  bool isLoaded() const { return m_isLoaded; }

  //! Return the canonical scanId (without the "d")
  string getCanonicalId() const;

  string id, meshFile, meshFileDebug, annotationFile, loadedMeshFileFullPath, sceneType;
  ml::TriMeshf mesh;
  ml::BoundingBox3f bbox;
  std::unique_ptr<VecSegPtr> segments;
  std::unique_ptr<VecSegPtr> rejectedSegments;
  segmentation::SegmentGroups segmentGroups;

 private:
  friend class ScanDatabase;

  bool m_isLoaded, m_isDebugMesh, m_hasGround;
  mesh::MeshHelper* m_meshHelper;
  mutable mesh::projection::VertexStackGrid* m_vStackGrid;
  mutable OccupancyGrid* m_occGrid;
};

}  // namespace core
}  // namespace sg


