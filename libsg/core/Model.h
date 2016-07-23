#pragma once

#include "libsg.h"  // NOLINT

#include <mLibCore.h>

#include "geo/geo.h"

namespace sg {
namespace core {

typedef segmentation::SegPtr SegPtr;
typedef segmentation::VecSegPtr VecSegPtr;

//! Encapsulates a loaded 3D model with mesh data
struct Model {
  Model();
  ~Model();

  //! VIP: delete copy and assignment since we only should have one copy of a model
  //! and the segments won't like being copied around
  Model(const Model&) = delete;
  Model& operator=(const Model&) = delete;

  //! Load mesh data from given pair of OBJ+MTL format files
  //! When optional parameter maxEdgeLength is >0 perform loop subdivision until below given threshold
  void load(const string& objFile, const string& mtlFile, float maxEdgeLength = 0.f);

  //! Load mesh data from given model file using Assimp library
  void Model::load(const string& file);

  //! Unloads mesh and voxel data
  void unload() {
    flattenedMesh.clear();
    allMeshes.clear();
    solidVoxels.free();
    surfaceVoxels.free();
    voxelCenters.resize(Eigen::NoChange, 0);
    isLoaded = false;
    isSubdivided = false;
  }

  //! Clears current segmentation
  void clearSegmentation();

  //! Load segmentation
  bool loadSegmentation(const string& filename, bool constrainZup, const string& format);

  //! Segment
  void segment(const segmentation::SegmentationParams& params);

  //! Convenience struct for loading OBJ+MTL material group geometry
  struct MeshData {
    ml::TriMeshf mesh;
    ml::Materialf material;
  };

  //! Returns whether this Model has the given category
  bool hasCategory(const string& cat) const {
    return find(categories.begin(), categories.end(), cat) != categories.end();
  }

  //! Returns whether this model has a segmentation currently loaded
  bool hasSegmentsLoaded() const {
    return !segments->empty();
  }

  //! Returns whether this model has voxels currently loaded
  bool hasSurfaceVoxelsLoaded() const {
    return surfaceVoxels.getNumElements() > 0;
  }

  //! Returns the number of voxels occupied in this model's surface
  size_t occupiedSurfaceVoxels() const {
    return surfaceVoxels.getNumOccupiedEntries();
  }

  //! Returns whether this model has voxels currently loaded
  bool hasSolidVoxelsLoaded() const {
    return solidVoxels.getNumElements() > 0;
  }

  //! Returns the number of voxels occupied in this model's surface
  size_t occupiedSolidVoxels() const {
    return solidVoxels.getNumOccupiedEntries();
  }

  //! Returns the voxel dimensions in model space. NOTE: assumes same dimensions apply for both surface and solid voxels
  ml::vec3f voxelDims() const {
    return ml::vec3f(voxelToModel.matrix2[0][0], voxelToModel.matrix2[1][1], voxelToModel.matrix2[2][2]);
  }

  //! Returns the voxel volume in model space. NOTE: assumes same dimensions apply for both surface and solid voxels
  float voxelVolume() const {
    const ml::vec3f vDims = voxelDims();
    return vDims[0] * vDims[1] * vDims[2];
  }

  //! Returns number of occupied voxels found in supporting plane at or above given vertical height index
  size_t numSupportingVoxels(size_t zSupport = 0) const;

  //! Returns model-space OBB for this Model (requires voxels since this is estimated by using voxelCenters)
  const geo::OBB& getOBB() const;

  string id;                      //! full "source.id" identifier of this model
  vec<string> categories;         //! categories assigned to this model
  float scaleUnit;                //! scale multiplier to convert model to real-world meter units
  ml::vec3f up;                   //! semantic "up" direction in model space
  ml::vec3f front;                //! semantic "front" direction in model space
  ml::vec3f dims;                 //! real world dimensions (NOTE: pre-scaled and pre-aligned)
  ml::TriMeshf flattenedMesh;     //! flattened mesh containing all the geometry
  ml::mat4f modelTransform;       //! Transform applied to create normalized flattenedMesh from original model
  vec<MeshData> allMeshes;        //! Model meshes separated by material groups (typically from OBJ/MTL definition)
  ml::BoundingBox3f bbox;         //! Bounding box containing flattenedMesh in model space
  std::unique_ptr<VecSegPtr> segments; //! Segmentation for this model
  ml::BinaryGrid3 surfaceVoxels;  //! Mesh surface voxels for this model
  ml::BinaryGrid3 solidVoxels;    //! Solid voxelization for this model (mesh is internally filled)
  ml::mat4f modelToVoxel;         //! Transform taking model coords to voxel coords
  ml::mat4f voxelToModel;         //! Transform taking voxel coords to model coords
  Eigen::Matrix3Xf voxelCenters;  //! [3 x n] matrix of model voxel centers in model coords
  bool isLoaded;                  //! flag indicating whether mesh data for this model has been loaded
  bool isSubdivided;              //! flag indicating whether mesh for this model has been subdivided
  bool isWhitelisted;             //! flag indicating whether the model is in a whitelist
 private:
  mutable geo::OBB* m_obb;  // cached OBB in model coordinates
};

}  // namespace core
}  // namespace sg


