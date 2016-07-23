#pragma once

#include <libsg.h>
#include <core/LabeledGrid.h>
#include <core/OccupancyGrid.h>
#include <vis/ColorIndex.h>
#include <vis/Heatmap.h>

#include "./mLibInclude.h"
#include "./SharedAppState.h"

namespace constants { struct BasicLightShaderConstantBuffer; }

// Contains various utility functions for Vizzer real-time visualization
namespace vizutil {

//! Writes out the first numFrames of depth data from video recording in depthFile as .sensor file and as .PLY file
void dumpDepthFramesToSensorFileAndPLY(const std::string& depthFile, unsigned int numFrames);

//! Helper function for performing orthographic projection of mesh from a top-down view and rendering into outBitmap
//! NOTE: d3d11ConstantBuffer is set to view from top-down position
void projectOrtho(const ml::D3D11TriMesh& mesh, ml::GraphicsDevice* graphics,
                  ml::D3D11ConstantBuffer<constants::BasicLightShaderConstantBuffer>* d3d11ConstantBuffer,
                  ml::D3D11RenderTarget* renderTarget, ml::ColorImageR8G8B8A8* outBitmap);
void projectOrtho(std::vector<std::reference_wrapper<const ml::D3D11TriMesh>> meshes, ml::GraphicsDevice* graphics,
                  ml::D3D11ConstantBuffer<constants::BasicLightShaderConstantBuffer>* d3d11ConstantBuffer,
                  ml::D3D11RenderTarget* renderTarget, ml::ColorImageR8G8B8A8* outBitmap);

// Helper to update UI with loaded segment groups. Param segmentGroupsReloaded determines whether we are clearing and
// starting from scratch or just updating some info on the segGroups
void updateSegGroupAnnotations(SharedAppState& state, bool segmentGroupsReloaded);  // NOLINT

//! Helper to visualize point sampling of mesh with a pointsMesh
void sampleMesh(ml::GraphicsDevice& g, const ml::TriMeshf& mesh, size_t numSamples,  // NOLINT
                ml::D3D11TriMesh* pointsMesh, const ml::vec4f color = ml::vec4f(1, 0, 0, 1));

//! Returns segments currently activated by skel in state.currScan
sg::core::Skeleton::SegmentsByJointPlusGaze getCurrentActiveSegs(const sg::core::Skeleton& skel,
                                                                 const SharedAppState& state);

//! Test writing out current skel's IG and PIG as GraphML files
void testWritingPIGGraphML(const sg::core::Skeleton& skel, const SharedAppState& state);

//! Helper to manipulate ModelInstance with keys
bool moveModelInstance(UINT key, sg::core::ModelInstance* mInst);

//! Helper to manipulate probe point
bool moveProbePoint(UINT key, ml::vec3f& p);

//! Project heatmap samples onto mesh vertices, taking average sample value within maxRadius of each vertex,
//! and set accumulated vertexWeights
void projectSamplesToMeshVerts(const sg::vec<sg::vis::PoseHeatMap::Sample>& samples,
                               const sg::vec<ml::TriMeshf::Vertex>& V,
                               float maxRadius, sg::vec<float>* vertexWeights);

//! Compute and print out similarity of IG created from Skeleton to all existing VerbSets
void reportInteractionSetKNNIGSim(const sg::core::Skeleton& skel, const SharedAppState& sas, const sg::util::Params& p);

//! Compute and print out similarity of IG created from Skeleton to PIGs for all existing VerbSets
void reportInteractionSetPIGSim(const sg::core::Skeleton& skel, const SharedAppState& sas, const sg::util::Params& p);

//! Create scan voxels trimesh
ml::TriMeshf scanVoxelsToTriMesh(const sg::core::Scan& scene,
                                 const sg::core::OccupancyGrid::OccupancyType occType = sg::core::OccupancyGrid::OccupancyType_Occupied);

//! Create labeled scan voxels trimesh using the given color index
ml::TriMeshf labeledScanVoxelsToTriMesh(const sg::core::LabeledGrid& labeledGrid,
                                        sg::vis::ColorIndex* pColorIndex = nullptr);

//! Create colored grid from labeled scan voxels
ml::Grid3<ml::RGBColor> labeledScanVoxelsToColoredGrid(const sg::core::Scan& scene,
                                                       const sg::core::LabeledGrid& labeledGrid,
                                                       sg::vis::ColorIndex* pColorIndex = nullptr);

//! Return a TriMeshf visualization sampled voxels from Skeleton skel with blue and red colors
//! Collision with scan voxelization is indicated by the latter
ml::TriMeshf visualizeSkelCollisions(const sg::core::Skeleton& skel, const sg::core::Scan& scene, int numPts = 100);

//! Return a TriMeshf visualization for the scored points using the colorGradient
ml::TriMeshf createPointCloudMesh(const std::vector<std::pair<sg::geo::Vec3f, float>>& scoredPoints,
                                  const ml::ColorGradient& colorGradient,
                                  const ml::vec4f& defaultColor,
                                  const float boxSize = 0.02);

//! Create mesh from skeleton with option to show joint coordinate frames
ml::TriMeshf skelToMeshWithJointWeights(const sg::core::Skeleton& skel, const std::array<double, sg::core::Skeleton::kNumJointGroups>& jointWeights, const ml::ColorGradient& colorGradient);


}  // namespace vizutil


