#pragma once

#include <mLibCore.h>

#include "libsg.h"  // NOLINT

namespace sg {
namespace vis {

//! Create TriMesh visualization of Skeleton by drawing boxes for joints (option for bones and gaze vector)
ml::TriMeshf toTriMesh(const core::Skeleton& skel, bool showJoints = true, bool showBones = true,
                       bool showGaze = true, bool showBodyPlane = true,
                       bool showCenterOfMass = true, bool showBoneCoords = false,
                       vec<ml::vec4f>* pColors = nullptr);

//! Create TriMesh visualization of SkelState
ml::TriMeshf toTriMesh2(const core::Skeleton& skel, bool showBoneCoords = true);

//! Create wireframe visualization of OBB. Optional param jitter introduces a proportional jitter (ratio of OBB dims)
//! Optional param thickness sets the wireframe thickness
ml::TriMeshf OBBwireframe(const geo::OBB& obb, const ml::vec4f& color, float jitter = 0, float thickness = 0.01f);

//! Create solid box visualization of OBB
ml::TriMeshf OBBbox(const geo::OBB& obb, const ml::vec4f& color);

// Options for visualizing an InteractionFrame
struct IFVisParams {
  ml::vec4f color = ml::vec4f(0, 0, 1, .5);
  float minBinWeight = 0.05f;
  bool renderJointPts = true;
  bool renderSkelPts = true;
  bool renderOccupancy = true;
  bool renderObjectLabels = true;
  float thickness = 0.01f;
  const double* pJointWeights = nullptr;
};

//! Create TriMesh visualization for InteractionFrame
ml::TriMeshf iframeWidget(const interaction::InteractionFrame& iframe, const IFVisParams& params);

//! Create TriMesh visualization for InteractionFrame of given iset
ml::TriMeshf iframeWidget(const interaction::InteractionSet& iset, 
                          const interaction::InteractionFrameType iframeType,
                          const IFVisParams& params);

//! Create TriMesh visualization for InteractionFrame
ml::TriMeshf iframePointsWidget(const interaction::InteractionFrame& iframe, const IFVisParams& params);

//! Create TriMesh corresponding to coordinate space defined by columns of xform
ml::TriMeshf coordAxesWidget(const ml::mat4f& xform, float scaleFactor = 1);

//! Create TriMesh corresponding to coordinate space defined by OBB
ml::TriMeshf coordAxesWidget(const geo::OBB& obb, float scaleFactor = 1);

//! Set camera to look at a particular target
void setCameraToLookAt(int windowWidth, int windowHeight,
                       const ml::vec3f& eye, const ml::vec3f& target,
                       ml::Cameraf* pCamera);

}  // namespace vis
}  // namespace sg


