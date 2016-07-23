#pragma once

#include "libsg.h"  // NOLINT

namespace sg {
namespace interaction {

//! Encapsulates IF construction params
struct InteractionFrameFactoryParams {
  explicit InteractionFrameFactoryParams(const util::Params& p);
  float halfWidth;                    // half-width of IF
  int numBins;                        // number of bins per dimension
  int numSkelSamplePoints;            // skel mesh samples
  int numInteractingOBBSamplePoints;  // active OBB point samples
  int numOccupancySamplePoints;       // occupancy grid samples
  float maxDistJointActivation;       // max dist to joint activation
  float maxDistGazeActivation;        // max dist to gaze activation
  float maxSegmentSizeRatio;          // ratio of scan obb length above which segments are not considered (0 for no filtering)
  int kNearestSegsPerJoint;           // max num of segs to activate per joint
  bool ignoreInferredJoints;          // ignore inferred joints
  bool computeJointsSurface;          // whether joints to surface IF should be computed (takes a while)
};

//! Responsible for constructing and manipulating InteractionFrame classes
class InteractionFrameFactory {
 public:
  explicit InteractionFrameFactory(const InteractionFrameFactoryParams& params,
                                   const core::Database& db);

  std::shared_ptr<InteractionFrames> createInteractionFrames(const vec<Interaction*>& interactions) const;

  void addInteractions(const vec<Interaction*>& interactions,
                       InteractionFrames* pFrames) const;

  void addInteraction(const Interaction& interaction,
                      InteractionFrames* pFrames) const;

  void addInteraction(const core::Scan& scan,
                      const core::SkelRange& skelRange,
                      InteractionFrames* pFrames) const;

  //! Add Skeleton points to InteractionFrame by uniform point sampling of Skeleton mesh
  //! This is a volumetric representation of the skeleton
  void addSkeletonPoints(const core::SkelRange& skelRange,
                         InteractionFrame* pFrame) const;

  //! Add all points in scan mesh interacting with Skeletons in skelRange
  //! (within maxDistToSeg/maxDistGaze of joint/gaze). pActRec is scratch var
  //! This is a surface representation of the interacting joints with some object
  void addInteractingScanMeshVertices(const core::Scan& scan,
                                      const core::SkelRange& skelRange,
                                      InteractionFrame* pFrame) const;

  //! Add randomly sampled points within segment OBBs activated by Skeletons in
  //! skelRange. Point samples of Skeleton meshes are also added to frame.
  //! This is a volumetric representation of the interacting joints with some object
  void addInteractingOBBSamplePoints(const core::Scan& scan,
                                     const core::SkelRange& skelRange,
                                     InteractionFrame* pFrame) const;

  //! Add sample occupied / unoccupied / unknown info
  //! NOTE: this is not interaction based 
  //!  (all voxels within the interaction frame is considered)
  void addOccupancy(const core::Scan& scan, const core::SkelRange& skelRange,
                    InteractionFrame* pFrame) const;
  //! Add sampled object and part labels from scan LabeledGrid annotations
  //! NOTE: this is not interaction based 
  //!  (all voxels within the interaction frame is considered)
  void addObjectLabels(const core::Scan& scan, const core::SkelRange& skelRange,
                       InteractionFrame* pFrame) const;

 private:
  InteractionFrameFactoryParams m_params;
  mutable std::shared_ptr<mesh::MeshActivationRecord> m_pActRec;  // scratch storage for NN queries
  const core::Database& m_db;  // for getting LabeledGrids
};

}  // namespace interaction
}  // namespace sg


