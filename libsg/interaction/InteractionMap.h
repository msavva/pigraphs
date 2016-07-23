#pragma once

#include <mLibCore.h>

#include "libsg.h"  // NOLINT
#include "core/ModelDatabase.h"  // for Model and ModelInstance
#include "geo/geo.h"
#include "interaction/SkeletonInteraction.h"
#include "mesh/mesh_nn.h"

namespace sg {
namespace interaction {

class InteractionMap {
 public:
  InteractionMap(): m_pModel(), m_meshNN(), m_rec() {}
  
  // Disallow copy and assignment
  // Need to make m_meshNN copyable if we want to copy
  InteractionMap(const InteractionMap&) = delete;
  InteractionMap& operator=(const InteractionMap&) = delete;

  //! Create an InteractionMap for the given model
  void init(const core::Model& model, const int kMaxNNpoints = 2048) {
    m_pModel = &model;
    m_meshNN.init(model.flattenedMesh);
    m_rec.init(kSkelParams.kNumJoints, Skeleton::kNumJoints,
               static_cast<int>(model.flattenedMesh.getVertices().size()), kMaxNNpoints);
  }

  //! Resets this InteractionMap by dropping all activation records
  void reset() {
    m_rec.reset();
  }

  //! Add interactions observed between mInst and skel
  void addInteraction(const core::ModelInstance& mInst, const core::SkelRange& skelRange, float maxContactDist,
                      float maxGazeDist) {
    // helper to map joints to JointGroupLR indices
    const auto jointIdx2JointGroup = [&] (int iJoint) { return kSkelParams.kJointMap[iJoint]; };

    // Get transforms for skel -> model space
    const auto& worldToModel = mInst.getParentToLocal();
    const ml::mat4f worldToModelML = geo::to<ml::mat4f>(worldToModel);
    const float maxRscaled = maxContactDist;  // TODO(MS): account for mInst scaling
    const float maxGazeDistScaled = maxGazeDist;  // TODO(MS): account for mInst scaling
    vec<geo::Vec3f> modelPoints(Skeleton::kNumJoints);

    for (const Skeleton& skel : skelRange) {
      for (int iJoint = 0; iJoint < Skeleton::kNumJoints; ++iJoint) {
        modelPoints[iJoint] = worldToModel * geo::vec3f(skel.jointPositions[iJoint]);
      }
      m_meshNN.accumulateContactActivation(&modelPoints[0][0], Skeleton::kNumJoints, maxRscaled, jointIdx2JointGroup,
                                           &m_rec);

      const Skeleton modelSkel = core::TransformedSkeleton(skel, worldToModelML).getSkeleton();
      m_meshNN.accumulateGazeActivation(modelSkel, maxGazeDistScaled, static_cast<int>(kSkelParams.kGazeJoint), &m_rec);
    }

  }

  void colorize(const core::ModelInstance& mInst, vec<ml::vec4f>* pColors) {
    const std::function<ml::vec4f(int)> colorFun = [&] (int i) {
      if (i == kSkelParams.kGazeJoint) { return kSkelParams.kJointColors[kSkelParams.kGazeJoint]; }
      return kSkelParams.kJointColors[i];
    };
    m_rec.colorizeVerticesAllIds(colorFun, pColors);
  }

 private:
  const core::Model* m_pModel;            //! Base model for this InteractionMap
  mesh::MeshHelper m_meshNN;              //! Helper for performing nearest neighbor lookups
  mesh::MeshActivationRecord m_rec;       //! Stores mesh activation state
};

}  // namespace interaction
}  // namespace sg


