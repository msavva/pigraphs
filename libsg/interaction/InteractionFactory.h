#pragma once

#include "libsg.h"  // NOLINT

#include "core/Skeleton.h"
#include "core/synth/ObjectLabeler.h"
#include "segmentation/Part.h"

namespace sg {
namespace interaction {

typedef map<const segmentation::SegPtr,
  std::reference_wrapper<const core::ModelInstance>> SegPtrToModelInstanceMap;
typedef map<const ConstPartPtr,
  std::reference_wrapper<const core::ModelInstance>> PartPtrToModelInstanceMap;
typedef std::shared_ptr<InteractionGraph> IGPtr;

//! Encapsulation struct for interaction parameters for computing active segments
struct InteractionParams {
  explicit InteractionParams(const sg::util::Params& p);
  bool ignoreInferredJoints;          //! Whether inferred joints should be ignored
  unsigned int kNearestSegsPerJoint;  //! Number of segments to keep per joint
  float maxDistToSegment;             //! Maximum contact distance to segment in meters
  float maxDistGaze;                  //! Maximum gaze distance in meters
  float maxSegmentSizeRatio;          //! Maximum ratio of segment size to room size to keep (use 0 for no limit)
};

//! Factory for creating interaction graphs and such
class InteractionFactory {
 public:
  explicit InteractionFactory(const util::Params& params)
    : m_params(params) { }

  using PartType = PartType;
  using Skeleton = Skeleton;
  using Scan = core::Scan;
  using SkelRange = core::SkelRange;
  using ModelInstance = core::ModelInstance;

  //! Convenience function for getting interaction params
  InteractionParams getInteractionParams() const {
    return InteractionParams(m_params);
  }

  //! Convenience function that retrieve "active segments" (contacted by joints and gazed by Skeletons in skelRange)
  void getActiveSegments(const InteractionParams& iParams,
                         const Scan& scan,
                         const SkelRange& skelRange,
                         Skeleton::SegmentsByJointPlusGaze* pSegs) const;


  //! Convenience function that retrieve "active segments" (contacted by joints and gazed by Skeletons in skelRange)
  Skeleton::SegmentsByJointPlusGaze getActiveSegments(const Scan& scan,
                                                      const SkelRange& skelRange) const;

  //! Convenience function that retrieve "active parts" (contacted by joints and gazed by Skeletons in skelRange)
  void getActiveParts(const InteractionParams& iParams,
                      const Scan& scan,
                      const SkelRange& skelRange,
                      const PartType partType,
                      Skeleton::PartsByJointPlusGaze* pParts) const;

  Skeleton::PartsByJointPlusGaze getActiveParts(const Scan& scan,
                                                const SkelRange& skelRange,
                                                const PartType partType) const;

  //! Convenience function that retrieve "active segments" (contacted by joints and gazed by Skeletons in skelRange)
  Skeleton::SegmentsByJointPlusGaze getActiveSegments(const vec<std::reference_wrapper<const ModelInstance>> modelInstances,
                                                      const SkelRange& skelRange,
                                                      SegPtrToModelInstanceMap* pSegToModelInstMap = nullptr) const;

  //! Convenience function that retrieve "active parts" (contacted by joints and gazed by Skeletons in skelRange)
  Skeleton::PartsByJointPlusGaze getActiveParts(const vec<std::reference_wrapper<const ModelInstance>> modelInstances,
                                                const SkelRange& skelRange,
                                                const PartType partType,
                                                PartPtrToModelInstanceMap* pPartToModelInstMap = nullptr) const;

  IGPtr createInteractionGraph(const Scan& scan,
                               const Skeleton& skel,
                               const PartType partType,
                               const set<VerbNoun> verbNouns = set<VerbNoun>()) const;

  IGPtr createInteractionGraph(const Scan& scan,
                               const Skeleton& skel,
                               const PartType partType,
                               const Skeleton::PartsByJointPlusGaze& partsByJoint,
                               const set<VerbNoun> verbNouns = set<VerbNoun>()) const;

  IGPtr createInteractionGraph(const Scan& scan,
                               const Skeleton& skel,
                               const PartType partType,
                               const Skeleton::SegmentsByJointPlusGaze& segsByJoint,
                               const set<VerbNoun> verbNouns = set<VerbNoun>()) const;

  IGPtr createInteractionGraph(const vec<std::reference_wrapper<const ModelInstance>> modelInstances,
                               const Skeleton& skel,
                               const PartType partType,
                               const core::synth::ObjectLabeler& objectLabeler,
                               const set<VerbNoun> verbNouns = set<VerbNoun>()) const;

  IGPtr createInteractionGraph(const vec<ModelInstance>& modelInstances,
                               const Skeleton& skel,
                               const PartType partType,
                               const core::synth::ObjectLabeler& objectLabeler,
                               const set<VerbNoun> verbNouns = set<VerbNoun>()) const;

 private:
  const util::Params& m_params;
};

Skeleton::PartsByJointPlusGaze segmentsToParts(const core::Scan& scan,
                                               const Skeleton::SegmentsByJointPlusGaze& segsByJoint,
                                               const PartType partType);

}  // namespace interaction
}  // namespace sg
