#include "common.h"  // NOLINT

#include "interaction/InteractionFactory.h"

#include "core/Model.h"
#include "core/ModelInstance.h"
#include "core/Scan.h"
#include "interaction/InteractionGraph.h"
#include "mesh/mesh_nn.h"
#include "segmentation/MeshSegment.h"
#include "segmentation/Part.h"

namespace sg {
namespace interaction {

using segmentation::Part;
using segmentation::ConstPartPtr;
using segmentation::SegPtr;
using segmentation::VecSegPtr;
using core::Scan;

void segmentsToParts(const Scan& scan,
                     const Skeleton::SegmentsByJointPlusGaze& segsByJoint,
                     const PartType partType,
                     Skeleton::PartsByJointPlusGaze* pPartsByJoints) {
  Skeleton::PartsByJointPlusGaze& partsByJoint = *pPartsByJoints; 
  for (auto& parts : partsByJoint) {
    parts.clear();
  }
  switch (partType) {
  case segmentation::kPartSegment:
    // Simply convert pointers
    for (size_t j = 0; j < partsByJoint.size(); ++j) {
      for (const SegPtr pSeg : segsByJoint.at(j)) {
        partsByJoint[j].push_back(std::static_pointer_cast<const Part>(pSeg));
      }
    }
    break;
  case segmentation::kPartSegmentGroup:
    // Group into segment groups
    for (size_t j = 0; j < partsByJoint.size(); ++j) {
      for (const SegPtr pSeg : segsByJoint.at(j)) {
        const segmentation::SegmentGroups::SegmentGroupHandle segGroupId = scan.segmentGroups.find(*pSeg);
        const ConstPartPtr pPart = scan.segmentGroups.getSegGroupPartPtr(segGroupId);
        if (pPart != nullptr && !util::contains(partsByJoint[j], pPart)) {
          partsByJoint[j].push_back(pPart);
        }
      }
    }
    break;
  case segmentation::kPartObject:
    // Group into objects;
    for (size_t j = 0; j < partsByJoint.size(); ++j) {
      for (const SegPtr pSeg : segsByJoint.at(j)) {
        const segmentation::SegmentGroups::SegmentGroupHandle segGroupId = scan.segmentGroups.find(*pSeg);
        const segmentation::SegmentGroup* pSegGroup = scan.segmentGroups.get(segGroupId);
        if (pSegGroup != nullptr) {
          const ConstPartPtr pPart = scan.segmentGroups.getObjectPartPtr(pSegGroup->objectId);
          if (pPart != nullptr && !util::contains(partsByJoint[j], pPart)) {
            partsByJoint[j].push_back(pPart);
          }
        }
      }
    }
    break;
  default:
    throw SG_EXCEPTION("Unsupported part type " + partType);
  }
}

Skeleton::PartsByJointPlusGaze segmentsToParts(const Scan& scan,
                                               const Skeleton::SegmentsByJointPlusGaze& segsByJoint,
                                               const PartType partType) {
  Skeleton::PartsByJointPlusGaze partsByJoint;
  segmentsToParts(scan, segsByJoint, partType, &partsByJoint);
  return partsByJoint;
}

void segmentsToParts(const Skeleton::SegmentsByJointPlusGaze& segsByJoint,
                     const PartType partType,
                     const SegPtrToModelInstanceMap& segToModelInstMap,
                     Skeleton::PartsByJointPlusGaze* pPartsByJoints,
                     PartPtrToModelInstanceMap* pPartToModelInstMap) {
  Skeleton::PartsByJointPlusGaze& partsByJoint = *pPartsByJoints; 
  for (auto& parts : partsByJoint) {
    parts.clear();
  }
  // TODO: Propagate from segPtrToModelInstanceMap to PartPtrToModelInstanceMap
  // TODO: Implement!!!
  switch (partType) {
  case segmentation::kPartSegment:
    // Simply convert pointers
    for (size_t j = 0; j < partsByJoint.size(); ++j) {
      for (const SegPtr pSeg : segsByJoint.at(j)) {
        partsByJoint[j].push_back(std::static_pointer_cast<Part>(pSeg));
      }
    }
    for (const auto& it : segToModelInstMap) {
      pPartToModelInstMap->insert({std::static_pointer_cast<Part>(it.first), it.second});
    }
    break;
  default:
    throw SG_EXCEPTION("Unsupported part type " + partType);
  }
}

Skeleton::PartsByJointPlusGaze segmentsToParts(const Skeleton::SegmentsByJointPlusGaze& segsByJoint,
                                               const PartType partType,
                                               const SegPtrToModelInstanceMap& segToModelInstMap,
                                               PartPtrToModelInstanceMap* pPartToModelInstMap) {
  Skeleton::PartsByJointPlusGaze partsByJoint;
  segmentsToParts(segsByJoint, partType, segToModelInstMap, &partsByJoint, pPartToModelInstMap);
  return partsByJoint;
}

InteractionParams::InteractionParams(const sg::util::Params& p)
{
  ignoreInferredJoints = p.get<bool>("Interaction.ignoreInferredJoints");
  kNearestSegsPerJoint = p.get<unsigned>("Interaction.kNearestSegsPerJoint");
  maxDistToSegment = p.get<float>("Interaction.maxDistToSegment");
  maxDistGaze = p.get<float>("Interaction.maxDistGaze");
  maxSegmentSizeRatio = p.get<float>("Interaction.maxSegmentSizeRatio");
}

void InteractionFactory::getActiveSegments(const InteractionParams& iparams,
                                           const Scan& scan,
                                           const SkelRange& skelRange,
                                           Skeleton::SegmentsByJointPlusGaze* pSegs) const
{
  scan.getActiveSegments(skelRange, iparams.ignoreInferredJoints,
                         iparams.kNearestSegsPerJoint,
                         iparams.maxDistToSegment,
                         iparams.maxDistGaze,
                         iparams.maxSegmentSizeRatio,
                         pSegs);
}

Skeleton::SegmentsByJointPlusGaze InteractionFactory::getActiveSegments(const Scan& scan,
                                                                        const SkelRange& skelRange) const {
  Skeleton::SegmentsByJointPlusGaze segsByJoint;
  const util::Params& p = m_params;
  const InteractionParams iparams(p);

  scan.getActiveSegments(skelRange, iparams.ignoreInferredJoints,
                         iparams.kNearestSegsPerJoint,
                         iparams.maxDistToSegment,
                         iparams.maxDistGaze,
                         iparams.maxSegmentSizeRatio,
                         &segsByJoint);
  return segsByJoint;
}

void InteractionFactory::getActiveParts(const InteractionParams& iParams,
                                        const Scan& scan,
                                        const SkelRange& skelRange,
                                        const PartType partType,
                                        Skeleton::PartsByJointPlusGaze* pParts) const {
  Skeleton::SegmentsByJointPlusGaze segsByJoint = getActiveSegments(scan, skelRange);
  segmentsToParts(scan, segsByJoint, partType, pParts);
}

Skeleton::PartsByJointPlusGaze InteractionFactory::getActiveParts(const Scan& scan,
                                                                  const SkelRange& skelRange,
                                                                  const PartType partType) const {
  Skeleton::SegmentsByJointPlusGaze segsByJoint = getActiveSegments(scan, skelRange);
  return segmentsToParts(scan, segsByJoint, partType);
}

Skeleton::SegmentsByJointPlusGaze InteractionFactory::getActiveSegments(const vec<std::reference_wrapper<const ModelInstance>> modelInstances,
                                                                        const SkelRange& skelRange,
                                                                        SegPtrToModelInstanceMap* pSegToModelInstMap /* = nullptr */) const {
  Skeleton::SegmentsByJointPlusGaze segsByJoint;
  const util::Params& p = m_params;

  const InteractionParams iparams(p);
  segmentation::SegmentationParams segParams(m_params);

  for (const ModelInstance& mInst : modelInstances) {
    // TODO: What segs to use for the model instance
    if (mInst.model.segments == nullptr) {
      SG_LOG_WARN << "No segments for model " << mInst.model.id;
      continue;
    }
    const VecSegPtr& rawSegs = *mInst.model.segments;

    // We need to get a copy of the mInst model segments with a transform applied
    // This will be the actual segs used in for the interaction graph
    VecSegPtr segs;
    segs.reserve(rawSegs.size());
    const geo::Transform& xform = mInst.getLocalTransform();
    for (const SegPtr rawSeg : rawSegs) {
      // TODO: Check parameters
      SegPtr seg = rawSeg->copy(segParams.constrainZup, 0, xform);
      segs.emplace_back(seg);
      if (pSegToModelInstMap != nullptr) {
        (*pSegToModelInstMap).insert({seg,mInst});
      }
    }

    mesh::MeshHelper meshHelper;
    meshHelper.init(mInst.model.flattenedMesh);

    for (const Skeleton& skel : skelRange) {
      meshHelper.getActiveSegments(
        segs, skel, iparams.ignoreInferredJoints,
        iparams.kNearestSegsPerJoint,
        iparams.maxDistToSegment,
        iparams.maxDistGaze,
        -1,
        &segsByJoint,
        true);
    }

  }
  return segsByJoint;
}

Skeleton::PartsByJointPlusGaze InteractionFactory::getActiveParts(const vec<std::reference_wrapper<const ModelInstance>> modelInstances,
                                                                  const SkelRange& skelRange,
                                                                  const PartType partType,
                                                                  PartPtrToModelInstanceMap* pPartToModelInstMap /* = nullptr*/) const {
  SegPtrToModelInstanceMap segPtrToModelInstanceMap;
  Skeleton::SegmentsByJointPlusGaze segsByJoint = getActiveSegments(modelInstances, skelRange, &segPtrToModelInstanceMap);
  return segmentsToParts(segsByJoint, partType, segPtrToModelInstanceMap, pPartToModelInstMap);
}


IGPtr InteractionFactory::createInteractionGraph(const Scan& scan,
                                                 const Skeleton& skel,
                                                 const PartType partType,
                                                 const set<VerbNoun> verbNouns /* = vec<VerbNoun>() */) const {
  const SkelRange skelRange(skel);
  Skeleton::PartsByJointPlusGaze partsByJoints = getActiveParts(scan, skelRange, partType);
  return createInteractionGraph(scan, skel, partType, partsByJoints, verbNouns);
}

IGPtr InteractionFactory::createInteractionGraph(const Scan& scan,
                                                 const Skeleton& skel,
                                                 const PartType partType,
                                                 const Skeleton::SegmentsByJointPlusGaze& segsByJoint,
                                                 const set<VerbNoun> verbNouns /* = vec<VerbNoun>() */) const {
  Skeleton::PartsByJointPlusGaze partsByJoint = segmentsToParts(scan, segsByJoint, partType);
  return createInteractionGraph(scan, skel, partType, partsByJoint, verbNouns);
}

IGPtr InteractionFactory::createInteractionGraph(const Scan& scan,
                                                 const Skeleton& skel,
                                                 const PartType partType,
                                                 const Skeleton::PartsByJointPlusGaze& partsByJoint,
                                                 const set<VerbNoun> verbNouns /* = vec<VerbNoun>() */) const {
  std::function<string(ConstPartPtr)> getPartLabel = [&] (ConstPartPtr pPart) {
    string label;
    switch (pPart->partType) {
      case segmentation::kPartSegment: {
        // Get noun:part label if seg is contained in annotated SegmentGroup
          const auto pSeg = std::static_pointer_cast<const segmentation::MeshSegment>(pPart);
          const auto segGroupId = scan.segmentGroups.find(*pSeg);
          if (segGroupId >= 0) {
            label = scan.segmentGroups.get(segGroupId)->label;
          }
          return label;
        }
      case segmentation::kPartSegmentGroup: {
        const auto pSegGroup = std::static_pointer_cast<const segmentation::SegmentGroup>(pPart);
        label = pSegGroup->label;
        return label;
      }
      case segmentation::kPartObject: {
        const auto pObject = std::static_pointer_cast<const segmentation::SegmentGroupObject>(pPart);
        label = pObject->label;
        return label;
      }
    }
    return label;
  };
  std::function<string(ConstPartPtr)> getPartTargetedBy = [&] (ConstPartPtr pPart) {
    string label = getPartLabel(pPart);
    vec<string> matchingVerbs;
    string noun = core::synth::ObjectLabeler::labelToAnnotationNoun(label);
    for (const VerbNoun& vn : verbNouns) {
      if (vn.noun == noun) {
        matchingVerbs.push_back(vn.verb);
      }
    }
    return util::join(matchingVerbs, ",");
  };
  const core::OccupancyGrid& occupancyGrid = scan.getOccupancyGrid();
  return std::make_shared<IG>(partType, skel, partsByJoint, 
                              getPartLabel, getPartTargetedBy,
                              verbNouns, &occupancyGrid);
}

IGPtr InteractionFactory::createInteractionGraph(const vec<std::reference_wrapper<const ModelInstance>> modelInstances,
                                                 const Skeleton& skel,
                                                 const PartType partType,
                                                 const core::synth::ObjectLabeler& objectLabeler,
                                                 const set<VerbNoun> verbNouns /* = set<VerbNoun>() */) const {

  const SkelRange skelRange(skel);
  PartPtrToModelInstanceMap partToModelInstanceMap;
  // Get interacting parts by joints
  Skeleton::PartsByJointPlusGaze partsByJoints = getActiveParts(modelInstances, skelRange, partType, &partToModelInstanceMap);
  // Define funciton for getting part labels
  std::function<string(ConstPartPtr)> getPartLabel = [&] (ConstPartPtr pPart) {
    // Get noun:part label from model instance
    const ModelInstance& modelInst = partToModelInstanceMap.at(pPart);
    string label = objectLabeler.getAnnotationNoun(modelInst.model.categories);
    return label;
  };
  std::function<string(ConstPartPtr)> getPartTargetedBy = [&] (ConstPartPtr pPart) {
    string label = getPartLabel(pPart);
    vec<string> matchingVerbs;
    string noun = core::synth::ObjectLabeler::labelToAnnotationNoun(label);
    for (const VerbNoun& vn : verbNouns) {
      if (vn.noun == noun) {
        matchingVerbs.push_back(vn.verb);
      }
    }
    return util::join(matchingVerbs, ",");
  };
  return std::make_shared<IG>(partType, skel, partsByJoints, getPartLabel, getPartTargetedBy, verbNouns);
}

IGPtr InteractionFactory::createInteractionGraph(const vec<ModelInstance>& modelInstances,
                                                 const Skeleton& skel,
                                                 const PartType partType,
                                                 const core::synth::ObjectLabeler& objectLabeler,
                                                 const set<VerbNoun> verbNouns /* = set<VerbNoun>() */) const {
  vec<std::reference_wrapper<const ModelInstance>> v;
  v.reserve(modelInstances.size());
  for (const ModelInstance& m : modelInstances) {
    v.emplace_back(m);
  }
  return createInteractionGraph(v, skel, partType, objectLabeler, verbNouns);
}

}  // namespace interaction
}  // namespace sg
