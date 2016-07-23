#include "common.h"  // NOLINT

#include "interaction/InteractionFeatures.h"

#include "core/Scan.h"
#include "interaction/Interaction.h"
#include "interaction/InteractionFactory.h"
#include "util/util.h"

using sg::core::Scan;
using sg::util::join;

namespace sg {
namespace interaction {

// TODO: Merge with functions in InteractionFactory
string getPartLabel(ConstPartPtr pPart) {
  string label;
  switch (pPart->partType) {
    case segmentation::kPartSegment: {
      label = pPart->label;
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

// TODO: Merge with functions in InteractionFactory
string getPartTargetedBy(const set<VerbNoun>& verbNouns, ConstPartPtr pPart) {
  string label = getPartLabel(pPart);
  vec<string> matchingVerbs;
  string noun = core::synth::ObjectLabeler::labelToAnnotationNoun(label);
  for (const VerbNoun& vn : verbNouns) {
    if (vn.noun == noun) {
      matchingVerbs.push_back(vn.verb);
    }
  }
  return join(matchingVerbs, ",");
};

ofstream& writeInteractionFeatsHeader(ofstream& ofs) {
  vec<string> featNames(kRelFeats.size());
  for (size_t i = 0; i < kRelFeats.size(); i++) {
    featNames[i] = kRelFeats[i].id;
  }
  ofs << "rec,id,iset,scan,timestamp,joint,segment,targetedBy," << join(featNames) << endl;
  return ofs;
}

ofstream& writeInteractionFeats(ofstream& ofs, const InteractionFactory& ifactory,
                                const vec<Interaction*>& interactions) {
  Skeleton::PartsByJointPlusGaze partsByJoint;
  const InteractionParams iparams = ifactory.getInteractionParams();
  vec<float> relFeats;
  PartType partType = PartType::kPartSegment;

  Joint combinedJoint;
  uset<ConstPartPtr> addedParts;

  size_t index = 0;
  for (const auto it : interactions) {
    if (it->scan == nullptr) {
      SG_LOG_WARN << "No scan for interactions!!! Cannot compute features!!!";
      continue;
    }

    const Scan& scan = *it->scan;

    string commonRowFieldsInter = it->recId + "," + to_string(index) + "," + it->isetId + "," + it->scanId + ",";
    for (const auto& skel : it->skelRange) {
      // Get activated parts
      core::SkelRange singleSkel(skel);
      ifactory.getActiveParts(iparams, scan, singleSkel, partType, &partsByJoint);

      string commonRowFieldsSkel = commonRowFieldsInter + to_string(skel.get().timestamp) + ",";
      // Compute and output features
      for (int iJoint = 0; iJoint < kSkelParams.kNumJoints; iJoint++) {
        if (iJoint == kSkelParams.kGazeJoint) {
          // Handle gaze 
          const auto& partPtrs = partsByJoint[Skeleton::JointType_Count]; 

          for (const auto partPtr : partPtrs) {
            gazeFeats(skel, partPtr, &relFeats);

            string targetedBy = getPartTargetedBy(it->verbNouns, partPtr);
            ofs << commonRowFieldsSkel
                << iJoint << "," << partPtr->id << "," << targetedBy << ","
                << join(relFeats) << endl;
          }
        } else {
          // Create combined joint
          createCombinedJoint(kSkelParams, iJoint, skel, &combinedJoint);

          // Make sure parts are not added multiple times for this joint
          addedParts.clear();
          for (size_t iSkelJoint: combinedJoint.rawIndices) {
            const auto& partPtrs = partsByJoint[iSkelJoint]; 
            for (const auto partPtr : partPtrs) {
              if (addedParts.count(partPtr) == 0) {
                contactFeats(skel, combinedJoint, partPtr, &relFeats);

                string targetedBy = getPartTargetedBy(it->verbNouns, partPtr);
                ofs << commonRowFieldsSkel
                    << iJoint << "," << partPtr->id << "," << targetedBy << ","
                    << join(relFeats) << endl;
                addedParts.insert(partPtr);
              }
            }
          }
        }
      }
    }
    index++;
  }
  return ofs;
}

}  // namespace interaction
}  // namespace sg
