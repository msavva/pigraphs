#pragma once

#include "libsg.h"  // NOLINT
#include "core/Skeleton.h"
#include "interaction/VerbNoun.h"

namespace sg {
namespace interaction {

//! Skeleton interactions with scan geometry during a time range
struct Interaction {
  Interaction()
    : recording(nullptr), scan(nullptr), startTime(-1), endTime(-1), skelRange(), isetId(""), interactionSet(nullptr), interactionSets() {}
  core::Recording* recording;
  const core::Scan* scan;
  string recId;
  string scanId;
  float startTime, endTime;
  core::SkelRange skelRange;
  set<VerbNoun> verbNouns;
  string isetId;  //! Aggregate isetId of interaction (ordered concatenation of verbs)
  string id;  //! unique id of form: recId_startTime_endTime_isetId

  //! Composite interaction set corresponding to this interaction
  const InteractionSet* interactionSet;

  //! Interaction sets that this interaction belongs to
  vec<InteractionSet*> interactionSets;

  //! Vector indexed by sampled skeleton over Interaction range and giving array of closest segments for each joint
  vec<core::Skeleton::SegmentsByJointPlusGaze> jointSegments;

  //! Set of all segments interacting with joints of sampled skeletons over this Interaction range
  set<std::shared_ptr<segmentation::MeshSegment>> activeSegments;

private:
  //! boost serialization function
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {  // NOLINT
    boost::serialization::split_member(ar, *this, version);
  }
  template <typename Archive>
  void save(Archive& ar, const unsigned int version) const {  // NOLINT
    ar & id;
    ar & isetId;
    ar & recId;
    ar & scanId;
    ar & startTime;
    ar & endTime;
  }
  template <typename Archive>
  void load(Archive& ar, const unsigned int version) {  // NOLINT
    ar & id;
    ar & isetId;
    ar & recId;
    ar & scanId;
    ar & startTime;
    ar & endTime;
    VerbNoun::getVerbNouns(isetId, &verbNouns);

    // Clear everything else that we can't recover
    recording = nullptr;
    scan = nullptr;
    skelRange = {};
    interactionSet = nullptr;
    interactionSets.clear();
    jointSegments.clear();
    activeSegments.clear();
  }

};

}  // namespace interaction
}  // namespace sg


