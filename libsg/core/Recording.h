#pragma once

#include <mLibCore.h>

#include "libsg.h"  // NOLINT
#include "core/Skeleton.h"

namespace sg {
namespace core {

//! Recording containing a stream of Skeletons as well as optional color and combined depth+bodyIndex frames
//! Frames are stored externally in video files but timestamps are recorded here
//! annotations are stored in an adjacent text file and loaded independently.
struct Recording {
  //! Multiplier to convert timestamps to seconds
  static const float kTimestampToSec;

  string id;
  ml::mat4f camera;
  uint64_t startTime;
  uint64_t endTime;
  vec<Skeleton> skeletons;
  vec<int64_t> colorTimestamps;
  vec<int64_t> depthTimestamps;

  // STATE - NOT STORED
  bool isLive;            //! Whether this Recording is currently being recorded
  bool isLoaded;          //! Whether this Recording has been loaded from file
  bool hasInteractions;   //! Whether this Recording has loaded up interactions
  bool hasActiveSegments; //! Whether active segments have been computed for this Recording
  bool isUnfloated;       //! Whether skeletons in this Recording have been post-processed for unfloating
  vec<interaction::Interaction*> interactions; //! stored independently and should not be archived

  Recording()
    : id("")
    , camera(ml::mat4f::identity())
    , startTime(0)
    , endTime(0)
    , skeletons()
    , colorTimestamps()
    , depthTimestamps()
    , isLive(false)
    , isLoaded(false)
    , hasInteractions(false)
    , hasActiveSegments(false)
    , isUnfloated(false) { }

  //! boost serialization function
  template<class Archive>
  void serialize(Archive& ar, const unsigned int) {  // NOLINT
    ar & id;
    ar & camera;
    ar & startTime;
    ar & endTime;
    ar & skeletons;
    ar & colorTimestamps;
    ar & depthTimestamps;
  }

  //! load from serialized Recording in file to rec and return whether success
  //! optional pointer to cameraExtrinsics matrix transforms all recorded skeletons (overriding saved camera)
  bool loadFromArchive(const string& file, const ml::mat4f* cameraExtrinsics = nullptr);

  //! Load from readable JSON file
  bool loadFromJSON(const string& file);

  //! Convert a vector of skeleton states captured at given fps into a recording
  bool loadFromSkelStates(const string& id, const vec<SkelState>& skelStates, float fps,
                          const ml::mat4f* camera = nullptr);

  //! Load interactions track for recording
  bool loadInteractions(const util::Params& params);

  //! Compute active segments of the interactions within this Recording against the given scene
  bool computeActiveSegments(const util::Params& params, const Scan& scene,
                             float maxDistToSegment, float maxDistGaze,
                             bool forceRecompute = false);

  //! Compute active segments of the interactions within this Recording against the given scene
  bool computeActiveSegments(const util::Params& params, const Scan& scene,
                             bool forceRecompute = false);

  //! Fix up this Recording's skeletons by making sure they are all statically supported within the given scene
  void unfloatSkeletons(const Scan& scan);

  //! save Recording to file
  bool save(const string& file) const;

  //! Number of skeletons stored in this Recording
  size_t numSkeletonFrames() const { return skeletons.size(); }

  //! Duration of this Recording in seconds
  float durationInSec() const {
    // NOTE: end and start time are in microseconds
    return (endTime - startTime) * 1.0e-6f;
  }

  //! Return the interactions filename
  string interactionsFilename() const;

  //! Return scan id in which this Recording was taken
  string baseScanId() const;

  //! Iterator to a Skeleton within this Recording
  typedef vec<Skeleton>::const_iterator SkelIter;

  //! Attempt to set skelItOut so it points to first Skeleton observed
  //! after tInSec seconds from start. Returns false if no such skeleton.
  bool getSkeletonAtTime(float tInSec, SkelIter& skelItOut) const;  // NOLINT

  //! Return first Skeleton after time tInSec, or last Skeleton if tInSec > tEnd
  const Skeleton& getSkeletonAtTime(float tInSec) const;

  //! Return SkelRange of skeletons in [tStart,tEnd) with a step size of stride
  SkelRange getSkeletonRange(float tStart, float tEnd, size_t stride) const;

  //! Outputs this Recording to passed ostream in JSON format and returns the ostream
  ostream& toJSON(ostream&, bool endlines) const;

 private:
  //! Helper function to finalize loading of recording (apply camera transform to Skeletons, set id, etc.)
  void finalizeLoad(const string& file);
  //! Helper to apply rotation to all skeletons
  void applyRotation(const ml::mat4f& xform);
  void applyTransform(const ml::mat4f& xform);

  //! Filename from which the interactins were loaded
  string m_interactionsFilename;
};

inline ostream& toJSON(ostream& os, const Recording& r, bool endlines = true) {  // NOLINT
  return r.toJSON(os, endlines);
}
inline ostream& operator<<(ostream& os, const Recording& r) {
  return r.toJSON(os, true);
}

}  // namespace core
}  // namespace sg


