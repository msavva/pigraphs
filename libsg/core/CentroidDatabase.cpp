#include "common.h"  // NOLINT

#include "core/CentroidDatabase.h"

// ReSharper disable once CppUnusedIncludeDirective
#include <ext-boost/serialization.h>

#include <mLibCore.h>

#include "core/Database.h"
#include "core/Recording.h"
#include "core/RecordingDatabase.h"
#include "core/Skeleton.h"
#include "interaction/Interaction.h"
#include "segmentation/MeshSegment.h"
#include "stats/centroidset.h"
#include "util/Params.h"

namespace sg {
namespace core {

BOOST_BINARY_SERIALIZABLE_IMPL(CentroidDatabase)

// Stores set of activated segments for each joint group (plus aggregated over all joints in last position)
typedef arr<set<const segmentation::MeshSegment*>, Skeleton::kNumJointGroups + 1> SegmentSetByJointGroup;

//! Helper function to accumulate segments that pass activation threshold per joint group
//  TODO(MS): Generalize so that aggregation can be over original joints too
SegmentSetByJointGroup accumulateSegments(const Database& database)  {
  SegmentSetByJointGroup segsPerJointGroup;

  set<const MeshSegment*>& allSegs = segsPerJointGroup[Skeleton::kNumJointGroups];  // All segments stored again here for convenience

  for (size_t iJointGroup = 0; iJointGroup < Skeleton::kNumJointGroups; iJointGroup++) {
    set<const MeshSegment*>& jointGroupSegs = segsPerJointGroup[iJointGroup];  // segments for this joint group

    // Count activation of segments in this joint group
    map<const MeshSegment*, float> segmentActivationCount;
    for (const string& recId : database.recordings.getLoadedRecordingIds()) {
      const Recording& rec = database.recordings.getRecording(recId, true, true);
      for (const interaction::Interaction* i : rec.interactions) {
        for (const auto& segsPerJoint : i->jointSegments) {
          const Skeleton::SegmentsByJointGroup segsPerJointGroup = Skeleton::groupSegmentsByJointGroup(segsPerJoint);
          for (const auto& seg : segsPerJointGroup[iJointGroup]) {
            // TODO(MS): WARNING - this count will do horrible things in case of pointer collision
            // if segments are deleted and recreated; should be fine when batching everything though
            const MeshSegment* pSeg = seg.get();
            if (segmentActivationCount.find(pSeg) == segmentActivationCount.end()) {
              segmentActivationCount[pSeg] = 0.0f;
            }
            segmentActivationCount[pSeg] += 1.0f;
          }
        }
      }
    }

    // Accept segments within activation threshold
    const float segmentUsageCutoff = database.params->get<float>("Clustering.segmentUsageCutoff") / database.params->get<UINT>("Interaction.skeletonSkip");
    for (const auto& p : segmentActivationCount) {
      if (p.second >= segmentUsageCutoff) {
        jointGroupSegs.insert(p.first);
        allSegs.insert(p.first);
      }
    }
  }

  return segsPerJointGroup;
}

void CentroidDatabase::create(const Database& database) {
  SG_LOG_INFO << "Extracting centroids...";
  const size_t totalCentroids = database.params->get<UINT>("Clustering.numCentroids");
  const double epsZCA = database.params->get<double>("Clustering.epsZCA");
  const SegmentFeatureGeneratorSimplistic gen;

  // Helper function to get numCentroids centroids out of set of MeshSegments
  const auto computeCentroids = [&] (const set<const MeshSegment*>& segs, const size_t numCentroids) {
    const size_t numFeats = gen.numFeatures();
    const size_t numSegs = segs.size();
    vecd segFeats(numSegs * numFeats);
    size_t featIdx = 0;
    for (const MeshSegment* seg : segs) {
      const vecd& feats = gen.generate(*seg);
      copy(feats.begin(), feats.end(), &segFeats[featIdx]);
      featIdx += numFeats;
    }
    return stats::unsupervisedFeatureLearning(segFeats, numSegs, numCentroids, epsZCA);
  };
  
  // Get sets of segments that pass activation threshold for each joint group
  SegmentSetByJointGroup segsByJointGroup = accumulateSegments(database);
  size_t totalSegs = 0;
  std::for_each(segsByJointGroup.begin(), segsByJointGroup.end() - 1, [&](const set<const MeshSegment*>& segs) {
    totalSegs += segs.size();
  });

  // For each joint group, get # centroids proportional to # of activated segs in that joint group and store away
  for (size_t iJointGroup = 0; iJointGroup < Skeleton::kNumJointGroups; iJointGroup++) {
    const set<const MeshSegment*> segs = segsByJointGroup[iJointGroup];
    if (segs.size() < 2) {  // TODO(MS): Handle trivial singleton case
      SG_LOG_WARN << "Skipping centroids for group: " << Skeleton::jointGroupName(iJointGroup)
                  << " since only " << to_string(segs.size()) << " segments activated.";
      continue;
    }
    const size_t numSegs = segs.size();
    const size_t numCentroids = static_cast<size_t>(ceilf(static_cast<float>(numSegs) / totalSegs * totalCentroids));
    const string centroidsId = Skeleton::jointGroupName(iJointGroup);
    _centroidSets[centroidsId] = computeCentroids(segs, numCentroids);
    SG_LOG_INFO << centroidsId << ": " << numSegs << "/" << totalSegs << " segs -> " << numCentroids << "/" << totalCentroids <<" centroids";
  }
  _centroidSets["ALL"] = computeCentroids(segsByJointGroup[Skeleton::kNumJointGroups], totalCentroids);
  SG_LOG_INFO << "ALL" << ": " << totalSegs << "/" << totalSegs << " segs -> " << totalCentroids << "/" << totalCentroids << " centroids";

  SG_LOG_INFO << " Extracting centroids Done." << endl;
}

vecd CentroidDatabase::computeCentroidActivations(const vecd& segFeats, size_t numSegs) const {
  SegmentFeatureGeneratorSimplistic gen;
  assert(segFeats.size() == numSegs * gen.numFeatures());

  SG_LOG_INFO << "Computing activations...";
  const vecd activations = _centroidSets.at("ALL").computeActivation(segFeats, numSegs);  // TODO(MS): Allow evaluation against specific joint group
  SG_LOG_INFO << "Computing activations Done.";
  return activations;
}

const stats::CentroidSet* CentroidDatabase::getCentroidSetForJointGroup(size_t jointGroupIdx) const {
  const string id = Skeleton::jointGroupName(jointGroupIdx);
  if (_centroidSets.count(id) == 0) {
    return nullptr;
  }
  return &_centroidSets.at(id);
}

const stats::CentroidSet* CentroidDatabase::getCentroidSetForAllJoints() const {
  const string id = "ALL";
  if (_centroidSets.count(id) == 0) {
    return nullptr;
  }
  return &_centroidSets.at(id);
}

}  // namespace core
}  // namespace sg
