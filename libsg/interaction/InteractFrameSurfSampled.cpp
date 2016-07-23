#include "common.h"  // NOLINT

#include "interaction/InteractFrameSurfSampled.h"

// ReSharper disable once CppUnusedIncludeDirective
#include <ext-boost/serialization.h>

#include "core/Model.h"
#include "core/ModelInstance.h"
#include "core/OccupancyGrid.h"
#include "core/Scan.h"
#include "interaction/SkeletonInteraction.h"
// ReSharper disable once CppUnusedIncludeDirective
#include "util/eigen_boost_serialization.h"
#include "util/timer.h"
#include "vis/vis.h"
#include "vis/ColorIndex.h"

namespace sg {
namespace interaction {

using geo::Vec3f; using geo::Matrix3f; using geo::OBB;

const string InteractionFrame::kSkeletonId = "Skeleton";

BOOST_BINARY_SERIALIZABLE_IMPL(InteractionFrame)

InteractionFrame::InteractionFrame(const Vec3f& halfDims /*= Vec3f(1, 1, 1)*/, int numBinsPerDim /*= 10*/)
  : m_halfDims(halfDims)
  , m_numBinsPerDim(numBinsPerDim)
  , m_binDim(m_halfDims[0] * 2.f / m_numBinsPerDim)
  , m_invBinSize(1.f / m_binDim)
  , m_numBinsPerDimSq(m_numBinsPerDim * m_numBinsPerDim)
  , m_obb(Vec3f::Zero(), m_halfDims, Matrix3f::Identity())
  , m_bins(m_numBinsPerDim * m_numBinsPerDimSq) { }

void InteractionFrame::addPoint(const string& id, const Vec3f& worldPoint, bool boundChecks /*= false*/) {
  const auto localPoint = m_obb.worldToLocal() * worldPoint;
  const int idx = binIndex(localPoint);
  if (idx < 0 && boundChecks) {
    cerr << "[InteractionFrame] Warning: point outside range: " << localPoint << endl;
  } else if (idx >= 0) {
    m_bins[idx].inc(id);
    m_points[id].push_back(localPoint);
    m_observationsCount.inc(id);
  } else {
    // No bounds checking and index out of bounds so just ignore point
  }
}

void InteractionFrame::addPoint(const string& id, const ml::vec3f& worldPoint, bool boundChecks /*= false*/) {
  addPoint(id, geo::vec3f(worldPoint), boundChecks);
}

void InteractionFrame::reposition(const Skeleton& skel) {
  const Vec3f bn = geo::vec3f(skel.bodyPlaneNormal),
              front = Vec3f(bn[0], bn[1], 0).normalized(),
              right = front.cross(Vec3f::UnitZ()),
              centroid = geo::vec3f(skel.centerOfMass());
  Matrix3f R;
  R.col(0) = right; R.col(1) = front; R.col(2) = Vec3f::UnitZ();
  m_obb = OBB(centroid, m_halfDims, R);
}

void InteractionFrame::recenter(const Vec3f& center) {
  m_obb = OBB(center, m_halfDims, Matrix3f::Identity());
}

vec<ml::vec3f> InteractionFrame::getWorldPoints(const string& id) const {
  if (m_points.count(id) == 0) {
    return vec<ml::vec3f>();
  }
  const auto& obbPoints = m_points.at(id);
  const size_t numPoints = obbPoints.size();
  vec<ml::vec3f> worldPoints(numPoints);
  const auto& worldT = m_obb.localToWorld();
  for (size_t iPoint = 0; iPoint < numPoints; ++iPoint) {
    worldPoints[iPoint] = geo::to<ml::vec3f>(worldT * obbPoints[iPoint]);
  }
  return worldPoints;
}

void InteractionFrame::clearPoints() {
  m_points.clear();
  m_bins.clear();
  m_bins.resize(m_numBinsPerDim * m_numBinsPerDimSq);
}

void InteractionFrame::clearPoints(const string& id) {
  if (m_points.count(id) > 0) {
    m_points.at(id).clear();
  }
  for (auto& bin : m_bins) {
    bin.resetCount(id);
  }
}

vec<InteractionFrame::Bin> InteractionFrame::getBins() const {
  vec<Bin> bins;
  for (int iBin = 0; iBin < m_bins.size(); ++iBin) {
    if (m_bins[iBin].totalCount() > 0) {  // TODO(MS): Cache counting if performance issue
      bins.push_back({m_bins[iBin], binCenter(iBin)});
    }
  }
  return bins;
}

// helper for selecting only major joints
bool isMajorJoint(int i) {
  using J = core::Skeleton::JointGroupLR;
  return (i == J::JointGroupLR_Head ||
          i == J::JointGroupLR_Hips ||
          i == J::JointGroupLR_Torso ||
          i == J::JointGroupLR_FootL ||
          i == J::JointGroupLR_FootR ||
          i == J::JointGroupLR_HandL ||
          i == J::JointGroupLR_HandR ||
          i == J::JointGroupLR_Gaze);
};

void InteractionFrame::getWeightedColorBins(const vis::IFVisParams& p,
                                            ColoredBinGrid* out,
                                            bool dropLowWeightBins /*= true*/) const {
  util::Timer timer("getWeightedColorBins");
  vis::ColorIndex cIndex;

  // Populate some information about each label
  struct LabelInfo
  {
    string id;
    string type;
    double weight;
    double normalization;
    ml::vec4f color;
  };
  std::unordered_map<string, LabelInfo> labelTypeMap;
  vec<string> jointNames(kSkelParams.kNumJoints);
  for (int iJoint = 0; iJoint < kSkelParams.kNumJoints; iJoint++) {
    const string& jointId = kSkelParams.kJointNames[iJoint];
    jointNames[iJoint] = jointId;
  }
  double normSkel = (p.renderSkelPts)? 1.0 / static_cast<double>(getBinWithMaxTotalCount(kSkeletonId).second) : 1.0;
  double normOcc = (p.renderOccupancy)? 1.0 / static_cast<double>(getBinWithMaxTotalCount({"OCC", "FREE", "UNK"}).second) : 1.0;
  for (int iJoint = 0; iJoint < kSkelParams.kNumJoints; iJoint++) {
    const string& jointId = kSkelParams.kJointNames[iJoint];
    double weight = 1.0;
    if (p.pJointWeights) {
      const int wgi = Skeleton::kJointGroupLRToJointGroup.at(iJoint);  // TODO(MS): Assumes kSkelParams is using JointGroupLR
      weight = p.pJointWeights[wgi];
    }
    double normJoint = (p.renderJointPts)? 1.0 / static_cast<double>(getBinWithMaxTotalCount(jointId).second) : 1.0;
    labelTypeMap[jointId] = { jointId, "Joint", weight, normJoint, kSkelParams.kJointColors[iJoint] };
  }
  labelTypeMap[kSkeletonId] = { kSkeletonId, kSkeletonId, 1.0, normSkel, cIndex.color(kSkeletonId, true).toVec4f() };
  labelTypeMap["OCC"] = { "OCC", "Occupancy", 1.0, normOcc, cIndex.color("OCC", true).toVec4f() };
  labelTypeMap["FREE"] = { "FREE", "Occupancy", 1.0, normOcc, cIndex.color("FREE", true).toVec4f() };
  labelTypeMap["UNK"] = { "UNK", "Occupancy", 1.0, normOcc, cIndex.color("UNK", true).toVec4f() };

  const auto& totalsCounter = getTotalsCounter();
  const auto addPoint = [&] (int iBin, const stats::Counter<string>& bin, const ml::vec3f& pos,
                             const string& id) {
    const auto& labelInfo = labelTypeMap[id]; 
    const size_t count = bin.count(id);
    //const float ratio = count * totalCountInv;
    //const int numSamplesPerObservation = 1000;  // TODO(MS): this really shouldn't be here
    //const float ratio = static_cast<float>(count) / totalsCounter.count(id) * numSamplesPerObservation;
    //const float binWeight2 = static_cast<float>(labelInfo.weight * ratio);
    const float binWeight = static_cast<float>(10 * labelInfo.weight * count * labelInfo.normalization);
    if (!dropLowWeightBins || binWeight > p.minBinWeight) {
      const float w = math::clamp((binWeight - p.minBinWeight) / (1 - p.minBinWeight), 0.f, 1.f);
      //cout << "bin " << iBin << " " << w << ", "<< binWeight << ", " << binWeight2 << endl;
      const auto c = labelInfo.color;
      const auto val = ml::vec4f(c.x, c.y, c.z, w);
      (*out)[binIndexToCoords(iBin)].emplace_back(ColoredBin{pos, val});
    }
  };

  const auto maxObjectLabelBin = getBinWithMaxTotalCount([&](const string& label) {
    return labelTypeMap.count(label) == 0;
  });
  double normObjectLabel = (p.renderOccupancy)? 1.0 / static_cast<double>(maxObjectLabelBin.second) : 1.0;
  for (int iBin = 0; iBin < m_bins.size(); ++iBin) {
    const auto& bin = m_bins[iBin];
    const auto pos = geo::to<ml::vec3f>(binCenter(iBin));

    //const float totalCountInv = 1.f / bin.counter.totalCount();
    // helper for adding entry to binCenters and binColors
    if (p.renderJointPts) {
      for (int iJoint = 0; iJoint < kSkelParams.kNumJoints; ++iJoint) {
        if (isMajorJoint(iJoint)) {
          const string jointName = kSkelParams.kJointNames[iJoint];
          addPoint(iBin, bin, pos, jointName);
        }
      }
    }
    if (p.renderSkelPts) {
      addPoint(iBin, bin, pos, kSkeletonId);
    }
    if (p.renderOccupancy) {
      //addPoint(iBin, bin, pos, "FREE");
      addPoint(iBin, bin, pos, "OCC");
      addPoint(iBin, bin, pos, "UNK");
    }
    if (p.renderObjectLabels) {
      for (const auto& it : bin) {
        const string& label = it.first;
        const size_t count = it.second;
        bool hasLabelInfo = labelTypeMap.count(label);
        bool isObjLabel = !hasLabelInfo || labelTypeMap.at(label).type == "ObjectLabel";
        if (isObjLabel) {
          if (!hasLabelInfo) {
            const ml::RGBColor& crgb = cIndex.color(label, true);
            ml::vec4f c = crgb.toVec4f();
            labelTypeMap[label] = { label, "ObjectLabel", 1.0, normObjectLabel, c};
            SG_LOG_INFO << "Adding object label " << label;
          }
          addPoint(iBin, bin, pos, label);
        }
      }
    }
  }

}

void InteractionFrame::getBinWorldCentroids(vec<geo::Vec3f>* out) const {
  if (out->size() != m_bins.size()) {
    out->resize(m_bins.size());
  }
  for (int iBin = 0; iBin < m_bins.size(); ++iBin) {
    (*out)[iBin] = m_obb.localToWorld() * binCenter(iBin);
  }
}

std::pair<int, size_t> InteractionFrame::getBinWithMaxTotalCount() const {
  pair<int, size_t> maxBin = {-1, 0};
  for (int iBin = 0; iBin < m_bins.size(); ++iBin) {
    const auto& bin = m_bins[iBin];
    const auto c = bin.totalCount();
    if (c > maxBin.second) {
      maxBin.first = iBin;
      maxBin.second = c;
    }
  }
  return maxBin;
}

std::pair<int, size_t> InteractionFrame::getBinWithMaxTotalCount(const vec<string>& labels) const {
  pair<int, size_t> maxBin = {-1, 0};
  for (int iBin = 0; iBin < m_bins.size(); ++iBin) {
    const auto& bin = m_bins[iBin];
    size_t c = 0;
    for (const string& label : labels) {
      c += bin.count(label);
    }
    if (c > maxBin.second) {
      maxBin.first = iBin;
      maxBin.second = c;
    }
  }
  return maxBin;
}

std::pair<int, size_t> InteractionFrame::getBinWithMaxTotalCount(std::function<bool(const string&)> filterFn) const {
  pair<int, size_t> maxBin = {-1, 0};
  for (int iBin = 0; iBin < m_bins.size(); ++iBin) {
    const auto& bin = m_bins[iBin];
    size_t c = 0;
    for (const auto& p : bin) {
      if (filterFn(p.first)) {
        c += p.second;
      }
    }
    if (c > maxBin.second) {
      maxBin.first = iBin;
      maxBin.second = c;
    }
  }
  return maxBin;
}

double InteractionFrame::support(const Skeleton& skel, const core::Scan* pScan) {
  if (!pScan) { return 0.0; }

  const ml::BinaryGrid3& scanVoxelGrid = pScan->getOccupancyGrid().unknownOrOccupied();
  const ml::mat4f& worldToGrid = pScan->getOccupancyGrid().worldToGrid();

  return support(skel, scanVoxelGrid, worldToGrid, true);
}

double InteractionFrame::support(const Skeleton& skel, const core::ModelInstance& modelInstance) {
  const ml::BinaryGrid3& voxelGrid = modelInstance.model.solidVoxels;
  const ml::mat4f worldToModel = geo::to<ml::mat4f>(modelInstance.getParentToLocal());
  const ml::mat4f worldToGrid =  modelInstance.model.modelToVoxel * worldToModel;

  return support(skel, voxelGrid, worldToGrid, false);
}

double InteractionFrame::support(const Skeleton& skel, 
                                         const ml::BinaryGrid3& voxelGrid,
                                         const ml::mat4f& worldToGrid,
                                         bool skipIfOutsideGrid) {
  vec<double> jointSupport(kSkelParams.kNumJoints, 0);    double skelSupport = 0;
  vec<size_t> jointBinCounts(kSkelParams.kNumJoints, 0);  size_t skelBinCount = 0;

  // TODO: Have boolean indicating if this repositioning needs to be done
  reposition(skel);

  // Get all nonempty frame bins and scan's voxel grid
  const auto bins = getBins();

  size_t numOutsideVoxelGrid = 0;
  size_t numBins = bins.size();
  for (const auto& bin : bins) {
    // Get bin center in voxel grid coords
    const auto binCenterWorld = m_obb.localToWorld() * bin.center;
    const ml::vec3f binCenterVoxel = worldToGrid * geo::to<ml::vec3f>(binCenterWorld);
    const ml::vec3i voxelCoord(static_cast<int>(binCenterVoxel.x),
                               static_cast<int>(binCenterVoxel.y),
                               static_cast<int>(binCenterVoxel.z));

    bool isValidVoxel = voxelGrid.isValidCoordinate(voxelCoord.x, voxelCoord.y, voxelCoord.z);
    if (!isValidVoxel) {
      numOutsideVoxelGrid++;
    // Skip if outside scan voxelization
      if (skipIfOutsideGrid) {
        continue;
      }
    }

    const bool voxelOccupied = isValidVoxel && voxelGrid.isVoxelSet(voxelCoord.x, voxelCoord.y, voxelCoord.z);
    //const double totalCountInv = 1.0 / bin.counter.totalCount();

    // If voxel set and joint interaction presence, add to support
    for (size_t iJoint = 0; iJoint < kSkelParams.kNumJoints; ++iJoint) {
      const string& jointId = kSkelParams.kJointNames[iJoint];
      const size_t jointC = bin.counter.count(jointId);
      if (jointC == 0) {  // No count for this joint
        continue;
      }
      const double sup = voxelOccupied ? jointC : 0.0;  // Good if occupied, bad if empty  // TODO(MS): weight this
      jointSupport[iJoint] += sup;
      jointBinCounts[iJoint]++;
    }

    // If voxel occupied and skeleton presence in frame, subtract from support
    const size_t skelC = bin.counter.count(kSkeletonId);
    if (skelC > 0) {
      skelSupport += voxelOccupied ? 0.0 : 1.0;  // Bad if occupied, good if empty
      skelBinCount++;
    }
  }

  //const double skelWeight = 0.1, totalJointWeight = 1 - skelWeight;

  // Add up total support
  double totalJointSupport = 0.0, totalSkelSupport = 0.0, totalSupport = 0.0;
  int numJointsContributing = 0;
  for (size_t iJoint = 0; iJoint < kSkelParams.kNumJoints; ++iJoint) {
    //const size_t jointCount = jointBinCounts[iJoint];
    const size_t jointCount = m_observationsCount.count(kSkelParams.kJointNames[iJoint]);
    if (jointCount == 0) { continue; }
    totalJointSupport += jointSupport[iJoint] / jointCount;
    numJointsContributing++;
  }
  if (skelBinCount > 0) {
    totalSkelSupport += skelSupport / skelBinCount;
  }
  if (numJointsContributing == 0) {
    totalSupport = totalSkelSupport;
  } else {
    totalSupport = totalSkelSupport + totalJointSupport / numJointsContributing;
  }

  SG_LOG_DEBUG << "IV numBins=" << numBins << ", numOutsideVoxelGrid=" << numOutsideVoxelGrid;

  SG_LOG_DEBUG << "IV totalSupport: " << totalSupport 
    << " with totalSkelSupport=" << totalSkelSupport
    << " totalJointSupport=" << totalJointSupport
    << " numJointsContributing=" << numJointsContributing;

  return totalSupport;
}

}  // namespace interaction
}  // namespace sg
