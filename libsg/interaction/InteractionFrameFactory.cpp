#include "common.h"  // NOLINT

#include "interaction/InteractionFrameFactory.h"

#include "core/Database.h"
#include "core/LabeledGrid.h"
#include "core/OccupancyGrid.h"
#include "core/Scan.h"
#include "core/synth/ObjectLabeler.h"
#include "interaction/InteractFrameSurfSampled.h"
#include "interaction/SkeletonInteraction.h"
#include "mesh/mesh_nn.h"
#include "mesh/sampling.h"
#include "segmentation/MeshSegment.h"
#include "vis/vis.h"

namespace sg {
namespace interaction {

InteractionFrameFactoryParams::InteractionFrameFactoryParams(const util::Params& p) {
  numSkelSamplePoints           = p.get<int>("InteractionFrame.numSkelSamplePoints");
  numInteractingOBBSamplePoints = p.get<int>("InteractionFrame.numInteractingOBBSamplePoints");
  numOccupancySamplePoints      = p.get<int>("InteractionFrame.numOccupancySamplePoints");
  halfWidth                     = p.get<float>("InteractionFrame.halfWidth");
  numBins                       = p.get<int>("InteractionFrame.numBins");
  ignoreInferredJoints          = p.get<bool>("Interaction.ignoreInferredJoints");
  maxDistGazeActivation         = p.get<float>("Interaction.maxDistGaze");
  maxDistJointActivation        = p.get<float>("Interaction.maxDistToSegment");
  maxSegmentSizeRatio           = p.get<float>("Interaction.maxSegmentSizeRatio");
  kNearestSegsPerJoint          = p.get<int>("Interaction.kNearestSegsPerJoint");
  computeJointsSurface          = p.get<bool>("InteractionFrame.computeJointsSurface");
}

InteractionFrameFactory::InteractionFrameFactory(const InteractionFrameFactoryParams& p,
                                                 const core::Database& db)
                                                 : m_params(p), m_db(db) { }

std::shared_ptr<InteractionFrames>
InteractionFrameFactory::createInteractionFrames(const vec<Interaction*>& interactions) const {
  const InteractionFrameFactoryParams& iffp = m_params;
  const geo::Vec3f ifDims(iffp.halfWidth, iffp.halfWidth, iffp.halfWidth);
  auto pFrames = std::make_shared<InteractionFrames>(ifDims, iffp.numBins);
  addInteractions(interactions, pFrames.get());
  pFrames->recenter(geo::Vec3f(0, 0, 0));
  return pFrames;
}

void InteractionFrameFactory::addInteractions(const vec<Interaction*>& interactions,
                                              InteractionFrames* pFrames) const {
  for (const Interaction* pIn : interactions) {
    addInteraction(*pIn->scan, pIn->skelRange, pFrames);
  }
}

void InteractionFrameFactory::addInteraction(const Interaction& interaction,
                                             InteractionFrames* pFrames) const {
  addInteraction(*interaction.scan, interaction.skelRange, pFrames);
}

void InteractionFrameFactory::addInteraction(const core::Scan& scan, 
                                             const core::SkelRange& skelRange, 
                                             InteractionFrames* pFrames) const {
  // For now we compute all interaction frames types 
  // (if too slow, can add options to disable some)
  vec<std::shared_ptr<InteractionFrame>>& iframes = pFrames->getInteractionFrames();
  for (size_t i = 0; i < iframes.size(); ++i) {
    if (iframes[i] == nullptr) {
      SG_LOG_WARN << "Cannot add interaction: IFrame " << i << " not initialized";
      continue;
    }
    InteractionFrame* pFrame = iframes[i].get();
    InteractionFrameType type = static_cast<InteractionFrameType>(i);
    switch (type) {
      case InteractionFrameType::kSkeleton:
        addSkeletonPoints(skelRange, pFrame);
        break;
      case InteractionFrameType::kJointsSurface:
        if (m_params.computeJointsSurface) {
          addInteractingScanMeshVertices(scan, skelRange, pFrame);
        }
        break;
      case InteractionFrameType::kJointsVolumetric:
        addInteractingOBBSamplePoints(scan, skelRange, pFrame);
        break;
      case InteractionFrameType::kOccupancy:
        addOccupancy(scan, skelRange, pFrame);
        break;
      case InteractionFrameType::kObjectLabel:
        addObjectLabels(scan, skelRange, pFrame);
        break;
      default:
        SG_LOG_WARN << "Unknown interaction frame type: " << type;
    }
  }
}

void InteractionFrameFactory::addSkeletonPoints(const core::SkelRange& skelRange,
                                                InteractionFrame* pFrame) const {
  for (const Skeleton& skel : skelRange) {
    pFrame->reposition(skel);
    const ml::TriMeshf skelMesh = vis::toTriMesh(skel, false, true, false, false, false, false);
    vec<geo::Vec3f> points;
    mesh::sampling::sampleTris(skelMesh, m_params.numSkelSamplePoints, &points);
    for (const auto& p : points) {
      pFrame->addPoint(pFrame->kSkeletonId, p);
    }
  }
}

void InteractionFrameFactory::addInteractingOBBSamplePoints(const core::Scan& scan,
                                                            const core::SkelRange& skelRange,
                                                            InteractionFrame* pFrame) const {
  for (const Skeleton& skel : skelRange) {
    pFrame->reposition(skel);
    Skeleton::SegmentsByJointPlusGaze segsByJoint;
    scan.getActiveSegments(skel, m_params.ignoreInferredJoints,
                           m_params.kNearestSegsPerJoint,
                           m_params.maxDistJointActivation,
                           m_params.maxDistGazeActivation,
                           m_params.maxSegmentSizeRatio,
                           &segsByJoint);
    size_t numSegs = 0;
    for (const vec<std::shared_ptr<segmentation::MeshSegment>>& segs : segsByJoint) {
      numSegs += segs.size();
    }
    if (numSegs > 0) {
      // TODO(MS): Consider using volume-weighted sample assignment
      size_t numSamplesPerSeg = m_params.numInteractingOBBSamplePoints / numSegs;
      for (int iJoint = 0; iJoint < Skeleton::kNumJoints + 1; ++iJoint) {
        const int iJointGroup = kSkelParams.kJointMap[iJoint];
        const string& jointId = kSkelParams.kJointNames[iJointGroup];
        for (const auto& seg : segsByJoint[iJoint]) {
          for (size_t iSample = 0; iSample < numSamplesPerSeg; ++iSample) {
            const auto p = seg->obb()->sample();
            pFrame->addPoint(jointId, p);
          }
        }
      }
    }
  }
}

void InteractionFrameFactory::addInteractingScanMeshVertices(const core::Scan& scan,
                                                             const core::SkelRange& skelRange,
                                                             InteractionFrame* pFrame) const {
  const auto iJoint2JointGroup = [&] (int iJoint) {
    return kSkelParams.kJointMap[iJoint];
  };
  const mesh::MeshHelper& meshNN = scan.getMeshHelper();
  const auto& V = scan.mesh.getVertices();
  const int kNumJointGroups = static_cast<int>(kSkelParams.kNumJoints);
  if(!m_pActRec) {
    m_pActRec = std::make_shared<mesh::MeshActivationRecord>();
  }
  m_pActRec->init(kNumJointGroups, Skeleton::kNumJoints,
                  static_cast<int>(V.size()), 2048);
  for (const Skeleton& skel : skelRange) {
    m_pActRec->reset();
    meshNN.accumulateContactActivation(&skel.jointPositions[0][0],
                                       Skeleton::kNumJoints,
                                       m_params.maxDistJointActivation,
                                       iJoint2JointGroup, m_pActRec.get());
    meshNN.accumulateGazeActivation(skel, m_params.maxDistGazeActivation,
                                    static_cast<int>(kSkelParams.kGazeJoint),
                                    m_pActRec.get());
    pFrame->reposition(skel);
    for (int iJointGroup = 0; iJointGroup < kNumJointGroups; ++iJointGroup) {
      const string& jointGroupName = kSkelParams.kJointNames[iJointGroup];
      const auto& actVerts = m_pActRec->actVertSets[iJointGroup];
      for (const int iV : actVerts) {
        pFrame->addPoint(jointGroupName, V[iV].position);
      }
    }
  }
}

void InteractionFrameFactory::addOccupancy(const core::Scan& scan,
                                           const core::SkelRange& skelRange,
                                           InteractionFrame* pFrame) const {
  const core::OccupancyGrid& G = scan.getOccupancyGrid();
  const ml::BinaryGrid3 free = G.free();
  const ml::BinaryGrid3 occupied = G.occupied();
  const ml::BinaryGrid3 unknown = G.unknown();
  const geo::Transform world2grid = geo::from(G.worldToGrid());
  vec<geo::Vec3f> binCentroids;
  for (const Skeleton& skel : skelRange) {
    pFrame->reposition(skel);
    const geo::OBB& obb = pFrame->getWorldOBB();
    for (int i = 0; i < m_params.numOccupancySamplePoints; ++i) {
      const geo::Vec3f pWorld = obb.sample();
      const geo::Vec3f& p = world2grid * pWorld;
      const ml::vec3ul pVoxel(static_cast<size_t>(p[0]),
                              static_cast<size_t>(p[1]),
                              static_cast<size_t>(p[2]));
      if (free.isValidCoordinate(pVoxel)) {  // validity should be same for all grids
        if (free.isVoxelSet(pVoxel)) {
          pFrame->addPoint("FREE", pWorld);
        } else if (occupied.isVoxelSet(pVoxel)) {
          pFrame->addPoint("OCC", pWorld);
        } else if (unknown.isVoxelSet(pVoxel)) {
          pFrame->addPoint("UNK", pWorld);
        }
      }
    }
  }
}

void InteractionFrameFactory::addObjectLabels(const core::Scan& scan,
                                              const core::SkelRange& skelRange,
                                              InteractionFrame* pFrame) const {
  using namespace core::synth;
  const core::OccupancyGrid& occ = scan.getOccupancyGrid();
  core::LabeledGrid labeledGrid(occ.voxelSize(), occ.getDimensions(), occ.worldToGrid());
  LabelOpts labelOpts;
  labelOpts.labelStrategy = kLabelsAnnotation;
  labelOpts.labelType = kLabelTypePart;
  m_db.getLabeler().labelVoxels(scan, nullptr, nullptr, labelOpts, &labeledGrid);
  const geo::Transform world2grid = geo::from(occ.worldToGrid());
  vec<geo::Vec3f> binCentroids;
  for (const Skeleton& skel : skelRange) {
    pFrame->reposition(skel);
    const geo::OBB& obb = pFrame->getWorldOBB();
    for (int i = 0; i < m_params.numOccupancySamplePoints; ++i) {
      const geo::Vec3f pWorld = obb.sample();
      const geo::Vec3f& p = world2grid * pWorld;
      const ml::vec3i pVoxel(static_cast<int>(p[0]),
                             static_cast<int>(p[1]),
                             static_cast<int>(p[2]));
      const string label = labeledGrid.getLabel(pVoxel);
      if (label != core::CATEGORY_NONE) {
        pFrame->addPoint(label, pWorld);
      }
    }
  }
}

}  // namespace interaction
}  // namespace sg
