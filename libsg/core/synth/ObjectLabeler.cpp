#include "common.h"  // NOLINT

#include "core/synth/ObjectLabeler.h"

#include "core/Database.h"
#include "core/OccupancyGrid.h"
#include "core/Recording.h"
#include "core/synth/SkeletonPoser.h"
#include "geo/OBB.h"
#include "interaction/Interaction.h"
#include "interaction/InteractionGraph.h"
#include "interaction/InteractionSet.h"
#include "interaction/ProtoInteractionGraph.h"
#include "interaction/similarity.h"
#include "io/io.h"
#include "segmentation/MeshSegment.h"
#include "vis/HeatMap.h"

namespace sg {
namespace core {
namespace synth {

using interaction::InteractionGraph;
using interaction::InteractionLink;
using interaction::InteractionNode;
using interaction::ProtoInteractionGraph;
using interaction::ProtoInteractionLink;
using interaction::ProtoInteractionNode;
using interaction::kSkelParams;
using segmentation::ConstPartPtr;
using segmentation::VecConstPartPtr;
using segmentation::ConstSegPtr;
using segmentation::VecConstSegPtr;
using segmentation::SegmentGroups;

const string& getSegLabel(const SegmentGroups& segGroups, const ConstSegPtr segPtr) {
  SegmentGroups::SegmentGroupHandle segGroupId = segGroups.find(*segPtr);
  if (segGroupId >= 0) {
    // Found segment group!
    const auto segGroupPtr = segGroups.get(segGroupId);
    return segGroupPtr->label;
  } else {
    return CATEGORY_NONE;
  }
}

void ObjectLabeler::init(Database* pDatabase) {
  const util::Params& params = *pDatabase->params;
  const string& dataDir = params.get<string>("localDataDir");
  const string& modelCatMapFile = dataDir + params.get<string>("categoryMappingFile");

  m_pDatabase = pDatabase;
  m_categoryIndex.clear();

  // Parse modelCatMapFile
  cout << "[ObjectLabeler] CategoryMapFile: " << modelCatMapFile << " : ";
  const vec<string> lines = io::getLines(modelCatMapFile);
  int lineno = 0;
  for (const auto& line : lines) {
    ++lineno;
    if (!line.empty()) {
      const vec<string> tokens = util::tokenize(line, ",");
      if (tokens.size() == 2) {
        cout << tokens[0] << "->" << tokens[1] << ", ";
        m_annoToModelCatMap[tokens[0]] = tokens[1];
        m_modelToAnnoCatMap[tokens[1]] = tokens[0];
        m_categoryIndex.add(tokens[1]);
      } else {
        SG_LOG_ERROR << "Unexpected line " << lineno;
      }
    }
  }
  cout << endl;
}

string ObjectLabeler::getCategory(const string& label, bool makeUpperIfUnknown) const {
  vec<string> parts = ml::util::split(label, ":");
  if (m_annoToModelCatMap.count(parts[0]) > 0) {
    return m_annoToModelCatMap.at(parts[0]);
  } else {
    if (parts[0] != CATEGORY_ANY && parts[0] != CATEGORY_NONE) {
      SG_LOG_ERROR << "Unknown category " << parts[0];
      if (makeUpperIfUnknown) {
        std::locale loc;
        char ch = parts[0][0];
        parts[0][0] = std::toupper(ch, loc);
        if (parts[0] != "Wall" && parts[0] != "Floor") {
          return parts[0];
        }
      }
    }
    return CATEGORY_NONE;
  }
}

string ObjectLabeler::getAnnotationNoun(const string& cat) const {
  if (m_modelToAnnoCatMap.count(cat) > 0) {
    return m_modelToAnnoCatMap.at(cat);
  } else {
    return cat;
  }
}

string ObjectLabeler::getAnnotationNoun(const vec<string>& cats) const {
  for (const string& cat : cats) {
    if (m_modelToAnnoCatMap.count(cat) > 0) {
      return m_modelToAnnoCatMap.at(cat);
    } 
  }
  return CATEGORY_NONE;
}

ObjectLabeler::LabelToSegMapT ObjectLabeler::convertToCategories(const LabelToSegMapT& segsByLabel) const {
  LabelToSegMapT segsByCategory;
  for (const auto& it : segsByLabel) {
    const string category = getCategory(it.first);
    segsByCategory[category].insert(segsByCategory[category].end(), it.second.begin(), it.second.end());
  }
  return segsByCategory;
}

// Pick a likely object/part label for a joint to be interacting with
vec<pair<string, double>> ObjectLabeler::predictInteractingLabels(
  const ProtoInteractionGraph& pig,
   vec<stats::Counter<string,double>>* pLabelProbs) const {
  stats::Counter<string, double> localLabelProbs;
  int nJoints = Skeleton::kNumJointGroupsLR;
  vec<pair<string, double>> jointCats(nJoints);
  if (pLabelProbs != nullptr) {
    pLabelProbs->clear();
    pLabelProbs->resize(nJoints);
  }
  for (int iJoint = 0; iJoint < nJoints; ++iJoint) {
    //TODO: Get joint weight from input
    float jointWeight = 1.f;  /// static_cast<float>(nJoints);
    stats::Counter<string, double>& labelProbs =
      (pLabelProbs != nullptr)? (*pLabelProbs)[iJoint]:localLabelProbs;
    labelProbs.clear();
    const auto& pigJointNode = pig.getProtoJoint(iJoint);
    if (iJoint == kSkelParams.kGazeJoint) {
      const auto& pigGazes = pig.getGazeEdges();
      for (const auto pigGaze : pigGazes) {
        const string label = pigGaze.first;
        const ProtoInteractionGraph::edge pigEdge = pigGaze.second;
        const ProtoInteractionLink& pigLink = pig.getEdgeBundle(pigEdge);
        //const ProtoInteractionNode& pigSegNode = pig.getVertexBundle(pigEdge.m_target);
        const float labelPresence = static_cast<float>(pigLink.interactionCount) / pigJointNode.interactionCount;
        const float segSim = 1.0f; // No segment similarity
        const float labelWeight = segSim * labelPresence;

        labelProbs.inc(label, labelWeight * jointWeight);
      }
    } else {
      const auto& pigContacts = pig.getContactEdges(iJoint);
      for (const auto pigContact : pigContacts) {
        const string label = pigContact.first;
        const ProtoInteractionGraph::edge pigEdge = pigContact.second;
        const ProtoInteractionLink& pigLink = pig.getEdgeBundle(pigEdge);
        //const ProtoInteractionNode& pigSegNode = pig.getVertexBundle(pigEdge.m_target);
        const float labelPresence = static_cast<float>(pigLink.interactionCount) / pigJointNode.interactionCount;
        const float segSim = 1.0f; // No segment similarity
        const float labelWeight = segSim * labelPresence;

        labelProbs.inc(label, labelWeight * jointWeight);
      }
    }
    // Get maximum label for joint
    if (labelProbs.empty()) {
      jointCats[iJoint] = make_pair(CATEGORY_NONE, 0.0);
    } else {
      jointCats[iJoint] = labelProbs.maxPair();
    }
  }
  return jointCats;
}

void ObjectLabeler::dumpJointLabelProbs(const ProtoInteractionGraph& pig,
                                        const string& filename) const {
  const auto& partIndex = m_pDatabase->interactions.getPartIndex();
  util::Index<string> jointGroupLRIndex;
  vec<stats::Counter<string, double>> jointLabelProbs;
  predictInteractingLabels(pig, &jointLabelProbs);
  util::Grid<double> grid(static_cast<int>(partIndex.size()),
                          static_cast<int>(jointLabelProbs.size()));
  grid.clear(0.0);
  for (int i = 0; i < jointLabelProbs.size(); ++i) {
    jointGroupLRIndex.add(kSkelParams.kJointNames[i]);
    for (const auto& p : jointLabelProbs[i]) {
      int r = partIndex.indexOf(p.first);
      if (r >= 0) {
        grid(r,i) = p.second;
      }
    }    
  }
  saveGrid(filename, "part", partIndex, "joint", jointGroupLRIndex, grid);
}

void ObjectLabeler::dumpJointCategoryProbs(const ProtoInteractionGraph& pig,
                                           const string& filename) const {
  const auto& nounIndex = m_pDatabase->interactions.getNounIndex();
  util::Index<string> jointGroupLRIndex;
  vec<stats::Counter<string, double>> jointLabelProbs;
  predictInteractingLabels(pig, &jointLabelProbs);
  util::Grid<double> grid(static_cast<int>(nounIndex.size()),
                          static_cast<int>(jointLabelProbs.size()));
  grid.clear(0.0);
  for (int i = 0; i < jointLabelProbs.size(); ++i) {
    jointGroupLRIndex.add(kSkelParams.kJointNames[i]);
    for (const auto& p : jointLabelProbs[i]) {
      vec<string> fields = util::tokenize(p.first, ":");
      int r = nounIndex.indexOf(fields[0]);
      if (r >= 0 && p.second > grid(r,i)) {
        grid(r, i) = p.second;
      }
    }
  }
  saveGrid(filename, "noun", nounIndex, "joint", jointGroupLRIndex, grid);
}

ObjectLabeler::LabelToSegMapT ObjectLabeler::labelWithPIG(const Scan& scan,
                                                          const Skeleton& skel,
                                                          const Skeleton::SegmentsByJointPlusGaze& segsByJoint,
                                                          const ProtoInteractionGraph& pig,
                                                          SegLabelScoresT* pSegLabelScores) const {
  // Create a Interaction Graph for this skel
  // TODO: Use custom labeler that leaves segments not labeled?
  const std::shared_ptr<InteractionGraph> pSkelIG =
    m_pDatabase->interactionFactory.createInteractionGraph(scan, skel, segmentation::kPartSegment, segsByJoint);
  const InteractionGraph& skelIG = *pSkelIG;

  // Figure out what model categories to use for the active segments
  // Predict the likely categories by using the IG and matching it to the PIG
  umap<ConstSegPtr, pair<string, float>> segToLabel;
  const auto& segGraphVertices = skelIG.getParts();
  stats::Counter<string, float> labelProbs;
  for (const auto segIt : segGraphVertices) {
    labelProbs.clear();
    const ConstPartPtr partPtr = segIt.first;
    assert(partPtr->partType == segmentation::kPartSegment);
    const ConstSegPtr segPtr = std::static_pointer_cast<const MeshSegment>(partPtr);
    const auto v = segIt.second;
    const auto& segNode = skelIG.getVertexBundle(v);
    vec<int> joints;
    for (const auto e : skelIG.getInEdges(v)) {
      const auto& sourceNode = skelIG.getVertexBundle(e.m_source);
      if (sourceNode.jointId >= 0) {
        joints.push_back(sourceNode.jointId);
      }
    }
    size_t nJoints = joints.size();
    float jointWeight = 1.f / static_cast<float>(nJoints);
    for (const auto iJoint : joints) {
      //const auto& jointNode = skelIG.getJointVertexBundle(iJoint);
      const auto& pigJointNode = pig.getProtoJoint(iJoint);
      if (iJoint == kSkelParams.kGazeJoint) {
        const auto& pigGazes = pig.getGazeEdges();
        for (const auto pigGaze : pigGazes) {
          const string label = pigGaze.first;
          const ProtoInteractionGraph::edge pigEdge = pigGaze.second;
          const ProtoInteractionLink& pigLink = pig.getEdgeBundle(pigEdge);
          const ProtoInteractionNode& pigSegNode = pig.getVertexBundle(pigEdge.m_target);
          const float labelPresence = static_cast<float>(pigLink.interactionCount) / pigJointNode.interactionCount;
          const float segSim = interaction::similarity::simSegment(pigSegNode, segNode);
          const float labelWeight = segSim * labelPresence;

          // TODO: Improve label weight based on segment similarity and link similarity
          labelProbs.inc(label, labelWeight * jointWeight);
        }
      } else {
        const auto& pigContacts = pig.getContactEdges(iJoint);
        for (const auto pigContact : pigContacts) {
          const string label = pigContact.first;
          const ProtoInteractionGraph::edge pigEdge = pigContact.second;
          const ProtoInteractionLink& pigLink = pig.getEdgeBundle(pigEdge);
          const ProtoInteractionNode& pigSegNode = pig.getVertexBundle(pigEdge.m_target);
          const float labelPresence = static_cast<float>(pigLink.interactionCount) / pigJointNode.interactionCount;
          const float segSim = interaction::similarity::simSegment(pigSegNode, segNode);
          const float labelWeight = segSim * labelPresence;

          // TODO: Improve label weight based on segment similarity and link similarity
          labelProbs.inc(label, labelWeight * jointWeight);
        }
      }
    }
    // Save label scores for output if requested
    if (pSegLabelScores != nullptr) {
      (*pSegLabelScores)[segPtr] = labelProbs;
    }
    // Take the most likely label for this segment
    segToLabel[segPtr] = labelProbs.maxPair();
    SG_LOG_TRACE << "guess seg " << segPtr->id << " " << segToLabel[segPtr].first << " "
         << segToLabel[segPtr].second << ", actual " << getSegLabel(scan.segmentGroups, segPtr);
  }

  LabelToSegMapT segsByLabel;
  for (const auto it : segToLabel) {
    const string label = it.second.first;
    segsByLabel[label].push_back(it.first);
  }

  return segsByLabel;
}

ObjectLabeler::LabelToSegMapT ObjectLabeler::labelWithIG(const Scan& scan,
                                                         const Skeleton& skel,
                                                         const Skeleton::SegmentsByJointPlusGaze& segsByJoint,
                                                         const InteractionGraph& ig,
                                                         SegLabelScoresT* pSegLabelScores) const {
  // Create a Interaction Graph for this skel
  // TODO: Use custom labeler that leaves segments not labeled?
  const std::shared_ptr<InteractionGraph> pSkelIG =
    m_pDatabase->interactionFactory.createInteractionGraph(scan, skel, segmentation::kPartSegment, segsByJoint);
  const InteractionGraph& skelIG = *pSkelIG;

  // Figure out what model categories to use for the active segments
  // Predict the likely categories by matching to the IG
  umap<ConstSegPtr, pair<string, float>> segToLabel;
  const auto& segGraphVertices = skelIG.getParts();
  stats::Counter<string, float> labelProbs;
  for (const auto segIt : segGraphVertices) {
    labelProbs.clear();
    const ConstPartPtr partPtr = segIt.first;
    assert(partPtr->partType == segmentation::kPartSegment);
    const ConstSegPtr segPtr = std::static_pointer_cast<const MeshSegment>(partPtr);
    const auto v = segIt.second;
    const auto& segNode = skelIG.getVertexBundle(v);
    vec<int> joints;
    for (const auto e : skelIG.getInEdges(v)) {
      const auto& sourceNode = skelIG.getVertexBundle(e.m_source);
      if (sourceNode.jointId >= 0) {
        joints.push_back(sourceNode.jointId);
      }
    }
    size_t nJoints = joints.size();
    float jointWeight = 1.f / static_cast<float>(nJoints);
    for (const auto iJoint : joints) {
      //const auto& jointNode = skelIG.getJointVertexBundle(iJoint);
      //const auto& igJointNode = ig.getJointVertexBundle(iJoint);
      if (iJoint == kSkelParams.kGazeJoint) {
        const auto& igGazeEdges = ig.getGazeEdges();
        for (const auto igEdge : igGazeEdges) {
          //const InteractionLink& igLink = ig.getEdgeBundle(igEdge);
          const InteractionNode& igSegNode = ig.getVertexBundle(igEdge.m_target);
          const string label = igSegNode.label;
          const float segSim = interaction::similarity::simSegment(igSegNode, segNode);
          const float labelWeight = segSim;

          // TODO: Improve label weight based on segment similarity and link similarity
          labelProbs.inc(label, labelWeight * jointWeight);
        }
      } else {
        const auto& igContacts = ig.getContactEdges(iJoint);
        for (const auto igEdge : igContacts) {
          //const InteractionLink& igLink = ig.getEdgeBundle(igEdge);
          const InteractionNode& igSegNode = ig.getVertexBundle(igEdge.m_target);
          const string label = igSegNode.label;
          const float segSim = interaction::similarity::simSegment(igSegNode, segNode);
          const float labelWeight = segSim;

          // TODO: Improve label weight based on segment similarity and link similarity
          labelProbs.inc(label, labelWeight * jointWeight);
        }
      }
    }
    // Save label scores for output if requested
    if (pSegLabelScores != nullptr) {
      (*pSegLabelScores)[segPtr] = labelProbs;
    }
    // Take the most likely label for this segment
    segToLabel[segPtr] = labelProbs.maxPair();
    SG_LOG_TRACE << "guess seg " << segPtr->id << " " << segToLabel[segPtr].first << " "
         << segToLabel[segPtr].second << ", actual " << getSegLabel(scan.segmentGroups, segPtr);
  }

  LabelToSegMapT segsByLabel;
  for (const auto it : segToLabel) {
    const string label = it.second.first;
    segsByLabel[label].push_back(it.first);
  }

  return segsByLabel;
}

ObjectLabeler::LabelToSegMapT ObjectLabeler::labelWithPIG(const Scan& scan,
                                                          const SkelRange& skelRange,
                                                          const ProtoInteractionGraph& PIG) const {
  // Go over interactiions and get aggregatedSegLabelScores
  SegLabelScoresT segLabelScores;
  SegLabelScoresT aggregatedSegLabelScores;
  size_t nSkels = 0;
  for (const Skeleton& skel : skelRange) {
    const Skeleton::SegmentsByJointPlusGaze& segsByJoints = 
      m_pDatabase->interactionFactory.getActiveSegments(scan, SkelRange({skel}));
    segLabelScores.clear();
    labelWithPIG(scan, skel, segsByJoints, PIG, &segLabelScores);
    for (const auto it : segLabelScores) {
      aggregatedSegLabelScores[it.first].add(it.second);
    }
    nSkels++;
  }
  SG_LOG_INFO << "[ObjectLabeler] Processed nSkels=" << nSkels;

  // Pick the maximum for each segment (currently independently)
  LabelToSegMapT segsByLabel;
  for (const auto it : aggregatedSegLabelScores) {
    const auto& segPtr = it.first;
    const auto& bestPair = it.second.maxPair();
    const string& label = bestPair.first;
    float score = bestPair.second;
    segsByLabel[label].push_back(segPtr);
    SG_LOG_INFO << "guess seg " << segPtr->id << " " << label << " "
      << score << ", actual " << getSegLabel(scan.segmentGroups, segPtr);
  }

  return segsByLabel;
}

ObjectLabeler::LabelToSegMapT ObjectLabeler::labelFromAnnotation(const Scan& scan,
                                                                 const VecConstSegPtr segs) const {
  LabelToSegMapT segsByLabel;
  for (const auto segPtr : segs) {
    // Find segment group this belongs to
    SegmentGroups::SegmentGroupHandle segGroupId = scan.segmentGroups.find(*segPtr);
    if (segGroupId >= 0) {
      // Found segment group!
      // Let's check the annotation
      const auto segGroupPtr = scan.segmentGroups.get(segGroupId);
      segsByLabel[segGroupPtr->label].push_back(segPtr);
    } else {
      // Well, let's just skip this segment
      SG_LOG_WARN << "Skipping segment " << segPtr->id << " without segment group";
    }
  }
  return segsByLabel;
}

// Takes aggregated scores for each segment and does final assignment of labels
void ObjectLabeler::scoresToLabels(const Scan& scan, 
                                   const SegLabelScoresT& aggregatedSegLabelScores,
                                   SegmentGroups* pSegGroups) const {
  // Pick the maximum for each segment (currently independently)
  map<string, VecConstSegPtr> segsByLabel;
  for (const auto it : aggregatedSegLabelScores) {
    const auto& segPtr = it.first;
    const auto& bestPair = it.second.maxPair();
    const string& label = bestPair.first;
    float score = bestPair.second;
    segsByLabel[label].push_back(segPtr);
    SG_LOG_INFO << "guess seg " << segPtr->id << " " << label << " "
      << score << ", actual " << getSegLabel(scan.segmentGroups, segPtr);
  }

  // TODO: Create segment groups out of adjacent segments with the same label
  // Sort our segments by size
   
  // For now just have one segment group per segment
  for (const auto it : segsByLabel) {
    for (const auto segIt : it.second) {
      const MeshSegment& seg = *segIt.get();
      pSegGroups->add(seg, it.first);
    }
  }
}

void ObjectLabeler::label(const Scan& scan, const vec<Recording*> recordings, SegmentGroups* pSegGroups) const {
  // TODO: Consider interaction set type

  // Clear old segment groups
  pSegGroups->clear();

  // Go over interactions and get aggregatedSegLabelScores
  SegLabelScoresT segLabelScores;
  SegLabelScoresT aggregatedSegLabelScores;
  size_t nRecordings = 0;
  size_t nInteractions = 0;
  size_t nInteractionFrames = 0;
  for (Recording* pRec : recordings) {
    SG_LOG_INFO << "[ObjectLabeler] Processing recording " << pRec->id;
    for (interaction::Interaction* pInteraction : pRec->interactions) {
      if (pInteraction->interactionSet == nullptr) {
        throw std::runtime_error("No interaction set for interaction " +
                                 pInteraction->isetId + ": make sure the database was created!!!");
      }
      if (pInteraction->interactionSet->protoInteractionGraph == nullptr) {
        throw std::runtime_error("No PIG for interaction " + pInteraction->interactionSet->id +
                                 ": make sure the database was created!!!");
      }
      SG_LOG_INFO << "[ObjectLabeler] Processing interaction " << pInteraction->isetId <<
        ", interactionSet " << pInteraction->interactionSet->id;
      const ProtoInteractionGraph& PIG = *pInteraction->interactionSet->protoInteractionGraph;
      for (int i = 0; i < pInteraction->skelRange.size(); i++) {
        const Skeleton& skel = pInteraction->skelRange[i];
        const Skeleton::SegmentsByJointPlusGaze& segsByJoints
          = m_pDatabase->interactionFactory.getActiveSegments(scan, SkelRange({skel}));
        segLabelScores.clear();
        labelWithPIG(scan, skel, segsByJoints, PIG, &segLabelScores);

        for (const auto it : segLabelScores) {
          aggregatedSegLabelScores[it.first].add(it.second);
        }
        nInteractionFrames++;
      }
      nInteractions++;
    }
    nRecordings++;
  }
  SG_LOG_INFO << "[ObjectLabeler] " << "Processed nRec=" << nRecordings 
    << ", nInter=" << nInteractions << ", nInterFrames=" << nInteractionFrames;

  scoresToLabels(scan, aggregatedSegLabelScores, pSegGroups);
}

void ObjectLabeler::label(const Scan& scan, const Skeleton& skel,
                          const ProtoInteractionGraph& pig, SegmentGroups* pSegGroups) const {
  // TODO: Consider interaction set type

  // Clear old segment groups
  pSegGroups->clear();

  // Go over interactions and get aggregatedSegLabelScores
  SegLabelScoresT segLabelScores;
  SegLabelScoresT aggregatedSegLabelScores;
  const Skeleton::SegmentsByJointPlusGaze& segsByJoints
    = m_pDatabase->interactionFactory.getActiveSegments(scan, SkelRange({skel}));
  segLabelScores.clear();
  labelWithPIG(scan, skel, segsByJoints, pig, &segLabelScores);

  for (const auto it : segLabelScores) {
    aggregatedSegLabelScores[it.first].add(it.second);
  }

  scoresToLabels(scan, aggregatedSegLabelScores, pSegGroups);
}

void ObjectLabeler::label(const Scan& scan, const vec<TransformedSkeleton>& posedSkels,
                          const ProtoInteractionGraph& pig,
                          SegmentGroups* pSegGroups) const {
  // Clear old segment groups
  pSegGroups->clear();

  // Go over interactions and get aggregatedSegLabelScores
  SegLabelScoresT segLabelScores;
  SegLabelScoresT aggregatedSegLabelScores;
  size_t nInteractions = 0;
  for (const TransformedSkeleton tskel : posedSkels) {
    const Skeleton& skel = tskel.getSkeleton();
    const Skeleton::SegmentsByJointPlusGaze& segsByJoints
          = m_pDatabase->interactionFactory.getActiveSegments(scan, SkelRange({skel}));
    segLabelScores.clear();
    labelWithPIG(scan, skel, segsByJoints, pig, &segLabelScores);
    for (const auto it : segLabelScores) {
      aggregatedSegLabelScores[it.first].add(it.second);
    }
    nInteractions++;
  }
  SG_LOG_INFO << "[ObjectLabeler] " << "Processed nInter=" << nInteractions;

  scoresToLabels(scan, aggregatedSegLabelScores, pSegGroups);
}

void ObjectLabeler::labelVoxels(const Scan& scan,
                                const Skeleton* pSkel,
                                Recording* currRecording, 
                                const LabelOpts& labelOpts,
                                LabeledGrid* pLabeledGrid) const {
  switch (labelOpts.labelStrategy) {
    case kLabelsAnnotation: 
      labelVoxelsFromAnnotation(scan, scan.segmentGroups, labelOpts, pLabeledGrid);
      break;
    case kLabelsRecording: 
      if (currRecording != nullptr) {
        vec<Recording*> recordings;
        recordings.push_back(currRecording);
        labelVoxelsFromRecordings(scan, recordings, labelOpts, pLabeledGrid);
      } else {
        SG_LOG_WARN << "No current recording";
      }
     break;
    case kLabelsRecordingsAll:
      labelVoxelsFromRecordings(scan, m_pDatabase->recordings.getLoadedRecordingsForScan(scan.getCanonicalId()), 
                                labelOpts, pLabeledGrid);
      break;
    case kLabelsSkeleton: 
      if (pSkel != nullptr) {
        const auto& isets = m_pDatabase->interactions.getAllInteractionSets();
        if (isets.count(labelOpts.interactionSetId) > 0) {
          const interaction::InteractionSet& iset = isets.at(labelOpts.interactionSetId);
          if (iset.protoInteractionGraph) {
            labelVoxelsFromSkeleton(scan, *pSkel, *iset.protoInteractionGraph, labelOpts, pLabeledGrid);
          } else {
            SG_LOG_WARN << "No PIG for iset " << iset.id;
          }
        } else {
          SG_LOG_WARN << "Unknown iset " << labelOpts.interactionSetId;
        }
      } else {
        SG_LOG_WARN << "No skeleton";
      }
      break;
    case kLabelsPredict:
      predictInteractionsAndLabelVoxels(scan, labelOpts, pLabeledGrid);
      break;
    default:
      SG_LOG_WARN << "Unsupported labeling strategy " << labelOpts.labelStrategy;
  }
}

void ObjectLabeler::labelVoxels(const Scan& scan, const vec<ModelInstance>& modelInsts,
                                const LabelType labelType, LabeledGrid* pLabeledGrid) const
{
  using namespace geo;
  ml::SparseGrid3<stats::Counter<int>> grid;
  const OccupancyGrid& occupancyGrid = scan.getOccupancyGrid();
  Transform world2scene = from(occupancyGrid.worldToGrid());
  Transform scene2world = from(occupancyGrid.gridToWorld());
  world2scene.translation() += Vec3f(0.5f, 0.5f, 0.5f);  // for correct rounding of floating point voxel coords to grid
  const float sceneVoxelSize = occupancyGrid.voxelSize();
  ml::vec3ul gridDims = occupancyGrid.getDimensions();

  // Consider placed model instances and use them to label the voxels
  // TODO: Order model instances by size and label from largest to smallest
  string label;
  util::Index<string> labelIndex;
  int mi = 0;
  for (const ModelInstance& mInst : modelInsts) {
    mi++;
    // Label voxel using category
    switch (labelType)
    {
//      case kLabelTypePart:
//        pLabeledGrid->setLabel(v.first, segGroupPtr->label);
//        break;
      case kLabelTypeObjectId:
        label = std::to_string(mi);
        break;
      case kLabelTypeCategory:
      default:
        label = mInst.category;
    }    
    int iLabel = labelIndex.indexOf(label, true);

    const Transform xform = world2scene * mInst.getLocalToParent();
    const Eigen::Matrix3Xf voxelCenters = xform * mInst.model.voxelCenters;
    const int numModelVoxels = static_cast<int>(voxelCenters.cols());
    for (int i = 0; i < numModelVoxels; i++) {
      const Vec3f& v = voxelCenters.col(i);
      int x = static_cast<int>(v[0]);
      int y = static_cast<int>(v[1]);
      int z = static_cast<int>(v[2]);
      if (x >= 0 && x < gridDims.x && y >= 0 && y < gridDims.y && z >= 0 && z < gridDims.z) {
        grid(x,y,z).inc(iLabel);
      }
    }
  }

  pLabeledGrid->clear();
  const auto& labels = labelIndex.labels();
  for (const auto& p : grid)
  {
    pLabeledGrid->setLabel(p.first, labels.at(p.second.argMax()));
  }
}

//! Label voxels in scan using recordings
void ObjectLabeler::labelVoxelsFromRecordings(const Scan& scan, const vec<Recording*> recordings,
                                              const LabelOpts& labelOpts, LabeledGrid* pLabeledGrid) const {
  SegmentGroups segGroups;
  label(scan, recordings, &segGroups);
  labelVoxelsFromAnnotation(scan, segGroups, labelOpts, pLabeledGrid);
}

void ObjectLabeler::labelVoxelsFromSkeleton(const Scan& scan, const Skeleton& skel,
                                            const ProtoInteractionGraph& pig,
                                            const LabelOpts& labelOpts, LabeledGrid* pLabeledGrid) const {
  SegmentGroups segGroups;
  label(scan, skel, pig, &segGroups);
  labelVoxelsFromAnnotation(scan, segGroups, labelOpts, pLabeledGrid);
}

void ObjectLabeler::predictInteractionsAndLabelVoxels(const Scan& scan,
                                                      const LabelOpts& labelOpts, LabeledGrid* pLabeledGrid) const {
  SegmentGroups segGroups;
  if (!labelOpts.interactionSetId.empty()) {
    // Interaction set is specified
    const sg::interaction::InteractionSet* pISet = m_pDatabase->interactions.getInteractionSet(labelOpts.interactionSetId);
    if (pISet) {
      if (pISet->protoInteractionGraph) {
        sg::interaction::VerbNounISetGroup isetGroup;
        isetGroup.init(pISet);
        m_pDatabase->interactions.getRelatedInteractionSets(&isetGroup);

        SkeletonPoserParams poserParams(&scan);
        if (labelOpts.pPoserParams) {
          poserParams = *labelOpts.pPoserParams;
        } else {
          SG_LOG_WARN << "No poser params, using default poser params to predict likely locations for action";
          poserParams.init(*m_pDatabase->params);
        }
        poserParams.pScan = &scan;
        poserParams.pISetGroup = &isetGroup;

        // Compute heatmap
        SkeletonPoser poser;
        poser.init(m_pDatabase);
        vec<TransformedSkeleton> predictedPoses;
        sg::vis::PoseHeatMap heatMap;
        poser.poseForScan(poserParams, &predictedPoses, &heatMap);
        label(scan, predictedPoses, *pISet->protoInteractionGraph, &segGroups);
      } else {
        // Sample points
        // For every sampled point compute most likely action and pose
        SG_LOG_WARN << "No PIG for " << labelOpts.interactionSetId;
      }
    } else {
      SG_LOG_WARN << "Unknown interaction set " << labelOpts.interactionSetId;
    }
  } else {
    // TODO: Sample positions and predict likely interaction at each location
    SG_LOG_WARN << "Prediction of likely interactions not yet supported.  Please provide interactionSetId";
  }
  labelVoxelsFromAnnotation(scan, segGroups, labelOpts, pLabeledGrid);
}

void ObjectLabeler::labelVoxelsBySegmentGroups(const Scan& scan, 
                                               const SegmentGroups& segGroups,
                                               const bool includeUnknown, 
                                               ml::SparseGrid3<int>* pGrid) const {
  using namespace geo;
  ml::SparseGrid3<int>& grid = *pGrid;
  const OccupancyGrid& occupancyGrid = scan.getOccupancyGrid();
  Transform world2scene = from(occupancyGrid.worldToGrid());
  Transform scene2world = from(occupancyGrid.gridToWorld());
  world2scene.translation() += Vec3f(0.5f, 0.5f, 0.5f);  // for correct rounding of floating point voxel coords to grid
  const float sceneVoxelSize = occupancyGrid.voxelSize();

  // Get voxels
  const ml::BinaryGrid3& sVoxels = (includeUnknown) ? occupancyGrid.unknownOrOccupied() : occupancyGrid.occupied();

  // Let's sort scan segments from largest to smallest
  vec<size_t> sortedSegGroupIndices = segGroups.getSortedIndices();

  for (size_t i : sortedSegGroupIndices) {
    int segGroupIdx = static_cast<int>(i);
    const auto segGroupPtr = segGroups.at(segGroupIdx);
    // Skip unlabeled
    string category = getCategory(segGroupPtr->label);
    if (category == CATEGORY_NONE || category == CATEGORY_ANY) {      
      continue;
    }

    // Figure out bounding box of segGroup in scan voxel space
    // Transform segs bbox into scan grid space
    const auto segGroupOBB = segGroupPtr->obb();
    // Create slightly larger obb using sceneVoxel size
    Vec3f r = segGroupOBB->axesLengths().array() + sceneVoxelSize;
    OBB obb(segGroupOBB->centroid(), r, segGroupOBB->normalizedAxes());

    BBox segGroupOBBInScene = transformBBox(world2scene, segGroupOBB->toAABB());

    const Vec3f minf = segGroupOBBInScene.min().array() - 0.5f;
    const Vec3f maxf = segGroupOBBInScene.max().array() + 0.5f;
    const Vec3i min = minf.cast<int>();
    const Vec3i max = maxf.cast<int>();

    // Label voxels, only marking voxels as belonging to this segment group if in obb
    for (int x = min[0]; x <= max[0]; ++x) {
      for (int y = min[1]; y <= max[1]; ++y) {
        for (int z = min[2]; z <= max[2]; ++z) {
          if (sVoxels.isValidCoordinate(x, y, z) && sVoxels.isVoxelSet(x, y, z)) {
            const ml::vec3i v(x, y, z);
            Vec3f centroid = scene2world * vec3f(ml::vec3f(v) + 0.5);
            if (obb.contains(centroid)) {
              grid[v] = segGroupIdx;
            }
          }
        }
      }
    }
  }
}

void ObjectLabeler::labelVoxelsFromAnnotation(const Scan& scan, 
                                              const SegmentGroups& segGroups,
                                              const LabelOpts& labelOpts, 
                                              LabeledGrid* pLabeledGrid) const {
  ml::SparseGrid3<int> segGroupGrid;
  labelVoxelsBySegmentGroups(scan, segGroups, labelOpts.includeUnknownOccupancy, &segGroupGrid);
  // Clear old labels
  pLabeledGrid->clear();
  // For now, just iterate over our segment group bounding boxes and label any voxels inside
  // as have the annotated label
  for (const auto& v : segGroupGrid) {
    int segGroupIdx = v.second;
    const auto segGroupPtr = segGroups.at(segGroupIdx);
    string category = getCategory(segGroupPtr->label);
    if (category == CATEGORY_NONE || category == CATEGORY_ANY) {
      continue;
    }

    // Label voxel using category
    switch (labelOpts.labelType)
    {
      case kLabelTypePart:
        pLabeledGrid->setLabel(v.first, segGroupPtr->label);
        break;
      case kLabelTypeObjectId:
        pLabeledGrid->setLabel(v.first, std::to_string(segGroupPtr->objectId));
        break;
      case kLabelTypeCategory:
      default:
        pLabeledGrid->setLabel(v.first, category);
        break;
    }
  }

  if (labelOpts.includeUnlabeled) {
    addUnlabeledOccupied(scan, pLabeledGrid);
  }
}

void ObjectLabeler::addUnlabeledOccupied(const Scan& scan, LabeledGrid* pLabeledGrid) const {
  const ml::BinaryGrid3& occupied = scan.getOccupancyGrid().occupied();
  for (int x = 0; x < occupied.getDimX(); ++x) {
    for (int y = 0; y < occupied.getDimY(); ++y) {
      for (int z = 0; z < occupied.getDimZ(); ++z) {
        const ml::vec3i v(x, y, z);
        if (!pLabeledGrid->contains(v) && occupied.isVoxelSet(x,y,z)) {
          pLabeledGrid->setLabel(v, CATEGORY_UNKNOWN);
        }
      }
    }
  }
}

ostream& operator<<(ostream& os, const LabelOpts& opts) {
  os << "{"
    << " includeUnknownOccupancy: " << opts.includeUnknownOccupancy << ", "
    << " includeUnlabeled: " << opts.includeUnlabeled << ", "
    << " labelType: " << opts.labelType << ", "
    << " labelStrategy: " << opts.labelStrategy << ", "
    << " interactionSetType: " << opts.interactionSetType
    << " interactionSetId: " << opts.interactionSetId << " }";
  return os;
}

}  // namespace synth
}  // namespace core
}  // namespace sg
