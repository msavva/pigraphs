#include "common.h"  // NOLINT

#include "interaction/InteractionDatabase.h"

#include <ext-boost/serialization.h>
#pragma warning(disable:4267)
// ReSharper disable once CppUnusedIncludeDirective
#include <boost/graph/adj_list_serialize.hpp>
#pragma warning(default:4267)
// pragmas to disable type conversion warning spewed by adj_list_serialize header

#include "core/Database.h"
#include "core/Recording.h"
#include "core/RecordingDatabase.h"
#include "core/ScanDatabase.h"
#include "interaction/InteractionFactory.h"
#include "interaction/InteractionGraph.h"
#include "interaction/ProtoInteractionGraph.h"
#include "interaction/InteractionFrameFactory.h"
#include "interaction/InteractFrameSurfSampled.h"
#include "io/io.h"
#include "segmentation/SegmentGroup.h"
#include "util/timer.h"
#include "util/util.h"
// ReSharper disable once CppUnusedIncludeDirective
#include "util/eigen_boost_serialization.h"


namespace sg {
namespace interaction {

void InteractionDatabase::init(const core::Database& db, bool computeIGs) {
  m_pDb = &db;
  generateInteractionSets(db.recordings);
  createIndices(&db.scans);
  computeStats();
  // Let's save out our stats while we are at it
  const string& outputDir = db.params->get<string>("workDir") + "/pigs/stats/";
  saveStats(outputDir);
  populateInteractionSetWeights();
  if (computeIGs) {
    computeInteractionGraphs();
  }
  if (m_params.get<bool>("Dataset.computeInteractionFrames")) {
    computeInteractionFrames();
  }
}

// Helper functions for add interaction sets
void addCompositeInteraction(map<string, InteractionSet>& allInteractionSets,
                             InteractionSetTypeToMap& interactionSetsByType,
                             const InteractionSetType type,
                             Interaction* pInteraction) {
  const string& id = "c/" + pInteraction->isetId;
  if (allInteractionSets.count(id) == 0) {
    InteractionSet& vSet = allInteractionSets[id];
    interactionSetsByType[ISetType_All].insert({id, vSet});
    interactionSetsByType[type].insert({id, vSet});
    vSet.id = id;
    vSet.type = type;
    for (const auto& vn : pInteraction->verbNouns) {
      vSet.verbNouns.insert(vn);
      vSet.verbs.insert(vn.verb);
      vSet.nouns.insert(vn.noun);
    }
  }
  allInteractionSets[id].interactions.push_back(pInteraction);
  pInteraction->interactionSet = &allInteractionSets[id];
  pInteraction->interactionSets.push_back(&allInteractionSets[id]);
}

void addVerbNounInteraction(map<string, InteractionSet>&  allInteractionSets,
                            InteractionSetTypeToMap& interactionSetsByType,
                            const InteractionSetType type,
                            Interaction* pInteraction) {
  for (const auto& vn : pInteraction->verbNouns) {
    const string& id = InteractionSet::getVerbNounISetId(vn);
    if (allInteractionSets.count(id) == 0) {
      InteractionSet& vSet = allInteractionSets[id];
      interactionSetsByType[ISetType_All].insert({id, vSet});
      interactionSetsByType[type].insert({id, vSet});
      vSet.id = id;
      vSet.type = type;
      vSet.verbNouns.insert(vn);
      vSet.verbs.insert(vn.verb);
      vSet.nouns.insert(vn.noun);
    }
    allInteractionSets[id].interactions.push_back(pInteraction);
    pInteraction->interactionSets.push_back(&allInteractionSets[id]);
  }
}

void addVerbInteraction(map<string, InteractionSet>&  allInteractionSets,
                        InteractionSetTypeToMap& interactionSetsByType,
                        const InteractionSetType type,
                        Interaction* pInteraction) {
  for (const auto& vn : pInteraction->verbNouns) {
    const string& id = InteractionSet::getVerbISetId(vn.verb);
    if (allInteractionSets.count(id) == 0) {
      InteractionSet& vSet = allInteractionSets[id];
      interactionSetsByType[ISetType_All].insert({id, vSet});
      interactionSetsByType[type].insert({id, vSet});
      vSet.id = id;
      vSet.type = type;
      vSet.verbs.insert(vn.verb);
    }
    // Same verb can have different nouns...
    InteractionSet& vSet = allInteractionSets[id];
    vSet.verbNouns.insert(vn);
    vSet.nouns.insert(vn.noun);
    vSet.interactions.push_back(pInteraction);
    pInteraction->interactionSets.push_back(&allInteractionSets[id]);
  }
}

void InteractionDatabase::generateInteractionSets(const core::RecordingDatabase& recordings) {
  // Clear existing InteractionSets
  m_allInteractionSets.clear();
  for (int i = 0; i < InteractionSet::kNumInteractionSetTypes; ++i) {
    m_interactionSetsByType[i].clear();
  }

  const vec<string> recIds = recordings.getLoadedRecordingIds();
  for (const string& recId : recIds) {
    const core::Recording& rec = recordings.getRecording(recId, true, false);
    for (Interaction* i : rec.interactions) {
      m_interactions[i->id] = util::ptr_to_shared<Interaction,Interaction>(i);
      addCompositeInteraction(m_allInteractionSets, m_interactionSetsByType, ISetType_Composite, i);
      addVerbNounInteraction(m_allInteractionSets, m_interactionSetsByType, ISetType_VerbNoun, i);
      addVerbInteraction(m_allInteractionSets, m_interactionSetsByType, ISetType_Verb, i);
    }
  }
}

void InteractionDatabase::registerAndPopulateISets(Interaction* pInteraction) {
  if (m_interactions.count(pInteraction->id)) {
    // We know about this interaction, populate it from our existing interaction
    std::shared_ptr<Interaction> pRegistered = m_interactions.at(pInteraction->id);
    pInteraction->interactionSet = pRegistered->interactionSet;
    pInteraction->interactionSets = pRegistered->interactionSets;
    // Also take the opportunity to populate recording/scan of our registered interaction
    if (pRegistered->recording == nullptr) {
      pRegistered->recording = pInteraction->recording;
      pRegistered->skelRange = pInteraction->skelRange;
    }
    if (pRegistered->scan == nullptr) {
      pRegistered->scan = pInteraction->scan;
    }
  } else {
    // Not registered, let's register it
    m_interactions[pInteraction->id] = util::ptr_to_shared<Interaction,Interaction>(pInteraction);
    addCompositeInteraction(m_allInteractionSets, m_interactionSetsByType, ISetType_Composite, pInteraction);
    addVerbNounInteraction(m_allInteractionSets, m_interactionSetsByType, ISetType_VerbNoun, pInteraction);
    addVerbInteraction(m_allInteractionSets, m_interactionSetsByType, ISetType_Verb, pInteraction);
  }  
}

void InteractionDatabase::populateInteractionSetWeights() {
  for (auto& it : m_allInteractionSets) {
    InteractionSet& iSet = it.second;
    iSet.computeJointActivationProbabilities();
    if (iSet.type == ISetType_Verb) {
      int iVerb = m_verbIndex.indexOf(*iSet.verbs.begin());
      for (int i = 0; i < Skeleton::kNumJointGroups; i++) {
        iSet.jointGroupWeights[i] = m_verbJointGroupMatchNounStats.condProb12(iVerb, i);
      }
    } else {
      iSet.jointGroupWeights.fill(0.0);
      double w = 1.0/iSet.verbNouns.size();
      for (const VerbNoun& verbNoun : iSet.verbNouns) {
        int iVerbNoun = m_verbNounIndex.indexOf(verbNoun.toString());
        for (int i = 0; i < Skeleton::kNumJointGroups; i++) {
          iSet.jointGroupWeights[i] += w*m_verbNounJointGroupMatchNounStats.condProb12(iVerbNoun, i);
        }
      }
    }
  }
}

void InteractionDatabase::getRelatedInteractionSets(VerbNounISetGroup* pISetGroup) const {
  // Find a set of verb interactionSet and verbNoun interactionSets
  pISetGroup->clearISets();
  pISetGroup->id = VerbNoun::getVerbNounSetId(pISetGroup->verbNouns);
  const auto& verbISetsMap = getInteractionSets(ISetType_Verb);
  const auto& verbNounISetsMap = getInteractionSets(ISetType_VerbNoun);
  const auto& compositeISetsMap = getInteractionSets(ISetType_Composite);
  for (const VerbNoun& vn : pISetGroup->verbNouns) {
    const string& vId = InteractionSet::getVerbISetId(vn.verb);
    if (verbISetsMap.count(vId) > 0) {
      pISetGroup->vISets.emplace_back(verbISetsMap.at(vId));
    }
    const string& vnId = InteractionSet::getVerbNounISetId(vn);
    if (verbNounISetsMap.count(vnId) > 0) {
      pISetGroup->vnISets.emplace_back(verbNounISetsMap.at(vnId));
    }
  }
  const string& cId = InteractionSet::getComponentISetId(pISetGroup->verbNouns);
  if (compositeISetsMap.count(cId) > 0) {
    pISetGroup->cISets.emplace_back(compositeISetsMap.at(cId));
    pISetGroup->pInteractionSet = &compositeISetsMap.at(cId).get();
  }
}

void InteractionDatabase::selectPigs(const PartType& partType, const string& aggrType) {
  for (auto& p : m_allInteractionSets) {
    p.second.protoInteractionGraph = p.second.pigs[partType][aggrType];    
  }
}

void InteractionDatabase::saveInteractionSummary(const string& csvFile) const {
  using util::join;
  io::ensurePathToFileExists(csvFile);
  ofstream ofs(csvFile);
  ofs << "type,id,verbs,nouns,count" << endl;
  for (int iType = 0; iType < InteractionSet::kNumInteractionSetTypes; ++iType) {
    if (iType != ISetType_All) {
      const string& typeName = InteractionSet::kInteractionSetNames[iType];
      for (const auto& interactionSetPair : m_interactionSetsByType[iType]) {
        const InteractionSet& is = interactionSetPair.second;
        ofs << typeName << "," << is.id << "," << join(is.verbs, "-")
            << "," << join(is.nouns, "-") << "," << is.interactions.size() << endl;
      }
    }
  }
  ofs.close();
}

void InteractionDatabase::saveInteractionRecordSummary(const string& csvFile) const {
  using util::join;
  io::ensurePathToFileExists(csvFile);
  ofstream ofs(csvFile);
  ofs << "type,id,verbs,nouns,rec,scan,start,end" << endl;
  for (int iType = 0; iType < InteractionSet::kNumInteractionSetTypes; ++iType) {
    if (iType != ISetType_All) {
      const string& typeName = InteractionSet::kInteractionSetNames[iType];
      for (const auto& iteractionSetPair : m_interactionSetsByType[iType]) {
        const InteractionSet& is = iteractionSetPair.second;
        for (const auto& it : is.interactions) {
          ofs << typeName << "," << is.id << "," << join(is.verbs, "-")
              << "," << join(is.nouns, "-") << ","
              << it->recId << "," << it->scanId << ","
              << it->startTime << "," << it->endTime << endl;
        }
      }
    }
  }
  ofs.close();
}

string InteractionDatabase::getInteractionSummaryFile() const {
  return m_params.get<string>("dataDir") + m_params.get<string>("interactionRecordsSummaryFile");
}

//// Functions for generating interaction graphs and interaction frams

using std::shared_ptr; using std::make_shared;

void InteractionDatabase::computeInteractionGraphs() {
  SG_LOG_INFO << "Computing InteractionGraphs...";
  util::Timer timer("computeInteractionGraphs");
  const string workDir = m_params.get<string>("workDir");
  // Get main PIG aggr strategy
  const string partTypeStr = m_params.get<string>("Interaction.partType"); 
  PartType defaultPartType = segmentation::getPartTypeFromString(partTypeStr); 
  const string pigAggrStrategyStr = m_params.get<string>("Interaction.PIGAggregatorStrategy");
  AggregationStrategy defaultPigAggrStrategy;
  vec<ProtoInteractionGraphAggregator> pigAggs;
  // Save PIGs for all aggr strategies
  for (int i = 0; i < kAggregationStrategyCount; ++i) {
    AggregationStrategy s = static_cast<AggregationStrategy>(i);
    if (kAggregationStrategyIds[i] == pigAggrStrategyStr) {
      defaultPigAggrStrategy = s;
    }
    pigAggs.emplace_back(s);
  }
  // Go over all interaction sets and create PIGs!
  for (auto& it : m_allInteractionSets) {
    InteractionSet& vSet = it.second;
    vSet.interactionGraphs.clear();
    vSet.interactionGraphs.resize(segmentation::kPartTypeCount);
    vSet.pigs.clear();
    vSet.pigs.resize(segmentation::kPartTypeCount);
    for (int iPartType = 0; iPartType < segmentation::kPartTypeCount; ++iPartType) {
      PartType partType = static_cast<PartType>(iPartType);
      const string partTypeName = segmentation::kPartTypeNames[partType];

      generateInteractionGraphs(vSet, partType, &vSet.interactionGraphs[partType]);

      for (const ProtoInteractionGraphAggregator& pigAgg : pigAggs) {
        shared_ptr<PIG> pPIG = make_shared<ProtoInteractionGraph>(vSet.id);
        const string& aggName = pigAgg.getAggregationStrategyName();
        vSet.pigs[partType][aggName] = pPIG;
        // Set main proto interaction graph here
        if (partType == defaultPartType && defaultPigAggrStrategy == pigAgg.getAggregationStrategy()) {
          vSet.protoInteractionGraph = pPIG;
        }
        for (const auto pIG : vSet.interactionGraphs[partType]) {
          pigAgg.addInteractionGraph(*pIG, pPIG.get());
        }
        pPIG->getGraphBundle().partType = partType;
        pPIG->getGraphBundle().pigType = aggName;

        // Output graphml and feats csv
        const string gdir = workDir + "/pigs/graphs/" + partTypeName + "/" + aggName, gfile = gdir + "/" + vSet.id + ".graphml";
        pPIG->writeGraphML(gfile);

        const string fdir = workDir + "/pigs/feats/" + partTypeName + "/" +  aggName, ffile = fdir + "/" + vSet.id + ".pig.csv";
        pPIG->dumpFeats(ffile);

        const auto& labeler = m_pDb->getLabeler();
        const string& jcdir = workDir + "/pigs/jc/" + partTypeName + "/" + aggName;
        const string& jcfile =  jcdir + "/" + vSet.id + ".pigjc.csv";
        labeler.dumpJointCategoryProbs(*pPIG, jcfile);
        const string& jlfile = jcdir + "/" + vSet.id + ".pigjl.csv";
        labeler.dumpJointLabelProbs(*pPIG, jlfile);
      }
    }
  }
}

void InteractionDatabase::computeInteractionFrames() {
  SG_LOG_INFO << "Computing InteractionFrames...";
  util::Timer timer("computeInteractionFrames");

  const InteractionFrameFactoryParams iffp(m_params);
  const InteractionFrameFactory iff(iffp, *m_pDb);
  for (auto& it : m_allInteractionSets) {
    util::Timer iftimer("computeInteractionFrames for " + it.first);
    InteractionSet& iSet = it.second;
    auto pFrame = iff.createInteractionFrames(iSet.interactions);
    iSet.protoInteractionFrame = pFrame;
  }
}

void InteractionDatabase::generateInteractionGraphs(const InteractionSet& vSet,
                                                    const PartType partType,
                                                    vec<shared_ptr<InteractionGraph>>* igs) const {
  igs->clear();
  for (const Interaction* pIn : vSet.interactions) {
    for (int i = 0; i < pIn->skelRange.size(); i++) {
      igs->push_back(m_interactionFactory.createInteractionGraph(
        *pIn->scan, pIn->skelRange[i], partType, pIn->jointSegments[i], vSet.verbNouns));
    }
  }
}

//// Functions for computing interaction statistics
void InteractionDatabase::createIndices(const core::ScanDatabase* pScans) {
  if (pScans != nullptr) {
    // Noun and Part indices
    m_nounIndex.clear();
    m_partIndex.clear();
    pScans->getAnnotationLabels(&m_partIndex);
    vec<string> fields;
    for (const string& label : m_partIndex.labels()) {
      util::tokenize(label, ":", &fields);
      m_nounIndex.add(fields[0]);
    }
    populateNounToParts();
    m_sceneTypeIndex.clear();
    for (const string& scanId : pScans->scanIds()) {
      const auto& scan = pScans->getScan(scanId);
      m_sceneTypeIndex.add(scan.sceneType);
    }
  }

  // Verb Index
  m_verbIndex.clear();
  const auto& verbInteractionSets = getInteractionSets(ISetType_Verb);
  for (const auto& it : verbInteractionSets) {
    const InteractionSet& interactionSet = it.second;
    for (const string& verb : interactionSet.verbs) {
      m_verbIndex.add(verb);
    }
  }

  // VerbNoun Index
  m_verbNounIndex.clear();
  const auto& verbNounInteractionSets = getInteractionSets(ISetType_VerbNoun);
  for (const auto& it : verbNounInteractionSets) {
    const InteractionSet& interactionSet = it.second;
    for (const VerbNoun& verbNoun : interactionSet.verbNouns) {
      m_verbNounIndex.add(verbNoun.toString());
    }
  }

  // Joint Group Index
  m_jointGroupIndex.clear();
  for (int i = 0; i < Skeleton::kNumJointGroups; i++) {
    m_jointGroupIndex.add(Skeleton::kJointGroupNames[i]);
  }
}

void InteractionDatabase::computeStats() {
  SG_LOG_INFO << "Computing InteractionDatabase stats...";
  util::Timer timer("InteractionDatabase stats");
  computeVerbStats();
  computeVerbNounStats();
  computeVerbJointStats();
  computeNounJointStats();
  computeVerbJointMatchNounStats();
  computeVerbNounJointMatchNounStats();
  computeSceneTypeStats();
}

void InteractionDatabase::computeSceneTypeStats() {
  m_nounSceneTypeStats.init(&m_nounIndex, &m_sceneTypeIndex);
  vec<string> fields;
  for (const string& scanId : m_pDb->scans.scanIds()) {
    const auto& scan = m_pDb->scans.getScan(scanId);
    int iSceneType = m_sceneTypeIndex.indexOf(scan.sceneType);
    set<int> iNouns;
    for (const auto& segGrp : scan.segmentGroups) {
      util::tokenize(segGrp.label, ":", &fields);
      const string& noun = fields[0];
      int iNoun = m_nounIndex.indexOf(noun);
      iNouns.insert(iNoun);
    }
    for (int iNoun : iNouns) {
      m_nounSceneTypeStats.rawCounts1[iNoun] += 1;
      m_nounSceneTypeStats.rawCooccurrenceCounts(iNoun, iSceneType) += 1;
    }
    m_nounSceneTypeStats.rawCounts2[iSceneType] += 1;
    m_nounSceneTypeStats.totalRawCounts += 1;
  }
  m_nounSceneTypeStats.smoothAndComputeStats(getLaplaceSmoothingParam());
}


void InteractionDatabase::computeVerbStats() {
  m_verbStats.init(&m_verbIndex);

  for (const auto& it : getInteractionSets()) {
    const InteractionSet& interactionSet = it.second;
    // Count either interactionSet size or total number of skeletons
//    int c = interactionSet.interactions.size();
    int c = 0;
    for (const auto interaction : interactionSet.interactions) {
      c += static_cast<int>(interaction->skelRange.size());
    }

    vec<int> iSetVerbIndices;
    for (const string& verb : interactionSet.verbs) {
      int iVerb = m_verbIndex.indexOf(verb);
      iSetVerbIndices.push_back(iVerb);
      m_verbStats.rawCounts[iVerb] += c;
    }
    m_verbStats.totalRawCounts += c;

    for (int i = 0; i < iSetVerbIndices.size(); i++) {
      for (int j = i + 1; j < iSetVerbIndices.size(); j++) {
        // TODO: ordering of i,j?
        m_verbStats.rawCooccurrenceCounts(iSetVerbIndices[i], iSetVerbIndices[j]) += c;
        m_verbStats.rawCooccurrenceCounts(iSetVerbIndices[j], iSetVerbIndices[i]) += c;
      }
    }
  }
  m_verbStats.smoothAndComputeStats(getLaplaceSmoothingParam());
}

void InteractionDatabase::computeVerbNounStats() {
  m_verbNounStats.init(&m_verbIndex, &m_nounIndex);

  for (const auto& it : getInteractionSets()) {
    const InteractionSet& interactionSet = it.second;
    // Count either interactionSet size or total number of skeletons
    //    int c = interactionSet.interactions.size();
    int c = 0;
    for (const auto interaction : interactionSet.interactions) {
      c += static_cast<int>(interaction->skelRange.size());
    }

    for (const VerbNoun& vn : interactionSet.verbNouns) {
      int iVerb = m_verbIndex.indexOf(vn.verb);
      int iNoun = m_nounIndex.indexOf(vn.noun);
      m_verbNounStats.rawCounts1[iVerb] += c;
      m_verbNounStats.rawCounts2[iNoun] += c;
      m_verbNounStats.rawCooccurrenceCounts(iVerb, iNoun) += c;
    }
    m_verbNounStats.totalRawCounts += c;
  }
  m_verbNounStats.smoothAndComputeStats(getLaplaceSmoothingParam());
}

typedef std::function<vec<int>(const InteractionSet&)> ISetToIndicesFn;
typedef std::function<map<string, vec<int>>(const InteractionSet&)> ISetToIndicesMapFn;
void computeISetJointStats(InteractionDatabase& interactions,
                           ISetToIndicesFn extractIndices,
                           const util::Index<string>& vIndex,
                           const util::Index<string>& jointGroupIndex,
                           stats::GridStats2DT<string, string>* pStats) {
  auto& stats = *pStats;
  stats.init(&vIndex, &jointGroupIndex);

  arr<bool, Skeleton::kNumJointGroups> jointGroupSeen;
  for (const auto& it : interactions.getInteractionSets()) {
    const InteractionSet& interactionSet = it.second;
    // Figure out what verbs this action goes with
    vec<int> indices = extractIndices(interactionSet);
    for (Interaction* interaction : interactionSet.interactions) {
      // Go through skeletons
      for (size_t iSkel = 0; iSkel < interaction->jointSegments.size(); iSkel++) {
        // Accumulate count for the verb
        for (int iV : indices) {
          stats.rawCounts1[iV]++;
        }
        stats.totalRawCounts++;

        // Accumulate count for the joint group
        const auto& jointSegments = interaction->jointSegments[iSkel];
        jointGroupSeen.fill(false);

        for (size_t iJoint = 0; iJoint < Skeleton::kNumJoints + 1; iJoint++) {
          if (!jointSegments[iJoint].empty()) {
            // Check if this joint group has been accounted for yet
            const int iJointGroup = static_cast<int>(Skeleton::kJointToJointGroup[iJoint]);
            if (!jointGroupSeen[iJointGroup]) {
              jointGroupSeen[iJointGroup] = true;
              stats.rawCounts2[iJointGroup]++;

              // Update for verbs
              for (int iV : indices) {
                stats.rawCooccurrenceCounts(iV, iJointGroup)++;
              }
            }
          }
        }
      }
    }
  }
  stats.smoothAndComputeStats(interactions.getLaplaceSmoothingParam());
}

void InteractionDatabase::computeVerbJointStats() {
  ISetToIndicesFn extractVerbIndices = [this](const InteractionSet & iSet) {
    // Figure out what verbs this action goes with
    vec<int> verbIndices;
    for (const string& verb : iSet.verbs) {
      verbIndices.push_back(m_verbIndex.indexOf(verb));
    }
    return verbIndices;
  };
  computeISetJointStats(*this,
                        extractVerbIndices,
                        m_verbIndex, m_jointGroupIndex,
                        &m_verbJointGroupStats);
}

void computeISetJointMatchNounStats(InteractionDatabase& interactions,
                                    ISetToIndicesFn extractIndices,
                                    ISetToIndicesMapFn extractIndicesMap,
                                    const util::Index<string>& vIndex,
                                    const util::Index<string>& jointGroupIndex,
                                    stats::GridStats2DT<string, string>* pStats) {
  auto& stats = *pStats;
  stats.init(&vIndex, &jointGroupIndex);

  map<segmentation::SegPtr, string> cachedSegmentObjLabels;
  arr<bool, Skeleton::kNumJointGroups> jointGroupSeen;
  vec<arr<bool, Skeleton::kNumJointGroups>> vJointGroupSeen;
  vJointGroupSeen.resize(vIndex.size());
  for (const auto& it : interactions.getInteractionSets()) {
    const InteractionSet& interactionSet = it.second;
    // Figure out what verbs this action goes with
    vec<int> indices = extractIndices(interactionSet);

    // For each noun, figure out what verbs goes with it
    map<string, vec<int>> nvIndices = extractIndicesMap(interactionSet);
    for (Interaction* interaction : interactionSet.interactions) {
      assert(interaction->scan != nullptr);
      if (interaction->scan == nullptr) {
        // Skip
        SG_LOG_WARN << "Skipping interaction " << interaction->id << " no scan";
        continue;
      }
      const segmentation::SegmentGroups& segGroups = interaction->scan->segmentGroups;
      // Go through skeletons
      for (size_t iSkel = 0; iSkel < interaction->jointSegments.size(); iSkel++) {
        // Accumulate count for the verb
        for (int iV : indices) {
          stats.rawCounts1[iV]++;
        }
        stats.totalRawCounts++;

        // Accumulate count for the joint group
        const auto& jointSegments = interaction->jointSegments[iSkel];
        for (int iV = 0; iV < vIndex.size(); iV++) {
          vJointGroupSeen[iV].fill(false);
        }
        jointGroupSeen.fill(false);

        for (size_t iJoint = 0; iJoint < Skeleton::kNumJoints + 1; iJoint++) {
          const int iJointGroup = static_cast<int>(Skeleton::kJointToJointGroup[iJoint]);
          if (!jointSegments[iJoint].empty()) {
            // Group joint segments by noun name
            map<string, segmentation::VecSegPtr> segsByCategory;
            for (const auto& segPtr : jointSegments[iJoint]) {
              // Get object label
              string catLabel = "";
              if (cachedSegmentObjLabels.count(segPtr) == 0) {
                // Not in cache, look again
                segmentation::SegmentGroups::SegmentGroupHandle segGroupId = segGroups.find(*segPtr);
                if (segGroupId >= 0) {
                  // Found segment group!
                  const auto segGroupPtr = segGroups.get(segGroupId);
                  string label = segGroupPtr->label;
                  string noun = core::synth::ObjectLabeler::labelToAnnotationNoun(label);
                  catLabel = noun;
                }
                cachedSegmentObjLabels[segPtr] = catLabel;
              } else {
                catLabel = cachedSegmentObjLabels.at(segPtr);
              }
              if (catLabel != "") {
                segsByCategory[catLabel].push_back(segPtr);
              }
            }

            for (const auto& catSegIt : segsByCategory) {
              const string catLabel = catSegIt.first;
              if (nvIndices.count(catLabel) > 0 && !catSegIt.second.empty()) {
                // Check if this joint group has been accounted for yet
                if (!jointGroupSeen[iJointGroup]) {
                  jointGroupSeen[iJointGroup] = true;
                  stats.rawCounts2[iJointGroup]++;
                }
                for (const int iV : nvIndices.at(catLabel)) {
                  // Check if this verb joint group has been accounted for yet
                  if (!vJointGroupSeen[iV][iJointGroup]) {
                    vJointGroupSeen[iV][iJointGroup] = true;
                    stats.rawCooccurrenceCounts(iV, iJointGroup)++;
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  stats.smoothAndComputeStats(interactions.getLaplaceSmoothingParam());
}

void InteractionDatabase::computeVerbJointMatchNounStats() {
  ISetToIndicesFn extractVerbIndices = [this] (const InteractionSet & iSet) {
    // Figure out what verbs this action goes with
    vec<int> verbIndices;
    for (const string& verb : iSet.verbs) {
      verbIndices.push_back(m_verbIndex.indexOf(verb));
    }
    return verbIndices;
  };

  ISetToIndicesMapFn extractNounToVerbIndicesMap = [this](const InteractionSet & iSet) {
    // Figure out what verbs this noun goes with
    map<string, vec<int>> nounVerbIndices;
    for (const VerbNoun& vn : iSet.verbNouns) {
      nounVerbIndices[vn.noun].push_back(m_verbIndex.indexOf(vn.verb));
    }
    return nounVerbIndices;
  };

  computeISetJointMatchNounStats(*this,
                                 extractVerbIndices,
                                 extractNounToVerbIndicesMap,
                                 m_verbIndex, m_jointGroupIndex,
                                 &m_verbJointGroupMatchNounStats);

}

void InteractionDatabase::computeVerbNounJointMatchNounStats() {
  ISetToIndicesFn extractVerbNounIndices = [this] (const InteractionSet & iSet) {
    // Figure out what verbNouns this action goes with
    vec<int> verbNounIndices;
    for (const VerbNoun& verbNoun : iSet.verbNouns) {
      verbNounIndices.push_back(m_verbNounIndex.indexOf(verbNoun.toString()));
    }
    return verbNounIndices;
  };

  ISetToIndicesMapFn extractNounToVerbNounIndicesMap = [this] (const InteractionSet & iSet) {
    // Figure out what verbs this noun goes with
    map<string, vec<int>> nounVerbIndices;
    for (const VerbNoun& vn : iSet.verbNouns) {
      nounVerbIndices[vn.noun].push_back(m_verbNounIndex.indexOf(vn.toString()));
    }
    return nounVerbIndices;
  };

  computeISetJointMatchNounStats(*this,
                                 extractVerbNounIndices,
                                 extractNounToVerbNounIndicesMap,
                                 m_verbNounIndex, m_jointGroupIndex,
                                 &m_verbNounJointGroupMatchNounStats);

}

void InteractionDatabase::computeNounJointStats() {
  ISetToIndicesFn extractNounIndices = [this] (const InteractionSet & iSet) {
    // Figure out what nouns this action goes with
    vec<int> nounIndices;
    for (const string& noun : iSet.nouns) {
      nounIndices.push_back(m_nounIndex.indexOf(noun));
    }
    return nounIndices;
  };

  computeISetJointStats(*this,
                        extractNounIndices,
                        m_nounIndex, m_jointGroupIndex,
                        &m_nounJointGroupStats);
}

void InteractionDatabase::loadStats(const string& dirname) {
  const string& basename = dirname + "verbs";
  const auto stodFn = [](const string & str) { return stod(str); };
  const auto stosizetFn = [] (const string & str) { return static_cast<size_t>(stoi(str)); };
  util::loadGrid<double>(basename + ".npmi.csv", stodFn, &m_verbStats.npmi);
  util::loadGrid<double>(basename + ".cp.csv", stodFn, &m_verbStats.condProb);
  util::loadGrid<size_t>(basename + ".counts.csv", stosizetFn, &m_verbStats.rawCooccurrenceCounts);
}

void saveGridStats(const string& basename,
                   const string& rowName, const util::Index<string>& rowIdx,
                   const string& colName, const util::Index<string>& colIdx,
                   const stats::GridStats2DT<string, string> stats) {
  saveGrid(basename + ".npmi.csv", rowName, rowIdx, colName, colIdx,
           stats.npmi);
  saveGridAsRows(basename + ".npmi-rows.csv", rowName, rowIdx, colName, colIdx,
                 stats.npmi);
  saveGrid(basename + ".cp1.csv", rowName, rowIdx, colName, colIdx,
           stats.condProb12);
  saveGridAsRows(basename + ".cp1-rows.csv", rowName, rowIdx, colName, colIdx,
                 stats.condProb12);
  saveGrid(basename + ".cp2.csv", rowName, rowIdx, colName, colIdx,
           stats.condProb21);
  saveGridAsRows(basename + ".cp2-rows.csv", rowName, rowIdx, colName, colIdx,
                 stats.condProb21);
  // Save marginals
  saveVector(basename + ".m1.csv", rowName, rowIdx, "prob",
             stats.marginalProb1);
  saveVector(basename + ".m2.csv", colName, colIdx, "prob",
             stats.marginalProb2);
}

void InteractionDatabase::saveStats(const string& dirname) const {
  const string& basename = dirname + "verbs";
  saveGrid(basename + ".npmi.csv", "verbNPMI", m_verbIndex,
           m_verbStats.npmi);
  saveGrid(basename + ".cp.csv", "verbCondProb", m_verbIndex,
           m_verbStats.condProb);
  saveGrid(basename + ".counts.csv", "verbCounts", m_verbIndex,
           m_verbStats.rawCooccurrenceCounts);

  saveGridStats(dirname + "verbNoun",
                "verb", m_verbIndex, "noun", m_nounIndex,
                m_verbNounStats);
  saveGridStats(dirname + "nounJointGroup",
                "noun", m_nounIndex, "joint", m_jointGroupIndex,
                m_nounJointGroupStats);
  saveGridStats(dirname + "verbJointGroup",
                "verb", m_verbIndex, "joint", m_jointGroupIndex,
                m_verbJointGroupStats);
  saveGridStats(dirname + "verbJointGroupMatchNoun",
                "verb", m_verbIndex, "joint", m_jointGroupIndex,
                m_verbJointGroupMatchNounStats);
  saveGridStats(dirname + "verbNounJointGroupMatchNoun",
                "verb", m_verbNounIndex, "joint", m_jointGroupIndex,
                m_verbNounJointGroupMatchNounStats);
  saveGridStats(dirname + "nounSceneType",
                "noun", m_nounIndex, "sceneType", m_sceneTypeIndex,
                m_nounSceneTypeStats);

  saveIndexCsv(dirname + "nounIndex.csv", m_nounIndex);
  saveIndexCsv(dirname + "partIndex.csv", m_partIndex);
}

bool InteractionDatabase::loadBinary(const string& file) {
  if (!io::fileExists(file)) {
    SG_LOG_ERROR << "File not found: " << file;
    return false;
  }
  bool loadSuccessful;
  ifstream ifs(file, std::ios::binary);
  {
    boost::archive::binary_iarchive ar(ifs);
    ar >> *this;
    if (m_version != INTERACTION_DB_VERSION) {
      SG_LOG_ERROR << "Incompatible interaction DB verion " << m_version 
        << ": Supported version " << INTERACTION_DB_VERSION;
      loadSuccessful = false;
    } else {
      loadSuccessful = true;
    }
  }
  ifs.close();
  if (!loadSuccessful) {
    // Move old cache db file to backup
    io::moveFile(file, file + ".bak", true /* allowOverwrite */);
  }
  return loadSuccessful;
}

bool InteractionDatabase::saveBinary(const string& file) const {
  io::ensurePathToFileExists(file);
  ofstream ofs(file, std::ios::binary);
  {
    boost::archive::binary_oarchive ar(ofs);
    ar << *this;
  }
  ofs.close();
  return true;
}

void InteractionDatabase::prepareSerializableSkeletons(vec<Skeleton>* pSkels, 
                                                       vec<geo::ScoredTransform>* pTransforms) const {
  // Go through interaction set and create skeletons
  geo::ScoredTransform st;
  util::Index<const Skeleton*> skelIndex;
  for (const auto& it : m_allInteractionSets) {
    const InteractionSet& iSet = it.second;
    for (const auto& tSkel : iSet.sampledSkeletons) {
      int i = skelIndex.add(tSkel.getOriginalSkeleton());
      st.index = i;
      st.label = iSet.id;
      st.score = tSkel.score;
      st.weight = tSkel.weight;
      st.xform = tSkel.transform();
      pTransforms->emplace_back(st);
    }
  }
  pSkels->clear();
  pSkels->resize(skelIndex.size());
  for (int i = 0; i < skelIndex.size(); ++i) {
    (*pSkels)[i] = *skelIndex[i];
  }
}

void InteractionDatabase::populateInteractionSetSkeletons(vec<Skeleton>& skels, 
                                                          const vec<geo::ScoredTransform>& transforms) {
  for (Skeleton& skel : skels) {
    skel.computeGaze();
  }
  for (const geo::ScoredTransform& t : transforms) {
    // Find appropriate interaction set
    InteractionSet& iSet = m_allInteractionSets.at(t.label);
    skels[t.index].computeGaze();
    core::TransformedSkeleton transformedSkel(skels[t.index], t.xform);
    transformedSkel.score = t.score;
    transformedSkel.weight = t.weight;
    iSet.sampledSkeletons.emplace_back(transformedSkel);
  }
}

void InteractionDatabase::populateNounToParts() {
  vec<string> fields;
  m_nounToParts.clear();
  for (const string& label : m_partIndex.labels()) {
    util::tokenize(label, ":", &fields);
    m_nounToParts[fields[0]].push_back(label);
  }
}

}  // namespace interaction
}  // namespace sg
