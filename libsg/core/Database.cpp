#include "common.h"  // NOLINT

#include "core/Database.h"

#include "core/ClassifierDatabase.h"
#include "core/features.h"
#include "core/MotionDatabase.h"
#include "core/Recording.h"
#include "core/RecordingDatabase.h"
#include "core/ScanDatabase.h"
#include "core/SkeletonDatabase.h"
#include "interaction/Interaction.h"
#include "io/io.h"
#include "util/timer.h"

using sg::interaction::Interaction;
using sg::interaction::InteractionSet;

namespace sg {
namespace core {

Database::Database(util::Params& _params)
  : params(&_params)
  , scans(_params, "/scans/scenes.json")
  , scansCornellPLY(_params, "/scans-cornell/scenes.json")
  , models(_params)
  , recordings(scans, _params)
  , interactionFactory(_params)
  , interactions(_params, interactionFactory)
  , reconScenes(_params, models)
  , skeletons(nullptr)
  , motions(nullptr)
  , m_isLoaded(false) {
  classifiers = new ClassifierDatabase(*this,
                                       _params.get<string>("dataDir") + _params.get<string>("classifiersDir"),
                                       _params.get<string>("workDir") + _params.get<string>("classifiersDir"),
                                       _params.get<string>("classifierSet"));
  skeletons = new SkeletonDatabase(_params);
  motions = new MotionDatabase(_params);
  featureGenerators.fill(nullptr);
  modelAnnotations.loadAnnotations(_params);
  m_labeler.init(this);
}

Database::~Database() {
  if (classifiers != nullptr) { delete classifiers; }
  if (skeletons != nullptr) { delete skeletons; }
  for (const SegmentFeatureGenerator* featGen : featureGenerators) {
    if (featGen) { delete featGen; }
  }
}

void Database::init(util::Params& p) {
  util::Timer t("[Database] init");
  //bool forceRetrainClassifiers = p.get<bool>("Dataset.forceRetrain");
  bool forceRecomputeCentroids = p.get<bool>("Dataset.forceRecomputeCentroids");
  bool computeInteractionGraphs = p.get<bool>("Dataset.computeInteractionGraphs");
  load(/* computeAllActiveSegments= */ true, computeInteractionGraphs);
  computeCentroids(forceRecomputeCentroids);
}

const SegmentFeatureGenerator* Database::getSegmentFeatureGenerator(const SegmentFeatureGenerator::FeatureType type)
const {
  if (featureGenerators[type] == nullptr) {
    if (type == SegmentFeatureGenerator::FeatureType_MeshCentroid) {
      const bool splitCentroidsPerJointGroup = params->get<bool>("Clustering.splitCentroidsPerJointGroup");
      featureGenerators[type] = new SegmentCentroidActivationFeatGen(centroids, splitCentroidsPerJointGroup);
    } else if (type == SegmentFeatureGenerator::FeatureType_MeshSimplistic) {
      featureGenerators[type] = new SegmentFeatureGeneratorSimplistic;
    } else if (type == SegmentFeatureGenerator::FeatureType_Wolf) {
      featureGenerators[type] = new SegmentFeatureGeneratorWolf;
    }
  }
  return featureGenerators[type];
}

void Database::load(bool computeAllActiveSegments, bool computeInteractionGraphs) {
  if (m_isLoaded) { return; }

  scenePriors.init(this);
  models.addParentCategories(scenePriors.getCategoryHierarchy());

  bool useCachedInteractions = params->get<bool>("useCachedInteractions");
  bool hasCachedInteractions = false;
  const string& cachedFilename = params->get<string>("dataCacheDir") + "interactionDb.cached";
  if (useCachedInteractions) {
    if (io::fileExists(cachedFilename)) {
      SG_LOG_INFO << "Loading interaction database from " << cachedFilename;
      bool loadSuccessful = interactions.loadBinary(cachedFilename);
      hasCachedInteractions = loadSuccessful;
    } else {
      SG_LOG_WARN << "Cannot find cached interaction db file " << cachedFilename;
    }
  }

  if (!hasCachedInteractions) {
    recordings.loadAllRecordings(computeAllActiveSegments);
    interactions.init(*this, computeInteractionGraphs);

    const auto& allInteractionSets = interactions.getAllInteractionSets();
    const unsigned int skelClusterCount = params->get<unsigned int>("Interaction.skeletonClusterCount");
    if (skelClusterCount > 0) {
      cout << "[Database] Clustering skeletons...";
      for (auto& it : allInteractionSets) {
        InteractionSet& iSet = it.second;
        iSet.clusterSkeletons(skelClusterCount);
      } 
      cout << "done" << endl;
    } else {
      cout << "[Database] Sampling skeletons...";
      const unsigned int skelSampleCount = params->get<unsigned int>("Interaction.skeletonCount");
      for (auto& it : allInteractionSets) {
        InteractionSet& iSet = it.second;
        iSet.sampleSkeletons(skelSampleCount);
      }
      cout << "done" << endl;
    }

    if (useCachedInteractions) {
      SG_LOG_INFO << "Caching interaction database to " << cachedFilename;
      interactions.saveBinary(cachedFilename);
      interactions.saveInteractionSummary(params->get<string>("dataCacheDir") + "interactions.summary.csv");
      interactions.saveInteractionRecordSummary(params->get<string>("dataCacheDir") + "interactionRecords.summary.csv");
    }
  }

  skeletons->init(recordings, interactions);

  m_isLoaded = true;
}

void Database::computeCentroids(bool forceRecompute) {
  const size_t totalCentroids = params->get<UINT>("Clustering.numCentroids");
  const bool splitCentroids = params->get<bool>("Clustering.splitCentroidsPerJointGroup");
  const string centroidFilename = "centroids-" + to_string(totalCentroids)
                                  + (splitCentroids ? "-split.bin" : "-nosplit.bin");
  const string defaultCentroidDir = classifiers->getClassifierDir();
  const string centroidDir = params->getWithDefault<string>("centroidDir", defaultCentroidDir);
  const string centroidFilepath = centroidDir + centroidFilename;
  if (ml::util::fileExists(centroidFilepath) && !forceRecompute) {
    centroids.load(centroidFilepath);
  } else {
    scans.loadAllScansWithRecordings(recordings);
    centroids.create(*this);
    centroids.save(centroidFilepath);
  }
}

void Database::loadClassifiers(bool forceRetrain) {
  cout << "Loading classifiers...";
  TrainOpts::RetrainLevel retrainLevel = (forceRetrain)? TrainOpts::RetrainLevel_UseLoaded : TrainOpts::RetrainLevel_None;
  classifiers->loadClassifiers(retrainLevel);
  cout << " Done." << endl;
}

void Database::saveAllActiveSegments(const string& csvFile) const {
  ofstream ofs(csvFile);
  ofs << "interaction,joint";
  SegmentFeatureGeneratorSimplistic featGen;
  for (const auto& f : featGen.fieldNames()) { ofs << "," + f; }
  ofs << endl;

  const vec<string> recIds = recordings.getLoadedRecordingIds();
  for (const string& recId : recIds) {
    const Recording& rec = recordings.getRecording(recId, true, false);
    for (Interaction* i : rec.interactions) {
      for (const Skeleton::SegmentsByJointPlusGaze& segsByJoint : i->jointSegments) {
        for (size_t iJoint = 0; iJoint < segsByJoint.size(); iJoint++) {
          const string interaction = i->isetId;
          const string jointName = Skeleton::jointName(iJoint);
          const auto& segs = segsByJoint[iJoint];
          for (const auto& seg : segs) {
            const vecd& feats = featGen.generate(*seg);
            ofs << interaction << "," << jointName;
            for (const double d : feats) { ofs << "," + to_string(d); }
            ofs << endl;
          }
        }
      }
    }
  }
  ofs.close();
}

void Database::describeDataset() const {
  map<string, int> recordingCount;
  map<string, float> recordingLength;
  map<string, set<string> > recordingScenes;
  ml::Console::log() << "Scene count: " << scans.scanIds().size() << endl;
  ml::Console::log() << "Recording count: " << recordings.getLoadedRecordingIds().size() << endl;
  for (auto& recID : recordings.getLoadedRecordingIds()) {
    auto& rec = recordings.getRecording(recID, true, true);
    for (const Interaction* interaction : rec.interactions) {
      if (recordingCount.count(interaction->isetId) == 0) {
        recordingCount[interaction->isetId] = 0;
        recordingLength[interaction->isetId] = 0.0;
      }
      recordingCount[interaction->isetId]++;
      recordingLength[interaction->isetId] += interaction->endTime - interaction->startTime;
      recordingScenes[interaction->isetId].insert(rec.baseScanId());
    }
  }
  for (auto& p : recordingCount) {
    ml::Console::log() << p.first << ": " << recordingScenes[p.first].size() << " scans, "
                       << recordingLength[p.first] / 60.0f << " minutes" << endl;
  }
}

SkeletonToInteractionsMap Database::getSkeletonInteractionRecords() const {
  SkeletonToInteractionsMap out;
  // Go over all interactions once - so use the default composite interaction sets to cover all interactions
  for (auto& it : interactions.getInteractionSets()) {
    InteractionSet& interactionSet = it.second;
    for (Interaction* interaction : interactionSet.interactions) {
      for (size_t iSkel = 0; iSkel < interaction->skelRange.size(); iSkel++) {
        const Skeleton& skel = interaction->skelRange[iSkel];
        const InteractionRecord snippet = {interaction, iSkel};
        out[&skel].push_back(snippet);
      }
    }
  }
  return out;
}

map<const MeshSegment*, vecd> Database::precomputeSegmentFeatures(const SegmentFeatureGenerator& geoGenerator) const {
  cout << "Computing segment features..." << endl;
  map<const MeshSegment*, vecd> out;
  // Go over all interactions once - so use the default composite interaction sets to cover all interactions
  for (auto& it : interactions.getInteractionSets()) {
    InteractionSet& interactionSet = it.second;
    for (auto& interaction : interactionSet.interactions) {
      for (const auto& seg : interaction->activeSegments) {
        if (out.find(seg.get()) == out.end()) {
          out[seg.get()] = geoGenerator.generate(*seg);
        }
      }
    }
  }
  return out;
}

}  // namespace core
}  // namespace sg
