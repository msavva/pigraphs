#include "common.h"  // NOLINT

#include "core/ClassifierDatabase.h"

#include "core/Database.h"
#include "core/learningUtil.h"
#include "io/io.h"
#include "util/util.h"

#include "../jace-proxy/wekautil.h"

// This forward declaration is CRAZYEEE! But it hides jace proxy headers from everyone else
namespace jace { namespace proxy { namespace edu { namespace stanford { namespace graphics { namespace wekautils { class Classer; }}}}}};
namespace wekautil { typedef jace::proxy::edu::stanford::graphics::wekautils::Classer Classifier; }

using sg::interaction::InteractionSet;
using sg::interaction::InteractionSetType;

namespace sg {
namespace core {

bool reportWarnings = false;

//! Returns important parameters for the classifier encoded in a string id1-val1_id2-val2_...
string getParamsPrefix(const util::Params& params, const string& type) {
  string s = "";
  const bool splitCentroidsPerJointGroup = params.get<bool>("Clustering.splitCentroidsPerJointGroup");
  const bool usingMeshCentroid = params.get<string>("Dataset.segmentPerJointFeatures") == "meshcentroid";
  const bool useActivation = params.get<bool>("Dataset.useActivation");
  if (type == "segmentPerJoint" || type == "segmentPerJointJoint") {
    s = s + "feats-" + params.get<string>("Dataset.segmentPerJointFeatures");
    if (usingMeshCentroid) {
      s = s + "_" + (splitCentroidsPerJointGroup ? "split1" : "split0");
    }
  } else if (type == "segmentJointsAggr") {
    bool useJointSegScores = params.get<bool>("Dataset.useJointSegScores");
    bool useJointSegFeatures = params.get<bool>("Dataset.useJointSegFeatures");
    if (useJointSegScores) {
      s = s + "feats-" + params.get<string>("Dataset.segmentPerJointFeatures");
      if (usingMeshCentroid) {
        s = s + "_" + (splitCentroidsPerJointGroup ? "split1" : "split0");
      }
      s = s + "_";
    }
    s = s + (useActivation? "a1":"a0") + "_"  + (useJointSegScores? "s1":"s0") + "_"  + (useJointSegFeatures? "f1":"f0");
  } else if (type == "segmentCentroidActivation") {
    s = s + (splitCentroidsPerJointGroup ? "split1" : "split0");
    s = s + "_" + (useActivation? "a1":"a0");
  }
  return s;
}

// Classifier Database
ClassifierDatabase::ClassifierDatabase(const Database& database, 
                                       const string& classifierDir, 
                                       const string& workDir,
                                       const string& classifierSet)
  : postTestCallbackFun(nullptr)
  , doPostTestCallback(false)
  , doCrossValidation(false)
  , m_database(database)
  , m_params(*database.params)
  , m_classifierDir(classifierDir)
  , m_workDir(workDir)
  , m_classifierSet(classifierSet)
  , m_currTimestampString(util::timeNowYMDHSstring())  
  , m_logAttachCount(0) {
    initClassifierManagers();
}

string ClassifierDatabase::makeOutputDatasetDir(const string& datasetTypeId) {
  const string cvString = (doCrossValidation) ? "_cv/" + m_params.get<string>("Dataset.testSetSceneIds") + "/" : "";
  const string params = getParamsPrefix(m_params, datasetTypeId);
  const string paramsPrefix = params.empty()? "default" : params;
  const string generalPath = getCurrentTimeStampString() + "/*/" + datasetTypeId + "/" + paramsPrefix + cvString + "/";
  const string path = util::replaceStringAll(generalPath, "*", "datasets");
  const string datasetsDir = m_workDir + "/" + path;

  //! We are preparing to dump dataset for classifiers into these directories
  //! Presumably we want to use it for training: save it away
  m_classifierPrefixes[datasetTypeId] = generalPath; 
  io::ensureDirExists(datasetsDir);
  //! Write out current parameters 
  m_params.write(datasetsDir + "parameters.txt");
  return datasetsDir;
}

string ClassifierDatabase::getClassifierPrefix(const string& type) const {
  auto classIt = m_classifierPrefixes.find(type);
  const bool found = classIt != m_classifierPrefixes.end();
  const string cvString = (doCrossValidation) ? "_cv/" + m_params.get<string>("Dataset.testSetSceneIds") + "/" : "";
  if (found) {
    return (classIt->second);
  } else {
    const string params = getParamsPrefix(m_params, type);
    if (params.empty()) {
      return m_classifierSet + "/*/" + type + "/" + "default" + cvString + "/";
    } else {
      return m_classifierSet + "/*/" + type + "/" + params + cvString + "/";
    }
  }
}

string ClassifierDatabase::getClassifierPath(const string& type, const string& subdir) const {
  string prefix = getClassifierPrefix(type);
  string path = util::replaceStringAll(prefix, "*", subdir);
  return path;
}

string ClassifierDatabase::getClassifierId(const string& type, const string& arg1) const {
  const string prefix = getClassifierPrefix(type);
  const string id = prefix + arg1;
  return id;
}

void ClassifierDatabase::loadClassifiers(TrainOpts::RetrainLevel forceRetrainLevel) {
  for (const auto& manager:m_classifierManagers) {
    try {
      SG_LOG_INFO << "Loading " << manager->name;
      manager->load(forceRetrainLevel);
    } catch (const std::exception& e) {
      SG_LOG_ERROR << "Error loading " << manager->name << ":" << e.what();
    }
  }
}

void ClassifierDatabase::loadClassifier(const string& type, TrainOpts::RetrainLevel forceRetrainLevel) {
  const auto& manager = getClassifierManager(type);
  if (manager != nullptr) {
    try {
      manager->load(forceRetrainLevel);
    } catch (const std::exception& e) {
      SG_LOG_ERROR << "Error loading " << manager->name << ":" << e.what();
    }
  } else {
    SG_LOG_ERROR << "Error loading unknown classifier type " << type;
  }
}

void ClassifierDatabase::trainClassifiers(const TrainOpts& trainOpts) {
  bool doAllVariations = m_params.get<bool>("Dataset.allVariations");
  for (const auto& manager:m_classifierManagers) {
    try {
      SG_LOG_INFO << "Training " << manager->name;
      attachLog();
      manager->train(trainOpts, doAllVariations);
      detachLog();
    } catch (const std::exception& e) {
      SG_LOG_ERROR << "Error training " << manager->name << ":" << e.what();
    }
  }
  SG_LOG_INFO << "Finished training classifiers";
}

ClassifierManager* ClassifierDatabase::getClassifierManager(const string& type) {
  const auto& it = m_classifierManagerIndices.find(type);
  const bool found = it != m_classifierManagerIndices.end();
  if (found) {
    return m_classifierManagers[it->second];
  } else {
    return nullptr;
  }
}

const ClassifierManager* ClassifierDatabase::getClassifierManager(const string& type) const {
  const auto& it = m_classifierManagerIndices.find(type);
  const bool found = it != m_classifierManagerIndices.end();
  if (found) {
    return m_classifierManagers[it->second];
  } else {
    return nullptr;
  }
}


void ClassifierDatabase::trainClassifier(const string& type, const TrainOpts& trainOpts) {
  bool doAllVariations = m_params.get<bool>("Dataset.allVariations");
  trainClassifier(type, trainOpts, doAllVariations);
}

void ClassifierDatabase::trainClassifier(const string& type, const TrainOpts& trainOpts, bool doAllVariations) {
  const auto& it = m_classifierManagerIndices.find(type);
  const bool found = it != m_classifierManagerIndices.end();
  if (found) {
    SG_LOG_INFO << "Training " << type;
    attachLog();
    m_classifierManagers[it->second]->train(trainOpts, doAllVariations);
    detachLog();
  } else {
    SG_LOG_ERROR << "Cannot train unknown classifier " << type;
  }
}

void ClassifierDatabase::crossValidateClassifiers() {
  bool doAllVariations = m_params.get<bool>("Dataset.allVariations");
  for (const auto& manager:m_classifierManagers) {
    try {
      SG_LOG_INFO << "Cross validating " << manager->name;
      attachLog();
      manager->crossValidate(doAllVariations);
      detachLog();
    } catch (const std::exception& e) {
      SG_LOG_ERROR << "Error training " << manager->name << ":" << e.what();
    }
  }
  SG_LOG_INFO << "Finished cross validating classifiers";
}

void ClassifierDatabase::crossValidateClassifier(const string& type) {
  bool doAllVariations = m_params.get<bool>("Dataset.allVariations");
  const auto& it = m_classifierManagerIndices.find(type);
  const bool found = it != m_classifierManagerIndices.end();
  if (found) {
    SG_LOG_INFO << "Cross validating " << type;
    attachLog();
    m_classifierManagers[it->second]->crossValidate(doAllVariations);
    detachLog();
  } else {
    SG_LOG_ERROR << "Cannot cross validate unknown classifier " << type;
  }
}

void ClassifierDatabase::testClassifiers() {
  bool doAllVariations = m_params.get<bool>("Dataset.allVariations");
  for (const auto& manager:m_classifierManagers) {
    try {
      SG_LOG_INFO << "Testing " << manager->name;
      attachLog();
      manager->test(doAllVariations);
      detachLog();
    } catch (const std::exception& e) {
      SG_LOG_ERROR << "Error testing " << manager->name << ":" << e.what();
    }
  }
  SG_LOG_INFO << "Finished testing classifiers";
}

void ClassifierDatabase::testClassifier(const string& type) {
  bool doAllVariations = m_params.get<bool>("Dataset.allVariations");
  const auto& it = m_classifierManagerIndices.find(type);
  const bool found = it != m_classifierManagerIndices.end();
  if (found) {
    SG_LOG_INFO << "Testing " << type;
    attachLog();
    m_classifierManagers[it->second]->test(doAllVariations);
    detachLog();
  } else {
    SG_LOG_ERROR << "Cannot test unknown classifier " << type;
  }
}

bool ClassifierDatabase::testWekaClasserOnTest(ClassifierBase* pClassifier, eval::BinaryConfusionMatrix* confM) const {
  const string& id = pClassifier->simpleId;
  const string testData = pClassifier->datasetDir + "/" + id + "_test.arff";
  if (!io::fileExists(testData)) {
    SG_LOG_ERROR << "[testWekaClasserOnTest] File not found, cannot test on file: " << testData;
    return false;
  }
  const string testReportFile = getEvalPath(pClassifier->type) + "/" + id + "_test.txt";
  io::ensurePathToFileExists(testReportFile);
  return pClassifier->test(testData, testReportFile, confM);
}

bool ClassifierDatabase::testWekaClasserOnTrain(ClassifierBase* pClassifier, eval::BinaryConfusionMatrix* confM) const {
  const string& id = pClassifier->simpleId;
  const string trainData = pClassifier->datasetDir + "/" + id + "_train.arff";
  if (!io::fileExists(trainData)) {
    SG_LOG_ERROR << "[testWekaClasserOnTrain] File not found, cannot test on file: " << trainData;
    return false;
  }
  const string trainReportFile = getEvalPath(pClassifier->type) + "/" + id + "_train.txt";
  io::ensurePathToFileExists(trainReportFile);
  return pClassifier->test(trainData, trainReportFile, confM);
}

void ClassifierDatabase::trainWekaClasser(ClassifierBase* pClassifier, bool forceRetrain,
                                          double biasToUniform, double sampleSizePercent) const {
  pClassifier->pWekaClasser = nullptr;
  wekautil::Classifier* pClasser = new wekautil::Classifier();
  *pClasser = jace::java_new<wekautil::Classifier>(
    pClassifier->wekaClassifierType,
    pClassifier->wekaClassifierOptions, 
    pClassifier->id);
   
  const string subDir = util::replaceStringAll(pClassifier->id, "*", "models");
  string classerModelFile = io::fileExists(subDir + ".model")
                          ? subDir + ".model" : m_classifierDir + "/" + subDir + ".model";
  int classifierOkay = false;
  try {
    if (io::fileExists(classerModelFile) && !forceRetrain) {
      SG_LOG_DEBUG << "Loading " << classerModelFile;
      if (pClasser->load(classerModelFile)) {
        pClassifier->datasetDir = getDatasetsPath(pClassifier->type, true);
        classifierOkay = true;
      } else {
        SG_LOG_ERROR << "Failed loading " << classerModelFile;
      }
    } else {
      // Use workDir instead
      classerModelFile = m_workDir + "/" + subDir + ".model";
      pClassifier->datasetDir = getDatasetsPath(pClassifier->type, false);

      // Check training data
      const string& trainData = pClassifier->datasetDir + "/" + pClassifier->simpleId + "_train.arff";
      if (io::fileExists(trainData)) {
        SG_LOG_INFO << "Training classifier from " << trainData << ", output " << classerModelFile + "/" + subDir;
        io::ensurePathToFileExists(classerModelFile);

        // Train classifier
        if (pClasser->train(trainData, biasToUniform, sampleSizePercent)) {
          SG_LOG_DEBUG << "Saving " << classerModelFile;
          if (pClasser->save(classerModelFile)) {
            classifierOkay = true;
          } else {
            SG_LOG_ERROR << "Failed saving " << classerModelFile;
          }
        } else {
          SG_LOG_ERROR << "Failed training from " << trainData;
        }
      } else {
        if (reportWarnings) {
          SG_LOG_WARN << "Skipping classifier training, data not found: " + trainData;
        }
      }       
    }
  } catch (...) {
    wekautil::handleJaceExceptions();
  }

  if (classifierOkay) {
    SG_LOG_DEBUG << "Set datasetDir to " << pClassifier->datasetDir;
    pClassifier->classifierDir = io::parentPath(classerModelFile);
    pClassifier->pWekaClasser = pClasser;
  } else {
    delete pClasser;
  }
}

void ClassifierDatabase::reportTestResults(const string& type,
                                           const string& classifierID,
                                           const string& classifierModelType) const {
  const string outputDir = getResultOutputPath(type);
  wekautil::Classifier::reportAllTestResults(outputDir, classifierID, classifierModelType);
}

// Classifier Managers
void ClassifierManager::prepareDataset(const TrainOpts& trainOpts) {
  const bool recomputeCentroids = m_database.params->get<bool>("Dataset.forceRecomputeCentroids");
  if (recomputeCentroids) {
    m_database.params->set<string>("centroidDir", m_database.classifiers->getCurrentWorkDir());
    m_database.computeCentroids(false);
  }
}

void ClassifierManager::train(const TrainOpts& trainOpts) {
  prepareDataset(trainOpts);
  load(trainOpts.forceRetrainLevel);
  if (trainOpts.doTest) {
    test();
  }
}

void ClassifierManager::trainAllVariations(const TrainOpts& trainOpts) {
  vec<ParameterSetting> variations = getParameterVariations();
  if (variations.empty()) {
    // Maybe just the default settings
    train(trainOpts);
    if (!trainOpts.doTest) {
      test();
    }
  } else {
    for (const auto& setting: variations) {
      // Set my settings
      setting.init(m_database.params);
      train(trainOpts);
      if (!trainOpts.doTest) {
        test();
      }
    }
  }
}

vec<ParameterSetting> ClassifierManager::getParameterVariations() {
  vec<ParameterSetting> settings;
  return settings;
}

using eval::BinaryConfusionMatrix;
using eval::BinaryConfusionMatrixSet;

void ClassifierManager::crossValidate() {
  // TODO(MS): Make thread safe
  m_database.classifiers->doCrossValidation = true;
  // Do leave one out (scene-wise) cross validation
  BinaryConfusionMatrixSet confMatMap;
  TrainOpts trainOpts(TrainOpts::RetrainLevel_UseDataset, TrainOpts::RecreateDatasetLevel_UseLoaded, false);
  for (const auto& scanId : m_database.scans.scanIds()) {
    // Train classifier with scanId as test scan
    m_database.params->set<string>("Dataset.testSetSceneIds", scanId);
    train(trainOpts);
    
    // Evaluate on test scene
    test(&confMatMap);
  }

  // Aggregate over test/train for each interaction type
  BinaryConfusionMatrixSet datasetInteractionMap;
  for (const auto& idMat : confMatMap) {
    const string& id = idMat.first;
    const BinaryConfusionMatrix& M = idMat.second;
    const vec<string> tokens = ml::util::split(id, ',');
    assert(tokens.size() == 3);
    //const string& type = tokens[0];
    const string& dataset = tokens[1];
    const string& interaction = tokens[2];
    const vec<string> interactionPieces = ml::util::split(interaction, '/');
    datasetInteractionMap[dataset + "," + interaction] += M;
    if (interactionPieces.size() > 1) {
      datasetInteractionMap[dataset + "," + interactionPieces[0]] += M;
    }
    datasetInteractionMap[dataset + "," + "all"] += M;
  }
  
  // Horrible hack to get one dir up
  const string path = m_database.classifiers->getEvalPath(name);
  string cleanPath = path;
  if (*(cleanPath.end() - 1) == '/') {
    while (*(cleanPath.end() - 1) == '/') {
      cleanPath = cleanPath.substr(0, cleanPath.size() - 1);
    }
  }
  cleanPath = ml::util::getSubstrBeforeLast(cleanPath, "/");

  // Now iterate and write out
  const string& outFilename = cleanPath + "/eval.csv";
  SG_LOG_INFO << "Writing cross-validating evaluation at " << outFilename;
  io::ensurePathToFileExists(outFilename);
  ofstream ofs(outFilename);
  ofs << "type,dataset,interaction," << BinaryConfusionMatrix::csvHeader() << endl;
  for (const auto& idMat : datasetInteractionMap) {
    const string& id = idMat.first;
    const BinaryConfusionMatrix& M = idMat.second;
    ofs << name << "," << id << "," << M << endl;
  }

  m_database.classifiers->doCrossValidation = false;
}

void ClassifierManager::crossValidateAllVariations() {
  vec<ParameterSetting> variations = getParameterVariations();
  if (variations.empty()) {
    // Maybe just the default settings
    crossValidate();
  } else {
    for (const auto& setting: variations) {
      // Set my settings
      setting.init(m_database.params);
      crossValidate();
    }
  }
}

void ClassifierManager::testAllVariations() {
  vec<ParameterSetting> variations = getParameterVariations();
  if (variations.empty()) {
    // Maybe just the default settings
    test();
  } else {
    for (const auto& setting: variations) {
      // Set my settings
      setting.init(m_database.params);
      load(TrainOpts::RetrainLevel_None);
      test();
    }
  }
}

void ClassifierManager::saveWeights(const string& filename) {
  SG_LOG_WARN << "No weights to save for " << name;
}

const ParameterSetting::InitFunc ParameterSetting::doNothing = [](){}; 
static const arr<bool,2> boolValues = { false, true };

// Helper function for pushing back centroid settings 
void addCentroidSettings(Database* database,
                         vec<ParameterSetting>* settings, 
                         const ParameterSetting& setting) {
  // TODO: Vary number of centroids... have centroid database give me the centroids path
  const size_t totalCentroids = database->params->get<UINT>("Clustering.numCentroids");
  const string centroidDbFilePrefix = "centroids-" + to_string(totalCentroids); 
  const bool recomputeCentroids = database->params->get<bool>("Dataset.forceRecomputeCentroids");

  ParameterSetting splitSetting = setting;
  if (recomputeCentroids) {
    splitSetting.addParam("centroidDir", database->classifiers->getCurrentWorkDir());
  }

  const string centroidDBfileSplit = centroidDbFilePrefix + "-split.bin";
  splitSetting.addParam<bool>("Clustering.splitCentroidsPerJointGroup", true);
  splitSetting.addParam<string>("Clustering.centroidDBfile", centroidDBfileSplit);
  splitSetting.addInit( [database](){ database->computeCentroids(false); } );

  ParameterSetting nosplitSetting = setting;
  const string centroidDBfileNosplit = centroidDbFilePrefix + "-nosplit.bin";
  nosplitSetting.addParam<bool>("Clustering.splitCentroidsPerJointGroup", false);
  nosplitSetting.addParam<string>("Clustering.centroidDBfile", centroidDBfileNosplit);
  splitSetting.addInit( [database](){ database->computeCentroids(false); } );

  settings->push_back(nosplitSetting);
  settings->push_back(splitSetting);
}

// Helper function for pushing back per joint segment feature settings
void addSegmentFeatureSettings(Database* database,
                               vec<ParameterSetting>* settings, 
                               const ParameterSetting& setting) {
  for (size_t iFeat = 0; iFeat < SegmentFeatureGenerator::kNumFeatureTypes; iFeat++) {
    const string& featureType = SegmentFeatureGenerator::kFeatureTypes[iFeat];
    ParameterSetting featSetting = setting;
    featSetting.addParam<string>("Dataset.segmentPerJointFeatures", featureType);
    if (iFeat == SegmentFeatureGenerator::FeatureType_MeshCentroid) {
      addCentroidSettings(database, settings, featSetting);
    } else {
      settings->push_back(featSetting);
    }
  }
}
class SegmentPerJointClassifierManager : public ClassifierManager {
public:
  SegmentPerJointClassifierManager(const string _name, Database& database, bool _useJointFeatures): 
    ClassifierManager(ClassifierType_SegmentJoint, _name, database), useJointFeatures(_useJointFeatures) {}
  void prepareDataset(const TrainOpts& opts) override {
    ClassifierManager::prepareDataset(opts);
    makeSegmentPerJointDataset(m_database, opts, useJointFeatures);
  }
  //! Load my many classifiers
  bool load(TrainOpts::RetrainLevel forceRetrainLevel) override {
    auto& classifiers = m_database.classifiers;
    bool hasError = false;
    for (auto& it : m_database.interactions.getAllInteractionSets()) {
      InteractionSet& interactionSet = it.second;
      for (size_t iJointGroup = 0; iJointGroup < Skeleton::kNumJointGroups; iJointGroup++) {
        if (useJointFeatures) {
          interactionSet.segJointPerJointGroupClassifier[iJointGroup]
          = classifiers->makeClassifier<SegmentSkelPerJointGroupClassifier>(name,
            interactionSet.id + "_" + Skeleton::jointGroupName(iJointGroup), 
            std::make_pair(interactionSet, iJointGroup), forceRetrainLevel, 1.0);
          if (interactionSet.segJointPerJointGroupClassifier[iJointGroup] == nullptr || !interactionSet.segJointPerJointGroupClassifier[iJointGroup]->isValid()) {
            hasError = true;
          }
        } else { 
          interactionSet.segPerJointGroupClassifier[iJointGroup]
          = classifiers->makeClassifier<SegmentPerJointGroupClassifier>(name,
            interactionSet.id + "_" + Skeleton::jointGroupName(iJointGroup), 
            std::make_pair(interactionSet, iJointGroup), forceRetrainLevel, 1.0);
          if (interactionSet.segPerJointGroupClassifier[iJointGroup] == nullptr || !interactionSet.segPerJointGroupClassifier[iJointGroup]->isValid()) {
            hasError = true;
          }
        }
      }
    }
    return !hasError;
  }
  //! Set up my parameter variations
  vec<ParameterSetting> getParameterVariations() override {
    vec<ParameterSetting> settings;
    ParameterSetting setting;
    addSegmentFeatureSettings(&m_database, &settings, setting);
    return settings;
  }
  void test(BinaryConfusionMatrixSet* confMatMap) override {
    auto& classifiers = m_database.classifiers;
    for (auto& it : m_database.interactions.getAllInteractionSets()) {
      InteractionSet& interactionSet = it.second;
      for (size_t iJointGroup = 0; iJointGroup < Skeleton::kNumJointGroups; iJointGroup++) {
        ClassifierBase* pClassifier = nullptr;
        if (useJointFeatures) {
          pClassifier = interactionSet.segJointPerJointGroupClassifier[iJointGroup];
        } else {
          pClassifier = interactionSet.segPerJointGroupClassifier[iJointGroup];
        }
        BinaryConfusionMatrix testM, trainM;
        classifiers->testWekaClasserOnTest(pClassifier, &testM);
        classifiers->testWekaClasserOnTrain(pClassifier, &trainM);
      }
    }
    if (classifiers->doPostTestCallback) {
      (*classifiers->postTestCallbackFun)(name);
    }
  }

public:
  const bool useJointFeatures;
};

class ScenePoseClassifierManager : public ClassifierManager {
 public:
  ScenePoseClassifierManager(const string _name, Database& database):
    ClassifierManager(ClassifierType_ScenePose, _name, database) {}
  void test(BinaryConfusionMatrixSet* confMatMap) override {
    auto& classifiers = m_database.classifiers;
    const string path = m_database.classifiers->getEvalPath(name);
    const string& outFilename = path + "/eval.csv";
    SG_LOG_INFO << "Writing testing evaluation at " << outFilename;
    io::ensurePathToFileExists(outFilename);
    ofstream ofs(outFilename);
    ofs << "type,dataset,interaction," << BinaryConfusionMatrix::csvHeader() << endl;
    BinaryConfusionMatrix interactionTypeTestM[InteractionSetType::ISetType_Count];
    BinaryConfusionMatrix interactionTypeTrainM[InteractionSetType::ISetType_Count];
    for (auto& it : m_database.interactions.getAllInteractionSets()) {
      InteractionSet& interactionSet = it.second;
      ScenePoseClassifier* pClassifier = getScenePoseClassifier(interactionSet);
      BinaryConfusionMatrix testM, trainM;
      classifiers->testWekaClasserOnTest(pClassifier, &testM);
      classifiers->testWekaClasserOnTrain(pClassifier, &trainM);
      const string testId = name + "," + "test," + interactionSet.id;
      const string trainId = name + "," + "train," + interactionSet.id;
      ofs << testId << "," << testM << endl;
      ofs << trainId << "," << trainM << endl;
      interactionTypeTestM[interactionSet.type] += testM;
      interactionTypeTrainM[interactionSet.type] += trainM;
      interactionTypeTestM[InteractionSetType::ISetType_All] += testM;
      interactionTypeTrainM[InteractionSetType::ISetType_All] += trainM;
      if (confMatMap) {
        (*confMatMap)[testId] += testM;
        (*confMatMap)[trainId] += trainM;
      }
    }
    for (int iType = 0; iType < InteractionSetType::ISetType_Count; ++iType) {
      const string& iTypeName = InteractionSet::kInteractionSetNames[iType];
      ofs << name << "," << "test," << iTypeName << "," << interactionTypeTestM[iType] << endl;
      ofs << name << "," << "train," << iTypeName << "," << interactionTypeTrainM[iType] << endl;
    }
    ofs.close();

    if (classifiers->doPostTestCallback) {
      (*classifiers->postTestCallbackFun)(name);
    }
  }
};

class SegmentJointsAggregatedClassifierManager: public ScenePoseClassifierManager {
public:
  SegmentJointsAggregatedClassifierManager(const string _name, Database& database): 
    ScenePoseClassifierManager(_name, database) {}

  void prepareDataset(const TrainOpts& opts) override {
    ScenePoseClassifierManager::prepareDataset(opts);
    // Make sure the correct per segment joint classifiers are loaded
    bool trainPerJointClassifiers = m_database.params->get<bool>("Dataset.trainPerJoint");
    if (trainPerJointClassifiers) {
      SG_LOG_INFO << "Training per joint classifiers";
      // Make sure the correct per segment joint classifiers are trained
      TrainOpts segJointTrainOpts(TrainOpts::RetrainLevel_UseLoaded, TrainOpts::RecreateDatasetLevel_UseLoaded, false);
      m_database.classifiers->trainClassifier("segmentPerJoint", segJointTrainOpts);
      m_database.classifiers->trainClassifier("segmentPerJointJoint", segJointTrainOpts);
    }
    m_database.classifiers->loadClassifier("segmentPerJoint", TrainOpts::RetrainLevel_UseLoaded);
    m_database.classifiers->loadClassifier("segmentPerJointJoint", TrainOpts::RetrainLevel_UseLoaded);
    makeSegmentJointsAggregatedDataset(m_database, opts);
  }

  bool load(TrainOpts::RetrainLevel forceRetrainLevel) override {
    bool hasError = false;
    auto& classifiers = m_database.classifiers;
    // Make sure the correct per segment joint classifiers are loaded
    classifiers->loadClassifier("segmentPerJoint", TrainOpts::RetrainLevel_UseLoaded);
    classifiers->loadClassifier("segmentPerJointJoint", TrainOpts::RetrainLevel_UseLoaded);

    for (auto& it : m_database.interactions.getAllInteractionSets()) {
      InteractionSet& vSet = it.second;
      vSet.segmentJointsAggregatedClassifier
        = classifiers->makeClassifier<SegmentJointsAggregatedClassifier>(name, vSet.id, vSet, forceRetrainLevel, 1.0);
      if (vSet.segmentJointsAggregatedClassifier == nullptr || !vSet.segmentJointsAggregatedClassifier->isValid()) {
          hasError = true;
      }
      // TODO(MS): Move this temporary hack into proper ClassifierDatabase instantiation
      vSet.segmentJointsLinearWeightedAggregatedClassifier = new SegmentJointsLinearWeightedAggregatedClassifier();
      vSet.segmentJointsLinearWeightedAggregatedClassifier->init(m_database, vSet);
      vSet.segmentJointsLinearWeightedAggregatedClassifier->id = "segmentJointsLWAggr"; 
    }
    return !hasError;
  }
  //! Set up my parameter variations
  vec<ParameterSetting> getParameterVariations() override {
    vec<ParameterSetting> settings;
    // iterate over useActivation, useJointSegScores, useJointSegFeatures (only if useJointSegScores)
    for (const bool useActivation:boolValues) {
      for (const bool useJointSegScores:boolValues) {
        ParameterSetting setting;
        setting.addParam<bool>("Dataset.useActivation", useActivation);
        setting.addParam<bool>("Dataset.useJointSegScores", useJointSegScores);
        if (useJointSegScores) {
          for (const bool useJointSegFeatures:boolValues) {
            ParameterSetting jsfSetting = setting;
            jsfSetting.addParam<bool>("Dataset.useJointSegFeatures", useJointSegFeatures);
            addSegmentFeatureSettings(&m_database, &settings, jsfSetting);
          }
        } else {
          setting.addParam<bool>("Dataset.useJointSegFeatures", false);
          settings.push_back(setting);
        }
      }
    }
    return settings;
  }
  ScenePoseClassifier* getScenePoseClassifier(const InteractionSet& interactionSet) const override { 
    return interactionSet.segmentJointsAggregatedClassifier; 
  }
  void saveWeights(const string& filename) override {
    load(TrainOpts::RetrainLevel_None);
    io::ensurePathToFileExists(filename);
    SG_LOG_INFO << "Outputting weights to " << filename;
    ofstream ofs(filename);
    vec<double> weights;
    // TODO: Get feature names
    for (auto& it : m_database.interactions.getAllInteractionSets()) {
      InteractionSet& vSet = it.second;
      ScenePoseClassifier* pClassifier = getScenePoseClassifier(vSet);
      if (pClassifier->getClassifierWeights(&weights)) {
        ofs << vSet.id << "," << util::join(weights, ",") << endl;
      }
    }
    ofs.close();
  }
};

class SegmentCentroidActivationClassifierManager : public ScenePoseClassifierManager {
public:
  SegmentCentroidActivationClassifierManager(const string& _name, Database& database): 
    ScenePoseClassifierManager(_name, database) { }
  void prepareDataset(const TrainOpts& opts) override {
    ScenePoseClassifierManager::prepareDataset(opts);
    makeSegmentCentroidActivationDataset(m_database, opts);
  }
  bool load(TrainOpts::RetrainLevel forceRetrainLevel) override {
    bool hasError = false;
    auto& classifiers = m_database.classifiers;
    for (auto& it : m_database.interactions.getAllInteractionSets()) {
      InteractionSet& vSet = it.second;
      vSet.segmentCentroidActivationAggregatedClassifier =
        classifiers->makeClassifier<SegmentCentroidActivationAggregatedClassifier>(
          name, vSet.id, vSet, forceRetrainLevel);
      if (vSet.segmentCentroidActivationAggregatedClassifier == nullptr || !vSet.segmentCentroidActivationAggregatedClassifier->isValid()) {
          hasError = true;
      }
    }
    return !hasError;
  }
  ScenePoseClassifier* getScenePoseClassifier(const InteractionSet& interactionSet) const override { 
    return interactionSet.segmentCentroidActivationAggregatedClassifier;
  }
  //! Set up my parameter variations
  vec<ParameterSetting> getParameterVariations() override {
    vec<ParameterSetting> settings;
    // iterate over useActivation
    for (const bool useActivation:boolValues) {
      ParameterSetting setting;
      setting.addParam<bool>("Dataset.useActivation", useActivation);
      addCentroidSettings(&m_database, &settings, setting);
    }
    return settings;
  }
};

class InteractionGraphKNNSimilarityClassifierManager : public ScenePoseClassifierManager {
public:
  InteractionGraphKNNSimilarityClassifierManager(const string& _name, Database& database)
    : ScenePoseClassifierManager(_name, database) { }
  void prepareDataset(const TrainOpts& opts) override { }
  bool load(TrainOpts::RetrainLevel forceRetrainLevel) override {
    for (auto& it : m_database.interactions.getAllInteractionSets()) {
      InteractionSet& vSet = it.second;
      if (!vSet.igKNNSimilarityClassifier) {
        vSet.igKNNSimilarityClassifier = std::make_shared<InteractionGraphKNNSimilarity>();
        vSet.igKNNSimilarityClassifier->init(m_database, vSet);
      }
    }
    return true;
  }
  ScenePoseClassifier* getScenePoseClassifier(const InteractionSet& interactionSet) const override {
    return interactionSet.igKNNSimilarityClassifier.get();
  }
};

void ClassifierDatabase::initClassifierManagers() {  
  Database& database = *const_cast<Database*>(&m_database);
  // Create new classifier managers
  m_classifierManagers.push_back(new SegmentPerJointClassifierManager("segmentPerJoint", database, false));
  m_classifierManagers.push_back(new SegmentPerJointClassifierManager("segmentPerJointJoint", database, true));
  m_classifierManagers.push_back(new SegmentJointsAggregatedClassifierManager("segmentJointsAggr", database));
//  m_classifierManagers.push_back(new SegmentPoseClassifierManager("segmentPose", database));
//  m_classifierManagers.push_back(new ScenePoseBowClassifierManager("scenePoseBoW", database));
  m_classifierManagers.push_back(new SegmentCentroidActivationClassifierManager("segmentCentroidActivation", database));
  m_classifierManagers.push_back(new InteractionGraphKNNSimilarityClassifierManager("IGKNNSim", database));

  // Create map of name to index
  for (size_t index = 0; index < m_classifierManagers.size(); index++) {
    const auto& manager = m_classifierManagers.at(index);
    m_classifierManagerIndices[manager->name] = index;  
  }
}

void ClassifierDatabase::updateCurrentTimeStampString() {
  m_currTimestampString = util::timeNowYMDHSstring();
}

string ClassifierDatabase::getCurrentWorkDir() const {
  return m_workDir + "/" + m_currTimestampString + "/";
}

void ClassifierDatabase::attachLog() {
  if (m_logAttachCount == 0) {
    util::add_file_log(getCurrentWorkDir() + "log_%Y%m%dT%H%M%S.txt" );
  }
  m_logAttachCount++;
}

void ClassifierDatabase::detachLog() { 
  m_logAttachCount--;
  if (m_logAttachCount <= 0) {
    util::remove_file_log(getCurrentWorkDir() + "log_%Y%m%dT%H%M%S.txt");
    m_logAttachCount = 0;
  }
}

const ScenePoseClassifier* ClassifierDatabase::getScenePoseClassifier(const string& clsType, const InteractionSet& interactionSet) const {
  if (clsType == "segmentJointsLWAggr") {
    return interactionSet.segmentJointsLinearWeightedAggregatedClassifier;
  }
  ClassifierManager* classifierManager = const_cast<ClassifierManager*>(getClassifierManager(clsType));
  if (classifierManager != nullptr) {
    classifierManager->load(TrainOpts::RetrainLevel::RetrainLevel_UseLoaded);
    return classifierManager->getScenePoseClassifier(interactionSet);
  } else {
    SG_LOG_ERROR << "Unknown classifier type " << clsType;
    return nullptr;
  }
}

void ClassifierDatabase::saveWeights(const string& clsType, const string& filename) {
  ClassifierManager* classifierManager = getClassifierManager(clsType);
  if (classifierManager != nullptr) {
    classifierManager->saveWeights(filename);
  } else {
    SG_LOG_ERROR << "Unknown classifier type " << clsType;
  }
}

string ClassifierDatabase::getClassifierDir() const {
  if (io::dirExists(m_classifierSet)) return m_classifierSet + "/";
  else return m_classifierDir + "/" + m_classifierSet + "/";
}

}  // namespace core
}  // namespace sg
