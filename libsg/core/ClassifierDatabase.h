#pragma once

#include "libsg.h"  // NOLINT
#include "core/Classifier.h"
#include "util/util.h"

namespace sg {
namespace core {

struct ParameterSetting {
  typedef std::function<void()> InitFunc;
  static const InitFunc doNothing;
  ParameterSetting() {
  }
  template <typename T>
  void addParam(const string name, const T val) {
    params.push_back(make_pair(name, boost::spirit::hold_any(val)));
  }
  void addInit(InitFunc initFunc) {
    initFuncs.push_back(initFunc);
  }
  void init(util::Params* overallParams) const {
    for (const auto& param : params) {
      overallParams->set(param.first, param.second);
    }
    for (const auto& initFunc: initFuncs) {
      initFunc();
    }
  }

  string name;
  vec<pair<const string, const boost::spirit::hold_any>> params;
  vec<InitFunc> initFuncs; 
};

//! Main classifier types
enum SceneGrokClassifierType {
  ClassifierType_ScenePose,
  ClassifierType_SegmentJoint,
  ClassifierType_Count
};

struct TrainOpts {
  //! forceRetrainLevel 
  //!  0 = retrain if needed
  //!  1 = retrain from dataset if not loaded
  //!  2 = retrain from dataset using prepared dataset
  enum RetrainLevel {
    RetrainLevel_None = 0,
    RetrainLevel_UseLoaded = 1,
    RetrainLevel_UseDataset = 2,
    RetrainLevel_Count = 3
  };

  //! recreateDataset 
  //!  0 = prepare dataset if needed
  //!  1 = prepare dataset if not loaded
  //!  2 = prepare dataset if not existing on disk
  enum RecreateDatasetLevel {
    RecreateDatasetLevel_None = 0,
    RecreateDatasetLevel_UseLoaded = 1,
    RecreateDatasetLevel_UseExisting = 2,
    RecreateDatasetLevel_Always = 3,
    recreateDatasetLevel_Count = 4
  };

  RetrainLevel forceRetrainLevel; 
  RecreateDatasetLevel recreateDatasetLevel;
  bool doTest;                //! Should we run test after train?
  string interactionSetType;  //! Just train for this interaction set type 

  TrainOpts()
    : forceRetrainLevel(RetrainLevel_UseDataset)
    , recreateDatasetLevel(RecreateDatasetLevel_Always)
    , doTest(false) { }

  explicit TrainOpts(bool doTest_)
    : forceRetrainLevel(RetrainLevel_UseDataset)
    , recreateDatasetLevel(RecreateDatasetLevel_Always)
    , doTest(doTest_) { }

  TrainOpts(RetrainLevel retrainLevel_, RecreateDatasetLevel recreateDatasetLevel_, bool doTest_)
    : forceRetrainLevel(retrainLevel_)
    , recreateDatasetLevel(recreateDatasetLevel_)
    , doTest(doTest_) { }
};

//! Type for post-test callback function which takes classifier type id as single argument
typedef std::function<void(const string&)> PostTestCallbackFun;

//! Manages the training of a classifier and loads and saves it into an appropriate places in the Database
class ClassifierManager {
 public:
  ClassifierManager(SceneGrokClassifierType _classifierType, const string& _name, Database& database)
    : name(_name), classifierType(_classifierType), m_database(database) { }
  virtual ~ClassifierManager() { }
    // ! Generates dataset and trains classifier
    void train(const TrainOpts& opts, bool doAllVariations) { 
      if (doAllVariations) { 
        trainAllVariations(opts); 
      } else { train(opts); }
    }
    virtual void train(const TrainOpts& opts);
    // ! Generates dataset and trains classifier on all variations
    virtual void trainAllVariations(const TrainOpts& opts);
    // ! Test my fabulous classifier and get great results, accumulates confusion matrices in confMatMap if passed in
    virtual void test(eval::BinaryConfusionMatrixSet* confMatMap = nullptr) = 0;
    // ! Tests all classifier variations
    virtual void testAllVariations();
    // ! Tests all classifier variations
    void test(bool doAllVariations) { 
      if (doAllVariations) { 
        testAllVariations(); 
      } else { test(); }
    }
    // ! Load classifier
    virtual bool load(TrainOpts::RetrainLevel forceRetrainLevel) = 0;
    // ! Prepare the dataset
    virtual void prepareDataset(const TrainOpts& opts) = 0;
    // ! Do leave one out cross validation for classifier
    void crossValidate(bool doAllVariations) { 
      if (doAllVariations) {
        crossValidateAllVariations();
      } else {
        crossValidate();
      }
    }
    virtual void crossValidate();
    // ! Do leave one out cross validation for classifier on all variations
    virtual void crossValidateAllVariations();
    // ! Parameter variations
    virtual vec<ParameterSetting> getParameterVariations();
    // ! Output weights
    virtual void saveWeights(const string& filename);

    //template<typename ClassifierT, typename ConditionT>
    //virtual ClassifierT* getClassifier(const ConditionT& condition) const { return nullptr; }

    virtual ScenePoseClassifier* getScenePoseClassifier(const interaction::InteractionSet& interactionSet) const {
      return nullptr;
    }

    const string name;
    SceneGrokClassifierType classifierType;
  protected:
    Database& m_database;
};

//! Database wrapping Classifier training, saving, loading and access
class ClassifierDatabase {
public:
  //! classifierDir = where the classifiers to be loaded are kept
  //! workDir = where to output files for this run
  //! classifierSet = what set of classifier to use (under classifierDir)
  //!   Directory structure underneath classifierDir/classifierSet
  //!     models   - where ".model" files are saved, 
  //!     datasets - where the dataset files used for generating the models are kept, 
  //!     results  - where evaluation results are saved
  ClassifierDatabase(const Database& database, 
                     const string& classifierDir, 
                     const string& workDir,
                     const string& classifierSet);
  ~ClassifierDatabase() {
    for (auto& it : m_classifiers) { if (it.second) { delete it.second; } }
    for (auto& manager : m_classifierManagers) { if (manager) { delete manager; } }
  }
  //! Creates underlying Weka classifier with id and conditioning type condition. Optional parameters biasToUniform and
  //! sampleSizePercent carry out biased resampling of the training dataset. Bias is [0,1] with 1 being uniform pos/neg,
  //! and sampleSizePercent is 0 to 100 percentage
  template<typename ClassifierT, typename ConditionT>
  ClassifierT* makeClassifier(const string& type, const string& simpleId, 
                              const ConditionT& condition, int forceRetrainLevel,
                              double biasToUniform = -1, double sampleSizePercent = -1) {
      // Get existing Classifier
      ClassifierT* pClassifier;
      const string id = getClassifierId(type, simpleId);
      if (forceRetrainLevel <= TrainOpts::RetrainLevel_UseLoaded && m_classifiers.count(id) > 0) {
        pClassifier = static_cast<ClassifierT*>(m_classifiers[id]);
      } else {
        bool forceRetrain = forceRetrainLevel > TrainOpts::RetrainLevel_None;
        // Create new Classifier
        pClassifier = new ClassifierT();
        pClassifier->init(m_database, condition);
        pClassifier->wekaClassifierType = m_params.get<string>("Classification.classifierType");
        pClassifier->wekaClassifierOptions = m_params.get<string>("Classification.classifierOptions");
        pClassifier->type = type;
        pClassifier->simpleId = simpleId;
        pClassifier->id = id;
        trainWekaClasser(pClassifier, forceRetrain, biasToUniform, sampleSizePercent);
        m_classifiers[id] = pClassifier;
      }
      return pClassifier;
  }

  template<typename ClassifierT>
  const ClassifierT* get(const string& id) const {
    auto classIt = m_classifiers.find(id);
    const bool found = classIt != m_classifiers.end();
    if (found) {
      return static_cast<ClassifierT*>(classIt->second);
    } else {
      cerr << "Tried to get unknown classifier with id=" + id << endl;
      return nullptr;
    }
  }

  template<typename ClassifierT>
  const ClassifierT* get(const string& type, const string& simpleId) const {
    string id = getClassifierId(type, simpleId);
    return get(id);
  }

  bool exists(const string& id) const {
    // Check if the classifier exists
    auto classIt = m_classifiers.find(id);
    const bool found = classIt != m_classifiers.end();
    return found;
  }

  bool exists(const string& type, const string& simpleId) const {
    string id = getClassifierId(type, simpleId);
    return exists(id);
  }

  //! load all classifiers
  void loadClassifiers(TrainOpts::RetrainLevel forceRetrainLevel);
  //! load classifier for the specified type
  void loadClassifier(const string& type, TrainOpts::RetrainLevel forceRetrainLevel);
  //! Train all classifiers
  void trainClassifiers(const TrainOpts& trainOpts = TrainOpts(true));
  //! Trains the classifier for the specified type
  void trainClassifier(const string& type, const TrainOpts& trainOpts = TrainOpts());
  void trainClassifier(const string& type, const TrainOpts& trainOpts, bool doAllVariations);
  //! Cross validate all the classifiers
  void crossValidateClassifiers();
  //! Cross validate the classifier for the specified type
  void crossValidateClassifier(const string& type);
  //! Test all the classifiers
  void testClassifiers();
  //! Test the classifier for the specified type
  void testClassifier(const string& type);

  //! Saves weights for the classifier type to the specified output file
  void saveWeights(const string& type, const string& filename);

  //! Runs train+test result reporting for classifiers with given base classifierID
  void reportTestResults(const string& type, const string& classifierID, const string& classifierModelType) const;

  // Helper functions for keeping track of where classifiers and their datasets are kept

  //! Returns the models directory for a classifier input type and m_params
  //string getModelsPath(const string& type, bool useClassifierDir) {
  //  if (useClassifierDir) {
  //    return m_classifierDir + getClassifierPath(type, "models");
  //  } else {
  //    return m_workDir + getClassifierPath(type, "models");
  //  }
  //}

  //! Returns the datasets base directory for a classifier input type and m_params
  string getDatasetsPath(const string& type, bool useClassifierDir) const {
    if (useClassifierDir) {
      return m_classifierDir + getClassifierPath(type, "datasets");
    } else {
      return m_workDir + getClassifierPath(type, "datasets");
    }
  }

  //! Returns the evaluation base directory for a classifier type and m_params
  string getEvalPath(const string& type) const {
    return getResultOutputPath(type);
  }

  //! Returns the result output directory for a classifier input type and m_params, and for specific test scanId
  string getResultOutputPath(const string& type) const {
    const string base = m_workDir + getClassifierPath(type, "results");
    return base;
  }

  //! Returns the classifier prefix based on the input type and m_params
  string getClassifierPrefix(const string& type) const;

  //! Returns the classifier path based on the input type and m_params
  string getClassifierPath(const string& type, const string& subdir) const;

  //! Get the classifier id given the classifier type and parameters
  string getClassifierId(const string& type, const string& simpleId) const;

  //! Get the classifer dir to use for loading classifiers
  string getClassifierDir() const;

  //! Get the current working dir
  string getCurrentWorkDir() const;

  string makeOutputDatasetDir(const string& datasetTypeId);

  //! Return current timestamp string to be used for identifying runs
  string getCurrentTimeStampString() const { return m_currTimestampString; }
  
  //! Update current timestamp string by resetting to current time
  void updateCurrentTimeStampString();

  //! Attach log
  void attachLog();

  //! Detach log
  void detachLog();

  //! Returns our classifier manager
  const ClassifierManager* getClassifierManager(const string& classifierType) const;
  ClassifierManager* getClassifierManager(const string& classifierType);

  //! Get a scan pose classifier for the given interactionSet
  const ScenePoseClassifier* getScenePoseClassifier(const string& clsType,
                                                    const interaction::InteractionSet& interactionSet) const;

  //! Test the weka classifier either on test or train set, and set confusion matrix in row-major order: (for 0=Negative,1=Positive, order is TN,FP,FN,TP)
  bool ClassifierDatabase::testWekaClasserOnTest(ClassifierBase* pClassifier, eval::BinaryConfusionMatrix* confM) const;
  bool ClassifierDatabase::testWekaClasserOnTrain(ClassifierBase* pClassifier, eval::BinaryConfusionMatrix* confM) const;

  //! This function gets called after every test run with the classifier type id as single argument
  PostTestCallbackFun* postTestCallbackFun;
  bool doPostTestCallback;

  //! Whether we are performing cross-validation runs TODO: Clean up this breakage of encapsulation
  bool doCrossValidation;

private:
  const Database& m_database;
  const util::Params& m_params;
  map<string, ClassifierBase*> m_classifiers;     //! Container for all classifiers: id -> Classifier
  map<string, string> m_classifierPrefixes;       //! Classifier prefix for type: type -> prefix
  vec<ClassifierManager*> m_classifierManagers;   //! Vector of classifier managers (order in which they should be trained)
  map<string, size_t> m_classifierManagerIndices; //! Classifier manager lookup by name: type -> index
  const string m_classifierDir;                   //! Directory where pretrained classifiers are stored
  const string m_workDir;                         //! Directory where new classifiers are stored
  const string m_classifierSet;                   //! Classifier set to load by default
  string m_currTimestampString;                   //! Timestamp string to use for all current runs

  int m_logAttachCount;  //! Count of number of times attach log was called

  //! Train the weka classifier
  void trainWekaClasser(ClassifierBase* pClassifier, bool forceRetrain,
                        double biasToUniform, double sampleSizePercent) const;

  //! Initializes the classifier managers
  void initClassifierManagers();
};

}  // namespace core
}  // namespace sg
