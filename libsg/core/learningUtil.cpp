#include "common.h"  // NOLINT

#include "core/learningUtil.h"

#include <functional>

#include "core/ClassifierDatabase.h"
#include "core/Database.h"
#include "core/Dataset.h"
#include "core/features.h"
#include "core/Recording.h"
#include "core/Scan.h"
#include "core/Skeleton.h"
#include "interaction/Interaction.h"
#include "io/io.h"
#include "segmentation/MeshSegment.h"
#include "util/Params.h"
#include "util/util.h"
#include "math/math.h"

using sg::interaction::InteractionSet;
using sg::interaction::InteractionSetType;

namespace sg {
namespace core {

//! Helper function for identifying if a skeleton belongs to a particular interactionSet
//! Returns true if tSkel has a interaction that belongs to interactionSet
//! TODO: Simplify now that a skeleton should only have one interaction
bool isSkeletonInInteractionSet(const SkeletonToInteractionsMap& skelToInteractions, 
                                const InteractionSet& interactionSet, 
                                const TransformedSkeleton& tSkel) {
  const auto& skelInteraction = skelToInteractions.at(tSkel.getOriginalSkeleton());
  bool interactionFound = false;
  switch (interactionSet.type) {
    case InteractionSetType::ISetType_Composite: {
        for (const auto& record : skelInteraction) {
          if (record.pInteraction->interactionSet->id == interactionSet.id) {
            interactionFound = true;
            break;
          }
        }
        break;
      }
    case InteractionSetType::ISetType_VerbNoun: {
        const auto& verbNoun = *interactionSet.verbNouns.begin();
        for (const auto& record : skelInteraction) {
          const auto& recVNs = record.pInteraction->interactionSet->verbNouns;
          if (recVNs.find(verbNoun) != recVNs.end()) {
            interactionFound = true;
            break;
          }
        }
        break;
      }
    case InteractionSetType::ISetType_Verb:
      const auto& verb = *interactionSet.verbs.begin();
      for (const auto& record : skelInteraction) {
        const auto& recVs = record.pInteraction->interactionSet->verbs;
        if (recVs.find(verb) != recVs.end()) {
          interactionFound = true;
          break;
        }
      }
      break;
  }
  return interactionFound;
};

//! Filter function to select training set instances out of datasets (its negation selects test set instances)
std::function<bool(const Dataset<TransformedSkeleton>::LabeledInstance&)> testSetFilterFun(const util::Params& params) {
  vec<string> testSceneIds = ml::util::split(params.get<string>("Dataset.testSetSceneIds"), ',');
  for (string& scanId : testSceneIds) { scanId = ml::util::removeChar(scanId, ' '); }
  SG_LOG_INFO << "Splitting dataset on testSceneIds: " << util::join(testSceneIds);
  const auto splitterFun = [=](const Dataset<TransformedSkeleton>::LabeledInstance& tSkelLabel) {
    const string& recId = tSkelLabel.first.getOriginalSkeleton()->rec->id;
    for (const string& scanId : testSceneIds) {
      const string recSceneId = ml::util::split(recId, '_')[0];
      if (recSceneId == scanId) { return true; }
    }
    return false;
  };
  return splitterFun;
}
//TODO(MS): This code duplication is extremely ugly!
std::function<bool(const Dataset<SegmentSkeletonJoint>::LabeledInstance&)> testSetFilterFun2(const util::Params& params) {
  vec<string> testSceneIds = ml::util::split(params.get<string>("Dataset.testSetSceneIds"), ',');
  for (string& scanId : testSceneIds) { scanId = ml::util::removeChar(scanId, ' '); }
  SG_LOG_INFO << "Splitting dataset on testSceneIds: " << util::join(testSceneIds);
  const auto splitterFun = [=](const Dataset<SegmentSkeletonJoint>::LabeledInstance& tSkelLabel) {
    const string& recId = tSkelLabel.first.skeleton.getOriginalSkeleton()->rec->id;
    for (const string& scanId : testSceneIds) {
      const string recSceneId = ml::util::split(recId, '_')[0];
      if (recSceneId == scanId) { return true; }
    }
    return false;
  };
  return splitterFun;
}

Dataset<TransformedSkeleton> createDataset(const SkeletonToInteractionsMap& skelToInteractions, const InteractionSet& vSet,
                                           const util::Params& params) {
    SG_LOG_INFO << "Creating training dataset for: " << vSet.id << "...";

    // Parameters
    const int negativeExampleCount = params.get<int>("Classification.negativeExampleCount");
    const int positiveExampleCount = params.get<int>("Classification.positiveExampleCount");
    const int perturbationExampleCount = (int)(params.get<double>("Classification.perturbationExamplePercent") * positiveExampleCount);

    // Create training set
    DatasetGenerator<TransformedSkeleton> trainingSetGen;
    vec<TransformedSkeleton> tSkels;
    tSkels.reserve(skelToInteractions.size());
    for (const auto& skelInter : skelToInteractions) {
      tSkels.push_back(TransformedSkeleton(*skelInter.first, ml::mat4f::identity()));
    }
    // Splitter function for positives/negatives
    namespace ph = std::placeholders;  // for _1, _2, _3 for binding future arguments
    auto posFilterFun = bind(isSkeletonInInteractionSet, skelToInteractions, vSet, ph::_1);

    Dataset<TransformedSkeleton>& trainingSet = trainingSetGen.createDataset(tSkels, posFilterFun);
    // Rebalance positives/negatives
    const auto& rebalanced
      = trainingSetGen.rebalancedInstances(trainingSet, positiveExampleCount, negativeExampleCount);
    trainingSet.clear();
    trainingSet.add(rebalanced);

    // Perturb for more negatives
    const auto& positives = trainingSet.positives();
    const Dataset<TransformedSkeleton>::LabeledInstanceXformFun& perturbFun = [] (const Dataset<TransformedSkeleton>::LabeledInstance& x) {
      // TODO: factor into parameter
      float perturbAngle = math::DEF_RAND.uniform_float_01() * 360.0f;
      float perturbRadius = 0.5f;
      ml::mat4f perturbation = ml::mat4f::translation(cosf(perturbAngle) * perturbRadius, sinf(perturbAngle) * perturbRadius, 0.0f);
      return std::make_pair(TransformedSkeleton(*x.first.getOriginalSkeleton(), perturbation), Dataset<TransformedSkeleton>::Label_NEG);
    };
    const auto& perturbed = trainingSetGen.perturbedInstances(positives, perturbationExampleCount, perturbFun);
    trainingSet.add(perturbed);

    return trainingSet;
}

Dataset<SegmentSkeletonJoint> createPerJointGroupTrainingSet(const SkeletonToInteractionsMap& skelToInteractions, 
                                                             const InteractionSet& vSet,
                                                             int iJointGroup,
                                                             const util::Params& params) {
    SG_LOG_INFO << "Creating training dataset for: interaction " << vSet.id << ", joint " << Skeleton::jointGroupName(iJointGroup);

    // Parameters
    const int negativeExampleCount = params.get<int>("Classification.negativeExampleCount");
    const int positiveExampleCount = params.get<int>("Classification.positiveExampleCount");

    // Create training set
    DatasetGenerator<SegmentSkeletonJoint> trainingSetGen;
    vec<TransformedSkeleton> tSkels;
    tSkels.reserve(skelToInteractions.size());
    for (const auto& skelInter : skelToInteractions) {
      tSkels.push_back(TransformedSkeleton(*skelInter.first, ml::mat4f::identity()));
    }
    // Joints for joint group
    const vec<size_t>& jointsForJointGroup = Skeleton::kJointGroupToJoints[iJointGroup];
    
    Dataset<SegmentSkeletonJoint> trainingSet;
    for (const TransformedSkeleton& tSkel : tSkels) {
      bool isPositive = isSkeletonInInteractionSet(skelToInteractions, vSet, tSkel);
      const InteractionRecord& snippet = skelToInteractions.at(tSkel.getOriginalSkeleton()).front();
      const auto& jointSegments = snippet.pInteraction->jointSegments[snippet.skelIndex];
      const auto& segsByJointGroup = Skeleton::groupSegmentsByJointGroup(jointSegments);
      const auto& segsForJointGroup = segsByJointGroup[iJointGroup];
      const auto& activeSegments = snippet.pInteraction->activeSegments;
      if (isPositive) {
        // If isPositive: Take segments belonging to other skeleton groups 
        // and not mine group and add them as negatives
        for (size_t iJoint:jointsForJointGroup) {
          const auto& segsForJoint = jointSegments[iJoint];
          // Add all as positives
          for (const auto& seg : segsForJoint) {
            SegmentSkeletonJoint ssj = {seg.get(), tSkel, iJoint};
            trainingSet.addPositive(ssj);
          }
        }
        const set<std::shared_ptr<segmentation::MeshSegment> >
          distinctSegsByJointGroup(segsForJointGroup.begin(), segsForJointGroup.end());
        for (const auto& seg : activeSegments) {
          if (!distinctSegsByJointGroup.count(seg)) {
            size_t i = static_cast<size_t>(floor(math::DEF_RAND.uniform_float_01()*jointsForJointGroup.size()));
            size_t iJoint = jointsForJointGroup[i];
            SegmentSkeletonJoint ssj = {seg.get(), tSkel, iJoint};
            trainingSet.addNegative(ssj);
          }
        }
      } else {
        // Otherwise, take all segments and add them as negative examples
        // Pick randomly from our joints
        for (const auto& seg:activeSegments) {
          size_t i = static_cast<size_t>(floor(math::DEF_RAND.uniform_float_01()*jointsForJointGroup.size()));
          size_t iJoint = jointsForJointGroup[i];
          SegmentSkeletonJoint ssj = {seg.get(), tSkel, iJoint};
          trainingSet.addNegative(ssj);
        }
      }
    }

    // Rebalance positives/negatives
    const auto& rebalanced
      = trainingSetGen.rebalancedInstances(trainingSet, positiveExampleCount, negativeExampleCount);
    trainingSet.clear();
    trainingSet.add(rebalanced);

    return trainingSet;
}

void printSegSkelCSVRowString(const SkeletonToInteractionsMap& interactionSnippets,
                              const map<const MeshSegment*, vecd>* segFeats,
                              const SegmentSkelFeatureGenerator* segPoseFeatGen,
                              const pair<TransformedSkeleton, int> instance,
                              bool printJointGroup, int selectedJointGroupIndex, ofstream& ofs) {
  //
  // What's happening here is a bit tricky. An interaction is sort of a cache
  // for a set of "close" segments. A single skeleton might map to multiple interactions,
  // but for the active set of segments, we are just taking the first interaction.
  //
  const TransformedSkeleton& tSkel = instance.first;
  const InteractionRecord& snippet = interactionSnippets.at(tSkel.getOriginalSkeleton()).front();
  const auto& jointSegments = snippet.pInteraction->jointSegments[snippet.skelIndex];
  const auto& segsByJointGroup = Skeleton::groupSegmentsByJointGroup(jointSegments);
  for (size_t iJointGroup = 0; iJointGroup < segsByJointGroup.size(); iJointGroup++) {
    if (selectedJointGroupIndex > 0 && selectedJointGroupIndex != iJointGroup) { continue; }  // Skip groups not selected for output
    const auto& segs = segsByJointGroup[iJointGroup];
    if (segs.empty() || segs[0] == nullptr) { continue; }
    ofs << instance.second;  // class
    if (printJointGroup) { ofs << "," << Skeleton::jointGroupName(iJointGroup); }
    if (segFeats) {  // segment features
      for (double f : segFeats->at(segs[0].get())) { ofs << "," << f; }
    }
    if (segPoseFeatGen) {  // pose-segment features
      for (double f : segPoseFeatGen->generate(SegmentSkeletonPair(segs[0].get(), &tSkel))) { ofs << "," << f; }
    }
    ofs << endl;
  }
}

void printSegSkelCSVRowString(const map<const MeshSegment*, vecd>* segFeats,
                              const SegmentSkelJointFeatureGenerator* segPoseFeatGen,
                              const pair<SegmentSkeletonJoint, int> instance,
                              bool printJointGroup, int selectedJointGroupIndex, ofstream& ofs) {
  const TransformedSkeleton& tSkel = instance.first.skeleton;
  const MeshSegment* seg = instance.first.segment;
  ofs << instance.second;  // class
  if (printJointGroup) { ofs << "," << Skeleton::jointGroupName(selectedJointGroupIndex); }
  if (segFeats) {  // segment features
    for (double f : segFeats->at(seg)) { ofs << "," << f; }
  }
  if (segPoseFeatGen) {  // pose-segment features
    for (double f : segPoseFeatGen->generate(instance.first)) { ofs << "," << f; }
  }
  ofs << endl;
}

void printSegSkelCSVHeaderString(const SegmentFeatureGenerator* segGenerator, 
                                 const SegmentSkelJointFeatureGenerator* segSkelGenerator,
                                 bool printJointIndex, ofstream& ofs) {
  ofs << "class";
  if (printJointIndex) { ofs << ",jointIndex"; }
  if (segGenerator) { for (const string& s : segGenerator->fieldNames()) { ofs << "," << s; } }
  if (segSkelGenerator) { for (const string& s : segSkelGenerator->fieldNames()) { ofs << "," << s; } }
  ofs << endl;
}

void dumpPerJointInstancesToCSVFile(const util::Params& params,
                                    const Dataset<SegmentSkeletonJoint>& dataset,
                                    const InteractionSet& vSet,
                                    const map<const MeshSegment*, vecd>& segFeats,
                                    const SegmentFeatureGenerator& segGenerator,
                                    const SegmentSkelJointFeatureGenerator& jointSegGenerator,
                                    const string& datasetDir,
                                    bool useJointFeatures, bool printjointGroupIndex, int iJointGroup) {
  // Open file and write header
  string filebase = datasetDir + "/" + vSet.id;
  filebase += "_" + Skeleton::jointGroupName(iJointGroup);
  const string& trainFilename = filebase + "_train.csv";
  const string& testFilename = filebase + "_test.csv";
  SG_LOG_INFO << "Creating dataset files: " << trainFilename;
  SG_LOG_INFO << "Creating dataset files: " << testFilename;
  io::ensurePathToFileExists(trainFilename);
  io::ensurePathToFileExists(testFilename);
  ofstream trainFile(trainFilename);
  ofstream testFile(testFilename);
  const SegmentSkelJointFeatureGenerator* pJointSegGenerator = (useJointFeatures) ? &jointSegGenerator : nullptr;
  printSegSkelCSVHeaderString(&segGenerator, pJointSegGenerator, printjointGroupIndex, trainFile);
  printSegSkelCSVHeaderString(&segGenerator, pJointSegGenerator, printjointGroupIndex, testFile);

  const auto& trainSet = dataset.filteredInstances(not1(testSetFilterFun2(params)));
  const auto& testSet = dataset.filteredInstances(testSetFilterFun2(params));

  // Print row per instance
  size_t iRow = 0;
  for (const auto& instance : trainSet) {
    printSegSkelCSVRowString(&segFeats, pJointSegGenerator, instance, printjointGroupIndex, iJointGroup, trainFile);
    if (iRow % 1000 == 0) { SG_LOG_INFO  << iRow << "/" << dataset.size(); }
    iRow++;
  }
  trainFile.close();
  for (const auto& instance : testSet) {
    printSegSkelCSVRowString(&segFeats, pJointSegGenerator, instance, printjointGroupIndex, iJointGroup, testFile);
    if (iRow % 1000 == 0) { SG_LOG_INFO  << iRow << "/" << dataset.size(); }
    iRow++;
  }
  testFile.close();

  // Convert to ARFF
  SG_LOG_INFO << "Convert to ARFF " << trainFilename;
  const string trainARFFfile = CSVtoARFF(trainFilename);
  //io::copyFile(trainARFFfile, datasetDir + trainARFFfile);
  SG_LOG_INFO << "Convert to ARFF " << testFilename;
  const string testARFFfile = CSVtoARFF(testFilename);
  //io::copyFile(testARFFfile, datasetDir + testARFFfile);
};

void printInteractingSkelCSVHeaderString(const SceneSkelFeatureGenerator& featGenerator, 
                                         ofstream& ofs) {
  ofs << "class";
  for (const string& s : featGenerator.fieldNames()) { ofs << "," << s; }
  ofs << endl;
}

void printInteractingSkelCSVRowString(const SkeletonToInteractionsMap& interactionSnippets,
                                      const SceneSkelFeatureGenerator& featGenerator,
                                      const pair<TransformedSkeleton, int> instance,
                                      ofstream& ofs) {
  const TransformedSkeleton& tSkel = instance.first;
  const InteractionRecord& snippet = interactionSnippets.at(tSkel.getOriginalSkeleton()).front();
  const Skeleton::SegmentsByJointPlusGaze& segmentsByJoint = snippet.pInteraction->jointSegments[snippet.skelIndex];
  const SceneSkel sceneSkel = { nullptr, &tSkel, &segmentsByJoint };  // NOTE: nullptr for scan since not needed
  ofs << instance.second;  // class
  for (double f : featGenerator.generate(sceneSkel)) { ofs << "," << f; }
  ofs << endl;
}

void dumpToCSVFile(const vec<Dataset<TransformedSkeleton>::LabeledInstance>& instances, 
                   const InteractionSet& vSet,
                   const SkeletonToInteractionsMap& interactionSnippets,
                   const SceneSkelFeatureGenerator& featGenerator,
                   const string& datasetsDir, const string& fileBasename) {
  // Open file and write header
  const string& filename = datasetsDir + "/" + fileBasename + ".csv";
  SG_LOG_INFO << "Creating dataset files: " << filename;
  io::ensurePathToFileExists(filename);
  ofstream csvFile(filename);
  printInteractingSkelCSVHeaderString(featGenerator, csvFile);

  // Print row per instance
  size_t iRow = 0;
  for (const auto& instance : instances) {
    printInteractingSkelCSVRowString(interactionSnippets, featGenerator, instance, csvFile);
    if (iRow % 1000 == 0) { SG_LOG_INFO << iRow << "/" << instances.size(); }
    iRow++;
  }
  // Close, convert to ARFF and copy to datasetsDir
  csvFile.close();
  SG_LOG_INFO << "Convert to ARFF " << filename;
  const string& arffFile = CSVtoARFF(filename);
  //io::copyFile(arffFile, datasetsDir + arffFile);
};

void dumpDatasets(const Database& database) {
  SG_LOG_INFO << "Dumping datasets...";
  const auto &params = *database.params;
  const string& datasetIds = params.get<string>("Dataset.datasetsToDump");
  for (const string& datasetId : ml::util::split(datasetIds, ',')) {
    const string id = ml::util::removeChar(datasetId, ' ');
    SG_LOG_INFO << "DatasetId=" << id;
    //if (id == "scenePoseBoW") { makeScenePoseBoWDataset(database); }
    //if (id == "segmentPerJoint") { makeSegmentPerJointDataset(database, false); }
    //if (id == "segmentPerJointJoint") { makeSegmentPerJointDataset(database, true); }
    if (id == "features") { makeFeaturesFiles(database); }
    if (id == "activeSegments") { database.saveAllActiveSegments(params.get<string>("workDir") + "features/activeSegments.csv"); }
    //if (id == "segmentJointsAggr") { makeSegmentJointsAggregatedDataset(database); }
    //if (id == "segmentCentroidActivation") { makeSegmentCentroidActivationDataset(database); }
  }
}

void makeSegmentPerJointDataset(const Database& database, const TrainOpts& trainOpts, bool useJointFeatures) {
  // Load database, create datasets dir, initialize feature generators
  const util::Params& params = *database.params;
  const bool printjointGroupIndex = params.get<bool>("Dataset.printJointIndex");
  const string datasetTypeId = (useJointFeatures)? "segmentPerJointJoint":"segmentPerJoint";
  const string datasetDir = database.classifiers->makeOutputDatasetDir(datasetTypeId);

  const string featureTypeName = params.get<string>("Dataset.segmentPerJointFeatures");
  const SegmentFeatureGenerator* pSegGenerator = database.getSegmentFeatureGenerator(SegmentFeatureGenerator::getFeatureType(featureTypeName));
  SegmentJointFeatureGeneratorBasic jointSegGenerator;

  // Map from Skeleton to vector of (Interaction,iSkel) records
  const auto interactionSnippets = database.getSkeletonInteractionRecords();

  // Compute and cache segment features
  const auto segFeats = database.precomputeSegmentFeatures(*pSegGenerator);

  // dump database.interactionSets into a simple list because OpenMP can't handle C++11 for loops
  vec<const InteractionSet*> interactionSets;
  database.interactions.getAllInteractionSets(&interactionSets);

//  #pragma omp parallel for
  for (int iInteractionSet = 0; iInteractionSet < interactionSets.size(); iInteractionSet++) {
    const InteractionSet& interactionSet = *interactionSets[iInteractionSet];
    for (int iJointGroup = 0; iJointGroup < Skeleton::kNumJointGroups; iJointGroup++) {
      if (trainOpts.recreateDatasetLevel <= TrainOpts::RecreateDatasetLevel_UseLoaded) {
        // Check if classifier exists - if so, skip creating of dataset
        if (database.classifiers->exists(datasetTypeId, interactionSet.id + "_" + Skeleton::jointGroupName(iJointGroup))) {
          continue;
        }
      }
      if (trainOpts.recreateDatasetLevel <= TrainOpts::RecreateDatasetLevel_UseExisting) {
        // Check if there is existing dataset... (TODO: check other files too)
        if (io::fileExists(datasetDir + "/" + interactionSet.id + "_" + Skeleton::jointGroupName(iJointGroup)+ "_train.arff")) {
          continue;
        }
      }
      const auto& trainingSet = createPerJointGroupTrainingSet(interactionSnippets, interactionSet, iJointGroup, params);
      if (trainingSet.positives().empty() || trainingSet.negatives().empty()) {
        SG_LOG_ERROR << "Warning, no positive or negative instances for training set of: " << interactionSet.id;
        continue;
      }

      dumpPerJointInstancesToCSVFile(params, trainingSet, interactionSet, segFeats, *pSegGenerator, jointSegGenerator,
        datasetDir, useJointFeatures, printjointGroupIndex, iJointGroup);
    }
  }
}

void dumpDataset(const Database& database, const InteractionSet& vSet, const SceneSkelFeatureGenerator& featGenerator,
                 const string& datasetDir) {
  const util::Params& params = *database.params;

  // Map from Skeleton to vector of (Interaction,iSkel) records
  const auto interactionSnippets = database.getSkeletonInteractionRecords();

  // Create dataset and split train/test
  const auto& dataset = createDataset(interactionSnippets, vSet, params);
  if (dataset.positives().empty() || dataset.negatives().empty()) {
    SG_LOG_ERROR << "Warning, no positive or negative instances for training set of: " << vSet.id;
    return;
  }
  const auto& trainSet = dataset.filteredInstances(not1(testSetFilterFun(params)));
  const auto& testSet = dataset.filteredInstances(testSetFilterFun(params));

  // Dump to files
  dumpToCSVFile(dataset.instances(), vSet, interactionSnippets, featGenerator, datasetDir, vSet.id);
  dumpToCSVFile(trainSet, vSet, interactionSnippets, featGenerator, datasetDir, vSet.id + "_train");
  dumpToCSVFile(testSet, vSet, interactionSnippets, featGenerator, datasetDir, vSet.id + "_test");
}

void makeSegmentCentroidActivationDataset(const Database& database, const TrainOpts& trainOpts) {
  const string& classifierTypeName = "segmentCentroidActivation";
  const string datasetDir = database.classifiers->makeOutputDatasetDir(classifierTypeName);
  const bool splitCentroidsPerJointGroup = database.params->get<bool>("Clustering.splitCentroidsPerJointGroup");
  const bool useActivation = database.params->get<bool>("Dataset.useActivation");
  for (const auto& it : database.interactions.getAllInteractionSets()) {
    const InteractionSet& vSet = it.second;
    if (trainOpts.recreateDatasetLevel <= TrainOpts::RecreateDatasetLevel_UseLoaded) {
      // Check if classifier exists - if so, skip creating of dataset
      if (database.classifiers->exists(classifierTypeName, vSet.id)) {
        continue;
      }
    }
    if (trainOpts.recreateDatasetLevel <= TrainOpts::RecreateDatasetLevel_UseExisting) {
      // Check if there is existing dataset... (TODO: check other files too)
      if (io::fileExists(datasetDir + "/" + vSet.id + "_train.arff")) {
        continue;
      }
    }
    SceneSkelCentroidActivationFeatGen fg(vSet, database.centroids, splitCentroidsPerJointGroup, useActivation);
    dumpDataset(database, vSet, fg, datasetDir);
  }
}

void makeSegmentJointsAggregatedDataset(const Database& database, 
                                        const TrainOpts& trainOpts) {
  const string& classifierTypeName = "segmentJointsAggr";
  // Load database, create datasets dir, initialize feature generators
  const util::Params& params = *database.params;
  const string datasetDir = database.classifiers->makeOutputDatasetDir(classifierTypeName);
  const bool useActivation = params.get<bool>("Dataset.useActivation");
  const bool useJointSegScores = params.get<bool>("Dataset.useJointSegScores");
  const bool useJointSegFeatures = params.get<bool>("Dataset.useJointSegFeatures");

  // Map from Skeleton to vector of (Interaction,iSkel) records
  const auto interactionSnippets = database.getSkeletonInteractionRecords();

  // dump database.interactionSets into a simple list because OpenMP can't handle C++11 for loops
  vec<const InteractionSet*> interactionSets;
  database.interactions.getAllInteractionSets(&interactionSets);

//  #pragma omp parallel for
  for (int iInteractionSet = 0; iInteractionSet < interactionSets.size(); iInteractionSet++) {
    const InteractionSet& interactionSet = *interactionSets[iInteractionSet];
    if (trainOpts.recreateDatasetLevel <= TrainOpts::RecreateDatasetLevel_UseLoaded) {
      // Check if classifier exists - if so, skip creating of dataset
      if (database.classifiers->exists(classifierTypeName, interactionSet.id)) {
        continue;
      }
    }
    if (trainOpts.recreateDatasetLevel <= TrainOpts::RecreateDatasetLevel_UseExisting) {
      // Check if there is existing dataset... (TODO: check other files too)
      if (io::fileExists(datasetDir + "/" + interactionSet.id + "_train.arff")) {
        continue;
      }
    }
    const auto& dataset = createDataset(interactionSnippets, interactionSet, params);
    if (dataset.positives().empty() || dataset.negatives().empty()) {
      SG_LOG_ERROR << "Warning, no positive or negative instances for training set of: " << interactionSet.id;
      continue;
    }

    const auto& trainSet = dataset.filteredInstances(not1(testSetFilterFun(params)));
    const auto& testSet = dataset.filteredInstances(testSetFilterFun(params));

    SceneSkelJointAggrFeatureGeneratorBasic featGenerator(interactionSet, useActivation, useJointSegScores, useJointSegFeatures);
    dumpToCSVFile(dataset.instances(), interactionSet, interactionSnippets, featGenerator, datasetDir, interactionSet.id);
    dumpToCSVFile(trainSet, interactionSet, interactionSnippets, featGenerator, datasetDir, interactionSet.id + "_train");
    dumpToCSVFile(testSet, interactionSet, interactionSnippets, featGenerator, datasetDir, interactionSet.id + "_test");
  }
}

void makeFeaturesFiles(const Database& database) {
  // Load database and initialize local objects and params
  const util::Params& params = *database.params;
  SegmentFeatureGeneratorSimplistic geoGenerator;
  SegmentSkelFeatureGeneratorBasic jointSegPoseGenerator;

  const string featsDir = params.get<string>("workDir") + "features/";
  io::ensureDirExists(featsDir);

  // Compute and cache segment features
  const map<const MeshSegment*, vecd>& segFeats = database.precomputeSegmentFeatures(geoGenerator);

  // dump database.interactionSets into a simple list because OpenMP can't handle C++11 for loops
  vec<const InteractionSet*> interactionSets;
  database.interactions.getAllInteractionSets(&interactionSets);

//  #pragma omp parallel for
  for (int iInteractionSet = 0; iInteractionSet < interactionSets.size(); iInteractionSet++) {
    const InteractionSet& interactionSet = *interactionSets[iInteractionSet];
    SG_LOG_INFO << "Creating features for: " << interactionSet.id;

    // Open file and write header
    const string filename = featsDir + interactionSet.id + ".csv";
    SG_LOG_INFO << "Outputing to: " << filename;
    io::ensurePathToFileExists(filename);
    ofstream geoFile(filename);
    geoFile << "recId,skelId,class,jointIndex";
    for (const string& s : geoGenerator.fieldNames()) { geoFile << "," << s; }
    for (const string& s : jointSegPoseGenerator.fieldNames()) { geoFile << "," << s; }
    geoFile << endl;

    size_t numProcessedSkels = 0;
    for (const interaction::Interaction* in : interactionSet.interactions) {
      for (size_t iSkel = 0; iSkel < in->skelRange.size(); iSkel++, numProcessedSkels++) {
        const Skeleton& skel = in->skelRange[iSkel];
        const TransformedSkeleton tSkel(skel, ml::mat4f::identity());
        const auto& jointSegments = in->jointSegments[iSkel];

        for (size_t iJoint = 0; iJoint < jointSegments.size(); iJoint++) {
          const auto& segs = jointSegments[iJoint];
          if (segs.empty() || segs[0] == nullptr) { continue; }
          geoFile << in->recording->id << "," << skel.timestamp << ",";
          geoFile << in->interactionSet->id << "," << iJoint;

          for (double f : segFeats.at(segs[0].get())) { geoFile << "," << f; }
          for (double f : jointSegPoseGenerator.generate(SegmentSkeletonPair(segs[0].get(), &tSkel))) { geoFile << "," << f; }
          geoFile << endl;
        }
      }
      if (numProcessedSkels % 1000 == 0) { SG_LOG_INFO << numProcessedSkels; }
    }
  }
}

}  // namespace core
}  // namespace sg
