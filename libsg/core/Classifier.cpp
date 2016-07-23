#include "common.h"  // NOLINT

#include "core/Classifier.h"

#include "core/Database.h"
#include "core/features.h"
#include "core/OccupancyGrid.h"
#include "core/Scan.h"
#include "core/Skeleton.h"
#include "geo/voxels.h"
#include "interaction/InteractionSet.h"
#include "io/io.h"
#include "segmentation/MeshSegment.h"
#include "util/Params.h"

#include "../jace-proxy/wekautil.h"

using sg::interaction::InteractionSet;

namespace sg {
namespace core {

string CSVtoARFF(const string& csvFile) {
  const string& arffFile = io::replaceExtension(csvFile, "arff");
  wekautil::common::CSVtoARFF(csvFile, arffFile);
  return arffFile;
}

ClassifierBase::ClassifierBase() { pWekaClasser = nullptr; }

ClassifierBase::~ClassifierBase() { if (pWekaClasser != nullptr) { delete pWekaClasser; } }

bool ClassifierBase::acceptsHeader(const string& header) const { return pWekaClasser->headerMatches(header); }

double ClassifierBase::classify(const vecd& feats) const {
  if (!isValid()) {
    return 0.0;  // TODO(MS): Do something more intelligent when underlying classifier is not available?
  }

  try {
    JNIEnv* env = jace::attach();
    jdoubleArray featsArr = env->NewDoubleArray(static_cast<jsize>(feats.size() + 1));  // Note +1 for class variable
    jsize featsSize = static_cast<jsize>(feats.size());
    env->SetDoubleArrayRegion(featsArr, 1, featsSize, &feats[0]);
    const double pPositive = pWekaClasser->classify(featsArr);
    env->DeleteLocalRef(featsArr);

    return pPositive;
  } catch (...) {
    wekautil::handleJaceExceptions();
    return 0.0;
  }
}

bool ClassifierBase::getClassifierWeights(vec<double>* pWeights) const {
  if (!isValid()) {
    return false;
  }

  bool ok = wekautil::getClassifierWeights(pWekaClasser, pWeights);
  return ok;
}

bool ClassifierBase::test(const string& testInstances, const string& reportFile, eval::BinaryConfusionMatrix* confMatrix) const {
  if (!isValid()) {
    return false;
  }

  try {
    /*JNIEnv* env =*/ jace::attach();
    jace::JArray<jace::proxy::types::JDouble> m(4);
    const string trainInstances = ml::util::replace(testInstances, "_test.", "_train.");
    pWekaClasser->test(trainInstances, testInstances, reportFile, m);
    (*confMatrix)[0] = m[0];
    (*confMatrix)[1] = m[1];
    (*confMatrix)[2] = m[2];
    (*confMatrix)[3] = m[3];
    return true;
  } catch (...) {
    wekautil::handleJaceExceptions();
    return false;
  }
}

void InteractionGraphKNNSimilarity::init(const Database& database, const InteractionSet& interactionSet) {
  id = "IGKNNSim";
  m_database = &database;
  m_pInteractionSet = &interactionSet;
  const auto& p = *m_database->params;
  m_NNpercent = p.get<float>("Interaction.igSimNNpercent");
  m_maxNumNNs = p.get<int>("Interaction.igSimNNmaxNumNNs");
  m_maxNumNNs = p.get<int>("Interaction.igSimNNmaxNumNNs");
  const string simType = p.get<string>("Interaction.igSimType");
  if (simType == "PIGIG") {
    m_simType = SimType_PIGIG;
  } else if (simType == "PIGIGW") {
    m_simType = SimType_PIGIGW;
    cout << "PIGIGW weights: ";
    for (const auto& w :interactionSet.jointGroupWeights) { cout << w << ","; } cout << endl;
  } else if (simType == "IFSUP") {
    m_simType = SimType_IFSUP;
  } else if (simType == "IGKNN") {
    m_simType = SimType_IGKNN;
  } else if (simType == "Manual") {
    m_simType = SimType_Manual;
  } else {
    SG_LOG_ERROR << "[InteractionGraphKNNSimilarity] Unknown similarity type '" << simType
                 << "'. Check Interaction.igSimType param";
  }
}

double InteractionGraphKNNSimilarity::classify(const SceneSkel& sceneSkel) const {
  const Skeleton& skel = sceneSkel.tSkel->getSkeleton();

  const int numSkelPts = 1000;
  const vec<ml::vec3f>& skelPts = skel.getPointsInOBBs(numSkelPts);
  const geo::Matrix3Xf skelPtsM = geo::Matrix3Xf::Map(&skelPts[0][0], 3, skelPts.size());
  const OccupancyGrid& occGrid = sceneSkel.scene->getOccupancyGrid();
  const ml::BinaryGrid3& sceneVoxels = occGrid.unknownOrOccupied();
  const geo::Transform worldToSceneVoxelCoords = geo::from(occGrid.worldToGrid());
  const geo::Matrix3Xf skelPtsMvoxelCoords = worldToSceneVoxelCoords * skelPtsM;

  // bail out if there's no static support for this Skel
  const float supportDistThresh = 0.2f;
  const float distToSupport = skel.avgDistToSupport(*sceneSkel.scene);
  if (distToSupport > supportDistThresh) { return 0.f; }

  // bail out if skel is colliding significantly with scene
  const float overlap = geo::ratioPointsInVoxels(skelPtsMvoxelCoords, sceneVoxels);
  const float overlapThresh = 0.2f;
  if (overlap > overlapThresh) { return 0.f; }

  // place skel on nearest support
  //Skeleton unfloatedSkel = skel;
  //unfloatedSkel.unfloat(*sceneSkel.scene);

  // create IG for current skel and invoke appropriate similarity method
  std::shared_ptr<interaction::InteractionGraph> pIG =
    m_database->interactionFactory.createInteractionGraph(*sceneSkel.scene, skel, segmentation::kPartSegment, *sceneSkel.activatedSegsByJoint);
  interaction::InteractionGraph& ig = *pIG;

  float sim = 0.f;
  switch (m_simType) {
  case SimType_IFSUP:
    sim = static_cast<float>(m_pInteractionSet->protoInteractionFrame->support(skel, sceneSkel.scene));
    break;
  case SimType_IGKNN:
    sim = m_pInteractionSet->similarity(ig, m_NNpercent, m_maxNumNNs);
    break;
  case SimType_PIGIG:
    sim = m_pInteractionSet->protoSimilarity(ig, false);
    break;
  case SimType_PIGIGW:
    sim = m_pInteractionSet->protoSimilarity(ig, true);
    break;
  case SimType_Manual: {
    const auto& segsByJointGroup = Skeleton::groupSegmentsByJointGroup(*sceneSkel.activatedSegsByJoint);
    const auto& hipSegs = segsByJointGroup[Skeleton::JointGroup_Hips];
    const auto& gazeSegs = segsByJointGroup[Skeleton::JointGroup_Gaze];
    if (hipSegs.size() && gazeSegs.size()) {
      const auto& seg = (*gazeSegs[0]);
      const float normZ = fabsf(seg.dominantNormal().z);
      const float diag = seg.diagonalLength();
      const bool bigVerticalSurf = normZ < 0.1f && diag > 0.5f;
      //const bool smallHorizSurf = normZ > 0.7f && seg.diagonalLength() < 0.6f;
      sim = bigVerticalSurf ? 1.f : 0.5f;
    }}
    break;
  case SimType_Count:
    break;
  default:
    break;
  }

  return sim;
}

double ScenePoseClassifierAggregatedSegments::classify(const SceneSkel& sceneSkel) const {
  vecd likelihoods;
  for (auto& seg : *sceneSkel.scene->segments) {
    // TODO(ms): Replace with Database-loaded classifier
    double likelihood = m_pInteractionSet->segmentPoseClassifier->classify(SegmentSkeletonPair(seg.get(), sceneSkel.tSkel));
    likelihoods.push_back(likelihood);
  }
  sort(likelihoods.begin(), likelihoods.end(), std::greater<double>());  // Sort in descending likelihood order

  // TODO: factor into parameter
  const size_t aggregateCount = 10;
  double sum = 0.0;
  for (size_t i = 0; i < std::min(likelihoods.size(), aggregateCount); i++) {
    sum += likelihoods[i];
  }
  return sum / static_cast<double>(aggregateCount);
}

double SegmentPoseClassifier::classify(const SegmentSkeletonPair& segSkelPair) const {
  SegmentSkelFeatureGeneratorBasic jointGen;
  SegmentFeatureGeneratorSimplistic geoGen;
  vecd feats = geoGen.generate(*segSkelPair.first);
  const vecd jointFeats = jointGen.generate(segSkelPair);
  feats.insert(feats.end(), jointFeats.begin(), jointFeats.end());
  return ClassifierBase::classify(feats);
}

ScenePoseClassifierBoW::~ScenePoseClassifierBoW() {
  if (m_gen) { delete m_gen; }
}

void ScenePoseClassifierBoW::init(const Database& database, const InteractionSet& interactionSet) {
  m_database = &database;
  m_interactionSet = &interactionSet;
  m_gen = new SceneSkelCentroidActivationFeatGen(*m_interactionSet, m_database->centroids,
    m_database->params->get<bool>("Clustering.splitCentroidsPerJointGroup"),
    m_database->params->get<bool>("Dataset.useActivation"));
}

double ScenePoseClassifierBoW::classify(const SceneSkel& pair) const {
  vecd feats = m_gen->generate(pair);
  return ClassifierBase::classify(feats);
}

void SegmentPerJointGroupClassifier::init(const Database& database, const pair<const InteractionSet&, size_t>& interactionSetJointGroup) {
  m_pInteractionSet = &interactionSetJointGroup.first;
  m_jointGroupIndex = interactionSetJointGroup.second;
  const string featureTypeName = database.params->get<string>("Dataset.segmentPerJointFeatures");
  m_featureType = SegmentFeatureGenerator::getFeatureType(featureTypeName);
  pFeatGenerator = database.getSegmentFeatureGenerator(m_featureType);
}

double SegmentPerJointGroupClassifier::classify(const MeshSegment& seg) const {
  vecd feats = pFeatGenerator->generate(seg);
  return ClassifierBase::classify(feats);
}

void SegmentSkelPerJointGroupClassifier::init(const Database& database, const pair<const InteractionSet&, size_t>& interactionSetJointGroup) {
  m_pInteractionSet = &interactionSetJointGroup.first;
  m_jointGroupIndex = interactionSetJointGroup.second;
  const string featureTypeName = database.params->get<string>("Dataset.segmentPerJointFeatures");
  m_segFeatureType = SegmentFeatureGenerator::getFeatureType(featureTypeName);
  pSegFeatGenerator = database.getSegmentFeatureGenerator(m_segFeatureType);
}

double SegmentSkelPerJointGroupClassifier::classify(const SegmentSkeletonJoint& segJoint) const {
  vecd feats;
  pSegFeatGenerator->append(*segJoint.segment, &feats);
  segJointGen.append(segJoint, &feats);
  return ClassifierBase::classify(feats);
}

void SegmentJointsAggregatedClassifier::init(const Database& database, const InteractionSet& interactionSet) {
  const util::Params& params = *database.params;
  m_ignoreInferredJoints = params.get<bool>("Interaction.ignoreInferredJoints");
  m_kNearestSegsPerJoint = params.get<UINT>("Interaction.kNearestSegsPerJoint");
  m_maxDistToSegment = params.get<float>("Interaction.maxDistToSegment");
  const bool useActivation = params.get<bool>("Dataset.useActivation");
  const bool useJointSegScores = params.get<bool>("Dataset.useJointSegScores");
  const bool useJointSegFeatures = params.get<bool>("Dataset.useJointSegFeatures");
  m_gen = new SceneSkelJointAggrFeatureGeneratorBasic(interactionSet, useActivation, useJointSegScores, useJointSegFeatures);
  m_pInteractionSet = &interactionSet;
}

SegmentJointsAggregatedClassifier::~SegmentJointsAggregatedClassifier() {
  if (m_gen) { delete m_gen; }
}

double SegmentJointsAggregatedClassifier::classify(const SceneSkel& x) const {
  vecd feats = m_gen->generate(x);
  return ClassifierBase::classify(feats);
}

void SegmentJointsLinearWeightedAggregatedClassifier::init(const Database& database, const InteractionSet& interactionSet) {
  SegmentJointsAggregatedClassifier::init(database, interactionSet);
  double weightsSum = 0.0;
  for (int iJointGroup = 0; iJointGroup < Skeleton::kNumJointGroups; iJointGroup++) {
    //const double jointGroupActivProb = m_pInteractionSet->jointGroupActivationProb[iJointGroup];
    //m_jointGroupWeights[iJointGroup] = abs(jointGroupActivProb - 0.5); 
    m_jointGroupWeights[iJointGroup] = m_pInteractionSet->jointGroupWeights[iJointGroup]; 
    weightsSum += m_jointGroupWeights[iJointGroup];
  }
  //SG_LOG_DEBUG << "Weights for joint aggregation classifier for interaction " << interationSet.id; 
  for (int iJointGroup = 0; iJointGroup < Skeleton::kNumJointGroups; iJointGroup++) {
    m_jointGroupWeights[iJointGroup] /= weightsSum;
    //SG_LOG_DEBUG << "Weight for " << Skeleton::kJointGroupNames[iJointGroup] << " " << m_jointGroupWeights[iJointGroup];
  }
}

double SegmentJointsLinearWeightedAggregatedClassifier::classify(const SceneSkel& x) const {
  //const Scan& scan = *x.scene;
  double score = 0;
  const Skeleton::SegmentsByJointGroup segs = Skeleton::groupSegmentsByJointGroup(*x.activatedSegsByJoint);

  for (int iJointGroup = 0; iJointGroup < Skeleton::kNumJointGroups; iJointGroup++) {
    pair<double,int> scoreAndCount = m_gen->jointInteractionScore(x, iJointGroup);
    double weight = m_jointGroupWeights[iJointGroup];
    const double jointGroupActivProb = m_pInteractionSet->jointGroupActivationProb[iJointGroup];
    if (scoreAndCount.second > 0) { score += weight * jointGroupActivProb * scoreAndCount.first; }
    else { score += weight*(1 - jointGroupActivProb); }
  }
  return score;
}

void SegmentCentroidActivationAggregatedClassifier::init(const Database& database, const InteractionSet& interactionSet) {
  const util::Params& params = *database.params;
  m_ignoreInferredJoints = params.get<bool>("Interaction.ignoreInferredJoints");
  m_kNearestSegsPerJoint = params.get<UINT>("Interaction.kNearestSegsPerJoint");
  m_maxDistToSegment = params.get<float>("Interaction.maxDistToSegment");
  const bool splitCentroidPerJoint = params.get<bool>("Clustering.splitCentroidsPerJointGroup");
  const bool useActivation = params.get<bool>("Dataset.useActivation");
  m_gen = new SceneSkelCentroidActivationFeatGen(interactionSet, database.centroids, splitCentroidPerJoint, useActivation);
  m_pInteractionSet = &interactionSet;
}

SegmentCentroidActivationAggregatedClassifier::~SegmentCentroidActivationAggregatedClassifier() {
  if (m_gen) { delete m_gen; }
}

//TODO(MS): This can be refactored to remove code duplication with other aggregated classifiers since the code is identical except for the type of m_gen
double SegmentCentroidActivationAggregatedClassifier::classify(const SceneSkel& x) const {
  vecd feats = m_gen->generate(x);
  return ClassifierBase::classify(feats);
}

}  // namespace core
}  // namespace sg
