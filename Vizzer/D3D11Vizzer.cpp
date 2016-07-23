#include "common.h"  // NOLINT
#include "./D3D11Vizzer.h"

#include <ann/SimpleNN.h>
#include <core/Database.h>
#include <core/ClassifierDatabase.h>
#include <core/Scene.h>
#include <core/SceneGrokker.h>
#include <core/PoseController.h>
#include <core/synth/synth.h>
#include <interaction/Interaction.h>
#include <interaction/InteractionFrameFactory.h>
#include <io/csv.h>
#include <io/io.h>
#include <segmentation/segmentation.h>
#include <segmentation/MeshSegment.h>
#include <util/log.h>
#include <util/timer.h>
#include <util/util.h>
#include <vis/vis.h>
#include <vis/VisLog.h>

#include "./vizutil.h"

using namespace sg;
using core::Recording; using core::Skeleton; using core::SkelRange;
using std::function; using std::make_pair; using std::make_shared;
using std::reference_wrapper; using std::shared_ptr;

D3D11Vizzer::D3D11Vizzer(SharedAppState& state, util::Params* params)
  : VizzerControls(this)
  , vs(*params)
  , m_state(state)
  , m_heatMap(new vis::PoseHeatMap())
  , m_segParams(*params)
  , m_segSelectedSegId(0)
  , m_segSelectedSegGroupId(-1)
  , m_currInteractions()
  , m_currInteraction(m_currInteractions.begin())
  , m_currInteractionSkelIdx(-1)
  , m_useInteractionActiveSegs(params->get<bool>("Interaction.usePrecomputedActiveSegs"))
  , m_integrationTimeWindowSec(.5f)
  , m_integrationJointIndex(0)
  , m_activationRadius(params->get<float>("Interaction.maxDistToSegment"))
  , m_vertexColorsDirty(true)
  , m_centroidIdx(0)
  , m_centroidActiveSegIdx(-1)
  , m_centroidMaxDist(1.5f)
  , m_pIsetPoseController(nullptr)
  , m_params(params)
  , m_showPredictedSkeleton(false)
  , m_voxelizedSkel(false)
  , m_showInfoOverlay(true)
  , m_probePointA(0, 0, 0)
  , m_probePointB(0, 0, 0)
  , m_showProbePoints(false) {
  const string testFunStr = params->getWithDefault<string>("testFun", "");
  vs.testFun = util::tokenize(testFunStr, " ");
  const string partTypeStr = params->get<string>("Interaction.partType");
  m_state.partType = sg::segmentation::getPartTypeFromString(partTypeStr);  
  // msgProcCallback = reinterpret_cast<ml::WindowWin32::MsgProcCallback>(TwEventWin);
}

D3D11Vizzer::~D3D11Vizzer() {
  if (m_pIsetPoseController) { delete m_pIsetPoseController; }
  if (m_heatMap) { delete m_heatMap; }
  // TwTerminate();
}

//void D3D11Vizzer::initTweakBar(ml::ApplicationData& app, TwBar* tw) {
//  TwInit(TW_DIRECT3D11, &app.graphics.castD3D11().getDevice());
//  TwWindowSize(app.window.getWidth(), app.window.getHeight());
//  tw = TwNewBar("Main");
//  TwDefine(" GLOBAL help='Help message goes here' ");
//  int barSize[2] = {224, 320};
//  TwSetParam(tw, NULL, "size", TW_PARAM_INT32, 2, barSize);
//  TwDefine("Main iconified=true");
//  //TwAddVarRW(tw, "Activation Radius", TW_TYPE_FLOAT, &m_activationRadius, "min=0 max=1 step=0.01 keyincr=+ keydecr=-");
//}

//// Callback function called by AntTweakBar to set activation radius
//void TW_CALL D3D11Vizzer::setActivationRadius(const void* value, void* /*clientData*/) {
//  m_activationRadius = *static_cast<const float*>(value);
//  // Also recompute active segments since different active radius
//  m_state.currRecording->computeActiveSegments(*m_params, *m_state.currScan, m_activationRadius,
//                                               m_params->get<float>("Interaction.maxDistGaze"), true);
//  updateVisualizations(app);
//}
//// Callback function called by AntTweakBar to get the sponge recursion level
//void TW_CALL D3D11Vizzer::getActivationRadius(void* value, void* /*clientData*/) {
//  *static_cast<float*>(value) = m_activationRadius;
//}

void D3D11Vizzer::initRecording(ml::ApplicationData& app, const string& recId,
                                const bool loadBaseScene,
                                const unsigned int cameraIndex /*= CameraView_Recording */) {
  cout << "[D3D11Vizzer] Initializing Recording " << recId << "...";
  const Recording& rec = m_state.database->recordings.getRecording(recId, true, true);
  m_state.currRecording = const_cast<Recording*>(&rec);

  const string& recFileBase = m_params->get<string>(".dataDir")
                              + m_params->get<string>(".recDir") + recId;

  // Load color and depth videos if they exist
  const string& colorFile = recFileBase + ".color.avi";
  if (ml::util::fileExists(colorFile)) {
    m_state.colorPlayer.open(colorFile);
    cout << "[+colorVideo] ";
  }
  const string& depthFile = recFileBase + ".depth.avi";
  if (ml::util::fileExists(depthFile)) {
    m_state.depthPlayer.open(depthFile);
    cout << "[+depthVideo] ";
  }

  // Set bound times: [0,1] range defined wrt to Skeleton track
  m_state.timeMinSec = m_state.timeNowSec = 0;
  m_state.timeMaxSec = m_state.currRecording->durationInSec();
  m_state.currRecording->getSkeletonAtTime(m_state.timeNowSec, m_currentSkelIt);  // Get iterator to initial skeleton

  cout << "done." << endl;
  if (loadBaseScene) {
    loadScan(app, rec.baseScanId());
    //updateMeshSegmentation();
  }

  if (m_state.currScan) {
    m_state.currRecording->unfloatSkeletons(*m_state.currScan);
  }

  setCamera(app, cameraIndex);

  m_state.sendStateInit();
  updateVisualizations(app);
}

void D3D11Vizzer::loadScan(ml::ApplicationData& app, const string& scanId,
                           const ml::mat4f* pXform /*= nullptr*/,
                           const ml::vec4f* pColorMult /*= nullptr*/) {
  cout << "[D3D11Vizzer] Initializing " << scanId << "...";
  const core::ScanDatabase& scanDb = util::startsWith(scanId, "cornell") ? m_state.database->scansCornellPLY : m_state.database->scans;
  if (!scanDb.scanExists(scanId) && !(scanId[0] == '.')) {  // manual override with '.'
    cout << "scene not found" << endl;
    return;
  }
  const core::Scan& scan = scanDb.getScan(scanId);
  cout << " (" << scan.sceneType << ")";
  m_state.currScan = const_cast<core::Scan*>(&scan);
  const vec<string> recordings = m_state.database->recordings.getRecordingIdsForScan(scanId);

  if (recordings.size() > 0) {
    initRecording(app, recordings[0], false);
  }

  // Load up mesh (with appropriate transform if given)
  if (pXform) {
    auto scanMesh = m_state.currScan->mesh;
    scanMesh.transform(*pXform);
    vs.mesh["scan"].load(app.graphics, scanMesh);
  } else {
    vs.mesh["scan"].load(app.graphics, m_state.currScan->mesh);
  }

  // color vertices
  const ml::TriMeshf& mesh = vs.mesh["scan"].getTriMesh();
  const auto& V = mesh.getVertices();
  const size_t numVerts = V.size();
  m_baseVertexColors.resize(numVerts);
  if (pColorMult) {  // scale colors by given color
    ml::util::fill(m_baseVertexColors, [&](UINT64 i) {
      const auto& c = V[i].color;
      const auto& m = *pColorMult;
      return ml::vec4f(c.x * m.x, c.y * m.y, c.z * m.z, c.w * m.w);
    });
  } else {  // just use original colors
    ml::util::fill(m_baseVertexColors, [&](UINT64 i) { return V[i].color; });
  }
  m_currVertexColors = m_baseVertexColors;
  m_vertexColorsDirty = true;

  // reinitialize vizzer state
  vs.sceneMeshActivationRec.init(Skeleton::kNumJointGroups, Skeleton::kNumJoints, static_cast<int>(numVerts),
                                 SharedAppState::kMaxANNpoints);
  setCamera(app, CameraView_Recording);
  m_state.sendStateInit();

  // If we have loaded segment groups propagate them to the UI
  if (m_state.currScan->segmentGroups.size() > 0) {
    vizutil::updateSegGroupAnnotations(m_state, /*reloadSegmentGroups=*/ true);
  }

  m_probePointA = m_state.currScan->bbox.getCenter();
  m_probePointB = m_probePointA; m_probePointB.x += 0.5f;

  if (m_pIsetPoseController != nullptr) {  // Reinitialize explorer for new scene
    const string& interactionSetType = m_pIsetPoseController->currentInteractionSetType();
    const string& interactionSetId = m_pIsetPoseController->currentInteractionSet().id;
    delete m_pIsetPoseController;
    m_pIsetPoseController = new core::ISetPoseController(m_state.database->interactions,
                                                         m_state.currScan);
    m_pIsetPoseController->setInteractionSet(interactionSetType, interactionSetId);
  }

  // Update/clear other meshes
  vs.mesh["heatmap"].load(app.graphics, ml::TriMeshf());
  vs.mesh.erase("scanVoxels");
  if (m_state.showScanVoxels) {
    showScanVoxels(app, core::OccupancyGrid::OccupancyType_Occupied,
                   false, core::synth::LabelOpts());
  }
  vs.mesh.erase("scanLabeledVoxels");
  if (m_state.showScanLabeledVoxels) {
    showScanVoxels(app, core::OccupancyGrid::OccupancyType_Occupied,
                   true, core::synth::LabelOpts());
  }

  cout << "done." << endl;
}

void D3D11Vizzer::loadModel(ml::ApplicationData& app, const string& loadType, const string& loadId) {
  if (!m_state.currScan) { SG_LOG_WARN << "Cannot loadModel, no scan is loaded"; return; }
  if (loadType == "id") {
    if (m_state.database->models.modelExists(loadId)) {
      const core::Model& model = m_state.database->models.getModel(loadId).get();
      core::synth::placeModelInstance(model, *m_state.currScan, &m_modelInstances);
    } else {
      cerr << "Cannot load " << loadType << ":" << loadId << ": unknown model id" << endl;
    }
  } else if (loadType == "category") {
    bool restrictToWhitelist = m_state.database->params->get<bool>("ModelPlacer.restrictToWhitelist");
    if (m_state.database->models.modelCountForCategory(loadId, restrictToWhitelist) > 0) {
      const core::Model& model = m_state.database->models.getRandomModelWithCategory(loadId, restrictToWhitelist).get();
      core::synth::placeModelInstance(model, *m_state.currScan, &m_modelInstances);
    } else {
      cerr << "Cannot load " << loadType << ":" << loadId << ": unknown category" << endl;
    }
  } else {
    cerr << "Cannot load " << loadType << ":" << loadId << ": unknown load type" << endl;
  }

  initModelInteractionMaps(app.graphics);
  updateModelSegmentations(app.graphics);
  updateModelVoxels(app.graphics);
}

void D3D11Vizzer::loadScene(ml::ApplicationData& app, const string& file) {
  bool ok = vs.scene.load(file, &m_state.database->models);
  if (ok) {
    m_modelInstances.clear();
    for (const auto& m : vs.scene.getModelInstances()) {
      m_modelInstances.push_back(m);
    }
    m_predictedSkeleton = vs.scene.getSkels()[0];
    m_showPredictedSkeleton = true;
    if (m_pIsetPoseController) {
      m_pIsetPoseController->setBaseSkeleton(m_predictedSkeleton);
    }
    initModelInteractionMaps(app.graphics);
    updateModelSegmentations(app.graphics);
    updateModelVoxels(app.graphics);
  } else {
    SG_LOG_ERROR << "Error loading scene from " << file;
  }
}

void D3D11Vizzer::loadReconScene(ml::ApplicationData& app, const string& sceneId) {
  bool ok = m_state.database->reconScenes.getScene(sceneId, &vs.scene);
  if (ok) {
    // load scan base for this scene
    const core::SceneSerialized& ss = m_state.database->reconScenes.getSceneSerialized(sceneId);
    const string scanPrefix = "vf.";
    geo::Transform scanXform;
    scanXform.setIdentity();
    scanXform.linear() *= ss.unit;
    for (const auto& m : ss.models) {
      if (m.id.compare(0, scanPrefix.size(), scanPrefix) == 0) {
        scanXform = scanXform * geo::from<ml::mat4f>(m.transform);
        const string scanId = m.id.substr(scanPrefix.size());
        const ml::vec4f colorMult(1, 1, 1, 0.5);
        loadScan(app, scanId, nullptr, &colorMult);
        break;  // NOTE: Assume only one scan specified
      }
    }
    // create transformed model instances
    const geo::Transform scanXformInv = scanXform.inverse();
    m_modelInstances.clear();
    for (const auto& m : vs.scene.getModelInstances()) {
      m_modelInstances.push_back(m);
      m_modelInstances.back().updateLocalTransformBy(scanXformInv);
    }
    // initialize other scene state
    initModelInteractionMaps(app.graphics);
    updateModelSegmentations(app.graphics);
    updateModelVoxels(app.graphics);
  } else {
    SG_LOG_ERROR << "Error loading scene " << sceneId;
  }
}

void D3D11Vizzer::saveScene(const string& file, double score) const {
  core::Scene scene;
  for (const auto& m : m_modelInstances) {
    scene.add(m);
  }
  scene.add(m_predictedSkeleton);
  scene.setScore(score);
  scene.serialize(file);
}

void D3D11Vizzer::savePLY(const string& file, const std::vector<string>& meshes,
                          bool rotateZupToYup /*= true*/) const {
  std::vector<ml::TriMeshf> outMeshes;
  const map<string, ml::D3D11TriMesh>& meshMap = vs.mesh;
  for (const string& id : meshes) {
    if (id == "models") {
      for (const auto& m : m_modelInteractionMapMeshes) {
        outMeshes.push_back(m.getTriMesh());
      }
    }
    if (meshMap.count(id)) {
      outMeshes.push_back(meshMap.at(id).getTriMesh());
    }
  }

  ml::TriMeshf jointMesh = ml::meshutil::createUnifiedMesh(outMeshes);
  if (rotateZupToYup) {
    ml::mat4f ztoyup;
    ztoyup.setRotationX(-90);
    jointMesh.transform(ztoyup);
  }

  cout << "Writing mesh to " << file << "...";
  ml::MeshIOf::saveToPLY(file, jointMesh.getMeshData());
  //ml::MeshIOf::saveToOBJ(objFile, jointMesh.getMeshData());
  cout << "done." << endl;
}

void D3D11Vizzer::clearScene(ml::ApplicationData& app) {
  m_modelInstances.clear();
  vs.iframes.clear();
  m_modelInteractionMaps.clear();
  m_modelInteractionMapMeshes.clear();
  m_segmentedModelInstanceMeshes.clear();
  m_voxelizedModelInstanceMeshes.clear();
  updateVisualizations(app);
}

void D3D11Vizzer::init(ml::ApplicationData& app) {
  // initTweakBar(app, vs.twBar);

  m_assets.init(m_state.database->models);
  m_renderer.init(app.graphics);

  // Initialize graphics objects
  m_vsBasicLight.load(app.graphics, m_state.workingDir + "../../shaders/basicLight.shader");
  m_psBasicLight.load(app.graphics, m_state.workingDir + "../../shaders/basicLight.shader");
  m_basicLightConstants.init(app.graphics);

  m_vsBasicLight.bind();
  m_psBasicLight.bind();
  m_basicLightConstants.bindVertexShader(0);

  m_font.init(app.graphics, "Calibri");
  m_gradientCool2Warm = ml::ColorGradient(ml::LodePNG::load("../../shaders/Cool2WarmBar.png"));
  m_gradientHeatmap = ml::ColorGradient(ml::LodePNG::load("../../shaders/heatmap.png"));
  m_renderTarget.load(app.graphics, 640, 640);

  // Set post test callback
  auto* callback = new function<void(const string&)> ([&](const string & classifierTypeId) {
    testClassifier(app, classifierTypeId);
  });
  m_state.database->classifiers->postTestCallbackFun = callback;

  // Initialize recording and scene
  if (m_params->exists("mesh")) {
    const string meshFile = m_params->get<string>("mesh");
    loadScan(app, meshFile);
  }
  if (m_params->exists("rec")) {
    const string recFile = m_params->get<string>("rec");
    initRecording(app, recFile, false);
  }
  m_interactionSynth.init(m_state.database);

  // Populate event handling map
  m_eventMap.populate(this, app);

  // Tell the UI about our loaded interactions
  m_state.sendStateUpdate("interactionSetsFile", m_state.database->interactions.getInteractionSummaryFile());
}

//! Test a classifier by generating appropriate heatmap and doing screen capture
void D3D11Vizzer::testClassifier(ml::ApplicationData& app, const string& classifierType) {
  ensureDatabase();
  vec<string> interactionSetIds;
  for (const auto& it : m_state.database->interactions.getAllInteractionSets()) {
    interaction::InteractionSet& interactionSet = it.second;
    interactionSetIds.push_back(interactionSet.id);
  }
  testClassifier(app, classifierType, interactionSetIds);
}

void D3D11Vizzer::testClassifier(ml::ApplicationData& app, const string& classifierType,
                                 const string& interactionSet) {
  vec<string> interactionSetIds;
  interactionSetIds.push_back(interactionSet);
  testClassifier(app, classifierType, interactionSetIds);
}

void D3D11Vizzer::testClassifier(ml::ApplicationData& app, const string& classifierType, 
                                 const vec<string>& interactionSetIds) {
  const core::ClassifierManager* classifierManager = m_state.database->classifiers->getClassifierManager(
                                                           classifierType);
  if (classifierManager == nullptr) {
    SG_LOG_ERROR << "Unknown classifierType " << classifierType;
    return;
  } else if (classifierManager->classifierType != core::ClassifierType_ScenePose) {
    SG_LOG_ERROR << "Cannot handle classifierType " << classifierType;
    return;
  }

  const string resultDir = m_state.database->classifiers->getResultOutputPath(classifierManager->name);
  io::ensureDirExists(resultDir);
  const string csvFilename = resultDir + "summary.csv";
  SG_LOG_INFO << "Writing test result summary to " << csvFilename;
  ofstream summaryCSV(csvFilename);
  summaryCSV << "classifierId,scanId,verbId," << eval::BinaryConfusionMatrix::csvHeader() << endl;

  // Also write out action descriptors
  map<string, ActionDescriptor> scanToActionDescMap; // This will store scanId -> ActionDescriptor
  const string actionDescFilename = resultDir + "actionDescriptors.csv";
  ofstream actionDescFile(actionDescFilename);
  // Header
  actionDescFile << "scanId";
  for (const auto& interactionSetId : interactionSetIds) {
    actionDescFile << "," << interactionSetId;
  }
  actionDescFile << endl;

  // TODO(MS): This probably won't behave reasonably if called from cross validation loop
  const vec<string> scanIds = ml::util::split(m_params->get<string>("Dataset.testSetSceneIds"), ',');
  for (const string& scanId : scanIds) {
    // Load target scene
    if (m_state.currScan->id != scanId) {
      loadScan(app, scanId);
      setCamera(app, CameraView_Top);
    }

    // Action descriptor for this scene
    ActionDescriptor& actionDesc = scanToActionDescMap[scanId];
    actionDescFile << scanId;

    // Iterate over requested interactions
    const auto& interactionSets = m_state.database->interactions.getAllInteractionSets();
    for (const auto& interactionSetId : interactionSetIds) {
      // Check if we have data for given interactionSetId
      if (interactionSets.count(interactionSetId) == 0) {
        SG_LOG_ERROR << "[D3D11Vizzer] InteractionSetId=" << interactionSetId << " not available -- skipping it...";
        continue;
      }
      const interaction::InteractionSet& interactionSet = interactionSets.at(interactionSetId);
      // Pull out classifier
      core::ScenePoseClassifier* pClassifier = classifierManager->getScenePoseClassifier(interactionSet);
      if (pClassifier == nullptr) { 
        SG_LOG_ERROR << "[D3D11Vizzer] Cannot get classifier for InteractionSetId="
          << interactionSetId << " not available -- skipping it...";
        continue; 
      }

      // Create output directory
      const string scanOutDir = resultDir + scanId + "/";
      const string outputBasename = scanOutDir + "/" + interactionSet.id;
      io::ensurePathToFileExists(outputBasename);
      SG_LOG_INFO << "Saving results to " << outputBasename;

      // Run test for this scene-verb combination and save confusion matrices at all cut offs
      const vec<eval::BinaryConfusionMatrix> confMats = testClassifier(app, pClassifier, interactionSet, outputBasename,
                                                              &actionDesc);
      for (const eval::BinaryConfusionMatrix& mat : confMats) {
        summaryCSV << classifierType << "," << scanId << "," << interactionSet.id << "," << mat << endl;
      }
    }
    // Now dump action descriptor row for this scene
    for (const auto& iSetIdActionDescPair : actionDesc) {
      actionDescFile << "," << iSetIdActionDescPair.second;
    }
    actionDescFile << endl;
  }
  summaryCSV.close();
  actionDescFile.close();
}

vec<eval::BinaryConfusionMatrix> D3D11Vizzer::testClassifier(ml::ApplicationData& app,
    core::ScenePoseClassifier* pClassifier,
    const interaction::InteractionSet& interactionSet, const string& outputBasename,
    ActionDescriptor* actionDescriptor) {

  // Clear, recompute heatmap, and save heatmap samples to image (on top of original scan)
  vs.mesh["heatmap"].load(app.graphics, ml::TriMeshf());
  updateInteractionHeatMap(app.graphics, interactionSet, *pClassifier);  

  vec<eval::BinaryConfusionMatrix> confusionMatrices;
  evaluateInteractionHeatMap(app, *m_heatMap, outputBasename, interactionSet.id, &confusionMatrices);

  // Compute total activation and store in action descriptor
  const double cutoff = -1.0; //-1 = sum of probability mass, 0.5 = % of pixels >= 0.5
  const double activation = core::SceneGrokker::computeSceneIntensity(*m_heatMap, cutoff);
  if (actionDescriptor) {
    (*actionDescriptor)[interactionSet.id] = activation;
  }
  return confusionMatrices;
}

void D3D11Vizzer::evaluateInteractionHeatMap(ml::ApplicationData& app,
                                             const vis::PoseHeatMap& heatMap,
                                             const string& outputBasename,
                                             const string& evaluateId,
                                             vec<eval::BinaryConfusionMatrix>* pConfusionMatrices) {
  // Set camera and bind basic light so our scan renders
  setCamera(m_renderTarget.getWidth(), m_renderTarget.getHeight(), CameraView_Top);
  bindBasicLight();
  vs.mesh["scan"].updateColors(m_baseVertexColors);
  // Capture and output to file
  vec<std::reference_wrapper<const ml::D3D11TriMesh>> meshes;
  meshes.emplace_back(vs.mesh["scan"]);
  meshes.emplace_back(vs.mesh["heatmap"]);
  ml::ColorImageR8G8B8A8 heatmapBitmap;
  vizutil::projectOrtho(meshes, &app.graphics, &m_basicLightConstants, &m_renderTarget, &heatmapBitmap);
  io::ensurePathToFileExists(outputBasename);
  ml::LodePNG::save(heatmapBitmap, outputBasename + "-heatmap.png");

  // Also dump interpolated mesh colorization from heatmap samples
  vec<vis::PoseHeatMap::Sample> heatMapSamples;
  heatMap.getSamples(&heatMapSamples);
  const auto& sceneMeshVerts = vs.mesh["scan"].getTriMesh().getVertices();
  vec<float> vertexWeights;
  vizutil::projectSamplesToMeshVerts(heatMapSamples, sceneMeshVerts, 0.2f, &vertexWeights);
  const size_t numVerts = sceneMeshVerts.size();
  for (size_t iV = 0; iV < numVerts; ++iV) {
    m_currVertexColors[iV] = ml::vec4f(m_gradientCool2Warm.value(vertexWeights[iV])) / 255.f;
  }
  vs.mesh["scan"].updateColors(m_currVertexColors);
  dumpMesh(outputBasename + "-mesh.ply");
  // Capture screen shot too
  ml::ColorImageR8G8B8A8 meshBitmap;
  vizutil::projectOrtho(vs.mesh["scan"], &app.graphics, &m_basicLightConstants, &m_renderTarget, &meshBitmap);
  ml::LodePNG::save(meshBitmap, outputBasename + "-mesh.png");

  if (!evaluateId.empty() && pConfusionMatrices != nullptr) {
      // Orthographic black & white projection
    for (size_t iV = 0; iV < numVerts; ++iV) {
      m_currVertexColors[iV] = ml::vec4f(vertexWeights[iV], vertexWeights[iV], vertexWeights[iV], 1.f);
    }
    vs.mesh["scan"].updateColors(m_currVertexColors);
    ml::ColorImageR8G8B8A8 orthBitmap;
    vizutil::projectOrtho(vs.mesh["scan"], &app.graphics, &m_basicLightConstants, &m_renderTarget, &orthBitmap);
    ml::LodePNG::save(orthBitmap, outputBasename + "-orth.png");

    // Save confusion matrices at full range of prediction cut-offs
    string scanId = m_state.currScan->getCanonicalId();
    vec<eval::BinaryConfusionMatrix> matrices = core::SceneGrokker::computeConfusionMatrixSet(
      m_state.database->params->get<string>("dataDir"),
      scanId, evaluateId, orthBitmap, 0.025, outputBasename + "-hybrid.png");
    SG_LOG_INFO << "Computed " << matrices.size() << " confusion matrices";
    ofstream csvFile(outputBasename + ".csv");
    csvFile << eval::BinaryConfusionMatrix::csvHeader() << endl;
    for (const auto& m : matrices) {
      csvFile << m << endl;
    }
    csvFile.close();
    *pConfusionMatrices = matrices;
  }
}

const Skeleton* D3D11Vizzer::getActiveSkeleton() const {
  const Skeleton* pCurrSkel = nullptr;
  if (m_showPredictedSkeleton) {
    pCurrSkel = &m_predictedSkeleton;
  } else if (!m_state.inPosingMode && m_state.showSkeletons && m_state.currRecording &&
             !m_state.currRecording->skeletons.empty()) {
    // Use current recording skeleton
    pCurrSkel = &(*m_currentSkelIt);
  } else if (m_pIsetPoseController && m_state.inPosingMode) {
    // Load scan pose iterator skeleton for rendering
    pCurrSkel = &m_pIsetPoseController->currentSkeleton();
  }
  return pCurrSkel;
}

void D3D11Vizzer::setCameraToLookAt(int width, int height, 
                                    const ml::vec3f& eye, const ml::vec3f& target) {
  vis::setCameraToLookAt(width, height, eye, target, &m_camera);
}

void D3D11Vizzer::setCamera(ml::ApplicationData& app, const unsigned cameraIdx) {
  setCamera(app.window.getWidth(), app.window.getHeight(), cameraIdx);
}

void D3D11Vizzer::setCamera(int width, int height, const unsigned cameraIdx) {
  switch (cameraIdx) {
    case CameraView_Current: { return; }
    case CameraView_Recording: {
      // Camera view from recording
      const ml::mat4f& m = (m_state.currRecording) ? m_state.currRecording->camera : ml::mat4f::identity();
      m_camera = ml::Cameraf(m, 60.0f, static_cast<float>(width) / height, 0.01f, 1000.0f, true);
      break;
    } 
    case CameraView_Side_Left: 
    case CameraView_Side_Right:
    case CameraView_Side_Front:
    case CameraView_Side_Back: 
    {
      if (m_state.currScan == nullptr) {  // no scan so just return
        SG_LOG_WARN << "setCamera did nothing since no scan is available";
        return;
      }
      // Side view
      const float distanceScale = 0.75f;
      //const float cameraHeightRatio = 1.0f;
      const float cameraHeightRatio = (1.f + sqrt(5.f)) / 2.f;  // PHI;
      const auto bb = m_state.currScan->bbox;
      const auto center = bb.getCenter();
      const float cameraHeight = bb.getExtentZ() * cameraHeightRatio + bb.getMinZ();
      const float maxExtent = bb.getMaxExtent();
      float dx = 0.0f;
      float dy = 0.0f;
      if (cameraIdx == CameraView_Side_Left) {
        dx = -1.5f;
      } else if (cameraIdx == CameraView_Side_Right) {
        dx = 1.5f;
      } else if (cameraIdx == CameraView_Side_Front) {
        dy = 1.5f;
      } else if (cameraIdx == CameraView_Side_Back) {
        dy = -1.5f;
      }
      float cameraX = center.x + dx*maxExtent * distanceScale;
      float cameraY = center.y + dy*maxExtent * distanceScale;
      ml::vec3f eye(cameraX, cameraY, cameraHeight);
      setCameraToLookAt(width, height, eye, center);
      break;
    }
    case CameraView_Top: {
      if (m_state.currScan == nullptr) {  // no scan so just return
        SG_LOG_WARN << "setCamera did nothing since no scan is available";
        return;
      }
      // Top view
      const float distanceScale = 1.05f;
      const auto bb = m_state.currScan->bbox;
      const auto center = bb.getCenter();
      const float maxExtent = std::max(bb.getExtentX(), bb.getExtentY());
      const float cameraHeight = bb.getMaxZ() + maxExtent * distanceScale;
      const float cameraX = center.x;
      const float cameraY = center.y;

      ml::vec3f eye(cameraX, cameraY, cameraHeight);
      setCameraToLookAt(width, height, eye, center);
      break;
    }
    case CameraView_Skeleton: {
      // Camera to look at the skeleton
      const Skeleton* pCurrSkel = getActiveSkeleton();
      if (pCurrSkel != nullptr) {
        const ml::vec3f target = 
          pCurrSkel->jointPositions[Skeleton::JointType_SpineShoulder];
        const ml::vec3f& bdir = pCurrSkel->bodyPlaneNormal;
        const ml::vec3f dir(bdir.x, bdir.y, bdir.z);
        const ml::vec3f pos = pCurrSkel->bodyPlaneCentroid + dir * 4.0;
        const float cameraHeight = std::max(1.7f, pos.z);
        const float cameraX = pos.x;
        const float cameraY = pos.y;

        ml::vec3f eye(cameraX, cameraY, cameraHeight);
        setCameraToLookAt(width, height, eye, target);
      } else {
        SG_LOG_WARN << "No skeleton for camera to look at";
      }
      break;
    }
    case CameraView_ISynth: {
      // Get bounding box of scene
      geo::BBox bb;
      for (const auto& mInst : m_modelInstances) {
        const auto& mBB = mInst.computeWorldBBox();
        bb.extend(mBB.min());
        bb.extend(mBB.max());
      }
      const Skeleton* pCurrSkel = getActiveSkeleton();
      ml::vec3f target = geo::to<ml::vec3f>(geo::Vec3f(bb.center()));
      ml::vec3f dir(1,0,0);
      if (pCurrSkel != nullptr) {
        for (const auto& j : pCurrSkel->jointPositions) {
          bb.extend(geo::Vec3f(j));
        }

        target = pCurrSkel->jointPositions[Skeleton::JointType_SpineShoulder];
        dir = pCurrSkel->bodyPlaneNormal;
      }
      ml::mat4f r;
      r.setRotationZ(60);
      dir = r * dir;
      const float maxExtent = std::max(bb.sizes().x(), bb.sizes().y());
      const ml::vec3f pos = pCurrSkel->bodyPlaneCentroid + dir * maxExtent * 2.5;
      const float cameraHeight = std::max(2.0f, pos.z);
      const float cameraX = pos.x;
      const float cameraY = pos.y;
      
      ml::vec3f eye(cameraX, cameraY, cameraHeight);
      setCameraToLookAt(width, height, eye, target);
      break;
    }
    default: 
      SG_LOG_WARN << "Ignoring unknown camera index " << cameraIdx;
  }
}

void D3D11Vizzer::updateVisualizations(ml::ApplicationData& app) {
  // Visualizations requiring access to a SkelRange should go in this block
  if (m_state.showActivationMap || m_state.hideUnactivatedMesh || m_state.showModelInteractionMaps ||
      m_state.showInteractionFrame) {
    // Get current skeleton range
    const float
      tEnd = m_state.timeNowSec,
      tStart = (tEnd - m_integrationTimeWindowSec) > 0.f ? tEnd - m_integrationTimeWindowSec : 0.f;

    SkelRange skelRange;
    if (m_state.inPosingMode) {
      skelRange = SkelRange(m_pIsetPoseController->currentSkeleton());
    } else if (m_state.currRecording) {
      skelRange = m_state.currRecording->getSkeletonRange(tStart, tEnd, 1);
    }

    // Update voxels occupied by skeleton joints
    vs.skelVolume.clear();
    for (const Skeleton& skel : skelRange) { vs.skelVolume.add(skel); }
    if (m_state.showModelInteractionMaps) {
      updateModelInteractionMaps(skelRange);
    }
    if (m_state.showInteractionFrame && !m_state.inPosingMode) {
      updateInteractionFrame(app.graphics, skelRange);
    }

    // Update mesh activation maps over this skeleton range
    if (m_state.showActivationMap || m_state.hideUnactivatedMesh) {
      updateMeshActivationMaps(app.graphics, skelRange);
    }
  }
  if (m_state.showInteractionFrame) {
    if (m_state.inPosingMode) {
      const interaction::InteractionSet& iset = m_pIsetPoseController->currentInteractionSet();
      const auto pIframe = iset.protoInteractionFrame;
      if (pIframe) {
        const Skeleton& skel = m_pIsetPoseController->currentSkeleton();
        cout << "IFrameSupport: " << pIframe->support(skel, m_state.currScan) << endl;
        vis::IFVisParams ifvp;
        ifvp.minBinWeight = m_params->get<float>("InteractionFrame.minBinWeight");
        if (m_params->get<bool>("InteractionFrame.useJointWeights")) {
          ifvp.pJointWeights = &iset.jointGroupWeights[0];
        }
        const auto mesh = iframeWidget(iset, m_state.iframeType, ifvp);
        vs.mesh["iframe"].load(app.graphics, mesh);
      }
    } else {  // assume playing back recording IF
      const Skeleton& skel = *m_currentSkelIt;
      cout << "IFrameSupport: " << vs.iframes.support(skel, m_state.currScan) << endl;
      vis::IFVisParams ifvp;
      const auto pIframe = vs.iframes.getInteractionFrame(m_state.iframeType);
      const auto mesh = iframeWidget(*pIframe, ifvp);
      vs.mesh["iframe"].load(app.graphics, mesh);
    }
  }

  if (m_state.showSegmentOBBs) { renderSegmentOBBs(app.graphics); }
  if (m_state.showSegmentGroupOBBs) { renderSegmentGroupOBBs(app.graphics); }
  if (m_state.showObjectOBBs) { renderObjectOBBs(app.graphics); }
  if (m_state.showBodyPoints) { recomputeBodyPoints(app.graphics); }
  if (m_state.showSegmentVerbProbability) { updateSegmentVerbProbabilityColors(); }
  if (m_state.showInteractionOBBs && m_state.currScan) { updateInteractionOBBs(app.graphics); }
}

// Given PoseHeatMap and a sample from it, set PoseExplorer to corresponding Skeleton
void setPoseToSample(const vis::PoseHeatMap& heatmap, const vis::PoseHeatMap::Sample& sample,
                     core::PoseController* pExplorer, int singleSkelIndex) {
  const auto& p = sample.pos();
  const float x = p.x();
  const float y = p.y();
  const float t = heatmap.thetas()[sample.iTheta];
  pExplorer->setPose(x, y, t, (singleSkelIndex < 0) ? sample.iSkeleton : singleSkelIndex, true);
}

void D3D11Vizzer::updateInteractionHeatMap(ml::GraphicsDevice& graphics,
                                           const interaction::InteractionSet& interactionSet,
                                           const core::ScenePoseClassifier& classifier) {
  const core::Scan& scan = *m_state.currScan;
  core::SceneGrokker grokker(*m_state.database, scan);

  // Get base skeleton set (constrain to currently shown skel if option set)
  vec<core::TransformedSkeleton> skels;
  const int singleSkelIndex = m_state.useSingleSkelForHeatmap ? m_pIsetPoseController->currentPoseIndex() : -1;
  if (singleSkelIndex >= 0) {  // Constrain to single skeleton sampling
    skels.push_back(interactionSet.sampledSkeletons[singleSkelIndex]);
  } else {
    skels = interactionSet.sampledSkeletons;
  }

  // Obtain heat map with classifier predictions
  grokker.makeHeatMap(classifier, skels, m_heatMap);
  updateInteractionHeatMap(graphics, *m_heatMap, true);
}

void D3D11Vizzer::updateInteractionHeatMap(ml::GraphicsDevice& graphics, 
                                           const vis::PoseHeatMap& heatMap, 
                                           bool showMaxSample) {
  const auto&   maxSample     = heatMap.maxLikelihoodSample();
  const auto&   maxSamplePos  = maxSample.pos();
  const double  maxLikelihood = maxSample.likelihood;
  SG_LOG_INFO << "[D3D11Vizzer] PoseHeatMap MaxLikelihood: " << maxLikelihood << " at "
              << maxSamplePos.format(geo::kEigenJSONFormat);

  // Create toroidal visualization
  vec<pair<ml::TriMeshf, ml::mat4f>> meshes;
  for (const vis::PoseHeatMap::Point& e : heatMap.points()) {
    // Torus colorization function
    const auto colorFunc = [&] (uint iTorusStack) {
      const bool
        showTheta       = m_state.interactionHeatMapShowAngle,
        renormalize     = m_state.interactionHeatMapNormalize;
      const int iTheta  = iTorusStack % heatMap.numThetaDivisions();
      const double
        normFactor      = renormalize ? (1.0 / maxLikelihood) : 1.0,
        val             = (showTheta ? e.likelihoodByTheta(iTheta) : e.likelihood()) * normFactor;
      const auto c      = m_gradientCool2Warm.value(val);

      return ml::vec4f(c.r / 255.f, c.g / 255.f, c.b / 255.f, 1.f);
    };

    const uint numStacks = heatMap.numThetaDivisions() + 1, numSlices = 10;
    const ml::TriMeshf mesh = ml::Shapesf::torus(ml::vec3f::origin, 0.08f, 0.01f, numStacks, numSlices, colorFunc);
    const ml::mat4f transform = ml::mat4f::translation(ml::vec3f(e.pos.x(), e.pos.y(), 1.0f));
    meshes.push_back(make_pair(mesh, transform));
  }

  // Set pose explorer to skeleton of maxSample and make indicator sphere at max sample
  if (showMaxSample) {
    const int singleSkelIndex = m_state.useSingleSkelForHeatmap ? m_pIsetPoseController->currentPoseIndex() : -1;
    setPoseToSample(heatMap, maxSample, m_pIsetPoseController, singleSkelIndex);
    const ml::TriMeshf maxSphere = ml::Shapesf::sphere(.05f, ml::vec3f(0, 0, 0), 10, 10, ml::vec4f(1, 0, 0, 1));
    meshes.push_back(make_pair(maxSphere, ml::mat4f::translation(maxSample.pos().x(), maxSample.pos().y(), 1.0f)));
  }
  vs.mesh["heatmap"].load(graphics, ml::meshutil::createUnifiedMesh(meshes));
}

void D3D11Vizzer::updateSegmentVerbProbabilityColors() {
  if (m_state.showSegmentVerbProbability && m_pIsetPoseController) {
    const core::SegmentPoseClassifier* pClassifier = m_pIsetPoseController->currentInteractionSet().segmentPoseClassifier;
    if (pClassifier != nullptr) {
      for (const auto& seg : *m_state.currScan->segments) {
        double value = pClassifier->classify(core::SegmentSkeletonPair(seg.get(), &m_pIsetPoseController->currentTransformedSkeleton()));
        for (size_t iVertex : seg->elements) {
          m_currVertexColors[iVertex] = m_gradientCool2Warm.value(value);
        }
      }
    } else {
      SG_LOG_ERROR << "No segment pose classifier";
    }
  }
  m_vertexColorsDirty = true;
}

using interaction::kSkelParams;

void D3D11Vizzer::updateMeshActivationMaps(ml::GraphicsDevice& g, const SkelRange& skelRange) {
  // Reset vertex colors
  m_currVertexColors = m_baseVertexColors;

  // Paint mesh with joint colors based on amount of activation
  if (m_state.showActivationMap) {
    const float maxGazeDist = m_params->get<float>("Interaction.maxDistGaze");
    m_state.currScan->getMeshHelper().accumulateActivation(skelRange, m_activationRadius, maxGazeDist,
                                                            &vs.sceneMeshActivationRec);
    if (m_state.showAllJointsActivation) {  // all joint groups
      const auto colorFun = [&] (int i) {
        if (i == Skeleton::JointGroup_Gaze) { return kSkelParams.kJointColors[Skeleton::JointGroupLR_Head]; }
        return kSkelParams.kJointColors[i];
      };
      vs.sceneMeshActivationRec.colorizeVerticesAllIds(colorFun, &m_currVertexColors);
    } else {  // single selected joint group
      const int iSelected = Skeleton::kJointToJointGroupLR[m_integrationJointIndex];
      const ml::vec4f& color = kSkelParams.kJointColors[iSelected];
      vs.sceneMeshActivationRec.colorizeVertices(iSelected, color, &m_currVertexColors);
    }
  }

  // Hide unactivated mesh regions
  if (m_state.hideUnactivatedMesh) {
    for (UINT iVertex = 0; iVertex < m_currVertexColors.size(); iVertex++) {
      if (m_currVertexColors[iVertex] == m_baseVertexColors[iVertex]) {
        m_currVertexColors[iVertex] = ml::vec4f(.1f, .1f, .1f, 1.f);
      }
    }
  }

  m_vertexColorsDirty = true;
}

void D3D11Vizzer::makeEvaluationTruthScenes(ml::ApplicationData& app) {
  const string evaluationDir = m_state.database->params->get<string>("dataDir") + "evaluation/";
  const string baseDir = evaluationDir + "verbImagesTruth/";
  auto verbList = ml::util::getFileLines(evaluationDir + "verbs.txt", 3);
  verbList.push_back("mask");

  for (const string& s : m_state.database->scans.scanIds()) {
    string baseFilename = baseDir + s + ".png";
    if (ml::util::fileExists(baseFilename)) {
      ml::Console::log(baseFilename + " already exists");
    } else {
      loadScan(app, s);
      ml::ColorImageR8G8B8A8 orthBitmap;
      vizutil::projectOrtho(vs.mesh["scan"], &app.graphics, &m_basicLightConstants, &m_renderTarget, &orthBitmap);
      ml::LodePNG::save(orthBitmap, baseFilename);
    }
    for (const string& verb : verbList) {
      string verbFilename = baseDir + s + "_" + verb + ".png";
      if (ml::util::fileExists(verbFilename)) {
        ml::Console::log(verbFilename + " already exists");
      } else {
        io::copyFile(baseFilename, verbFilename);
      }
    }
  }
}

void D3D11Vizzer::renderToBitmap(ml::ApplicationData& app,
                                 vec<reference_wrapper<const ml::D3D11TriMesh>> meshes,
                                 ml::ColorImageR8G8B8A8* pBitmap) {
  bindBasicLight();
  m_renderTarget.bind();
  m_renderTarget.clear(ml::vec4f(1,1,1,0));
  for (const ml::D3D11TriMesh& mesh: meshes) {
    mesh.render();
  }
  m_renderTarget.captureColorBuffer(*pBitmap);
  // Restore graphics bindings to screen
  app.graphics.castD3D11().bindRenderTarget();
}

void D3D11Vizzer::renderToFile(ml::ApplicationData& app,
                               vec<reference_wrapper<const ml::D3D11TriMesh>> meshes,
                               const string& filename) {
  ml::ColorImageR8G8B8A8 bitmap;
  renderToBitmap(app, meshes, &bitmap);
  io::ensurePathToFileExists(filename);
  ml::LodePNG::save(bitmap, filename);
}


void D3D11Vizzer::bindBasicLight() {
  m_vsBasicLight.bind();
  m_psBasicLight.bind();
  m_basicLightConstantsBuffer.worldViewProj = m_camera.getCameraPerspective();
  m_basicLightConstants.updateAndBind(m_basicLightConstantsBuffer, 0);
}

// NOTE: This is called by the message loop and surrounded by renderBeginFrame() and renderEndFrame()
void D3D11Vizzer::render(ml::ApplicationData& app) {
  m_timer.frame();
  processEvents(app);
  renderOnly(app);
}

void D3D11Vizzer::renderFrame(ml::ApplicationData& app) {
  m_timer.frame();
  app.graphics.renderBeginFrame();
  renderOnly(app);
  app.graphics.renderEndFrame(false);
}

void D3D11Vizzer::renderOnly(ml::ApplicationData& app) {
  app.graphics.clear(ml::vec4f(1, 1, 1, 0), 1.0f);
  // helper for printing status text
  const auto statusTxt = [&] (const string& s, const int vpos, const ml::RGBColor& c) {
    if (m_showInfoOverlay) {
      m_font.drawString(app.graphics, s, ml::vec2i(5, vpos), 14.f, c);
    }
  };

  // Update mesh colors
  if (m_vertexColorsDirty) {
    vs.mesh["scan"].updateColors(m_currVertexColors);
    m_vertexColorsDirty = false;
  }

  const ml::mat4f cameraPerspective = m_camera.getCameraPerspective();
  // Render scan elements
  if (m_state.showModels) {
    for (const auto& modelInst : m_modelInstances) {
      m_renderer.renderModel(m_assets, app.graphics, cameraPerspective, modelInst.model.id,
                             geo::to<ml::mat4f>(modelInst.getLocalTransform()),
                             modelInst.color);
    }
  }

  bindBasicLight();

  if (m_state.showModelInteractionMaps) {
    for (const auto& m : m_modelInteractionMapMeshes) {
      m.render();
    }
  }

  if (m_state.showScan) { vs.mesh["scan"].render(); }
  if (m_state.showBodyPoints) { vs.mesh["bodyPoints"].render(); }
  if (m_state.showSegmentOBBs) { vs.mesh["OBBs"].render(); }
  if (m_state.showSegmentGroupOBBs) { vs.mesh["segGroupOBB"].render(); }
  if (m_state.showObjectOBBs) { vs.mesh["objOBB"].render(); }
  if (m_state.showInteractionOBBs) { vs.mesh["interactionOBB"].render(); }
  if (m_state.showInteractionHeatMap) { vs.mesh["heatmap"].render(); }
  if (true) {  // TODO(MS): Unhack
    vs.mesh["test"].render();
  }
  if (m_state.showScanVoxels && vs.mesh.count("scanVoxels")) {
    vs.mesh["scanVoxels"].render();
  }
  if (m_state.showScanLabeledVoxels && vs.mesh.count("scanVoxelsLabeled")) {
    vs.mesh["scanVoxelsLabeled"].render();
  }

  if (m_state.showModelSegmentation) {
    for (const auto& m : m_segmentedModelInstanceMeshes) {
      m.render();
    }
  }
  if (m_state.showModelVoxels) {
    for (const auto& m : m_voxelizedModelInstanceMeshes) {
      m.render();
    }
  }
  // Load skeleton for rendering
  const Skeleton* pCurrSkel = getActiveSkeleton();
  if (m_pIsetPoseController && m_state.inPosingMode) {
    // Display current interaction set id
    const string& expStr = m_pIsetPoseController->currentInteractionSet().id;
    statusTxt(expStr, 105, ml::RGBColor::Red);
  }

  // Render skeleton
  if (pCurrSkel != nullptr) {
    const Skeleton& skel = *pCurrSkel;
    if (m_voxelizedSkel) {
      vs.mesh["skeleton"].load(app.graphics, vizutil::visualizeSkelCollisions(skel, *m_state.currScan));
    } else {
      const ml::TriMeshf skelMesh = vis::toTriMesh(skel, true, true, m_state.showGaze, false, false, false);
      vs.mesh["skeleton"].load(app.graphics, skelMesh);
    }
    if (skel.isSitting()) {
      statusTxt("sit", 30, ml::RGBColor::Blue);
    } else {
      statusTxt("stand", 30, ml::RGBColor::Red);
    }
    vs.mesh["skeleton"].render();
  }

  if (m_state.showInteractionFrame) {
    vs.mesh["iframe"].render();
  }

  // Print current info string at top
  string infoStr = "Time:" + to_string(m_state.timeNowSec)
                    + "\tFPS: " + to_string(m_timer.framesPerSecond())
                    + "\tJoint (Z/X): " + Skeleton::jointGroupName(m_integrationJointIndex)
                    + "\tInteraction: " + ((m_pIsetPoseController) ? m_pIsetPoseController->currentInteractionSet().id : "");
  statusTxt(infoStr, 5, ml::RGBColor::Red);

  // Print centroid selection info
  if (m_state.showClusterAssignment && m_state.database &&
      m_state.database->centroids.getCentroidSetForAllJoints() != NULL &&
      m_state.database->centroids.getCentroidSetForAllJoints()->k > 0) {
    string centrInfo = "centrIdx [yu]: " + to_string(m_centroidIdx)
                       + "\tmaxDist [hj]: " + to_string(m_centroidMaxDist);
    statusTxt(centrInfo, 55, ml::RGBColor::Red);
  }

  // Print centroid-segment assignment info
  if (m_state.showClusterAssignment &&
      m_centroidActiveSegIdx >= 0 &&
      m_centroidActiveSegIdx < m_centroidActSegIndices.size() &&
      m_centroidActSegIndices[m_centroidActiveSegIdx] < m_state.currScan->segments->size()) {
    const auto& seg = *(*m_state.currScan->segments)[m_centroidActSegIndices[m_centroidActiveSegIdx]];
    core::SegmentFeatureGeneratorSimplistic segGenerator;
    const auto fieldNames = segGenerator.fieldNames();
    const auto segFeatures = segGenerator.generate(seg);
    const auto& centroidSet = *m_state.database->centroids.getCentroidSetForAllJoints();
    const vec<double> centroidWhite = centroidSet.getCentroid(m_centroidIdx);
    const vec<double> centroidOrig = centroidSet.invertWhitening(centroidWhite, 1);
    const string header = "field\tsegment\tcent(UW)\tcent(W)";
    statusTxt(header, 80, ml::RGBColor::Blue);
    for (size_t featIdx = 0; featIdx < fieldNames.size(); featIdx++) {
      string text = fieldNames[featIdx] + "\t" +
                    to_string(seg.rawFeatureCache[featIdx]) + "\t" +
                    to_string(centroidOrig[featIdx]) + "\t" +
                    to_string(centroidWhite[featIdx]);
      statusTxt(text, 105 + static_cast<int>(featIdx) * 25, ml::RGBColor::Red);
    }
  }

  // Print Interaction info
  if (m_currInteractions.size() > 0) {
    string annStr = "Vs: ";
    for (const interaction::Interaction* i : m_currInteractions) { annStr += " " + i->isetId; }
    if (m_currInteraction != m_currInteractions.end()) { annStr += " [selected: " + (*m_currInteraction)->isetId + "]"; }
    statusTxt(annStr, 55, ml::RGBColor::Red);
  }

  // Render probe points
  if (m_showProbePoints) {
    auto vecstr = [ ] (const string& id, const ml::vec3f& p) {
      return id + ":" + to_string(p.x) + "," + to_string(p.y) + "," + to_string(p.z);
    };
    const auto& a = m_probePointA; const auto& b = m_probePointB;
    m_renderer.renderSphere(app.graphics, cameraPerspective, a, .025f, ml::ColorUtils::colorById<ml::vec4f>(0));
    m_renderer.renderSphere(app.graphics, cameraPerspective, b, .025f, ml::ColorUtils::colorById<ml::vec4f>(1));
    const string pointPosInfo = vecstr("RED", a) + " --- " + vecstr("GREEN", b);
    statusTxt(pointPosInfo, 50, ml::RGBColor::Red);
  }

  // Playback stepping
  if (m_state.playingRange) {
    setCurrentTime(app, m_state.timeNowSec + m_state.rangeTimeStepSec);
    m_state.sendStateUpdate();  // Inform UI of time step
    if (m_state.timeNowSec > m_state.rangeTimeEndSec) { m_state.playingRange = false; }
  }

  // Draw tweak bars
  if (m_showInfoOverlay) {
    // TwDraw();
  }
}

void D3D11Vizzer::processEvents(ml::ApplicationData& app) {
  if (m_state.terminate) { PostQuitMessage(0); }
  m_state.checkForUpdates();
  m_eventMap.processSignalQueue();
}

void D3D11Vizzer::resize(ml::ApplicationData& app) {
  m_camera.updateAspectRatio(static_cast<float>(app.window.getWidth()) / app.window.getHeight());
  //m_font.reset(app.graphics);
}

void D3D11Vizzer::ensureDatabase() {
  if (!m_state.database || !m_pIsetPoseController) { createDatabase(); }
}

void D3D11Vizzer::createDatabase() {
  if (!m_state.database) {
    m_state.database = new core::Database(*m_params);
  }
  // These should have been be done in the main
  m_state.database->load(/* computeAllActiveSegments= */ true, /* computeInteractionGraphs= */ true);
  m_state.database->computeCentroids(/* forceRecompute = */ false);
  //bool forceRetrain = m_params->get<bool>("Dataset.forceRetrain");
  //m_state.database->loadClassifiers( forceRetrain );
  if (!m_pIsetPoseController) {
    m_pIsetPoseController = new core::ISetPoseController(m_state.database->interactions,
                                                         m_state.currScan);
    m_state.sendStateUpdate("interactionSet", 
                            m_pIsetPoseController->currentInteractionSetType() + ":" +
                            m_pIsetPoseController->currentInteractionSet().id);
  }
  m_state.database->describeDataset();
}

void D3D11Vizzer::selectSegmentGroup(ml::ApplicationData& app, int segGroupId) {
  m_segSelectedSegGroupId = segGroupId;
  if (m_state.showSegmentation) {
    const auto* sgPtr = m_state.currScan->segmentGroups.get(segGroupId);
    if (sgPtr != nullptr) {
      cout << "Segment group " << segGroupId << " selected: ";
      for (int idx : sgPtr->segments()) { cout << idx << " " << endl; }
      segmentation::getSegmentationColors(m_state.currScan->mesh, sgPtr->segments(), &m_currVertexColors);
    } else {
      vec<int> none;
      segmentation::getSegmentationColors(m_state.currScan->mesh, none, &m_currVertexColors);
    }
    m_vertexColorsDirty = true;
  }
  updateVisualizations(app);
}

void D3D11Vizzer::updateMeshSegmentation() {
  m_state.currScan->updateSegmentation(m_segParams);
  meshSegmentationChanged();
}

void D3D11Vizzer::loadMeshSegmentation(const string& filename) {
  m_state.currScan->loadSegmentation(filename, m_segParams.constrainZup);
  meshSegmentationChanged();
}

void D3D11Vizzer::meshSegmentationChanged(bool segmentGroupsReloaded) {
  // If mesh segmentation has changed and we are showing segmentations, update the vertex colors
  if (m_state.showSegmentation) {
    segmentation::getSegmentationColors(m_state.currScan->mesh, m_segSelectedSegId, &m_currVertexColors);
    m_vertexColorsDirty = true;
  }
  // Also recompute all active segments in current recording since segmentation has changed
  m_state.currRecording->computeActiveSegments(*m_params, *m_state.currScan, m_activationRadius,
                                               m_params->get<float>("Interaction.maxDistGaze"), true);

  // Segment groups may have changed
  vizutil::updateSegGroupAnnotations(m_state, segmentGroupsReloaded);
}

bool D3D11Vizzer::isSegmentSelected(int segId) const {
  if (segId == m_segSelectedSegId) {
    return true;
  } else {
    const auto* sgPtr = m_state.currScan->segmentGroups.get(m_segSelectedSegGroupId);
    if (sgPtr != nullptr) {
      return sgPtr->contains(segId);
    }
    return false;
  }
}

ml::vec4f D3D11Vizzer::getSegmentColor(const segmentation::MeshSegment& seg) {
  if (isSegmentSelected(seg.id)) {
    return ml::vec4f(1, 1, 1, 1);
  }
  switch (m_state.colorSegmentBy) {
  case SharedAppState::Color_ByLabel:
  case SharedAppState::Color_ByCategory:
  case SharedAppState::Color_ByObjectId: {
    const auto segGroupHandle = m_state.currScan->segmentGroups.find(seg);
    const segmentation::SegmentGroup* segGroupPtr = m_state.currScan->segmentGroups.at(segGroupHandle);
    if (segGroupPtr != nullptr) {
     return getSegmentGroupColor(*segGroupPtr);
    }
  }
  // Fall through
  default:
    return ml::ColorUtils::colorById<ml::vec4f>(static_cast<int>(seg.id));
  }
}

ml::vec4f D3D11Vizzer::getSegmentGroupColor(const segmentation::SegmentGroup& segGroup) {
  if (segGroup.id == m_segSelectedSegGroupId) {
    return ml::vec4f(1, 1, 1, 1);
  }
  vis::ColorIndex* pColorIndex;
  switch (m_state.colorSegmentBy) {
  case SharedAppState::Color_ByLabel:
    pColorIndex = &m_partLabelColors;
    return pColorIndex->color(segGroup.label).toVec4f();
  case SharedAppState::Color_ByCategory:
    pColorIndex = &m_categoryColors;
    return pColorIndex->color(m_state.database->getLabeler().getCategory(segGroup.label)).toVec4f();
  case SharedAppState::Color_ByObjectId:
    return ml::ColorUtils::colorById<ml::vec4f>(static_cast<int>(segGroup.objectId));
  default:
    return ml::ColorUtils::colorById<ml::vec4f>(static_cast<int>(segGroup.id));
  }
}

ml::vec4f D3D11Vizzer::getObjectColor(const segmentation::SegmentGroupObject& obj) {
  vis::ColorIndex* pColorIndex;
  switch (m_state.colorSegmentBy) {
  case SharedAppState::Color_ByLabel:
  case SharedAppState::Color_ByCategory:
    pColorIndex = &m_categoryColors;
    return pColorIndex->color(m_state.database->getLabeler().getCategory(obj.label)).toVec4f();
  case SharedAppState::Color_ByObjectId:
  default:
    return ml::ColorUtils::colorById<ml::vec4f>(static_cast<int>(obj.id));
  }
}

void D3D11Vizzer::findSimilarMeshSegmentsForJointInteraction() {
  if (m_integrationJointIndex < 0 || m_integrationJointIndex >= Skeleton::kNumJoints) {
    cout << "ERROR: Invalid joint index " << m_integrationJointIndex
         << " - Please set a joint index from 0 to " << Skeleton::kNumJoints << endl;
    return;
  }
  string jointName = Skeleton::jointName(m_integrationJointIndex);
  cout << "Finding most similar mesh segments for joint interaction with joint " << jointName << endl;
  if (m_currInteractions.size() == 0) {
    cout << "ERROR: Please load and select interactions!" << endl;
    return;
  }
  if (m_state.jointSegKnnUseCurr) {
    cout << "Find most similar mesh segments for current interaction" << endl;
    segmentation::VecSegPtr targets;
    for (const auto& jointSegments : (*m_currInteraction)->jointSegments) {
      const auto& segs = jointSegments[m_integrationJointIndex];
      if (segs.empty()) { continue; }
      const auto jointSegment = segs[0];
      if (jointSegment != nullptr) { targets.push_back(jointSegment); }
    }
    findSimilarMeshSegments(*m_state.currScan->segments, targets);
  } else {
    const string interactionId = (*m_currInteraction)->isetId;
    const auto& interactions = m_state.currRecording->interactions;
    cout << "Find most similar mesh segments for interaction " << interactionId << endl;
    if (interactions.size() > 0) {
      segmentation::VecSegPtr targets;
      for (const auto& interaction : interactions) {
        if (interaction->isetId == interactionId) {
          // Aha! matching interaction found!
          for (const auto& jointSegments : interaction->jointSegments) {
            const auto& segs = jointSegments[m_integrationJointIndex];
            if (segs.empty()) { continue; }
            const auto seg = segs[0];
            if (seg != nullptr) { targets.push_back(seg); }
          }
        }
      }
      if (targets.size() > 0) {
        findSimilarMeshSegments(*m_state.currScan->segments, targets);
      } else {
        cout << "No interactions for interactionId " + interactionId + " joint " + jointName << endl;
      }
    } else {
      cout << "ERROR: Please load interactions!" << endl;
    }
  }
}

void D3D11Vizzer::findSimilarMeshSegmentsForSelectedSegment() {
  cout << "Finding most similar mesh segments to segment " << m_segSelectedSegId << endl;
  size_t nSegs = m_state.currScan->segments->size();
  if (m_segSelectedSegId >= 0 && m_segSelectedSegId < nSegs) {
    findSimilarMeshSegments(*m_state.currScan->segments, *(*m_state.currScan->segments)[m_segSelectedSegId]);
  } else {
    if (nSegs == 0) {
      cout << "ERROR: No segments - Please segment first" << endl;
    } else {
      cout << "ERROR: Invalid segment id " << m_segSelectedSegId
           << " - Please set a segment id from 0 to " << nSegs << endl;
    }
  }
}

struct MeshSegmentDistance :
  ann::FeatureDistance<segmentation::MeshSegment, core::SegmentFeatureGeneratorSimplistic> { };

void D3D11Vizzer::findSimilarMeshSegments(const segmentation::VecSegPtr& segments,
                                          const segmentation::MeshSegment& target,
                                          const size_t k, const size_t radius) {

  MeshSegmentDistance distance;
  ann::BruteForceNearestNeighbors<segmentation::MeshSegment, shared_ptr<segmentation::MeshSegment>, double>
  nn(distance);
  vec<pair<size_t, double>> out_pairs;

  nn.queryKNN(segments, target, &out_pairs, k, radius);
  visualizeMeshSegments(segments, out_pairs, [] (double x ) { return 1.0 / (1.0 + x); });
}

void D3D11Vizzer::findSimilarMeshSegments(const segmentation::VecSegPtr& segments,
                                          const segmentation::VecSegPtr& targets,
                                          const size_t k, const size_t radius) {
  MeshSegmentDistance distance;
  ann::BruteForceNearestNeighbors<segmentation::MeshSegment, shared_ptr<segmentation::MeshSegment>, double>
  nn(distance);
  vec<pair<size_t, double>> out_pairs;

  nn.queryKNNMin(segments, targets, &out_pairs, k, radius);
  visualizeMeshSegments(segments, out_pairs, [] (double x) { return 1.0 / (1.0 + x); });
}

void D3D11Vizzer::classifySegmentsPerJoint() {
  if (!m_state.database || m_currInteraction == m_currInteractions.end() || *m_currInteraction == nullptr) { return; }
  if (m_integrationJointIndex >= Skeleton::kNumJointGroups) {
    cerr << "Ignoring invalid joint group index: " << m_integrationJointIndex << endl;
    return;
  }
  const string& interactionId = (*m_currInteraction)->isetId;
  const string& classifierId = m_state.database->classifiers->getClassifierId("segmentPerJoint", 
    interactionId + "_" + Skeleton::jointGroupName(m_integrationJointIndex));
  const auto* classer = m_state.database->classifiers->get<core::SegmentPerJointGroupClassifier>(classifierId);
  if (classer == nullptr || !classer->isValid()) { return; }
  vec<pair<size_t, double>> out_pairs;
  for (size_t iSeg = 0; iSeg < m_state.currScan->segments->size(); iSeg++) {
    const auto seg = (*m_state.currScan->segments)[iSeg];
    out_pairs.push_back(make_pair(iSeg, classer->classify(*seg)));
  }
  visualizeMeshSegments(*m_state.currScan->segments, out_pairs);
  cout << "Classifying segments with " << classifierId << endl;
}

void D3D11Vizzer::classifySegmentsByJointsAggregated() {
  if (!m_state.database || *m_currInteraction == nullptr) { return; }
  const string& interactionId = (*m_currInteraction)->isetId;
  const string& classifierId = m_state.database->classifiers->getClassifierId("segmentJointsAggr", interactionId);
  const auto* classer = m_state.database->classifiers->get<core::SegmentPerJointGroupClassifier>(classifierId);
  if (classer == nullptr || !classer->isValid()) { return; }

  vec<pair<size_t, double>> out_pairs;
  for (size_t iSeg = 0; iSeg < m_state.currScan->segments->size(); iSeg++) {
    const auto& seg = (*m_state.currScan->segments)[iSeg];
    out_pairs.push_back(make_pair(iSeg, classer->classify(*seg)));
  }
  visualizeMeshSegments(*m_state.currScan->segments, out_pairs);
  cout << "Classifying segments with " << classifierId << endl;
}

void D3D11Vizzer::visualizeMeshSegments(const segmentation::VecSegPtr& segments,
                                        const vec<pair<size_t, double>>& pairs,
                                        function<double(double)> convertScore) {
  for (size_t i = 0; i < pairs.size(); i++) {
    size_t iseg = pairs[i].first;
    double val = convertScore(pairs[i].second);
    const auto& seg = segments[iseg];
    for (size_t iVertex : seg->elements) {
      m_currVertexColors[iVertex] = m_gradientCool2Warm.value(val);
    }
  }
  m_vertexColorsDirty = true;
}

void D3D11Vizzer::recomputeBodyPoints(ml::GraphicsDevice& graphics) {
  // Make sure image plane coords matrix is initialized
  if (m_imgPlanePts.empty()) {
    m_imgPlanePts.create(constants::KINECT_ONE_DEPTH_HEIGHT, constants::KINECT_ONE_DEPTH_WIDTH, CV_32FC4);
    for (int j = 0; j < m_imgPlanePts.rows; j++) {
      for (int i = 0; i < m_imgPlanePts.cols; i++) {
        cv::Vec4f& v = m_imgPlanePts.at<cv::Vec4f>(j, i);
        const ml::vec4f p
          = constants::KINECT_ONE_INTRINSICS_INV * ml::vec4f(static_cast<float>(i), static_cast<float>(j), 0.0f, 1.f);
        v[0] = p.x;
        v[1] = p.y;
        v[2] = p.z;
        v[3] = p.w;
      }
    }
  }
  cv::flip(m_depthBodyIndex, m_temp, 1);
  reprojectDepthFrame(m_temp, &m_xyzPts);
  const ml::TriMeshf& box = ml::Shapesf::box(0.01f);
  vs.mesh["bodyPoints"].load(graphics, ml::meshutil::createPointCloudTemplate(box, m_xyzPts));
}

void D3D11Vizzer::reprojectDepthFrame(const cv::Mat& depthAndBody,
                                      vec<ml::vec3f>* points) const {
  // Reconstruct depth and body index matrices
  cv::Mat ch[3];
  cv::split(depthAndBody, ch);
  cv::Mat depth(constants::KINECT_ONE_DEPTH_WIDTH, constants::KINECT_ONE_DEPTH_HEIGHT, CV_8UC2);
  const cv::Mat body = ch[0];
  cv::merge(&ch[1], 2, depth);
  cv::Mat depthMerged(depth.rows, depth.cols, CV_16UC1);
  memmove(depthMerged.data, depth.data, 2 * depth.rows * depth.cols);
  const cv::Mat depthMeters = cv::Mat_<float>(depthMerged) * 0.001f;
  const ml::mat4f& extrinsics = m_state.currRecording->camera;

  // Preallocate points
  points->clear();
  points->reserve(50000);

  for (int j = 0; j < depth.rows; j++) {
    for (int i = 0; i < depth.cols; i++) {
      const uchar b = body.at<uchar>(j, i);
      if (b == 0xff) { continue; }  // No body point here
      const float d = depthMeters.at<float>(j, i);
      if (d == 0) { continue; }  // No depth value
      const cv::Vec4f& p = m_imgPlanePts.at<cv::Vec4f>(j, i);
      points->push_back(extrinsics * ml::vec3f(d * p[0], d * p[1], d * p[3]));
    }
  }
}

void D3D11Vizzer::updateInteractionOBBs(ml::GraphicsDevice& graphics) {
  sg::interaction::InteractionParams iparams(*m_params);
  iparams.maxDistToSegment = m_activationRadius;

  vs.mesh["interactionOBB"].reset();  // Clear old OBBs

  // Retrieve active segments
  if (m_state.inPosingMode) {  // Current explorer Skeleton
    const Skeleton& skel = m_pIsetPoseController->currentSkeleton();
    if (m_state.currScan) {
      m_state.database->interactionFactory.getActiveParts(
        iparams, *m_state.currScan, SkelRange(skel), m_state.partType, &vs.activeParts);
    }
  } else if (m_useInteractionActiveSegs && m_currInteraction != m_currInteractions.end()) {
    // Use precomputed active segments from Interaction
    const interaction::Interaction& interaction = **m_currInteraction;
    if (m_currInteractionSkelIdx < 0) {
      cerr << "No skeleton to use for activated segs." << endl;
      return; // No matching skeleton
    }
    //const Skeleton& skel = interaction.skelRange[m_currInteractionSkelIdx];
    if (m_state.currScan) {
      vs.activeParts = interaction::segmentsToParts(
        *m_state.currScan, interaction.jointSegments[m_currInteractionSkelIdx], m_state.partType);
    }
  } else {  // Get closest skeleton in current Recording stream and compute active segments
    const Skeleton& skel = m_state.currRecording->getSkeletonAtTime(m_state.timeNowSec);
    if (m_state.currScan) {
      m_state.database->interactionFactory.getActiveParts(
        iparams, *m_state.currScan, SkelRange(skel), m_state.partType, &vs.activeParts);
    }
  }

  // Create OBB meshes for each activated segment colored by joint group color and load for rendering
  vec<ml::TriMeshf> meshes;
  for (size_t iJoint = 0; iJoint < vs.activeParts.size(); iJoint++) {
    ml::vec4f color = Skeleton::kJointGroupLRBaseColors[Skeleton::kJointToJointGroupLR[iJoint]];
    ml::vec4f colort = color;
    colort.a = 0.5f;
    for (const auto& seg : vs.activeParts[iJoint]) {
      meshes.push_back(vis::OBBbox(*seg->obb(), colort));
      meshes.push_back(vis::OBBwireframe(*seg->obb(), color, 0.f, 0.01f));
    }
  }
  vs.mesh["interactionOBB"].load(graphics, ml::meshutil::createUnifiedMesh(meshes));
}

void D3D11Vizzer::updateInteractionFrame(ml::GraphicsDevice& graphics, const SkelRange& skelRange) {
  vs.iframes.clear();
  interaction::InteractionFrameFactoryParams iffp(*m_params);
  interaction::InteractionFrameFactory iff(iffp, *m_state.database);

  if (m_state.currScan) {
    iff.addInteraction(*m_state.currScan, skelRange, &vs.iframes);
  }

  vis::IFVisParams ifvp;
  ifvp.minBinWeight = m_params->get<float>("InteractionFrame.minBinWeight");
  const auto pIframe = vs.iframes.getInteractionFrame(m_state.iframeType);
  const auto mesh = vis::iframePointsWidget(*pIframe, ifvp);
  vs.mesh["iframe"].load(graphics, mesh);
}

void addAnnotationPointCloud(const interaction::ModelAnnotationDatabase& db, const core::ModelInstance& mInst,
                             vis::ColorIndex* cIndex, ml::TriMeshf* outMesh) {
  vec<ml::TriMeshf> meshes;
  meshes.push_back(*outMesh);
  map<string, vec<ml::vec3f>> typeToPts;
  const string mId = "wss." + mInst.model.id;
  if (db.getAnnotatedModelIds().count(mId) > 0) {
    SG_LOG_INFO << "Creating annotations point cloud for: " << mId;
    const interaction::ModelAnnotation& mann = db.getAnnotation(mId);
    for (const auto& entry : mann.entries) {
      const interaction::ModelAnnotation::Annotation& ann = entry.second;
      vec<ml::vec3f>& pts = typeToPts[ann.type];
      for (const auto& p : ann.points) {
        pts.push_back(mInst.model.modelTransform * p.pos);
      }
    }
  }
  for (const auto& typeAndPts : typeToPts) {
    const string type = typeAndPts.first;
    const auto& pts = typeAndPts.second;
    ml::TriMeshf box = ml::Shapesf::box(0.02f);
    ml::TriMeshf mesh = ml::meshutil::createPointCloudTemplate(box, pts);
    mesh.setColor(cIndex->color(type));
    mesh.setHasColors(true);
    meshes.push_back(mesh);
  }
  *outMesh = ml::meshutil::createUnifiedMesh(meshes);
}

void D3D11Vizzer::initModelInteractionMaps(ml::GraphicsDevice& graphics) {
  // Initialize InteractionMaps and their meshes
  const size_t numModelInstances = m_modelInstances.size();
  m_modelInteractionMaps.resize(numModelInstances);
  m_modelInteractionMapMeshes.resize(numModelInstances);
  for (size_t i = 0; i < numModelInstances; ++i) {
    const auto& mInst = m_modelInstances[i];
    if (!m_modelInteractionMaps[i]) {
      // Allocate some memory for our interaction map
      m_modelInteractionMaps[i] = make_shared<interaction::InteractionMap>();
    }
    m_modelInteractionMaps[i]->init(mInst.model);
    ml::TriMeshf mesh = mInst.model.flattenedMesh;
    if (m_state.showOIMAnnotations) {
      addAnnotationPointCloud(m_state.database->modelAnnotations, mInst, &m_interactionLabelColors, &mesh);
    }
    const geo::Transform& xform = mInst.getLocalTransform();
    //xform.translate(geo::Vec3f(0, 0, 2.5f));
    mesh.transform(geo::to<ml::mat4f>(xform));
    m_modelInteractionMapMeshes[i].load(graphics, mesh);
  }
}

void D3D11Vizzer::updateModelInteractionMaps(const SkelRange& skelRange) {
  for (size_t i = 0; i < m_modelInteractionMaps.size(); ++i) {
    const core::ModelInstance& mInst = m_modelInstances[i];
    interaction::InteractionMap& imap = *m_modelInteractionMaps[i];
    imap.reset();
    const float maxGazeDist = m_params->get<float>("Interaction.maxDistGaze");
    imap.addInteraction(mInst, skelRange, m_activationRadius, maxGazeDist);
    const ml::vec4f gray(0.78f, 0.78f, 0.78f, 1.f);
    const ml::vec4f color = mInst.color; //ml::ColorUtils::colorById<ml::vec4f>(static_cast<int>(i));
    vec<ml::vec4f> colors;
    const size_t numVerts = mInst.model.flattenedMesh.getVertices().size();
    if (m_state.showActivationMap) {
      colors = vec<ml::vec4f>(numVerts, gray);
      imap.colorize(mInst, &colors);
    } else {
      colors = vec<ml::vec4f>(numVerts, color);
    }
    ml::D3D11TriMesh& imapMesh = m_modelInteractionMapMeshes[i];
    imapMesh.updateColors(colors);
  }
}

void D3D11Vizzer::scoreAndColorModelSegments(ml::GraphicsDevice& graphics, bool resegment,
                                             const SegmentScoringFn& scoringFn) {
  size_t numModelInstances = m_modelInstances.size();
  m_segmentedModelInstanceMeshes.resize(numModelInstances);
  vec<int> none;

  if (resegment) {
    // Mark old segmentation as being invalid...
    m_state.database->models.clearSegmentation();
  }
  
  for (size_t i = 0; i < numModelInstances; i++) {
    auto& mInst = m_modelInstances.at(i);
    // Make sure model is segmented
    m_state.database->models.ensureSegmentation(mInst.model.id, m_segParams, m_state.loadModelSegmentation);
    const auto& model = mInst.model;

    if (model.hasSegmentsLoaded()) {
      // Create colorized mesh for model instance
      auto& segMesh = m_segmentedModelInstanceMeshes[i];
      segmentation::VecSegPtr segPtrs;
      mInst.getSegments(&segPtrs, true, 0);
      SG_LOG_INFO << "Got " << segPtrs.size() << " segments";
      vec<double> scores;
      scoringFn(segPtrs, &scores);
      SG_LOG_INFO << "Got " << scores.size() << " scores";
      vec<ml::vec4f> colors(model.flattenedMesh.getVertices().size());
      segmentation::getSegmentationColors(model.flattenedMesh, scores, m_gradientCool2Warm, &colors);
      SG_LOG_INFO << "Got " << colors.size() << " colors";

      // Transform flattened mesh into world space
      ml::TriMeshf mesh = model.flattenedMesh;
      const geo::Transform& xform = mInst.getLocalTransform();
      mesh.transform(geo::to<ml::mat4f>(xform));
      segMesh.load(graphics, mesh);
      segMesh.updateColors(colors);
    } else {
      cerr << "Error updating segments for " << model.id << endl;
    }
  }
}

void D3D11Vizzer::updateModelSegmentations(ml::GraphicsDevice& graphics, bool resegment) {
  size_t numModelInstances = m_modelInstances.size();
  m_segmentedModelInstanceMeshes.resize(numModelInstances);
  vec<int> none;

  if (resegment) {
    // Mark old segmentation as being invalid...
    m_state.database->models.clearSegmentation();
  }
  
  for (size_t i = 0; i < numModelInstances; i++) {
    auto& mInst = m_modelInstances.at(i);
    // Make sure model is segmented
    m_state.database->models.ensureSegmentation(mInst.model.id, m_segParams, m_state.loadModelSegmentation);
    const auto& model = mInst.model;

    if (model.hasSegmentsLoaded()) {
      // Create colorized mesh for model instance
      auto& segMesh = m_segmentedModelInstanceMeshes[i];
      vec<ml::vec4f> colors(model.flattenedMesh.getVertices().size());
      segmentation::getSegmentationColors(model.flattenedMesh, none, &colors);

      // Transform flattened mesh into world space
      ml::TriMeshf mesh = model.flattenedMesh;
      const geo::Transform& xform = mInst.getLocalTransform();
      mesh.transform(geo::to<ml::mat4f>(xform));
      segMesh.load(graphics, mesh);
      segMesh.updateColors(colors);
    } else {
      cerr << "Error updating segments for " << model.id << endl;
    }
  }
}

void D3D11Vizzer::updateModelVoxels(ml::GraphicsDevice& graphics) {
  util::Timer timer("updateModelVoxels");
  size_t numModelInstances = m_modelInstances.size();
  m_voxelizedModelInstanceMeshes.resize(numModelInstances);

  for (size_t iInst = 0; iInst < numModelInstances; iInst++) {
    auto& mInst = m_modelInstances.at(iInst);
    // Make sure model is voxelized
    m_state.database->models.ensureVoxelization(mInst.model.id);
    const auto& model = mInst.model;

    if (model.hasSolidVoxelsLoaded()) {
      // Create mesh for model voxelization
      const ml::mat4f worldToVoxelInv = geo::to<ml::mat4f>(mInst.getLocalTransform()) * model.voxelToModel;
      ml::TriMeshf voxelMesh(model.solidVoxels, worldToVoxelInv);
      voxelMesh.computeNormals();
      voxelMesh.setColor(ml::vec4f(1, 0, 0, 1));

      auto& voxMesh = m_voxelizedModelInstanceMeshes[iInst];
      voxMesh.load(graphics, voxelMesh);
    } else {
      cerr << "Error loading voxels for " << model.id << endl;
    }
  }
}

void D3D11Vizzer::setCurrentInteractions(const float t) {
  const auto& allInteractions = m_state.currRecording->interactions;
  const float tolerance = 0.001f;
  if (allInteractions.empty()) { return; }
  m_currInteractions.clear();
  for (const auto it:allInteractions) {
    if (t >= it->startTime - tolerance && t <= it->endTime + tolerance) {
      m_currInteractions.push_back(it);
    }
  }
  //cout << "Time " << t << ", current interactions: " << m_currInteractions.size() 
  //  << ", all: " << allInteractions.size() << endl;
  m_currInteraction = m_currInteractions.begin();
  if (m_currInteraction != m_currInteractions.end()) {  // Also set skeleton index if non-empty interactions
    const auto& skeletons = (*m_currInteraction)->skelRange;
    const auto& it = find_if(skeletons.begin(), skeletons.end(),
    [&] (const Skeleton& s) { return s.timestamp >= m_currentSkelIt->timestamp; });
    if (it == skeletons.end()) {
      m_currInteractionSkelIdx = -1;  // Fell out of range
    } else {
      m_currInteractionSkelIdx = static_cast<int>(distance(skeletons.begin(), it));
    }
  }
}

void D3D11Vizzer::renderSegmentOBBs(ml::GraphicsDevice& graphics) {
  if (!m_state.currScan) { SG_LOG_WARN << "[renderSegmentOBBs]: No scan available"; return; }
  // ReSharper disable once CppDefaultInitializationWithNoUserConstructor
  const core::SegmentFeatureGeneratorSimplistic gen;
  vec<ml::TriMeshf> meshes;
  const segmentation::VecSegPtr& segs = *m_state.currScan->segments;

  m_centroidActSegIndices.clear();
  //const auto& V = m_state.currScan->mesh.getVertices();
  for (size_t iSeg = 0; iSeg < segs.size(); iSeg++) {
    const segmentation::MeshSegment& seg = *segs[iSeg];
    if (seg.diagonalLength() >= m_segParams.minSegDiag) {
      ml::vec4f color = getSegmentColor(seg);
      //ml::vec4f color = (isSegmentSelected(seg.id)) ? ml::vec4f(1, 1, 1, 1)
      //                 : ml::ColorUtils::colorById<ml::vec4f>(static_cast<int>(V[seg.elements[0]].texCoord[0]));

      float jitter = 0.01f;
      if (m_state.database && m_state.showClusterAssignment &&
          m_state.database->centroids.getCentroidSetForAllJoints() != nullptr &&
          m_state.database->centroids.getCentroidSetForAllJoints()->k > 0) {
        const stats::CentroidSet centroids = *m_state.database->centroids.getCentroidSetForAllJoints();
        const vec<double> centroidWhite = centroids.getCentroid(m_centroidIdx);
        const vec<double> centroidOrig = centroids.invertWhitening(centroidWhite, 1);
        const vec<double> feats = gen.generate(seg);
        double dist = 0;
        for (size_t i = 0; i < centroidOrig.size(); ++i) {
          const double d = centroidOrig[i] - feats[i];
          dist += d * d;
        }

        if (m_centroidActiveSegIdx == m_centroidActSegIndices.size()) {
          color = ml::vec4f(0, 0, 1, 1);
        }
        const double similarity = std::max(1 - (dist / m_centroidMaxDist), 0.0);
        const auto rgb = m_gradientHeatmap.value(similarity);
        color = ml::vec4f(rgb.r / 255.f, rgb.g / 255.f, rgb.b / 255.f, 1.f);
        if (dist < m_centroidMaxDist) {
          m_centroidActSegIndices.push_back(iSeg);
        }
        jitter = 0.0f;
      }
      meshes.push_back(vis::OBBwireframe(*seg.obb(), color, jitter));
    }
  }
  float jitter = 0.01f;
  for (const auto& mInst : m_modelInstances) {
    int i = 0;
    for (const auto& seg : *mInst.model.segments) {
      ml::TriMeshf mesh = vis::OBBwireframe(*seg->obb(), ml::ColorUtils::colorById<ml::vec4f>(i), jitter);
      mesh.transform(geo::to<ml::mat4f>(mInst.getLocalTransform()));
      meshes.push_back(mesh);
      ++i;
    }
  }

  vs.mesh["OBBs"].load(graphics, ml::meshutil::createUnifiedMesh(meshes));
}

void D3D11Vizzer::renderSegmentGroupOBBs(ml::GraphicsDevice& graphics) {
  if (!m_state.currScan) { SG_LOG_WARN << "[renderSegmentGroupOBBs]: No scan available"; return; }
  vec<ml::TriMeshf> meshes;
  float jitter = 0.01f;

  for (int i = 0; i < m_state.currScan->segmentGroups.size(); ++i) {
    const auto segGroupPtr = m_state.currScan->segmentGroups.at(i);    
    ml::vec4f color = getSegmentGroupColor(*segGroupPtr);
    meshes.push_back(vis::OBBwireframe(*segGroupPtr->obb(), color, jitter));
  }

  vs.mesh["segGroupOBB"].load(graphics, ml::meshutil::createUnifiedMesh(meshes));
}

void D3D11Vizzer::renderObjectOBBs(ml::GraphicsDevice& graphics) {
  if (!m_state.currScan) { SG_LOG_WARN << "[renderObjectOBBs]: No scan available"; return; }
  vec<ml::TriMeshf> meshes;
  float jitter = 0.01f;

  const auto& objMap = m_state.currScan->segmentGroups.getObjectMap();

  for (const auto& pair : objMap) {
    const auto objPtr = pair.second;
    ml::vec4f color = getObjectColor(*objPtr);
    meshes.push_back(vis::OBBwireframe(*objPtr->obb(), color, jitter));
  }

  vs.mesh["objOBB"].load(graphics, ml::meshutil::createUnifiedMesh(meshes));
}

void D3D11Vizzer::setCurrentTime(ml::ApplicationData& app, const float tNowInSec) {
  // TODO(ms): Factor out and generalize
  m_state.timeNowSec = tNowInSec;

  // We use ratio to seek instead of time in order to counteract dropped frames (assume uniform dropping)
  const double ratioSkel = m_state.timeNowSec / m_state.timeMaxSec;  // [0,1] time ratio on skeleton stream
  auto ratioSkel2ratioVid = [&](const double rSkel, const vec<int64_t>& ts) {
    const int64_t delta = m_state.currRecording->skeletons[0].timestamp - ts[0];
    MLIB_ASSERT_STR(delta >= 0, "D3D11Vizzer: non-monotonically increasing timestamps for skeletons")
    //const int64_t durationSkel = m_state.currRecording->skeletons.back().timestamp -
    //                             m_state.currRecording->skeletons[0].timestamp;
    const int64_t durationVid = ts.back() - ts[0];
    const double deltaRatioVid = static_cast<double>(delta) / durationVid;
    // WARNING + TODO(ms): Assumes that ending times for video and skeleton are the same
    return  deltaRatioVid + rSkel * (1 - deltaRatioVid);
  };

  if (m_state.showColorFrames) {
    cvutil::VideoPlayer& colorPlayer = m_state.colorPlayer;
    const double ratioVid = ratioSkel2ratioVid(ratioSkel, m_state.currRecording->colorTimestamps);
    if (colorPlayer.isOpened() && colorPlayer.seekToRatio(ratioVid)) {
      const bool isRead = colorPlayer.read(m_color);
      cv::flip(m_color, m_color, 1);
      if (isRead) { cv::imshow("Color", m_color);  cv::waitKey(1); }
    }
  }
  if (m_state.showDepthFrames) {
    cvutil::VideoPlayer& depthPlayer = m_state.depthPlayer;
    const double ratioVid = ratioSkel2ratioVid(ratioSkel, m_state.currRecording->depthTimestamps);
    if (depthPlayer.isOpened() && depthPlayer.seekToRatio(ratioVid)) {
      const bool isRead = depthPlayer.read(m_depthBodyIndex);
      cv::flip(m_depthBodyIndex, m_depthBodyIndex, 1);
      if (isRead) { cv::imshow("Depth", m_depthBodyIndex);  cv::waitKey(1); }
    }
  }

  //cout << m_state.timeNowSec << "s" << ", ratio=" << ratioSkel <<endl;
  if (!m_state.currRecording->getSkeletonAtTime(m_state.timeNowSec, m_currentSkelIt)) {
    // Failed to get skeleton so set to beginning
    m_currentSkelIt = m_state.currRecording->skeletons.begin();
  }

  setCurrentInteractions(tNowInSec);

  updateVisualizations(app);

  if (m_state.showKnnIGSim) {
    vizutil::reportInteractionSetKNNIGSim(*m_currentSkelIt, m_state, *m_params);
  }

  if (m_state.showPIGSim) {
    vizutil::reportInteractionSetPIGSim(*m_currentSkelIt, m_state, *m_params);
  }
}

void D3D11Vizzer::dumpMesh(const string& filename) {
  ml::MeshDataf meshData = vs.mesh["scan"].getMeshData();
  //meshData.computeVertexNormals();
  SG_LOG_INFO << "writing mesh " << filename << "... ";
  io::ensurePathToFileExists(filename);
  ml::MeshIOf::saveToFile(filename, meshData);
  SG_LOG_INFO << "writing mesh done!";
}

bool D3D11Vizzer::setLabelOptsFromArgs(const std::vector<string>& args,
                                       const sg::interaction::InteractionSet* pISet,
                                       sg::core::synth::LabelOpts* opts,
                                       sg::core::OccupancyGrid::OccupancyType* occType) {
  bool showLabeled = false;
  opts->labelStrategy = sg::core::synth::kLabelsAnnotation;
  opts->labelType = sg::core::synth::LabelType::kLabelTypeCategory;
  opts->includeUnlabeled = true;
  opts->includeUnknownOccupancy = true;
  opts->interactionSetId = (pISet != nullptr)? pISet->id : "";

  // Set poser params (used when we need to predict action)
  opts->pPoserParams = std::make_shared<sg::core::synth::SkeletonPoserParams>();
  opts->pPoserParams->init(*m_state.database->params);
  //opts->pPoserParams->selectSkelStrategy = params.selectSkelStrategy;
  opts->pPoserParams->classifierType = m_state.currClassifierType;
  //opts->pPoserParams->pSkels = params.pSkels;

  for (int i = 1; i < args.size(); ++i) {
    if (args[i] == "labeled") {
      showLabeled = true;
    } else if (args[i] == "part")  {
      showLabeled = true;
      opts->labelType = sg::core::synth::kLabelTypePart;
    } else if (args[i] == "object")  {
      showLabeled = true;
      opts->labelType = sg::core::synth::kLabelTypeObjectId;
    } else if (args[i] == "models") {
      showLabeled = true;
      opts->labelStrategy = sg::core::synth::kLabelsModels;
    } else if (args[i] == "skeleton") {
      showLabeled = true;
      opts->labelStrategy = sg::core::synth::kLabelsSkeleton;
    } else if (args[i] == "recording") {
      showLabeled = true;
      opts->labelStrategy = sg::core::synth::kLabelsRecording;
    } else if (args[i] == "recordingsAll") {
      showLabeled = true;
      opts->labelStrategy = sg::core::synth::kLabelsRecordingsAll;
    } else if (args[i] == "predict") {
      showLabeled = true;
      opts->labelStrategy = sg::core::synth::kLabelsPredict;
    } else if (args[i] == "unknown") {
      *occType = sg::core::OccupancyGrid::OccupancyType_Unknown;
      opts->includeUnknownOccupancy = true;
    } else if (args[i] == "unknownOrOccupied") {
      *occType = sg::core::OccupancyGrid::OccupancyType_UnknownOrOccupied;
      opts->includeUnknownOccupancy = true;
    } else {
      SG_LOG_ERROR << "Unknown arg: " << args[i] << endl;
    }
  }

  return showLabeled;
};

void D3D11Vizzer::showScanVoxels(ml::ApplicationData& app, const core::OccupancyGrid::OccupancyType occType,
                                 bool useLabels, const LabelOpts& labelOpts) {
  using core::OccupancyGrid;
  if (!m_state.currScan) { SG_LOG_WARN << "[showScanVoxels]: No scan available"; return; }
  const auto& scan = *m_state.currScan;
  //const core::synth::ModelPlacer& modelPlacer = m_interactionSynth.getModelPlacer();
  string outputBasename = m_params->get<string>("workDir") + "labeler/test";
  sg::core::synth::InteractionSynthParams::VisualizeHeatMapFn visualizeHeatMap =
    [&] (const sg::vis::PoseHeatMap& heatMap) {
      updateInteractionHeatMap(app.graphics, heatMap, false);
      evaluateInteractionHeatMap(app, heatMap, outputBasename + "/label", "", nullptr);
      return 0;
    };
  if (labelOpts.pPoserParams) {
    labelOpts.pPoserParams->pVisualizeHeatMapFn = &visualizeHeatMap;
  }

  SG_LOG_INFO << "showScanVoxels: occType=" << occType 
    << ", useLabels=" << useLabels << ", labelOpts=" << labelOpts;
  if (useLabels) {
    if (labelOpts.labelStrategy == core::synth::kLabelsRecording || 
        labelOpts.labelStrategy == core::synth::kLabelsRecordingsAll ||
        labelOpts.labelStrategy == core::synth::kLabelsSkeleton || 
        labelOpts.labelStrategy == core::synth::kLabelsPredict) {
      // Need to have interaction sets and PIGs
      ensureDatabase();
    }
    const core::OccupancyGrid& occ = scan.getOccupancyGrid();
    core::LabeledGrid labeledGrid(occ.voxelSize(), occ.getDimensions(), occ.worldToGrid());
    const auto currentSkel = getActiveSkeleton();
    if (labelOpts.labelStrategy == core::synth::kLabelsModels) {
      m_state.database->getLabeler().labelVoxels(scan, m_modelInstances, labelOpts.labelType, &labeledGrid);
    } else {
      m_state.database->getLabeler().labelVoxels(scan, currentSkel, m_state.currRecording, labelOpts, &labeledGrid);
    }
    SG_LOG_INFO << "showScanVoxels: labeled=" << labeledGrid.getGrid().size();

    arr<vis::ColorIndex*,sg::core::synth::kLabelTypeCount> colorIndices =
      { &m_categoryColors, &m_partLabelColors, &m_objectIdColors };
    vis::ColorIndex* pColorIndex = colorIndices[labelOpts.labelType]; 
    ml::TriMeshf gridMesh = vizutil::labeledScanVoxelsToTriMesh(labeledGrid, pColorIndex);
    vs.mesh["scanVoxelsLabeled"].load(app.graphics, gridMesh);
    pColorIndex->saveColorLegend(m_params->get<string>("workDir") + "labelColors.html");
    pColorIndex->saveCSV(m_params->get<string>("workDir") + "labelColors.csv");
  } else {
    ml::TriMeshf gridMesh = vizutil::scanVoxelsToTriMesh(scan,  occType);
    vs.mesh["scanVoxels"].load(app.graphics, gridMesh);
  }
}

const interaction::InteractionSet* D3D11Vizzer::getCurrentInteractionSet() const {
  if (m_state.inPosingMode) {  // If in posing mode use skelExplorer to get current pose
    return &m_pIsetPoseController->currentInteractionSet();
  } else if (!m_currInteractions.empty()) {  // Assume in replay mode with available interaction
    const interaction::Interaction& interaction = **m_currInteraction;
   return interaction.interactionSet;
  } else {
    return nullptr;
  }
}

void D3D11Vizzer::retrieveAndPlace(ml::ApplicationData& app, core::synth::ModelPlacerParams& params) {
  // Retrieve appropriate skeleton range
  SkelRange skelRange;
  if (m_state.inPosingMode) {
    skelRange = SkelRange({m_pIsetPoseController->currentSkeleton()});
  } else if (params.useSkelRange && m_currInteraction != m_currInteractions.end()) {
    const interaction::Interaction& interaction = **m_currInteraction;
    skelRange = interaction.skelRange;
  } else {
    skelRange = SkelRange({*m_currentSkelIt});
  }

  params.pScan = m_state.currScan;
  params.pSkelRange = &skelRange;
  params.pInteractionSet = nullptr;

  // Set InteractionSet if available
  params.pInteractionSet = getCurrentInteractionSet();

  core::synth::ModelAlignerParams::RenderAlignmentStateFn renderCallbackFun
    = [&] (const core::synth::AlignmentState& state) {
    renderAlignmentState(app, state);
  };
  params.alignParams.pRenderFun = (params.debugViz) ? &renderCallbackFun : nullptr;

  const core::synth::ModelPlacer& modelPlacer = m_interactionSynth.getModelPlacer();
  modelPlacer.place(params, &m_modelInstances);

  // Re-initialize InteractionMaps and their meshes, the model segmentations and voxels
  initModelInteractionMaps(app.graphics);
  updateModelSegmentations(app.graphics);
  updateModelVoxels(app.graphics);
}

double D3D11Vizzer::interactionSynth(ml::ApplicationData& app, const vec<string>& args) {
  ensureDatabase();

  core::synth::InteractionSynthParams isParams;
  // Set defaultsv
  isParams.pScan = nullptr;
  isParams.classifierType = m_state.currClassifierType;
  isParams.iscorerParams.init(*m_params);
  isParams.maxPoseIters = m_params->get<int>("ISynth.maxPoseIters");

  const interaction::InteractionSet* pCurrISet = getCurrentInteractionSet();
  if (pCurrISet != nullptr) {
    isParams.text = pCurrISet->id;
  } else {
    isParams.text = "";
  }

  // Parse arguments
  bool debugViz = false;
  bool logViz = false;
  bool keepExisting = false;
  vec<string> fields;
  string outputBasename = m_params->get<string>("workDir") + "isynth/test";
  for (const string& arg : args) {
    util::tokenize(arg, "=", &fields);
    const string& name = fields[0];
    const string& value = (fields.size() > 1)? fields[1] : "";
    if (name == "fromScan") {
      // Let's be from scene!  Set scene!
      isParams.pScan = m_state.currScan;
    } else if (name == "compose") {
      // Composite action
      isParams.doComposition = true;
    } else if (name == "retarget") {
      // Retarget action
      isParams.retarget = true;
    } else if (name == "backoff") {
      // Backoff action
      isParams.doBackoff = true;
    } else if (name == "skipModelPlacement") {
      // No model placement
      isParams.doModelPlacement = false;
    } else if (name == "predictSupportSurfaces") {
      isParams.predictSupportSurfaces = true;
    } else if (name == "predictSegmentLabels") {
      isParams.predictSegmentLabels = true;
    } else if (name == "debug") {
      debugViz = true;
    } else if (name == "log") {
      logViz = true;
    } else if (name == "keepExisting") {
      keepExisting = true;
    } else if (name == "naive") {
      isParams.placeWithPig = false;
    } else if (name == "parserType") {
      if (!value.empty()) {
        isParams.parserType = value;
      } else {
        SG_LOG_WARN << "No parserType specified: using " << isParams.parserType;
      }
    } else if (name == "selectModel") {
      if (!value.empty()) {
        if (value == "random") {
          isParams.selectModelStrategy = core::synth::SelectModelStrategy::kRandom;
        } else if (value == "first") {
          isParams.selectModelStrategy = core::synth::SelectModelStrategy::kFirst;
        } else {
          SG_LOG_WARN << "Invalid selectModel specified: using " << isParams.selectModelStrategy;
        }
      } else {
        SG_LOG_WARN << "No selectModel specified: using " << isParams.selectModelStrategy;
      }
    } else if (name == "selectSkel") {
      if (!value.empty()) {
        core::synth::SelectSkelStrategy s = core::synth::getSelectSkelStrategyFromString(value.c_str());
        if (s == core::synth::SelectSkelStrategy::kCount) {
          SG_LOG_WARN << "Invalid selectSkel specified: using " << isParams.selectSkelStrategy ;
        } else {
          isParams.selectSkelStrategy = s;
        }
      } else {
        SG_LOG_WARN << "No selectSkel specified: using " << isParams.selectSkelStrategy ;
      }
    } else if (name == "classifierType") {
      if (!value.empty()) {
        isParams.classifierType = value;
      } else {
        SG_LOG_WARN << "No classifierType specified: using " << isParams.classifierType;
      }
    } else if (name == "i") {
      // Input text
      isParams.text = value;
    } else if (name == "o") {
      // Output directory
      if (io::isAbsolutePath(value) || util::startsWith(value, m_params->get<string>("workDir"))) {
        outputBasename = value;
      } else {
        outputBasename = m_params->get<string>("workDir") + "isynth/" + value;
      }
    } else {
      SG_LOG_WARN << "Unknown option: " << arg;
    }
  }

  if (isParams.text.empty()) {
    SG_LOG_WARN << "No interaction set: Please specify interaction using i=";
    return 0.0;
  }

  if (isParams.pScan == nullptr && m_state.currScan != nullptr) {
    // Get bounding box of scan and set center with z at 0 at the targetPos
    isParams.targetPos = geo::Vec3f(m_state.currScan->bbox.getCenter());
    isParams.targetPos.z() = 0.0f;
  } else {
    isParams.targetPos = geo::Vec3f(0, 0, 0);
  }

  // Setup debug visualization and logging
  core::synth::InteractionSynthParams::VisualizeHeatMapFn visualizeHeatMap =
    [&] (const vis::PoseHeatMap& heatMap) {
      updateInteractionHeatMap(app.graphics, heatMap, false);
      evaluateInteractionHeatMap(app, heatMap, outputBasename + "/is", "", nullptr);
      return 0;
    };
  isParams.pVisualizeHeatMapFn = &visualizeHeatMap;
  core::synth::ModelAlignerParams::RenderAlignmentStateFn renderAlignmentCallbackFun
    = [&] (const core::synth::AlignmentState& state) {
    renderAlignmentState(app, state);
  };
  core::synth::ModelPlacerParams::RenderPlacementStateFn renderPlacementCallbackFun
    = [&] (const core::synth::PlacementState& state) {
    renderPlacementState(app, state);
  };
  isParams.pRenderAlignmentStateFn = (debugViz) ? &renderAlignmentCallbackFun : nullptr;
  isParams.pRenderPlacementStateFn = (debugViz) ? &renderPlacementCallbackFun : nullptr;
  vis::VisLog vislog;
  if (logViz) {
    isParams.pVislog = &vislog;
    vislog.init(outputBasename);
    util::add_file_log(vislog.getFilename("log.txt"));
  }
  double finalScore;
  isParams.pFinalScore = &finalScore;

  if (!keepExisting) {
    // Clear the current models
    clearScene(app);
  }

  // Synthesize
  vec<core::TransformedSkeleton> skels;
  m_interactionSynth.parseAndSynthesize(isParams, &skels, &m_modelInstances);

  // Show the skeleton
  if (skels.size() > 0) {
    SG_LOG_INFO << "Setting posed skeleton with score of " << skels[0].score;
    m_predictedInfo = "Posed " + interaction::VerbNoun::getVerbNounSetId(isParams.verbNouns);
    m_predictedSkeleton = skels[0].getSkeleton();
    m_showPredictedSkeleton = true;
    m_pIsetPoseController->setBaseSkeleton(m_predictedSkeleton);
  }

  // Re-initialize InteractionMaps and their meshes, the model segmentations and voxels
  SG_LOG_INFO << "update model interaction maps";
  initModelInteractionMaps(app.graphics);
  SG_LOG_INFO << "update model segmentations";
  updateModelSegmentations(app.graphics);
  SG_LOG_INFO << "update model voxels";
  updateModelVoxels(app.graphics);
  if (logViz) {
    vislog.close();
  }
  SG_LOG_INFO << "synth done!";
  if (logViz) {
    saveScene(vislog.getFilename("scene.json"), finalScore);
    savePLY(vislog.getFilename("scene.ply"), {"skeleton", "models"});
    util::remove_file_log(vislog.getFilename("log.txt"));
  }
  return finalScore;
}

void D3D11Vizzer::testInteractionSynth(ml::ApplicationData& app, const vec<string>& args) {
  const string defaultTestsFile = m_params->get<string>("localDataDir")
                                + m_params->get<string>("testISetsFile");
  const int numPerTest = args.size() > 0 ? stoi(args[0]) : 1;
  const string file = args.size() > 1 ? args[1] : defaultTestsFile;
  const string outDir = m_params->get<string>("workDir") + io::basename(file) + "/";
  bool useAllKnown = false;

  if (!io::fileExists(file)) {
    if (file == "known") {
      useAllKnown = true;      
    } else {
      SG_LOG_ERROR << "Could not find interaction synth test file: " << file;
      return;
    }
  }

  io::ensureDirExists(outDir);
  util::add_file_log(outDir + "log.txt");
  SG_LOG_INFO << "Writing snapshot tests to " << outDir;

  struct Entry {
    string type;
    string id;
  };
  vec<Entry> entries;
  if (useAllKnown) {
    SG_LOG_INFO << "Getting all known interaction sets";
    // Get entries from our list of interaction sets
    ensureDatabase();
    for (const auto& p : m_state.database->interactions.getAllInteractionSets()) {
      entries.push_back({"generalize", p.first});
    } 
  } else {
    // Read entries from file
    SG_LOG_INFO << "Reading test interaction sets from " << file;
    using io::csv::CSVReader;
    CSVReader<2, io::csv::trim_chars<' '>, io::csv::double_quote_escape<',','\"'> > csv(file);
    csv.read_header(io::csv::ignore_extra_column, "type", "id");
    string type, id;
    while (csv.read_row(type, id)) {
      entries.push_back({type,id});
    }
  }

  SG_LOG_INFO << "Preparing to test " << entries.size() << " entries, " << numPerTest << " each";
  for (const Entry& entry: entries) {
    vec<string> basicSynthArgs{
      "i=" + entry.id,
      "selectModel=random",
      "selectSkel=Average",
      "predictSupportSurfaces",
      "log"  // enables output
    };
    SG_LOG_INFO << "Generate interaction for: " << entry.type << " " << entry.id;
    if (entry.type == "generalize") {
      basicSynthArgs.push_back("backoff");
    } else if (entry.type == "retarget") {
      basicSynthArgs.push_back("retarget");
    } else if (entry.type == "compose") {
      basicSynthArgs.push_back("retarget");
      basicSynthArgs.push_back("compose");
    } else {
      SG_LOG_INFO << "Unknown entry type. Skipping: " << entry.id;
      continue;
    }
    for (int i = 0; i < numPerTest; ++i) {
      const string iDir = outDir + entry.id + "_" + to_string(i);
      vec<string> synthArgs = basicSynthArgs;
      synthArgs.push_back("o=" + iDir + "/");
      try {
        interactionSynth(app, synthArgs);
      } catch(std::exception& e) {
        SG_LOG_ERROR << "Exception at ISynth test i=" << i << " : " << e.what();
      }
      setCamera(app, CameraView_ISynth);
      // render out models and skeleton
      vec<ml::TriMeshf> modelMeshes;
      for (const auto& mInst : m_modelInstances) {
        ml::TriMeshf mesh = mInst.model.flattenedMesh;
        const geo::Transform& xform = mInst.getLocalTransform();
        mesh.transform(geo::to<ml::mat4f>(xform));
        mesh.setColor(mInst.color);
        modelMeshes.push_back(mesh);
      }
      vs.mesh["test"].load(app.graphics, ml::meshutil::createUnifiedMesh(modelMeshes));
      vs.mesh["skeleton"].load(app.graphics,
                                vis::toTriMesh(m_predictedSkeleton, true, true, false, false, false, false));
      vec<reference_wrapper<const ml::D3D11TriMesh>> meshesToRender;
      meshesToRender.push_back(vs.mesh["test"]);
      meshesToRender.push_back(vs.mesh["skeleton"]);
      renderToFile(app, meshesToRender, iDir + ".png");
    }
  }
  util::remove_file_log(outDir + "log.txt");
}

void D3D11Vizzer::renderAlignmentState(ml::ApplicationData& app, const core::synth::AlignmentState& state) {
  vec<ml::TriMeshf> meshes;
  const auto boxMesh = [ ] (const geo::BBox& bbox, const ml::vec4f& c) {
    ml::BoundingBox3f mlbbox(geo::to<ml::vec3f>(bbox.min()), geo::to<ml::vec3f>(bbox.max()));
    return ml::Shapesf::box(mlbbox, c);
  };
  const auto ptMesh = [] (const geo::Matrix3Xf& P, const ml::vec4f& c) {
    const auto box = ml::Shapesf::box(.02, c);
    const size_t numPts = P.cols();
    vec<ml::vec3f> pts(numPts);  vec<ml::vec4f> colors(numPts);
    for (size_t iP = 0; iP < numPts; iP++) {
      const geo::Vec3f& p = P.col(iP);
      pts[iP] = geo::to<ml::vec3f>(p);
      colors[iP] = c;
    }
    return ml::meshutil::createPointCloudTemplate(box, pts, colors);
  };

  meshes.push_back(boxMesh(state.segsBBox, ml::vec4f(1, 0, 0, 0.3)));
  //meshes.push_back(boxMesh(state.modelBBox, ml::vec4f(0, 0, 1, 0.3)));
  meshes.push_back(ptMesh(state.modelVoxelPts, ml::vec4f(0, 0, 1, 0.3)));
  if (state.pInst != nullptr) {
    state.pInst->setLocalTransform(state.xform);
    const geo::OBB mInstOBB = state.pInst->computeWorldOBB();
    meshes.push_back(vis::OBBwireframe(mInstOBB, ml::vec4f(0, 0, 1, 1)));
  }

  vs.mesh["test"].load(app.graphics, ml::meshutil::createUnifiedMesh(meshes));
  setCamera(app, CameraView_Top);
  renderFrame(app);
};

void D3D11Vizzer::renderPlacementState(ml::ApplicationData& app, const core::synth::PlacementState& state) {
  vec<ml::TriMeshf> meshes;

  if (state.pSkel) {
    const auto skelMesh = vis::toTriMesh(*state.pSkel, true, true, true, true, true);
    meshes.push_back(skelMesh);
  }
  if (state.pSkelOBB) {
    const auto skelOBBMesh = vis::OBBwireframe(*state.pSkelOBB, ml::vec4f(1,0,0,1));
    meshes.push_back(skelOBBMesh);
  }
  if (!state.placed.empty()) {
    for (const auto pInst : state.placed) {
      const auto& mInst = *pInst;
      // Model instance OBB
      const geo::OBB mInstOBB = mInst.computeWorldOBB();
      meshes.push_back(vis::OBBwireframe(mInstOBB, ml::vec4f(0, 0, 1, 1)));

      // Model Instance mesh
      ml::TriMeshf mesh = mInst.model.flattenedMesh;
      const geo::Transform& xform = mInst.getLocalTransform();
      mesh.transform(geo::to<ml::mat4f>(xform));
      meshes.push_back(mesh);
    }
  }
  if (!state.placementScores.empty()) {
    meshes.push_back(vizutil::createPointCloudMesh(state.placementScores, m_gradientCool2Warm,
                                                   ml::vec4f(0, 0, 1, 0.3)));
  }

  vs.mesh["test"].load(app.graphics, ml::meshutil::createUnifiedMesh(meshes));
  setCamera(app, CameraView_Top);
  renderFrame(app);
  if (state.pVislog != nullptr) {
    // Save to visualization log
    const string imagename = state.pVislog->getNextImageName();
    const string imageFilename = state.pVislog->getFilename(imagename);
    vec<reference_wrapper<const ml::D3D11TriMesh>> meshesToRender;
    meshesToRender.push_back(vs.mesh["test"]);
    renderToFile(app, meshesToRender, imageFilename);
    vis::VisImage image({imagename, "test"});
    state.pVislog->log(state.description, image);
  }
};
