#include "common.h"  // NOLINT

#include "./VizzerEventMap.h"

#include <string>

#include <core/ClassifierDatabase.h>
#include <core/MotionDatabase.h>
#include <core/PoseController.h>
#include <core/SkelState.h>
#include <core/SkeletonDatabase.h>
#include <interaction/ProtoInteractionGraph.h>
#include <interaction/InteractionFeatures.h>
#include <interaction/InteractionFrames.h>
#include <io/mitsuba_vol.h>
#include <segmentation/segmentation.h>
#include <segmentation/SurfacePredictor.h>

#include "./D3D11Vizzer.h"

using std::cout;  using std::cerr;  using std::endl;  using std::string;
using std::vector;

using sg::core::SkelState;  using sg::core::Skeleton;

typedef sg::ui::EventMap<string>::Tokens Tokens;

#define CHECK_ARGS(args, num)                         \
  if (args.size() < num) {                            \
    return sg::ui::ErrorCodes::E_INVALID_ARGS;        \
  }

void VizzerEventMap::populate(D3D11Vizzer* _viz, ml::ApplicationData& app) {
  using E = sg::ui::ErrorCodes;
  m_viz = _viz;
  SharedAppState& state = _viz->m_state;
  const sg::util::Params& params = *_viz->m_params;
  m_eventMap["D3D:help"] = [this] (const Tokens &args) {
    vector<string> keys;
    m_eventMap.getSignals(&keys);
    for (const auto& key : keys) {
      cout << key << endl;
    }
    return E::E_OK;
  };
  m_eventMap["D3D:setTestFun"] = [&] (const Tokens &args) {
    CHECK_ARGS(args, 2);
    vector<string> rest(args.begin() + 1, args.end());
    m_viz->vs.testFun = rest;
    cout << "Test command set to D3D:" << sg::util::join(rest, " ") << endl;
    return E::E_OK;
  };
  m_eventMap["D3D:printParams"] = [&] (const Tokens &args) {
    if (args.size() > 1) {
      const sg::util::Params& p = params.extract(args[1]);
      p.toJSON(cout) << endl;
    } else {
      params.toJSON(cout) << endl;
    }
    return E::E_OK;
  };
  m_eventMap["D3D:setParam"] = [this] (const Tokens &args) {
    // BEWARE
    // This command changes parameters!!!  It is not safe to randomly change parameters!!!
    CHECK_ARGS(args, 3);
    const string& name = args[1];
    const string& value = args[2];
    if (m_viz->m_params->exists(name)) {
      if (m_viz->m_params->isLeaf(name)) {
        m_viz->m_params->set<string>(name, value);
        const sg::util::Params& p = m_viz->m_params->extract(name);
        p.toJSON(cout) << endl;
        return E::E_OK;
      } else {
        cout << "Not leaf param: " << name;
        return E::E_INVALID_ARGS;
      }
    } else {
      cout << "Unknown param: " << name;
      return E::E_INVALID_ARGS;
    }
  };
  m_eventMap["D3D:predictSurfaces"] = [&] (const Tokens &args) {
    CHECK_ARGS(args, 2);
    const string& type = args[1];
    sg::segmentation::SurfacePredictor surfacePredictor;
    if (type == "flat") {
      std::reference_wrapper<const sg::geo::Direction> dir = sg::geo::DIR_UP;
      //= sg::geo::DIR_UP;
      if (args.size() > 2) {
        const string& dirName = args[2];
        if (sg::geo::kDirectionIndexMap.count(dirName) > 0) {
          dir = sg::geo::lookupDirection(dirName);
        } else {
          SG_LOG_WARN << "Unknown direction " << dirName;
        }
      }
      const auto scoringFn = [&] (const sg::segmentation::VecSegPtr& segPtrs, sg::vec<double>* pScores) {
        surfacePredictor.predictFlatSurfaces(segPtrs, dir.get().vec, pScores);
      };
      m_viz->scoreAndColorModelSegments(app.graphics, false, scoringFn);
    } else if (type == "support") {
      float bboxSz = 0.01f;
      if (args.size() > 2) {
        bboxSz = stof(args[2]);
      }
      const sg::geo::Vec3f c;
      const sg::geo::Vec3f r(bboxSz,bboxSz,bboxSz);
      const sg::geo::Matrix3f R = sg::geo::Matrix3f::Identity();
      sg::geo::OBB obb(c,r,R);
      const auto scoringFn = [&] (const sg::segmentation::VecSegPtr& segPtrs, sg::vec<double>* pScores) {
        surfacePredictor.predictSupportSurfaces(segPtrs, &obb, pScores);
      };
      m_viz->scoreAndColorModelSegments(app.graphics, false, scoringFn);
    } else {
      cerr << "Unknown surface prediction type " << type << endl;
    }
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:setColorSegmentBy"] = [&] (const Tokens &args) {
    CHECK_ARGS(args, 2);
    const string& mode = args[1];
    if (mode == "id") {
      state.colorSegmentBy = SharedAppState::Color_ById;
    } else if (mode == "label") {
      state.colorSegmentBy = SharedAppState::Color_ByLabel;
    } else if (mode == "category") {
      state.colorSegmentBy = SharedAppState::Color_ByCategory;
    } else if (mode == "objectId") {
      state.colorSegmentBy = SharedAppState::Color_ByObjectId;
    } else {
      cerr << "Unknown color segment by " << mode << endl;
    }
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:setShowScanLabeledVoxels"] = [&] (const Tokens &args) {
    CHECK_ARGS(args, 2);
    state.showScanLabeledVoxels = ml::convert::toBool(args[1]);
    if (state.showScanLabeledVoxels && m_viz->vs.mesh.count("scanLabeledVoxels") == 0) {
      m_viz->showScanVoxels(app, sg::core::OccupancyGrid::OccupancyType_Occupied, 
                             true, sg::core::synth::LabelOpts());
    }
    return E::E_OK;
  };
  m_eventMap["D3D:setShowScanVoxels"] = [&] (const Tokens &args) {
    CHECK_ARGS(args, 2);
    state.showScanVoxels = ml::convert::toBool(args[1]);
    if (state.showScanVoxels && m_viz->vs.mesh.count("scanVoxels") == 0) {
      m_viz->showScanVoxels(app, sg::core::OccupancyGrid::OccupancyType_Occupied,
                             false, sg::core::synth::LabelOpts());
    }
    return E::E_OK;
  };
  m_eventMap["D3D:makeSceneGray"] = [&] (const Tokens &args) {
    auto& VC = m_viz->m_baseVertexColors;
    float opacity = (args.size() > 1) ? stof(args[1]) : .7f;
    std::fill(VC.begin(), VC.end(), ml::vec4f(0.466f, 0.466f, 0.466f, opacity));
    m_viz->m_currVertexColors = VC;
    m_viz->m_vertexColorsDirty = true;
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:writeIFvol"] = [&] (const Tokens &args) {
    m_viz->ensureDatabase();
    const string isetId = args[1];
    const auto& iset = state.database->interactions.getInteractionSets().at(isetId).get();
    const string workDir = params.get<string>("workDir");
    //const auto pIF = m_viz->vs.iframes.getInteractionFrame(state.iframeType);
    const auto pIF = iset.protoInteractionFrame;
    if (pIF) {
      const auto& iframe = *pIF.get()->getInteractionFrame(state.iframeType);
      sg::vis::IFVisParams ifvp;
      ifvp.minBinWeight = m_viz->m_params->get<float>("InteractionFrame.minBinWeight");
      if (m_viz->m_params->get<bool>("InteractionFrame.useJointWeights")) {
        ifvp.pJointWeights = &iset.jointGroupWeights[0];
      }
      ifvp.renderObjectLabels = true; ifvp.renderOccupancy = true; ifvp.renderSkelPts = true;
      write_mitsuba_vol(iframe, ifvp, workDir + "if");
    }
    return E::E_OK;
  };
  m_eventMap["D3D:writePLY"] = [&] (const Tokens &args) {
    const string workDir = params.get<string>("workDir");
    int i = 0;  string plyFile;
    do {
      i++;
      plyFile = workDir + "scene" + std::to_string(i) + ".ply";
    } while (sg::io::fileExists(plyFile));
    m_viz->savePLY(plyFile, args);
    return E::E_OK;
  };

  m_eventMap["D3D:writeSkelPLYs"] = [&] (const Tokens &args) {
    if (!state.currRecording) {
      SG_LOG_ERROR << "No recording available!";
      return E::E_INVALID_ARGS;
    }

    const string workDir = params.get<string>("workDir");
    const string dir = workDir + "skels/" + state.currRecording->id + "/";
    const string basename = dir + "skel";
    sg::io::ensurePathToFileExists(basename);

    ml::mat4f yToZup;
    yToZup.setRotationX(-90);
    const auto& skels = state.currRecording->skeletons;
    const size_t numSkels = skels.size();
    cout << "dumping skeleton PLYs..." << endl;
    for (size_t i = 0; i < numSkels; i++) {
      const Skeleton& skel = skels[i];
      ml::TriMeshf mesh = sg::vis::toTriMesh(skel, true, true, false, false, false, false);
      mesh.transform(yToZup);
      string plyFile = basename + std::to_string(i) + ".ply";
      ml::MeshIOf::saveToPLY(plyFile, mesh.getMeshData());
      cout << plyFile << endl;
    }

    cout << "done" << endl;
    return E::E_OK;
  };

  m_eventMap["D3D:saveAverageSkels"] = [&] (const Tokens &args) {
    m_viz->ensureDatabase();
    const string workDir = params.get<string>("workDir");
    const string dir = workDir + "averageSkels/";
    const auto& skelDb = *state.database->skeletons;
    ml::mat4f yToZup;
    yToZup.setRotationX(90);
    sg::vis::IFVisParams ifvp;
    ifvp.renderSkelPts = false; ifvp.renderObjectLabels = false;  ifvp.renderOccupancy = false;
    ifvp.minBinWeight = m_viz->m_params->get<float>("InteractionFrame.minBinWeight");

    const auto& isets = state.database->interactions.getAllInteractionSets();
    vector<string> ids;  sg::util::mapToKeyVec(isets, ids);
    for (const string& id : ids) {
      const auto& iset = isets.at(id).get();
      Skeleton skel;
      skelDb.getAverageSkeleton(iset.id, &skel);
      skel = sg::core::TransformedSkeleton(skel, yToZup).getSkeleton();
      auto meshWeights = vizutil::skelToMeshWithJointWeights(skel, iset.jointGroupWeights, m_viz->m_gradientHeatmap);
      //meshWeights.transform(yToZup);
      auto mesh = sg::vis::toTriMesh(skel, true, true, false, false, false, false);
      //mesh.transform(yToZup);
      sg::interaction::InteractionFrame iframe = *iset.protoInteractionFrame->getInteractionFrame(sg::interaction::InteractionFrameType::kJointsSurface).get();
      iframe.reposition(skel);
      if (m_viz->m_params->get<bool>("InteractionFrame.useJointWeights")) {
        ifvp.pJointWeights = &iset.jointGroupWeights[0];
      }
      ml::TriMeshf iframeMesh = iframeWidget(iframe, ifvp);
      //iframeMesh.transform(ztoyup);
      const string basename = dir + iset.id;
      sg::io::ensurePathToFileExists(basename);
      SG_LOG_INFO << "Writing average skel for " << iset.id;
      ml::MeshIOf::saveToPLY(basename + ".ply", mesh.getMeshData());
      ml::MeshIOf::saveToPLY(basename + ".weights.ply", meshWeights.getMeshData());
      ml::MeshIOf::saveToPLY(basename + ".iframe.ply", iframeMesh.getMeshData());
    }
    return E::E_OK;
  };

  m_eventMap["D3D:saveSkelDistributions"] = [&] (const Tokens &args) {
    m_viz->ensureDatabase();
    const string workDir = params.get<string>("workDir");
    const string dir = workDir + "averageSkels/";
    const auto& skelDb = *state.database->skeletons;
    const auto& isets = state.database->interactions.getAllInteractionSets();
    vector<string> ids;  sg::util::mapToKeyVec(isets, ids);
    for (const string& id : ids) {
      if (id[0] != 'v') { continue; }
      const auto& iset = isets.at(id).get();
      vector<std::pair<Skeleton, double>> skels;
      skelDb.sampleSkeletons(iset.id, 100, &skels);
      // normalize lps
      double minLP = 1e60, maxLP = -1e60;
      for (const auto& p : skels) {
        if (p.second < minLP) minLP = p.second;
        if (p.second > maxLP) maxLP = p.second;
      }
      for (auto& p : skels) {
        p.second = (p.second - minLP) / (maxLP - minLP);
      }
      Skeleton avgSkel;
      skelDb.getAverageSkeleton(iset.id, &avgSkel);
      ml::TriMeshf avgMesh = sg::vis::toTriMesh(avgSkel, true, true, false, false, false, false);
      vector<ml::TriMeshf> meshes;
      const auto box = ml::Shapesf::box(0.01f, Skeleton::kBoneColor);
      const int numPtsPerSkel = 100;
      for (const auto& p : skels) {
        const Skeleton& s = p.first;
        const double prob = p.second;
        const vector<ml::vec3f> pts = s.getPointsInOBBs(numPtsPerSkel);
        //const ml::RGBColor c = m_viz->m_gradientHeatmap.value(prob);
        //vector<ml::vec4f> colors(numPtsPerSkel, ml::vec4f(c.x/255.f, c.y/255.f, c.z/255.f, 1.f));
        const ml::TriMeshf ptCloud = ml::meshutil::createPointCloudTemplate(box, pts);
        meshes.push_back(ptCloud);
      }
      //meshes.push_back(avgMesh);
      const ml::TriMeshf mesh = ml::meshutil::createUnifiedMesh(meshes);
      const string basename = dir + iset.id;
      sg::io::ensurePathToFileExists(basename);
      SG_LOG_INFO << "Writing average skel for " << iset.id;
      ml::MeshIOf::saveToPLY(basename + ".distribution.ply", mesh.getMeshData());
    }
    return E::E_OK;
  };


  m_eventMap["D3D:saveScanPLYs"] = [&] (const Tokens &args) {
    sg::vec<string> scanIds;
    const size_t numArgs = args.size();
    if (numArgs > 1) {  // save each scan id given
      for (size_t iArg = 1; iArg < numArgs; ++iArg) {
        scanIds.push_back(args[iArg]);
      }
    } else {  // save all scans
      scanIds = state.database->scans.scanIds();
    }
    state.database->scans.saveScansAsPLYs(scanIds);

    return E::E_OK;
  };
  m_eventMap["D3D:saveScanBinvoxes"] = [&] (const Tokens &args) {
    sg::vec<string> scanIds;
    const size_t numArgs = args.size();
    if (numArgs > 1) {  // save each scan id given
      for (size_t iArg = 1; iArg < numArgs; ++iArg) {
        scanIds.push_back(args[iArg]);
      }
    } else {  // save all scans
      scanIds = state.database->scans.scanIds();
    }
    state.database->scans.saveScanBinvoxes(scanIds);

    return E::E_OK;
  };
  m_eventMap["D3D:cacheAllModels"] = [&] (const Tokens &args) {
    state.database->models.ensureAllModelsCached();
    return E::E_OK;
  };
  m_eventMap["D3D:computeModelStats"] = [&] (const Tokens &args) {
    string file = (args.size() > 1) ? args[1] : "modelStats.csv";
    state.database->models.computeModelStats(params.get<string>("workDir") + "/" + file);
    return E::E_OK;
  };
  m_eventMap["D3D:computeModelOverlap"] = [&] (const Tokens &args) {
    // Compute overlap of models int the scene
    float overlap = sg::core::voxelOverlapRaw(m_viz->m_modelInstances);
    SG_LOG_INFO << "Overall model overlap: " << overlap;
    return E::E_OK;
  };

  /// SKELETON DATABASE ///
  m_eventMap["D3D:isetScores"] = [&] (const Tokens &args) {
    m_viz->ensureDatabase();
    const auto& scoredIsets = state.database->skeletons->predictInteractionSetLikelihoods(*m_viz->m_currentSkelIt);
    for (const auto& i : scoredIsets) {
      SG_LOG_INFO << i.first << " : " << i.second;
    }
    return E::E_OK;
  };
  m_eventMap["D3D:sampleSkel"] = [&] (const Tokens &args) {
    m_viz->ensureDatabase();
    Skeleton skel;
    const string isetId = (args.size() > 1) ? args[1] : "";
    const bool ok = state.database->skeletons->sampleSkeleton(isetId, &skel);
    if (!ok) { return E::E_INVALID_ARGS; }
    m_viz->m_pIsetPoseController->setAutoupdateSkeleton(false);
    m_viz->m_pIsetPoseController->setBaseSkeleton(skel);
    //const auto& mesh = vizutil::createMeshFromSkeleton(skel, true);
    //m_viz->vs.mesh["test"].load(app.graphics, mesh);

    return E::E_OK;
  };
  m_eventMap["D3D:averageSkel"] = [&] (const Tokens &args) {
    m_viz->ensureDatabase();
    Skeleton skel;
    const string isetId = (args.size() > 1) ? args[1] : "";
    const auto& skelDb = *state.database->skeletons;
    const bool ok = skelDb.getAverageSkeleton(isetId, &skel);
    if (!ok) { return E::E_INVALID_ARGS; }
    m_viz->m_pIsetPoseController->setAutoupdateSkeleton(false);
    m_viz->m_pIsetPoseController->setBaseSkeleton(skel);
    const auto& iset = state.database->interactions.getAllInteractionSets().at(isetId).get();
    const auto mesh = vizutil::skelToMeshWithJointWeights(skel, iset.jointGroupWeights, m_viz->m_gradientHeatmap);
    m_viz->vs.mesh["test"].load(app.graphics, mesh);

    return E::E_OK;
  };
  m_eventMap["D3D:readASF"] = [&] (const Tokens &args) {
    auto& mdb = *m_viz->m_state.database->motions;
    if (args.size() > 1) {
      const string str = args[1];
      vector<string> ids;
      if (ml::util::startsWith(str, "f")) {  // motion id
        ids.push_back(str.substr(1));
      } else {  // keyword search
        ids = mdb.getMotionIdsByKeyword(str);
      }
      for (const string id : ids) {
        mdb.getMotionInfo(id).toJSON(cout);
        cout << endl;
      }
      if (ids.size()) {
        const auto& rec = mdb.getMotion(ids[0]);
        m_viz->m_state.database->recordings.addRecording(rec, rec.id);
        m_viz->initRecording(app, rec.id, false, D3D11Vizzer::CameraView_Current);
      }
    } else {
      mdb.ensureAllMotionsLoaded();
      sg::core::SkeletonDatabase sdb(*m_viz->m_params);
      sdb.init(mdb);
      const string cacheFile = m_viz->m_params->get<string>("dataCacheDir") + "motionsSkelDb.cached";
      sdb.saveBinary(cacheFile);
    }
    return E::E_OK;
  };

  m_eventMap["D3D:saveInteractionSummary"] = [&] (const Tokens &args) {
    state.database->interactions.saveInteractionSummary(
      m_viz->m_params->get<string>("workDir") + "interactions.summary.csv");
    state.database->interactions.saveInteractionRecordSummary(
      m_viz->m_params->get<string>("workDir") + "interactionRecords.summary.csv");
    return E::E_OK;
  };

  m_eventMap["D3D:saveSegFeats"] = [&] (const Tokens & args) {
    m_viz->ensureDatabase();
    const string dir = params.get<string>("workDir") + "feats/";
    state.database->scans.saveSegmentFeatures(dir);
    return E::E_OK;
  };

  m_eventMap["D3D:saveSkelFeats"] = [&] (const Tokens &args) {
    m_viz->ensureDatabase();
    const string dir = params.get<string>("workDir") + "feats/";
    state.database->skeletons->saveSkelFeats(dir);
    return E::E_OK;
  };

  m_eventMap["D3D:saveInteractionFeats"] = [&] (const Tokens &args) {
    m_viz->ensureDatabase();
    const string dirname = m_viz->m_params->get<string>("workDir") + "feats/";
    std::ofstream ofsALL(dirname + "ALL.interaction-feats.csv");
    sg::interaction::writeInteractionFeatsHeader(ofsALL);
    for (const string recId : state.database->recordings.getRecordingIds()) {
      const sg::core::Recording& rec = state.database->recordings.getRecording(recId, true, false);
      const string csvFile = dirname + "interactions/" + recId + ".feats.csv";
      sg::io::ensurePathToFileExists(csvFile);
      std::ofstream ofs(csvFile);
      sg::interaction::writeInteractionFeatsHeader(ofs);
      sg::interaction::writeInteractionFeats(ofs, state.database->interactionFactory, rec.interactions);
      sg::interaction::writeInteractionFeats(ofsALL, state.database->interactionFactory, rec.interactions);
    }
    return E::E_OK;
  };

  m_eventMap["D3D:saveRecordingStartTimes"] = [&] (const Tokens &args) {
    m_viz->ensureDatabase();
    const string dirname = m_viz->m_params->get<string>("workDir") + "/";
    sg::io::ensurePathToFileExists(dirname);
    std::ofstream ofs(dirname + "ALL.recordingStartTimes.csv");
    ofs << "recId,deltaStart" << endl;
    for (const string recId : state.database->recordings.getRecordingIds()) {
      const sg::core::Recording& rec = state.database->recordings.getRecording(recId, true, false);
      int64_t colorStart = rec.colorTimestamps[0];
      int64_t skelStart = rec.skeletons[0].timestamp;
      int64_t deltaStart = skelStart - colorStart;
      ofs << rec.id << "," << deltaStart * sg::core::Recording::kTimestampToSec << endl;
    }
    return E::E_OK;
  };

  m_eventMap["D3D:showScanVoxels"] = [&] (const Tokens &args) {
    sg::core::synth::LabelOpts labelOpts;
    sg::core::OccupancyGrid::OccupancyType occType = sg::core::OccupancyGrid::OccupancyType_Occupied;
    const sg::interaction::InteractionSet* pISet = m_viz->getCurrentInteractionSet();
    bool showLabeled = m_viz->setLabelOptsFromArgs(args, pISet, &labelOpts, &occType);
    if (showLabeled) {
      state.showScanLabeledVoxels = true;
      state.sendStateUpdate("showScanLabeledVoxels", "true");
    } else {
      state.showScanVoxels = true;
      state.sendStateUpdate("showScanVoxels", "true");
    }
    m_viz->showScanVoxels(app, occType, showLabeled, labelOpts);
    return E::E_OK;
  };
  m_eventMap["D3D:saveScanVoxels"] = [&] (const Tokens &args) {
    sg::core::synth::LabelOpts labelOpts;
    const sg::interaction::InteractionSet* pISet = m_viz->getCurrentInteractionSet();
    sg::core::OccupancyGrid::OccupancyType occType = sg::core::OccupancyGrid::OccupancyType_Occupied;
    bool showLabeled = m_viz->setLabelOptsFromArgs(args, pISet, &labelOpts, &occType);

    if (labelOpts.labelStrategy == sg::core::synth::kLabelsRecording ||
      labelOpts.labelStrategy == sg::core::synth::kLabelsRecordingsAll ||
      labelOpts.labelStrategy == sg::core::synth::kLabelsSkeleton ||
      labelOpts.labelStrategy == sg::core::synth::kLabelsPredict) {
      // Need to have interaction sets and PIGs
      m_viz->ensureDatabase();
    }

    string runId = "";
    bool needRecordings = false;
    for (int i = 1; i < args.size(); ++i) {
      if (args[i] == "recordingsAll") { needRecordings = true; }
      runId += args[i] + (i == args.size() - 1 ? "" : "-");
    }
    if (needRecordings) {
      state.database->recordings.loadAllRecordingsWithInteractions(state.database->interactions);
    }
    const vector<string> scanIds = state.database->scans.scanIds();
    state.database->scans.saveScanLabeledVoxels(scanIds, state.database->getLabeler(), labelOpts, runId);

    return E::E_OK;
  };
  m_eventMap["D3D:loadScanVoxels"] = [&] (const Tokens &args) {
    if (args.size() < 2) { return E::E_INVALID_ARGS; }
    const sg::core::LabeledGrid grid(args[1]);
    sg::vis::ColorIndex* pColorIndex = &m_viz->m_partLabelColors;
    ml::TriMeshf gridMesh = vizutil::labeledScanVoxelsToTriMesh(grid, pColorIndex);
    // TODO(MS): Use simplified version of showScanVoxels instead
    m_viz->vs.mesh["scanVoxelsLabeled"].load(app.graphics, gridMesh);
    return E::E_OK;
  };
  m_eventMap["D3D:evalVoxels"] = [&] (const Tokens &args) {
    if (args.size() < 2) { return E::E_INVALID_ARGS; }
    const string baseDir = m_viz->m_params->get<string>(".workDir") + "voxels/";
    const string voxelsDir = baseDir + args[1];
    const string truthDir = baseDir + "truth/";
    SG_LOG_INFO << "Evaluating voxel predictions in " << voxelsDir << " ; gold is at " << truthDir;
    const auto results = sg::core::compare(voxelsDir, truthDir);
    return E::E_OK;
  };
  m_eventMap["D3D:loadCornellPLY"] = [&] (const Tokens &args) {
    if (args.size() < 2) { return E::E_INVALID_ARGS; }
    const string id = args[1];
    SG_LOG_INFO << "Loading " << id;
    //const sg::core::Scan& s = state.database->scansCornellPLY.getScan(id);
    m_viz->loadScan(app, id);
    return E::E_OK;
  };
  m_eventMap["D3D:place"] = [&] (const Tokens &args) {
    sg::core::synth::ModelPlacerParams placerParams;
    placerParams.predictSegmentLabels = false;
    placerParams.predictAction = false;
    placerParams.useSkelRange = false;
    placerParams.debugViz = false;
    // Restrict models to whitelist
    placerParams.restrictModelsToWhitelist = m_viz->m_state.database->params->get<bool>("ModelPlacer.restrictToWhitelist");
    for (int i = 1; i < args.size(); ++i) {
      const string& arg = args[i];
      if (arg == "predictSegs") {
        placerParams.predictSegmentLabels = true;
      } else if (arg == "predictAction") {
        placerParams.predictAction = true;
      } else if (arg == "skelRange") {
        placerParams.useSkelRange = true;
      } else if (arg[0] == '-') {
        const string filterCat = arg.substr(1);
        placerParams.categoriesToPlace.insert(filterCat);
      } else if (arg == "debug") {
        placerParams.debugViz = true;
      } else {
        cout << "Unknown arg: " << args[i] << endl;
        return E::E_INVALID_ARGS;
      }
    }
    m_viz->ensureDatabase();
    m_viz->retrieveAndPlace(app, placerParams);
    return E::E_OK;
  };
  m_eventMap["D3D:setIFrameType"] = [&] (const Tokens &args) {
    CHECK_ARGS(args, 2);
    auto iframeType = sg::interaction::getInteractionFrameTypeFromString(args[1].c_str());
    if (iframeType < sg::interaction::InteractionFrameType::kCount) {
      SG_LOG_INFO << "Setting interaction frame type to " << iframeType;
      state.iframeType = iframeType;
      m_viz->updateVisualizations(app);
      return E::E_OK;
    } else {
      SG_LOG_WARN << "Invalid interaction frame type " << args[1];
      return E::E_INVALID_ARGS;
    }
  };
  m_eventMap["D3D:setPartType"] = [&] (const Tokens &args) {
    CHECK_ARGS(args, 2);
    auto partType = sg::segmentation::getPartTypeFromString(args[1]);
    if (partType < sg::segmentation::PartType::kPartTypeCount) {
      SG_LOG_INFO << "Setting part type to " << partType;
      state.database->params->set<string>("Interaction.partType", args[1]);
      state.partType = partType;
      m_viz->updateVisualizations(app);
      return E::E_OK;
    } else {
      SG_LOG_WARN << "Invalid part type " << args[1];
      return E::E_INVALID_ARGS;
    }
  };
  // Event messages from UIWindow!
  m_eventMap["D3D:setMode"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    string mode = args[1];
    if (mode == "replay") {
      state.mode = SharedAppState::Mode_Replay;
      state.inPosingMode = false;
    } else if (mode == "pose") {
      state.mode = SharedAppState::Mode_Pose;
      state.inPosingMode = true;
      m_viz->ensureDatabase();
    } else {
      cerr << "Unknown mode " << mode << endl;
    }
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:segConstrainZup"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    m_viz->m_segParams.constrainZup = ml::convert::toBool(args[1]);
    m_viz->updateMeshSegmentation();
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:timeScrollA"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    const float t = ml::math::lerp(state.timeMinSec, state.timeMaxSec, std::stof(args[1]));
    m_viz->setCurrentTime(app, t);
    return E::E_OK;
  };
  m_eventMap["D3D:timeScrollB"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    const float t = ml::math::lerp(state.timeMinSec, state.timeMaxSec, std::stof(args[1]));
    m_viz->setCurrentTime(app, t);
    return E::E_OK;
  };
  m_eventMap["D3D:playRange"] = [&] (const Tokens & args) {
    // Stop playback if already playing, or start playing range back
    if (state.playingRange) {
      state.playingRange = false;
    } else {
      CHECK_ARGS(args, 2);
      const vector<string>& ts = ml::util::split(args[1], ",");
      state.rangeTimeStartSec = ml::math::lerp(state.timeMinSec, state.timeMaxSec, std::stof(ts[0]));
      state.rangeTimeEndSec = ml::math::lerp(state.timeMinSec, state.timeMaxSec, std::stof(ts[1]));
      state.rangeTimeStepSec = 1.0f / 30.f;
      state.playingRange = true;
      // If current time is outside range, rewind to beginning
      if (state.timeNowSec < state.rangeTimeStartSec || state.timeNowSec > state.rangeTimeEndSec) {
        m_viz->setCurrentTime(app, state.rangeTimeStartSec);
      }
    }
    return E::E_OK;
  };
  m_eventMap["D3D:showSkeletons"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    state.showSkeletons = ml::convert::toBool(args[1]);
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:showGaze"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    state.showGaze = ml::convert::toBool(args[1]);
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:showSegmentation"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    state.showSegmentation = ml::convert::toBool(args[1]);
    if (!state.showSegmentation) {
      m_viz->m_currVertexColors = m_viz->m_baseVertexColors;
    } else {
      sg::segmentation::getSegmentationColors(state.currScan->mesh, m_viz->m_segSelectedSegId, &m_viz->m_currVertexColors);
    }
    m_viz->m_vertexColorsDirty = true;
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:segmentationKthresh"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    m_viz->m_segParams.kthresh = std::stof(args[1]);
    m_viz->updateMeshSegmentation();
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:segmentationMinSegVertices"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    m_viz->m_segParams.minSegVerts = std::stoi(args[1]);
    m_viz->updateMeshSegmentation();
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:segColorWeight"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    m_viz->m_segParams.colorWeight = std::stof(args[1]);
    m_viz->updateMeshSegmentation();
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:segKnn"] = [&] (const Tokens & args) {
    m_viz->findSimilarMeshSegmentsForSelectedSegment();
    return E::E_OK;
  };
  m_eventMap["D3D:jointSegKnn"] = [&] (const Tokens & args) {
    m_viz->findSimilarMeshSegmentsForJointInteraction();
    return E::E_OK;
  };
  m_eventMap["D3D:selectSegId"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    m_viz->m_segSelectedSegId = std::stoi(args[1]);
    sg::segmentation::getSegmentationColors(state.currScan->mesh, m_viz->m_segSelectedSegId, &m_viz->m_currVertexColors);
    m_viz->m_vertexColorsDirty = true;
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:minSegDiag"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    m_viz->m_segParams.minSegDiag = std::stof(args[1]);
    m_viz->updateMeshSegmentation();
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:showSegmentOBBs"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    state.showSegmentOBBs = ml::convert::toBool(args[1]);
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:showSegmentGroupOBBs"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    state.showSegmentGroupOBBs = ml::convert::toBool(args[1]);
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:showObjectOBBs"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    state.showObjectOBBs = ml::convert::toBool(args[1]);
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:showClusterAssignment"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    state.showClusterAssignment = ml::convert::toBool(args[1]);
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:showInteractionOBBs"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    state.showInteractionOBBs = ml::convert::toBool(args[1]);
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:showActivationMap"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    state.showActivationMap = ml::convert::toBool(args[1]);
    if (!state.showActivationMap) {
      m_viz->m_currVertexColors = m_viz->m_baseVertexColors;
    }
    m_viz->updateVisualizations(app);
    m_viz->m_vertexColorsDirty = true;
    return E::E_OK;
  };
  m_eventMap["D3D:hideUnactivated"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    state.hideUnactivatedMesh = ml::convert::toBool(args[1]);
    m_viz->updateVisualizations(app);
    m_viz->m_vertexColorsDirty = true;
    return E::E_OK;
  };
  m_eventMap["D3D:showOIMAnnotations"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    state.showOIMAnnotations = ml::convert::toBool(args[1]);
    m_viz->initModelInteractionMaps(app.graphics);
    m_viz->updateVisualizations(app);
    m_viz->m_vertexColorsDirty = true;
    return E::E_OK;
  };
  m_eventMap["D3D:jointIndex"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    m_viz->m_integrationJointIndex = std::stoi(args[1]);
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:setCamera"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    const unsigned cameraIdx = std::stoi(args[1]);
    m_viz->setCamera(app, cameraIdx);
    return E::E_OK;
  };
  m_eventMap["D3D:integrationWindow"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    m_viz->m_integrationTimeWindowSec = std::stof(args[1]);
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:activationRadius"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    m_viz->m_activationRadius = std::stof(args[1]);
    m_viz->m_params->set<float>("Interaction.maxDistToSegment", m_viz->m_activationRadius);
    // Also recompute active segments since different active radius
    state.currRecording->computeActiveSegments(params, *state.currScan, m_viz->m_activationRadius,
                                               params.get<float>("Interaction.maxDistGaze"), true);
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:showScan"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    state.showScan = ml::convert::toBool(args[1]);
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:showBodyPoints"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    state.showBodyPoints = ml::convert::toBool(args[1]);
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:showModels"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    state.showModels = ml::convert::toBool(args[1]);
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:showModelInteractionMaps"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    state.showModelInteractionMaps = ml::convert::toBool(args[1]);
    return E::E_OK;
  };
  m_eventMap["D3D:showModelSegmentation"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    state.showModelSegmentation = ml::convert::toBool(args[1]);
    return E::E_OK;
  };
  m_eventMap["D3D:loadModelSegmentation"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    state.loadModelSegmentation = ml::convert::toBool(args[1]);
    m_viz->updateModelSegmentations(app.graphics, true);
    return E::E_OK;
  };
  m_eventMap["D3D:showModelVoxels"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    state.showModelVoxels = ml::convert::toBool(args[1]);
    return E::E_OK;
  };
  m_eventMap["D3D:showInteractionFrame"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    state.showInteractionFrame = ml::convert::toBool(args[1]);
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:saveSeg"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    string filename = args[1];
    const auto& V = state.currScan->mesh.getVertices();
    vector<int> vertexSegIndices(V.size());
    for (size_t i = 0; i < V.size(); i++) {
      vertexSegIndices[i] = static_cast<int>(V[i].texCoord[0]);
    }
    sg::segmentation::saveSegIndicesCsv(filename, vertexSegIndices);
    return E::E_OK;
  };
  m_eventMap["D3D:loadSeg"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    string filename = args[1];
    m_viz->loadMeshSegmentation(filename);
    return E::E_OK;
  };
  m_eventMap["D3D:createDatabase"] = [&] (const Tokens & args) {
    m_viz->createDatabase();
    return E::E_OK;
  };
  m_eventMap["D3D:nextPose"] = [&] (const Tokens & args) {
    m_viz->ensureDatabase();
    m_viz->m_pIsetPoseController->nextPose();
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:nextInteractionSet"] = [&] (const Tokens & args) {
    m_viz->ensureDatabase();
    m_viz->m_pIsetPoseController->nextInteractionSet();
    state.sendStateUpdate("interactionSet",
                          m_viz->m_pIsetPoseController->currentInteractionSetType() + ":" +
                          m_viz->m_pIsetPoseController->currentInteractionSet().id);
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:setPoseInteractionSet"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 3);
    m_viz->ensureDatabase();
    m_viz->m_pIsetPoseController->setInteractionSet(args[1], args[2]);
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:segPerJointClassify"] = [&] (const Tokens & args) {
    m_viz->classifySegmentsPerJoint();
    return E::E_OK;
  };
  m_eventMap["D3D:showInteractionHeatmap"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    state.showInteractionHeatMap = ml::convert::toBool(args[1]);
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:reportActiveOBBs"] = [&] (const Tokens & args) {
    for (int iJoint = 0; iJoint < Skeleton::kNumJoints + 1; ++iJoint) {
      const auto& segs = m_viz->vs.activeParts[iJoint];
      for (const auto& seg : segs) {
        string label = seg->label;
        cout << Skeleton::kJointNames[iJoint] << ": " << label << " " << *seg->obb() << endl;
      }
    }
    return E::E_OK;
  };
  m_eventMap["D3D:showSegmentVerbProbability"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    state.showSegmentVerbProbability = ml::convert::toBool(args[1]);
    m_viz->updateVisualizations(app);
    return E::E_OK;
  };
  m_eventMap["D3D:dumpTestImages"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    state.database->classifiers->doPostTestCallback = ml::convert::toBool(args[1]);
    return E::E_OK;
  };
  m_eventMap["D3D:updateHeatmap"] = [&] (const Tokens & args) {
    m_viz->ensureDatabase();
    const sg::interaction::InteractionSet& interactionSet
      = m_viz->m_pIsetPoseController->currentInteractionSet();

    const string& clsType = state.currClassifierType;
    const sg::core::ScenePoseClassifier* pClassifier
      = state.database->classifiers->getScenePoseClassifier(clsType, interactionSet);
    if (pClassifier != nullptr) {
      if (state.showInteractionHeatMap) {
        m_viz->updateInteractionHeatMap(app.graphics, interactionSet, *pClassifier);
        m_viz->updateVisualizations(app);
      }
    } else {
      return E::E_GENERAL_ERROR;
    }
    return E::E_OK;
  };
  m_eventMap["D3D:nextRecording"] = [&] (const Tokens & args) {
    if (!state.currScan) {
      return E::E_GENERAL_ERROR;
    }

    // get rec ids for current scan
    const string& scanId = state.currScan->getCanonicalId();
    const vector<string> recIds = state.database->recordings.getRecordingIdsForScan(scanId);
    if (recIds.size() < 2) {
      return E::E_GENERAL_ERROR;
    }

    // iterate to next rec id or back to beginning if at end
    const string& currRecId = state.currRecording->id;
    auto currRecIt = std::find(recIds.begin(), recIds.end(), currRecId);
    if (currRecIt + 1 == recIds.end()) {
      currRecIt = recIds.begin();
    } else {
      currRecIt = currRecIt + 1;
    }

    // load recording and return
    m_viz->initRecording(app, *currRecIt, false);
    return E::E_OK;
  };
  m_eventMap["D3D:dumpPIGFeats"] = [&] (const Tokens & args) {
    // Dump PIG features in workDir/pigs/feats
    // Use 'all' to iterate through all interaction sets and dump all pig features
    // Otherwise, only features for the currently selected interaction set is dumped
    m_viz->ensureDatabase();
    bool dumpAll = false;
    for (const string& arg : args) {
      if (arg == "all") {
        dumpAll = true;
      }
    }
    const string& outputDir = params.get<string>("workDir") + "/pigs/feats/";

    if (dumpAll) {
      for (const auto& it : state.database->interactions.getAllInteractionSets()) {
        const sg::interaction::InteractionSet& is = it.second;
        for (const auto& pigsForPart : is.pigs) {
          for (const auto& pigPair : pigsForPart) {
            const string& partTypeName = 
              sg::segmentation::kPartTypeNames[pigPair.second->getPartType()];
            const string& pigType = pigPair.second->getPigType();
            const string& filename = outputDir + partTypeName + "/" + pigType + "/" + is.id + ".pig";
            pigPair.second->dumpFeats(filename + ".csv");
            pigPair.second->dumpFeatsSummary(filename + ".summary.csv");
          }
        }
      }
    } else {
      const sg::interaction::InteractionSet* pCurrISet = m_viz->getCurrentInteractionSet();
      if (pCurrISet != nullptr) {
        for (const auto& pigsForPart : pCurrISet->pigs) {
          for (const auto& pigPair : pigsForPart) {
            const string& partTypeName =
              sg::segmentation::kPartTypeNames[pigPair.second->getPartType()];
            const string& pigType = pigPair.second->getPigType();
            const string& filename = outputDir + partTypeName + "/" + pigType + "/" + pCurrISet->id + ".pig";
            pCurrISet->protoInteractionGraph->dumpFeats(filename + ".csv");
            pCurrISet->protoInteractionGraph->dumpFeatsSummary(filename + ".summary.csv");
          }
        }
      }
    }
    return E::E_OK;
  };
  m_eventMap["D3D:dumpCatProbs"] = [&] (const Tokens & args) {
    // Save per category probabilties for interaction sets
    // Use 'all' to iterate through all interaction sets 
    // Otherwise, only statistics for the currently selected interaction set is dumped
    m_viz->ensureDatabase();
    bool dumpAll = false;
    for (const string& arg : args) {
      if (arg == "all") {
        dumpAll = true;
      }
    }
    const string& outputDir = params.get<string>("workDir") + "/pigs/jc/";

    const auto& labeler = m_viz->m_state.database->getLabeler();
    if (dumpAll) {
      for (const auto& it : state.database->interactions.getAllInteractionSets()) {
        for (const auto& pigsForPart : it.second.get().pigs) {
          for (const auto& pigPair : pigsForPart) {
            const string& partTypeName =
              sg::segmentation::kPartTypeNames[pigPair.second->getPartType()];
            const string& pigType = pigPair.second->getPigType();
            const sg::interaction::InteractionSet& is = it.second;
            const string& filename = outputDir + partTypeName + "/" + pigType + "/" + is.id + ".pigjc.csv";
            labeler.dumpJointCategoryProbs(*pigPair.second, filename);
            const string& filename2 = outputDir + partTypeName + "/" + pigType + "/" + is.id + ".pigjl.csv";
            labeler.dumpJointLabelProbs(*pigPair.second, filename2);
          }
        }
      }
    } else {
      const sg::interaction::InteractionSet* pCurrISet = m_viz->getCurrentInteractionSet();
      if (pCurrISet != nullptr) {
        for (const auto& pigsForPart : pCurrISet->pigs) {
          for (const auto& pigPair : pigsForPart) {
            const string& partTypeName =
              sg::segmentation::kPartTypeNames[pigPair.second->getPartType()];
            const string& pigType = pigPair.second->getPigType();
            const string& filename = outputDir + partTypeName + "/" + pigType + "/" + pCurrISet->id + ".pigjc.csv";
            labeler.dumpJointCategoryProbs(*pigPair.second, filename);
            const string& filename2 = outputDir + partTypeName + "/" + pigType + "/" + pCurrISet->id + ".pigjl.csv";
            labeler.dumpJointLabelProbs(*pigPair.second, filename2);
          }
        }
      }
    }
    return E::E_OK;
  };
  m_eventMap["D3D:saveInteractionStats"] = [&] (const Tokens & args) {
    // Save aggregated statistics for interactions
    m_viz->ensureDatabase();
    const string& outputDir = params.get<string>("workDir") + "/pigs/stats/";
    state.database->interactions.saveStats(outputDir);
    return E::E_OK;
  };
  m_eventMap["D3D:saveInteractionSet"] = [&] (const Tokens & args) {
    // Save interaction set
    m_viz->ensureDatabase();
    const sg::interaction::InteractionSet* pCurrISet = m_viz->getCurrentInteractionSet();
    if (pCurrISet != nullptr) {
      const string& filename = params.get<string>("workDir") + "isets/" + pCurrISet->id + ".bin";
      pCurrISet->saveBinary(filename);
    }
    return E::E_OK;
  };
  m_eventMap["D3D:saveWeights"] = [&] (const Tokens & args) {
    // Save weights for the selected classifier
    m_viz->ensureDatabase();
    const string& clsId = state.currClassifierType;
    const string& outputFilename = 
      sg::util::replaceStringAll(state.database->classifiers->getResultOutputPath(clsId) + "weights.csv",
        params.get<string>("dataDir"), params.get<string>("workDir"));
//    const string& outputFilename = state.database->classifiers->getClassifierDir() + "weights.csv";
    state.database->classifiers->saveWeights(clsId, outputFilename);
    return E::E_OK;
  };
  m_eventMap["D3D:train"] = [&] (const Tokens & args) {
    // Train the selected classifier
    m_viz->ensureDatabase();
    const string& clsId = state.currClassifierType;
    state.database->classifiers->trainClassifier(clsId);
    return E::E_OK;
  };
  m_eventMap["D3D:trainSelected"] = [&] (const Tokens & args) {
    // Train the selected classifier
    m_viz->ensureDatabase();
    const string& clsId = state.currClassifierType;
    state.database->classifiers->trainClassifier(clsId);
    return E::E_OK;
  };
  m_eventMap["D3D:trainAll"] = [&] (const Tokens & args) {
    // Train all classifiers
    m_viz->ensureDatabase();
    state.database->classifiers->trainClassifiers();
    return E::E_OK;
  };
  m_eventMap["D3D:testSelected"] = [&] (const Tokens & args) {
    // Test my classifier
    m_viz->ensureDatabase();
    const string& clsId = state.currClassifierType;
    state.database->classifiers->testClassifier(clsId);
    return E::E_OK;
  };
  m_eventMap["D3D:crossValidate"] = [&] (const Tokens & args) {
    // Cross validate
    m_viz->ensureDatabase();
    const string& clsId = state.currClassifierType;
    state.database->classifiers->crossValidateClassifier(clsId);
    return E::E_OK;
  };
  m_eventMap["D3D:testInteraction"] = [&] (const Tokens & args) {
    // Test my classifier with current interaction
    m_viz->ensureDatabase();
    const string& clsId = state.currClassifierType;
    m_viz->testClassifier(app, clsId, m_viz->m_pIsetPoseController->currentInteractionSet().id);
    return E::E_OK;
  };
  m_eventMap["D3D:testSnaps"] = [&] (const Tokens & args) {
    vector<string> rest(args.begin() + 1, args.end());
    m_viz->testInteractionSynth(app, rest);
    return E::E_OK;
  };
  m_eventMap["D3D:synth"] = [&] (const Tokens & args) {
    vector<string> rest(args.begin() + 1, args.end());
    m_viz->interactionSynth(app, rest);
    return E::E_OK;
  };
  m_eventMap["D3D:setShowPredictedSkeleton"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    m_viz->m_showPredictedSkeleton = ml::convert::toBool(args[1]);
    return E::E_OK;
  };
  m_eventMap["D3D:setShowInteractionScore"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    m_viz->m_state.showInteractionScore = ml::convert::toBool(args[1]);
    return E::E_OK;
  };
  m_eventMap["D3D:setLogLevel"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    sg::util::set_log_severity(args[0], args[1]);
    return E::E_OK;
  };
  m_eventMap["D3D:dumpMesh"] = [&] (const Tokens & args) {
    // Dumps the current state of the mesh
    m_viz->dumpMesh();
    return E::E_OK;
  };
  m_eventMap["D3D:loadScan"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    // Load and initialize the scan
    m_viz->loadScan(app, args[1]);
    return E::E_OK;
  };
  m_eventMap["D3D:loadModel"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 3);
    // Load a model - arg[1] is the loadType, arg[2] is the loadId
    m_viz->loadModel(app, args[1], args[2]);
    return E::E_OK;
  };
  m_eventMap["D3D:loadReconScene"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    // Load a scene - arg[1] is the scene id
    m_viz->loadReconScene(app, args[1]);
    return E::E_OK;
  };
  m_eventMap["D3D:loadScene"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    // Load a scene - arg[1] is the file
    m_viz->loadScene(app, args[1]);
    return E::E_OK;
  };
  m_eventMap["D3D:saveScene"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    // Save a scene - arg[1] is the file
    m_viz->saveScene(args[1]);
    return E::E_OK;
  };

  // Segment group annotations
  m_eventMap["D3D:segGroup:guessObjectIds"] = [&] (const Tokens & args) {
    if (state.currScan->segmentGroups.size() > 0) {
      // Guess object ids for this scan
      state.currScan->segmentGroups.guessObjectIds();
      // Update object ids in the UI Window
      vizutil::updateSegGroupAnnotations(state, /*reloadSegmentGroups=*/ false);
      // Update visualization of scan
      m_viz->updateVisualizations(app);
    }
    return E::E_OK;
  };
  m_eventMap["D3D:segGroup:clear"] = [&] (const Tokens & args) {
    // Clears the segment group annotations of the current scene
    state.currScan->segmentGroups.clear();
    cout << "Segment groups cleared, segGroupCount="
      << state.currScan->segmentGroups.size() << endl;;
    m_viz->selectSegmentGroup(app, -1);
    return E::E_OK;
  };
  m_eventMap["D3D:segGroup:relabel"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 4);
    // Relabels the segment group
    int sgId = std::stoi(args[1]);
    const string& relabelField = args[2];
    const string& newLabel = args[3];
    bool okay = false;
    if (relabelField == "label") {
      okay = state.currScan->segmentGroups.relabel(sgId, newLabel);
    } else if (relabelField == "objectId") {
      okay = state.currScan->segmentGroups.setObjectId(sgId, std::stoi(newLabel));
    } else {
      cout << "Unknown field " << relabelField;
    }
    m_viz->selectSegmentGroup(app, sgId);
    if (okay) {
      cout << "Segment group " << sgId << " " << relabelField << " labeled as " << newLabel << endl;
    } else {
      cout << "Error labeling segment group " << sgId << " " << relabelField << " as " << newLabel << endl;
    }
    return E::E_OK;
  };
  m_eventMap["D3D:segGroup:delete"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    // Deletes the segment group
    int sgId = std::stoi(args[1]);
    state.currScan->segmentGroups.remove(sgId);
    cout << "Segment group " << sgId << " deleted, segGroupCount="
      << state.currScan->segmentGroups.size() << endl;
    if (sgId == m_viz->m_segSelectedSegGroupId) {
      m_viz->selectSegmentGroup(app, -1);
    } else {
      m_viz->selectSegmentGroup(app, m_viz->m_segSelectedSegGroupId);
    }
    return E::E_OK;
  };
  m_eventMap["D3D:segGroup:select"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    // Select the segGroup
    m_viz->selectSegmentGroup(app, std::stoi(args[1]));
    return E::E_OK;
  };
  m_eventMap["D3D:segGroup:save"] = [&] (const Tokens & args) {
    // Save the segment group annotations
    CHECK_ARGS(args, 2);
    state.currScan->saveSegmentGroups(args[1]);
    return E::E_OK;
  };
  m_eventMap["D3D:segGroup:load"] = [&] (const Tokens & args) {
    CHECK_ARGS(args, 2);
    // Load the segment group annotations
    bool loaded = state.currScan->loadSegmentGroups(args[1],
                                                    params.get<float>("Segmentation.segGroupAbsorbThreshold"),
                                                    params.get<bool>("Segmentation.constrainZup"),
                                                    params.get<bool>("SegmentGroup.loadSegments"),
                                                    params.get<bool>("SegmentGroup.recomputeOBBs"));

    if (loaded) {
      // Segmentation has changed - update visualization
      m_viz->meshSegmentationChanged(true);
      m_viz->updateVisualizations(app);
    } else {
      cout << "Error loading segment groups from " << args[1] << endl;
      return E::E_GENERAL_ERROR;
    }
    return E::E_OK;
  };
}

void VizzerEventMap::processSignalQueue() {
  SharedAppState& state = m_viz->m_state;
  m_eventMap.processSignalQueue(&state.appMessages[SharedAppState::UI_D3D], &state.mutex);
}
