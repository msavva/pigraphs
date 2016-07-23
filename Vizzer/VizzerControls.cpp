#include "common.h"  // NOLINT

#include "./VizzerControls.h"

#include <core/PoseController.h>
#include <core/SkeletonDatabase.h>
#include <core/synth/ModelAligner.h>
#include <core/synth/synth.h>
#include <segmentation/MeshSegment.h>
#include <segmentation/segmentation.h>

#include "./vizutil.h"
#include "./D3D11Vizzer.h"

using std::string;  using std::cout;  using std::cerr;  using std::endl;
using sg::core::Skeleton;

VizzerControls::VizzerControls(D3D11Vizzer* pVizzer)
  : m_viz(pVizzer) { }

void VizzerControls::keyDown(ml::ApplicationData& app, unsigned int key) {
  D3D11Vizzer& v = *m_viz;  // save some characters!

  if (key == KEY_T) {  // Handle test function hook
    if (v.vs.testFun.size() > 0) {
      const string text = "D3D:" + sg::util::join(v.vs.testFun, " ");
      v.m_state.sendMessage(SharedAppState::UI_D3D, text);
    } else {
      cout << "No test function set. Use 'v setTestFun fun args' to set." << endl;
    }
  }

  if (key == KEY_O) {  // toggle overlay
    v.m_showInfoOverlay = !v.m_showInfoOverlay;
  }

  if (key == KEY_P) {
    //if (GetAsyncKeyState(VK_SHIFT)) {  // Toggle showing probes
    //  v.m_showProbePoints = !v.m_showProbePoints;
    //} else if (v.m_showProbePoints) {  // Report probe similarity
    //  sg::interaction::InteractionNode a, b;
    //  sg::interaction::jointVolumeFeats(*v.m_state.currScan, v.m_probePointA, a);
    //  sg::interaction::jointVolumeFeats(*v.m_state.currScan, v.m_probePointB, b);
    //  const float simVol = sg::graph::similarity::simVolume(a, b);
    //  cout << "*** ptA: " << v.m_probePointA << endl
    //       << a.heightHistogramOccupied << a.heightHistogramTotal << endl;
    //  cout << "*** ptB: " << v.m_probePointB << endl
    //       << b.heightHistogramOccupied << b.heightHistogramTotal << endl;
    //  cout << "*** simVolume: " << simVol << endl;
    //}
  }
  if (v.m_showProbePoints) {  // If probes are showing, handle probe manipulation
    ml::vec3f& p = GetAsyncKeyState(VK_SHIFT) ? v.m_probePointB : v.m_probePointA;
    vizutil::moveProbePoint(key, p);
  }
  if (key == KEY_Z && v.m_integrationJointIndex > 0) {
    v.m_integrationJointIndex = (v.m_integrationJointIndex - 1) % Skeleton::kNumJoints;
  }
  if (key == KEY_X && v.m_integrationJointIndex < Skeleton::kNumJoints - 1) {
    v.m_integrationJointIndex = (v.m_integrationJointIndex + 1) % Skeleton::kNumJoints;
  }
  if (key == KEY_Z || key == KEY_X) {
    v.m_voxelizedSkel = !v.m_voxelizedSkel;
  }

  // Helper to return vec of all active segs
  const auto flattenedActiveSegs = [&v] () {
    const auto& segsByJointPlusGaze = vizutil::getCurrentActiveSegs(*v.m_currentSkelIt, v.m_state);
    sg::segmentation::VecConstSegPtr vecSegs;
    std::set<sg::segmentation::SegPtr> seenSegs;
    for (int i = 0; i < Skeleton::kNumJoints; i++) {
      sg::segmentation::VecSegPtr segs = segsByJointPlusGaze[i];
      for (const sg::segmentation::SegPtr& seg : segs) {
        if (seenSegs.count(seg) == 0) {
          vecSegs.push_back(seg);
          seenSegs.insert(seg);
        }
      }
    }
    return vecSegs;
  };

  if (key == KEY_B) {
    auto& vStackGrid = v.m_state.currScan->getVertexStackGrid();
    const auto vizMesh = sg::mesh::projection::toTriMesh(vStackGrid, 0.1f);
    v.vs.mesh["test"].load(app.graphics, vizMesh);
  }

  if (key == KEY_F) {
    namespace syn = sg::core::synth;
    const auto segs = flattenedActiveSegs();
    if (v.m_modelInstances.size() > 0) {
      cout << "Placing modelInstance[0] at maxScoreTransform position" << endl;
      const Skeleton& skel = *v.m_currentSkelIt;
      const sg::core::SkelRange skelRange({skel});
      const sg::core::Scan& scan = *v.m_state.currScan;
      sg::core::ModelInstance& mInst = v.m_modelInstances[0];
      const sg::geo::OBB segsOBB = sg::segmentation::computeSegsOBB(segs);
      const syn::ModelAlignerParams alignParams(*v.m_params);
      syn::ModelAligner aligner;
      aligner.init(alignParams);
      const auto maxOverlapXform = aligner.alignToScan(scan, skelRange, mInst, segsOBB);
      mInst.setLocalTransform(maxOverlapXform);
    } else {
      cout << "Placing Chair into scene" << endl;
      bool restrictToWhitelist = v.m_state.database->params->get<bool>("ModelPlacer.restrictToWhitelist");
      const auto& model = v.m_state.database->models.getFirstModelWithCategory("Chair", restrictToWhitelist);
      if (model) { syn::placeModelInstance(model.get(), segs, &v.m_modelInstances); }
      else {
        SG_LOG_WARN << "Can't find a Chair to add to scene";
      }
    }
    // Also re-initialize InteractionMaps and their meshes, the model segmentations and voxels
    v.initModelInteractionMaps(app.graphics);
    v.updateModelSegmentations(app.graphics);
    v.updateModelVoxels(app.graphics);
  }

  if (key == KEY_V) {
    sg::core::synth::ModelPlacerParams placerParams;
    placerParams.predictAction = false;
    placerParams.predictSegmentLabels = false;
    if (GetAsyncKeyState(VK_SHIFT)) {
      placerParams.predictSegmentLabels = true;
    } else if (GetAsyncKeyState(VK_CONTROL)) {
      placerParams.predictAction = true;
      placerParams.predictSegmentLabels = true;
    }
    placerParams.useSkelRange = false;
    placerParams.debugViz = false;
    // Restrict models to whitelist
    placerParams.restrictModelsToWhitelist = v.m_state.database->params->get<bool>("ModelPlacer.restrictToWhitelist");
    v.retrieveAndPlace(app, placerParams);
  }

  if (key == KEY_I) {
    if (GetAsyncKeyState(VK_SHIFT)) {
      v.clearScene(app);
    } else {
      //app.graphics.castD3D11().toggleWireframe();
      v.m_state.movingModelInstance = !v.m_state.movingModelInstance;
      cout << "movingModelInst=" << v.m_state.movingModelInstance << endl;
    }
  }

  // Model instance manipulation
  const int mid = v.m_params->getWithDefault<int>("mid", 0);
  bool modelMoved = false;
  if (v.m_modelInstances.size() > 0 && v.m_state.movingModelInstance) {
    modelMoved = vizutil::moveModelInstance(key, &v.m_modelInstances[mid]);
    if (modelMoved) {
      // replace interaction maps for any new transforms on ModelInstances
      v.initModelInteractionMaps(app.graphics);
      // Make sure the model segmentations and voxels are updated too
      // TODO: Only update for this model instance
      //v.updateModelSegmentations(app.graphics);  // UNHACK
      //v.updateModelVoxels(app.graphics);  // UNHACK
      // Compute placement score
      const sg::core::SkelRange skelRange({*v.m_currentSkelIt});
      const float overlapRatio
        = sg::core::synth::placementScore(v.m_modelInstances[0], flattenedActiveSegs(), skelRange);
      cout << "overlap=" << overlapRatio << endl;
    }
  }

  if (v.m_state.database && v.m_state.database->centroids.getCentroidSetForAllJoints()) {
    const size_t numCentroids = v.m_state.database->centroids.getCentroidSetForAllJoints()->k;
    if (numCentroids > 0) {
      if (key == KEY_UP) { v.m_centroidIdx = (v.m_centroidIdx - 1) % numCentroids; }
      if (key == KEY_DOWN) { v.m_centroidIdx = (v.m_centroidIdx + 1) % numCentroids; }
      if (key == KEY_LEFT) { v.m_centroidMaxDist -= 0.05f; }
      if (key == KEY_RIGHT) { v.m_centroidMaxDist += 0.05f; }

      const size_t numActSegs = v.m_centroidActSegIndices.size();
      if (numActSegs > 0) {
        if (key == KEY_PGUP) { v.m_centroidActiveSegIdx = (v.m_centroidActiveSegIdx - 1) % numActSegs; }
        if (key == KEY_PGDN) { v.m_centroidActiveSegIdx = (v.m_centroidActiveSegIdx + 1) % numActSegs; }
        if (key == KEY_PGUP || key == KEY_PGDN) { v.updateVisualizations(app); }
      }

      if (key == KEY_UP || key == KEY_DOWN || key == KEY_LEFT || key == KEY_RIGHT) {
        v.m_centroidActiveSegIdx = -1;
        v.updateVisualizations(app);
      }
    }
  }

  bool skelMoved = false;
  if (v.m_pIsetPoseController && !v.m_state.movingModelInstance) {
    skelMoved = v.m_pIsetPoseController->update(key);
    if (skelMoved) {
      if (v.m_state.database->skeletons) {
        const string isetId = v.m_pIsetPoseController->currentInteractionSet().id;
        const auto& sDist = v.m_state.database->skeletons->getSkelDistribution(isetId);
        const Skeleton& s = v.m_pIsetPoseController->currentSkeleton();
        sg::core::SkelState ss;
        skel2state(s, &ss);
        ss = makeHierarchical(ss);
        SG_LOG_INFO << "lp=" << sDist.logprob(ss);
      }
      v.updateSegmentVerbProbabilityColors();
      v.updateVisualizations(app);
      const Skeleton& skel = v.m_pIsetPoseController->currentSkeleton();
      if (v.m_state.showPIGSim) {
        vizutil::reportInteractionSetPIGSim(skel, v.m_state, *v.m_params);
      }
      if (v.m_state.showKnnIGSim) {
        vizutil::reportInteractionSetKNNIGSim(skel, v.m_state, *v.m_params);
      }
    }
  }
  if ((skelMoved || modelMoved) && v.m_pIsetPoseController && 
      v.m_state.showInteractionScore) {
    const Skeleton& s = v.m_pIsetPoseController->currentSkeleton();
    const auto& iset = v.m_pIsetPoseController->currentInteractionSet();
    sg::core::synth::InteractionScorer iscorer;
    sg::core::synth::InteractionScorerParams params;
    params.init(*v.m_state.database->params);
    iscorer.init(v.m_state.database, params);
    double score = iscorer.score(iset, v.m_modelInstances, s);
    SG_LOG_INFO << "Interaction score: " << score;
  }
}

void VizzerControls::keyPressed(ml::ApplicationData& app, unsigned int key) {
  float distance = 0.2f;
  float theta = 2.5f;

  D3D11Vizzer& v = *m_viz;
  if (key == KEY_W) { v.m_camera.move(distance); }
  if (key == KEY_S) { v.m_camera.move(-distance); }
  if (key == KEY_A) { v.m_camera.strafe(-distance); }
  if (key == KEY_D) { v.m_camera.strafe(distance); }
  if (key == KEY_E) { v.m_camera.jump(distance); }
  if (key == KEY_Q) { v.m_camera.jump(-distance); }
  if (key == KEY_R) { v.m_camera.roll(GetAsyncKeyState(VK_SHIFT) ? -1 : 1 * theta); }
}

void VizzerControls::mouseDown(ml::ApplicationData& app, ml::MouseButtonType button) {
  D3D11Vizzer& v = *m_viz;
  if (app.input.mouse.buttons[ml::MouseButtonLeft] &&
      (GetAsyncKeyState(VK_SHIFT) || GetAsyncKeyState(VK_CONTROL))) {
    // Shift or Ctrl left-click selects a segment
    const ml::vec2f p(static_cast<float>(app.input.mouse.pos.x) / app.window.getWidth(),
                      static_cast<float>(app.input.mouse.pos.y) / app.window.getHeight());
    sg::mesh::IntersectionRecord rec;
    if (v.m_state.currScan->intersect(v.m_camera, p, &rec)) {
      const int segId = rec.segIdx;
      const auto& seg = v.m_state.currScan->getSegment(segId);
      assert(seg->id == segId);
      v.m_segSelectedSegId = segId;

      // Also send state change to UI to update selected segId
      v.m_state.sendStateUpdate("iSelectSeg", std::to_string(segId));
      cout << "id=" << segId
        << " c="   << seg->centroid<ml::vec3f>()
        << " nrm=" << seg->dominantNormal()
        << " npt=" << seg->points.size()
        << " dia=" << seg->diagonalLength()
        << " axl=" << seg->axesLengths<ml::vec3f>()
        << endl;

      sg::core::SegmentFeatureGeneratorWolf gen;
      const auto feats = gen.generate(*seg);
      for (int i = 0; i < feats.size(); ++i) { cout << feats[i] << ","; }
      cout << endl;

      // Trying to click segments - set show segmentation to be true
      v.m_state.showSegmentation = true;
      v.m_state.sendStateUpdate("showSegmentation", "true");
      if (GetAsyncKeyState(VK_CONTROL)) {
        if (GetAsyncKeyState(VK_SHIFT)) {
          // For Ctrl-Shift left-click we take the currently selected segment
          // and add/remove it from the currently selected segment group
          const auto segGroupPtr = v.m_state.currScan->segmentGroups.get(v.m_segSelectedSegGroupId);
          if (segGroupPtr != nullptr) {
            v.m_state.currScan->segmentGroups.toggle(v.m_segSelectedSegGroupId, *seg);
            v.m_state.sendAnnotationUpdate("segGroup", "select", v.m_segSelectedSegGroupId,
                                           segGroupPtr->label, segGroupPtr->info(), segGroupPtr->objectId);
            v.selectSegmentGroup(app, v.m_segSelectedSegGroupId);
          }
        } else {
          // For Ctrl left-click, we select or start a new segment group 
          // and saves it away in the annotations
          int sgId = v.m_state.currScan->segmentGroups.find(*seg);
          if (sgId < 0) {
            // Empty - start a new segment group
            const string label = "seg" + std::to_string(segId);
            sgId = v.m_state.currScan->segmentGroups.add(*seg, label);
            if (sgId >= 0) {
              // Successfully added
              bool absorb = v.m_params->get<bool>("Segmentation.segGroupAbsorb");
              if (absorb) {
                int absorbed = v.m_state.currScan->segmentGroups.absorb(sgId, *v.m_state.currScan->segments);
                cout << "Absorbed " << absorbed << " segments";
              }
              const auto segGroupPtr = v.m_state.currScan->segmentGroups.get(sgId);
              v.m_state.sendAnnotationUpdate("segGroup", "add", 
                                             sgId, segGroupPtr->label, segGroupPtr->info(), segGroupPtr->objectId);
            }
          } else {
            // Existing segment group
            const auto segGroupPtr = v.m_state.currScan->segmentGroups.get(sgId);
            v.m_state.sendAnnotationUpdate("segGroup", "select", sgId, 
                                           segGroupPtr->label, segGroupPtr->info(), segGroupPtr->objectId);
          }
          v.selectSegmentGroup(app, sgId);
        }
      } else {
        sg::segmentation::getSegmentationColors(v.m_state.currScan->mesh, v.m_segSelectedSegId, &v.m_currVertexColors);
        v.m_vertexColorsDirty = true;
        v.updateVisualizations(app);
      }

    }
  }
}

void VizzerControls::mouseWheel(ml::ApplicationData& app, int wheelDelta) {
  D3D11Vizzer& v = *m_viz;  // save some characters!
  const float distance = 0.002f;
  v.m_camera.move(distance * wheelDelta);
}

void VizzerControls::mouseMove(ml::ApplicationData& app) {
  D3D11Vizzer& v = *m_viz;  // save some characters!
  const float distance = 0.01f;
  const float theta = 0.25f;

  ml::vec2i posDelta = app.input.mouse.pos - app.input.prevMouse.pos;

  if (app.input.mouse.buttons[ml::MouseButtonRight]) {
    v.m_camera.strafe(distance * posDelta.x);
    v.m_camera.jump(-distance * posDelta.y);
  }

  if (app.input.mouse.buttons[ml::MouseButtonLeft]) {
    v.m_camera.lookRight(-theta * posDelta.x);
    v.m_camera.lookUp(theta * posDelta.y);
  }
}
