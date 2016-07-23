#include "common.h"  // NOLINT

#include "./SharedAppState.h"

#include <core/Database.h>
//#include <util/util.h>

SharedAppState::SharedAppState()
  : currClassifierType("IGKNNSim")
  , workingDir(ml::util::getWorkingDirectory() + "/")
  , mode(0)
  , isVizStarted(false)
  , terminate(false)
  , paused(false)
  , showSegmentVerbProbability(false)
  , showSkeletons(true)
  , showGaze(false)
  , showSegmentation(false)
  , showModelSegmentation(false)
  , loadModelSegmentation(true)
  , showModelVoxels(false)
  , showActivationMap(false)
  , showAllJointsActivation(false)
  , showAverageSkeleton(false)
  , hideUnactivatedMesh(false)
  , showSegmentOBBs(false)
  , showSegmentGroupOBBs(false)
  , showObjectOBBs(false)
  , showClusterAssignment(false)
  , showScan(true)
  , showScanVoxels(false)
  , showScanLabeledVoxels(false)
  , showBodyPoints(false)
  , showModels(false)
  , showInteractionOBBs(true)
  , showInteractionFrame(false)
  , showModelInteractionMaps(true)
  , showInteractionHeatMap(false)
  , inPosingMode(false)
  , showColorFrames(false)
  , showDepthFrames(false)
  , playingRange(false)
  , movingModelInstance(false)
  , interactionHeatMapNormalize(true)
  , interactionHeatMapShowAngle(true)
  , jointSegKnnUseCurr(false)
  , showKnnIGSim(false)
  , showPIGSim(false)
  , showInteractionScore(false)
  , showOIMAnnotations(false)
  , useSingleSkelForHeatmap(true)
  , timeNowSec(-1)
  , timeMinSec(-1)
  , timeMaxSec(-1)
  , rangeTimeStartSec(-1)
  , rangeTimeEndSec(-1)
  , rangeTimeStepSec(-1)
  , colorSegmentBy(Color_ById)
  , partType(sg::segmentation::kPartSegment)
  , iframeType(sg::interaction::InteractionFrameType::kJointsVolumetric)
  , currScan(nullptr)
  , currRecording(nullptr)
  , database(nullptr) { }

void SharedAppState::sendMessage(UITarget target, const std::string& message) {
  mutex.lock();
  appMessages[target].push_back(message);
  mutex.unlock();
}

void SharedAppState::checkForUpdates() {
  mutex.lock();
  ui.readMessages();
  for (const std::string& s : ui.messages()) {
    std::vector<std::string> parts = ml::util::split(s, " ");
    std::string command = parts[0];
    if (ml::util::startsWith(command, "D3D:")) { appMessages[UI_D3D].push_back(s); }
    else if (ml::util::startsWith(command, "CV:")) { appMessages[UI_CV].push_back(s); }
    else {
      if (parts.size() == 1) {
        if (command == "terminate") { terminate = true; }
        else if (command == "pause") { paused = !paused; }
      } else {
        std::vector<std::string> parameters = ml::util::split(parts[1], ",");
        if (command == "mode") { mode = ml::convert::toInt(parameters[0]); }
        else if (command == "showColorFrames") { showColorFrames = ml::convert::toBool(parameters[0]); }
        else if (command == "showDepthFrames") { showDepthFrames = ml::convert::toBool(parameters[0]); }
        else if (command == "heatmapNormalize") { interactionHeatMapNormalize = ml::convert::toBool(parameters[0]); }
        else if (command == "heatmapShowAngle") { interactionHeatMapShowAngle = ml::convert::toBool(parameters[0]); }
        else if (command == "jointSegKnnUseCurr") { jointSegKnnUseCurr = ml::convert::toBool(parameters[0]); }
        else if (command == "currentClassifierType") { currClassifierType = parameters[0]; }
        else if (command == "useSingleSkelForHeatmap") { useSingleSkelForHeatmap = ml::convert::toBool(parameters[0]); }
        else if (command == "showAllJointsActivation") { showAllJointsActivation = ml::convert::toBool(parameters[0]); }
        else if (command == "showKnnIGSim") { showKnnIGSim = ml::convert::toBool(parameters[0]); }
        else if (command == "showPIGSim") { showPIGSim = ml::convert::toBool(parameters[0]); }
      }
    }
  }
  ui.messages().clear();
  mutex.unlock();
}

void SharedAppState::sendStateInit() {
  std::vector<std::pair<std::string, std::string>> params;
  if (currRecording != nullptr) {
    params.push_back(std::make_pair("recDuration", std::to_string(currRecording->durationInSec())));
    params.push_back(std::make_pair("recId", currRecording->id));
    params.push_back(std::make_pair("recInteractionsFilename", currRecording->interactionsFilename()));
  }
  if (currScan != nullptr) {
    params.push_back(std::make_pair("scanId", currScan->id));
  }
  if (database != nullptr) {
    const std::string scanDir =
      database->params->get<std::string>("dataDir") + database->params->get<std::string>("scanDir");
    const std::string recDir =
      database->params->get<std::string>("dataDir") + database->params->get<std::string>("recDir");
    params.push_back(std::make_pair("scanDir", scanDir));
    params.push_back(std::make_pair("recDir", recDir));
  }
  sendUpdate("stateInit", params);
}

void SharedAppState::sendStateUpdate() {
  std::vector<std::pair<std::string, std::string>> params;
  params.push_back(std::make_pair("mode", std::to_string(mode) ));
  params.push_back(std::make_pair("time", std::to_string(timeNowSec / timeMaxSec) ));
  sendUpdate("stateUpdate", params);
}

void SharedAppState::sendStateUpdate(const std::string& param, const std::string& value) {
  std::vector<std::pair<std::string, std::string>> params;
  params.push_back(std::make_pair(param, value));
  sendUpdate("stateUpdate", params);
}

void SharedAppState::sendAnnotationUpdate(const std::string& annoType, 
                                          const std::string& action, 
                                          int idx, 
                                          const std::string& label,
                                          const std::string& info,
                                          const int objectId) {
  std::vector<std::pair<std::string, std::string>> params;
  params.push_back(std::make_pair("annoType", annoType));
  params.push_back(std::make_pair("action", action));
  params.push_back(std::make_pair("idx", std::to_string(idx)));
  params.push_back(std::make_pair("label", label));
  params.push_back(std::make_pair("info", info));
  params.push_back(std::make_pair("objectId", std::to_string(objectId)));
  sendUpdate("annotationUpdate", params);
}

void SharedAppState::sendAnnotationUpdate(const std::string& annoType, const std::string& action) {
  std::vector<std::pair<std::string, std::string>> params;
  params.push_back(std::make_pair("annoType", annoType));
  params.push_back(std::make_pair("action", action));
  sendUpdate("annotationUpdate", params);
}

void SharedAppState::sendUpdate(const std::string& name,
                                const std::vector<std::pair<std::string, std::string>>& params) {
  std::ostringstream s;
  s << name << " ";
  for (const auto& iter : params) {
    s << iter.first << "=" << iter.second << "|";
  }
  //std::cout << s.str() << std::endl;
  mutex.lock();
  ui.sendMessage(s.str().c_str());
  mutex.unlock();
}




