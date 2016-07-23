#pragma once

#include <string>
#include <thread>
#include <vector>

#include <mLibCore.h>
#include <core/Recording.h>
#include <core/Scan.h>
#include <cv/VideoPlayer.h>

struct SharedAppState {
  enum UITarget { UI_D3D = 0, UI_CV, UI_TARGET_COUNT };
  enum ModeType {
    Mode_Replay = 0,  // "replay"
    Mode_Pose  = 1,   // "pose"
    Mode_Count  = 2
  };
  enum ColorSegmentBy {
    Color_ById = 0,
    Color_ByLabel = 1,
    Color_ByCategory = 2,
    Color_ByObjectId = 3
  };

  SharedAppState();

  // Parameters and UI state
  std::string currClassifierType;
  const std::string workingDir;
  ml::UIConnection ui;
  std::recursive_mutex mutex;
  unsigned int mode;
  std::thread vizzerThread;
  bool
    isVizStarted,
    terminate,
    paused,
    showSegmentVerbProbability,
    showSkeletons,
    showGaze,
    showSegmentation,
    showModelSegmentation,
    loadModelSegmentation,
    showModelVoxels,
    showActivationMap,
    showAllJointsActivation,
    showAverageSkeleton,
    hideUnactivatedMesh,
    showSegmentOBBs,
    showSegmentGroupOBBs,
    showObjectOBBs,
    showClusterAssignment,
    showScan,
    showScanVoxels,
    showScanLabeledVoxels,
    showBodyPoints,
    showModels,
    showInteractionOBBs,
    showInteractionFrame,
    showModelInteractionMaps,
    showInteractionHeatMap,
    inPosingMode, // This should be kept in sync with the mode
    showColorFrames, showDepthFrames,
    playingRange,
    movingModelInstance,
    interactionHeatMapNormalize,
    interactionHeatMapShowAngle,
    jointSegKnnUseCurr,
    showKnnIGSim,
    showPIGSim,
    showInteractionScore,
    showOIMAnnotations,
    useSingleSkelForHeatmap;
  float
    timeNowSec,
    timeMinSec,
    timeMaxSec,
    rangeTimeStartSec,
    rangeTimeEndSec,
    rangeTimeStepSec;
  ColorSegmentBy colorSegmentBy;
  sg::segmentation::PartType partType;
  sg::interaction::InteractionFrameType iframeType;
  std::vector<std::string> appMessages[UI_TARGET_COUNT];

  // Input data
  static const int kMaxANNpoints = 2048;
  sg::core::Scan* currScan;
  sg::core::Recording* currRecording;
  sg::cvutil::VideoPlayer
    colorPlayer,
    depthPlayer;

  sg::core::Database* database;

  void sendMessage(UITarget target, const std::string& message);

  void checkForUpdates();

  void sendStateInit();

  void sendStateUpdate();

  void sendStateUpdate(const std::string& param, const std::string& value);

  void sendAnnotationUpdate(const std::string& annoType,
                            const std::string& action);

  void sendAnnotationUpdate(const std::string& annoType,
                            const std::string& action, 
                            int idx, 
                            const std::string& label,
                            const std::string& info,
                            const int objectId);

  void sendUpdate(const std::string& name, const std::vector<std::pair<std::string, std::string>>& params);
};


