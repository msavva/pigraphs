#pragma once

// #include <AntTweakBar.h>

#include <core/Classifier.h>
#include <core/Scene.h>
#include <core/synth/InteractionSynth.h>
#include <cv/cvutil.h>
#include <interaction/InteractionFrames.h>
#include <interaction/InteractionMap.h>
#include <interaction/SkeletonVolume.h>

#include "./constants.h"
#include "./mLibInclude.h"
#include "./SharedAppState.h"
#include "./VizzerControls.h"
#include "./VizzerEventMap.h"
#include "./AssetManager.h"
#include "./AssetRenderer.h"

typedef std::map<std::string, double> ActionDescriptor;

// Encapsulates vis state members
struct VizzerState {
  explicit VizzerState(const sg::util::Params& p)
    : iframes(p.get<float>("InteractionFrame.halfWidth"),
              p.get<int>("InteractionFrame.numBins")) { }

  // Interaction frames
  sg::interaction::InteractionFrames iframes;
  sg::mesh::MeshActivationRecord sceneMeshActivationRec;

  // Skeleton Interaction Volume
  sg::interaction::SkeletonVolume skelVolume;
  // Currently activated (interacted with) parts
  sg::core::Skeleton::PartsByJointPlusGaze activeParts;

  std::map<std::string, ml::D3D11TriMesh> mesh;

  sg::core::Scene scene;

  //// AntTweakBar pointer
  // TwBar* twBar;

  // Strings that compose into console command to call when "test" shortcut is invoked
  std::vector<std::string> testFun;
};

class D3D11Vizzer : public ml::ApplicationCallback, private VizzerControls {
 public:
  D3D11Vizzer(SharedAppState& state, sg::util::Params* params);
  void init(ml::ApplicationData& app) override;
  void render(ml::ApplicationData& app) override;
  void keyDown(ml::ApplicationData& app, UINT key) override { VizzerControls::keyDown(app, key); };
  void keyPressed(ml::ApplicationData& app, UINT key) override { VizzerControls::keyPressed(app, key); }
  void mouseDown(ml::ApplicationData& app, ml::MouseButtonType button) override { VizzerControls::mouseDown(app, button); }
  void mouseMove(ml::ApplicationData& app) override { VizzerControls::mouseMove(app); }
  void mouseWheel(ml::ApplicationData& app, int wheelDelta) override { VizzerControls::mouseWheel(app, wheelDelta); }
  void resize(ml::ApplicationData& app) override;
  ~D3D11Vizzer();

  VizzerState vs;

  friend class VizzerEventMap;  // for setting up events that change private members
  friend class VizzerControls;  // for handling interactions (keys, mouse)

  //! Performs only rendering, without checking for events
  void renderOnly(ml::ApplicationData& app);

  //! Renders a single frame (complete begin-render-end)
  void renderFrame(ml::ApplicationData& app);

 private:
  typedef sg::core::Recording::SkelIter SkelIter;

  //void initTweakBar(ml::ApplicationData& app, TwBar* tw);
  void loadScan(ml::ApplicationData& app, const std::string& scanId,
                const ml::mat4f* pXform = nullptr, const ml::vec4f* pColorMult = nullptr);
  void loadModel(ml::ApplicationData& app, const std::string& loadType, const std::string& loadId);
  void loadScene(ml::ApplicationData& app, const std::string& file);
  void loadReconScene(ml::ApplicationData& app, const std::string& sceneId);
  void saveScene(const std::string& file, const double score = 0.0) const;
  void savePLY(const std::string& file, const std::vector<std::string>& meshes, bool rotateZupToYup = true) const;
  void clearScene(ml::ApplicationData& app);
  void initRecording(ml::ApplicationData& app, const std::string& recId, const bool loadBaseScene,
                     const unsigned int cameraIndex = CameraView_Recording);
  void processEvents(ml::ApplicationData& app);

  void setCurrentTime(ml::ApplicationData& app, const float tNowInSec);
  void updateInteractionOBBs(ml::GraphicsDevice& graphics);
  void updateInteractionFrame(ml::GraphicsDevice& graphics, const sg::core::SkelRange& skelRange);
  void initModelInteractionMaps(ml::GraphicsDevice& graphics);
  void updateModelInteractionMaps(const sg::core::SkelRange& skelRange);
  void setCurrentInteractions(const float t);

  void updateModelSegmentations(ml::GraphicsDevice& graphics, bool resegment = false);
  void updateModelVoxels(ml::GraphicsDevice& graphics);

  typedef std::function<void(const sg::segmentation::VecSegPtr&,sg::vec<double>*)> SegmentScoringFn;
  void scoreAndColorModelSegments(ml::GraphicsDevice& graphics, bool resegment, const SegmentScoringFn& scoringFn);

  void selectSegmentGroup(ml::ApplicationData& app, int segGroupId);
  void updateVisualizations(ml::ApplicationData& app);
  void updateInteractionHeatMap(ml::GraphicsDevice& g, const sg::interaction::InteractionSet& verbSet,
                                const sg::core::ScenePoseClassifier& classifier);
  void updateInteractionHeatMap(ml::GraphicsDevice& graphics,
                                const sg::vis::PoseHeatMap& heatMap,
                                bool showMaxSample);
  void updateMeshActivationMaps(ml::GraphicsDevice& g, const sg::core::SkelRange& skelRange);
  void updateSegmentVerbProbabilityColors();

  void makeEvaluationTruthScenes(ml::ApplicationData& app);
  void reprojectDepthFrame(const cv::Mat& depthAndBody, std::vector<ml::vec3f>* points) const;
  void renderSegmentOBBs(ml::GraphicsDevice& graphics);
  void renderSegmentGroupOBBs(ml::GraphicsDevice& graphics);
  void renderObjectOBBs(ml::GraphicsDevice& graphics);
  void recomputeBodyPoints(ml::GraphicsDevice& graphics);
  //! Create and load the database
  void createDatabase();
  //! Ensures database has been created and loaded
  void ensureDatabase();

  // Update mesh segmentation for parameter changes 
  void updateMeshSegmentation(); 
  // Load mesh segmentation from file
  void loadMeshSegmentation(const std::string& filename);
  // Mesh segmentation has changed - update active segments and such
  void meshSegmentationChanged(bool segmentGroupsReloaded = false);
  // Whether the segment is the selected segment or a segment in the selected group
  bool isSegmentSelected(int segId) const;

  // Returns appropriate color for segment for visualization
  ml::vec4f getSegmentColor(const sg::segmentation::MeshSegment& seg);
  // Returns appropriate color for segment group for visualization
  ml::vec4f getSegmentGroupColor(const sg::segmentation::SegmentGroup& segGroup);
  // Returns appropriate color for object for visualization
  ml::vec4f getObjectColor(const sg::segmentation::SegmentGroupObject& object);

  void classifySegmentsPerJoint();
  void classifySegmentsByJointsAggregated();
  void findSimilarMeshSegmentsForSelectedSegment();
  void findSimilarMeshSegmentsForJointInteraction();
  void findSimilarMeshSegments(const sg::segmentation::VecSegPtr& segments,
                               const sg::segmentation::MeshSegment& target, const size_t k = 0,
                               const size_t radius = 0.1);
  void findSimilarMeshSegments(const sg::segmentation::VecSegPtr& segments,
                               const sg::segmentation::VecSegPtr& target, const size_t k = 0,
                               const size_t radius = 0.1);
  void visualizeMeshSegments(const sg::segmentation::VecSegPtr& segments,
                             const std::vector<std::pair<size_t,double>>& pairs,
                             std::function<double(double)> convertScore = [] (double x) { return x; });
  //! Test a classifier by generating appropriate heatmap and doing screen capture
  void testClassifier(ml::ApplicationData& app, const std::string& classifierType);
  void testClassifier(ml::ApplicationData& app, const std::string& classifierType,
                      const std::string& interactionSet);
  void testClassifier(ml::ApplicationData& app, const std::string& classifierType,
                      const std::vector<std::string>& interactionSets);
  std::vector<sg::eval::BinaryConfusionMatrix> testClassifier(ml::ApplicationData& app,
                                                        sg::core::ScenePoseClassifier* pClassifier,
                                                        const sg::interaction::InteractionSet& verbSet,
                                                        const std::string& outputBase,
                                                        ActionDescriptor* actionDescriptor);
  //! Evaluate the heatmap and save images away 
  //! (evaluation is only done if evaluateId is not empty and pConfusionMatrices is not null)
  void evaluateInteractionHeatMap(ml::ApplicationData& app,
                                  const sg::vis::PoseHeatMap& heatMap,
                                  const std::string& outputBasename,
                                  const std::string& evaluateId,
                                  std::vector<sg::eval::BinaryConfusionMatrix>* pConfusionMatrices);
  //! Show scan voxelss
  using LabelOpts = sg::core::synth::LabelOpts;
  bool setLabelOptsFromArgs(const std::vector<std::string>& args,
                            const sg::interaction::InteractionSet* pISet,
                            sg::core::synth::LabelOpts* opts,
                            sg::core::OccupancyGrid::OccupancyType* occType);
  void showScanVoxels(ml::ApplicationData& app, const sg::core::OccupancyGrid::OccupancyType occType, 
                       bool useLabels, const LabelOpts& labelOpts = LabelOpts());
  //! Retrieve and place models
  void retrieveAndPlace(ml::ApplicationData& app, sg::core::synth::ModelPlacerParams& params);

  //! Synthesize interaction
  double interactionSynth(ml::ApplicationData& app, const std::vector<std::string>& args);
  //! Run interaction snapshot synth tests
  void testInteractionSynth(ml::ApplicationData& app, const std::vector<std::string>& args);

  const sg::interaction::InteractionSet* getCurrentInteractionSet() const;

  //! Bind basic light for rendering
  void bindBasicLight();
  //! Render and capture to bitmap
  void renderToBitmap(ml::ApplicationData& app,
                      std::vector<std::reference_wrapper<const ml::D3D11TriMesh>> meshes,
                      ml::ColorImageR8G8B8A8* pBitmap);
  //! Render and save to file
  void renderToFile(ml::ApplicationData& app,
                    std::vector<std::reference_wrapper<const ml::D3D11TriMesh>> meshes,
                    const std::string& filename);


  //! Set the camera to a predefined camera
  //! Camera 0 = Camera associated with the recording
  //! Camera 1 = Top down view
  //! Camera 2-5 = Side view 
  enum CameraView {
    CameraView_Recording = 0,
    CameraView_Top = 1,
    CameraView_Side_Left = 2,
    CameraView_Side_Right = 3,
    CameraView_Side_Front = 4,
    CameraView_Side_Back = 5,
    CameraView_Skeleton = 6,
    CameraView_ISynth = 7,
    CameraView_Current = 8,
    CameraView_Count = 9
  };
  void setCamera(int width, int height, const unsigned cameraIdx);
  void setCamera(ml::ApplicationData& app, const unsigned cameraIdx);

  //! Set the camera to look at something
  void setCameraToLookAt(int width, int height, const ml::vec3f& eye, const ml::vec3f& target);

  //! Returns active skeleton
  const sg::core::Skeleton* getActiveSkeleton() const;

  void dumpMesh(const std::string& filename = "default.ply");

  //! Callback for visualizing alignment state
  void renderAlignmentState(ml::ApplicationData& app, const sg::core::synth::AlignmentState& state);

  //! Callback for visualizing placement state
  void renderPlacementState(ml::ApplicationData& app, const sg::core::synth::PlacementState& state);

  VizzerEventMap m_eventMap;

  SharedAppState& m_state;

  AssetManager m_assets;
  AssetRenderer m_renderer;

  std::vector<sg::core::ModelInstance> m_modelInstances;  // Model instances to render
  std::vector<std::shared_ptr<sg::interaction::InteractionMap>> m_modelInteractionMaps;
  std::vector<ml::D3D11TriMesh> m_modelInteractionMapMeshes;
  std::vector<ml::D3D11TriMesh> m_segmentedModelInstanceMeshes;
  std::vector<ml::D3D11TriMesh> m_voxelizedModelInstanceMeshes;

  ml::D3D11VertexShader m_vsBasicLight;
  ml::D3D11PixelShader m_psBasicLight;

  ml::D3D11Font m_font;
  ml::FrameTimer m_timer;

  ml::D3D11ConstantBuffer<constants::BasicLightShaderConstantBuffer> m_basicLightConstants;
  constants::BasicLightShaderConstantBuffer m_basicLightConstantsBuffer;

  ml::Cameraf m_camera;
  ml::D3D11RenderTarget m_renderTarget;

  ml::ColorGradient m_gradientCool2Warm, m_gradientHeatmap;
  sg::vis::PoseHeatMap* m_heatMap;

  // Time manipulation
  SkelIter m_currentSkelIt;

  // Segmentation
  sg::segmentation::SegmentationParams m_segParams;
  int m_segSelectedSegId;
  int m_segSelectedSegGroupId;

  // Annotations
  std::vector<sg::interaction::Interaction*> m_currInteractions;
  std::vector<sg::interaction::Interaction*>::const_iterator m_currInteraction;
  int m_currInteractionSkelIdx;
  bool m_useInteractionActiveSegs;

  // Affordance map integration over time
  float m_integrationTimeWindowSec;  // Number of frames to integrate centered around current time
  int m_integrationJointIndex;  // Index of skeleton joint to integrate

  float m_activationRadius;
  std::vector<ml::vec4f> m_baseVertexColors;
  std::vector<ml::vec4f> m_currVertexColors;
  bool m_vertexColorsDirty;

  int m_centroidIdx;
  int m_centroidActiveSegIdx;
  std::vector<size_t> m_centroidActSegIndices;
  float m_centroidMaxDist;

  // Video frames
  cv::Mat m_depthBodyIndex, m_color, m_imgPlanePts, m_temp;
  std::vector<ml::vec3f> m_xyzPts;

  sg::core::ISetPoseController* m_pIsetPoseController;
  sg::util::Params* m_params;

  sg::core::synth::InteractionSynth m_interactionSynth;
  sg::vis::ColorIndex m_categoryColors;
  sg::vis::ColorIndex m_partLabelColors;
  sg::vis::ColorIndex m_objectIdColors;
  sg::vis::ColorIndex m_interactionLabelColors;
  std::string m_predictedInfo;
  sg::core::Skeleton m_predictedSkeleton;
  bool m_showPredictedSkeleton;

  bool m_voxelizedSkel;
  bool m_showInfoOverlay;

  // Point probing for similarity testing
  ml::vec3f m_probePointA, m_probePointB;
  bool m_showProbePoints;
};


