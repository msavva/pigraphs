#include "common.h"  // NOLINT

#include "./vizutil.h"

#include <core/Database.h>
#include <core/OccupancyGrid.h>
#include <core/Scan.h>
#include <cv/cvutil.h>
#include <geo/geo.h>
#include <interaction/InteractionGraph.h>
#include <interaction/ProtoInteractionGraph.h>
#include <mesh/sampling.h>
#include <vis/Heatmap.h>

#include "./constants.h"
#include "./SharedAppState.h"
#include "./calibratedSensorData.h"

namespace vizutil {

using namespace sg;
using core::Recording; using core::Skeleton; using core::SkelRange;
using core::ModelInstance; using core::Scan; using core::OccupancyGrid;
using IG = interaction::InteractionGraph;

void dumpDepthFramesToSensorFileAndPLY(const string& depthFile, unsigned int numFrames) {
  cout << "Dumping sensor and ply file for " << depthFile << endl;
  cvutil::VideoPlayer depthPlayer;
  if (ml::util::fileExists(depthFile)) {
    depthPlayer.open(depthFile);
    cout << "Loaded depth video: " << ml::util::fileNameFromPath(depthFile) << endl;
  } else {
    cerr << "Could not load depth video: " << depthFile << endl;
    return;
  }

  ml::CalibratedSensorData cs;
  depthPlayer.seekToFrame(0);

  for (unsigned int i = 0; i < numFrames; i++) {
    cv::Mat frame, bodyIdx, d0, d1;
    cv::Mat split[] = {bodyIdx, d0, d1};
    depthPlayer.read(frame);
    cv::split(frame, split);
    cv::Mat depth;
    cv::Mat merge[] = {split[1], split[2]};
    cv::merge(merge, 2, depth);

    float* depthData = new float[depth.rows * depth.cols];
    for (int j = 0; j < depth.rows * depth.cols; j++) {
      depthData[j] = static_cast<float>(reinterpret_cast<const uint16_t*>(depth.ptr())[j]) / 1000.0f;
    }
    cs.m_DepthImages.push_back(depthData);

    if (i == 0) {
      cs.m_DepthNumFrames = numFrames;
      cs.m_DepthImageHeight = depth.rows;
      cs.m_DepthImageWidth = depth.cols;
      cs.m_CalibrationDepth.setIdentity();

      //data: [ 3.6750353136295951e+002, 0., 2.6709172115436729e+002, 0.,
      //      3.6633843419010759e+002, 1.9722506230697877e+002, 0., 0., 1. ]
      float fx = 367.5f;
      float fy = -366.3f;	//note the minus which ensures the Y-flip between scan and skeleton data
      float mx = 267.1f;
      float my = 197.2f;

      cs.m_CalibrationDepth.m_Intrinsic = ml::mat4f(
        fx, 0.0f, mx, 0.0f,
        0.0f, fy, my, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f);
      cs.m_CalibrationDepth.m_IntrinsicInverse = cs.m_CalibrationDepth.m_Intrinsic.getInverse();
    }
  }

  const string
    basename    = io::basename(depthFile),
    sensorFile  = basename + ".sensor",
    plyFile     = basename + ".ply";

  // write ply
  cout << cs << endl;
  cout << "Dumping to " << plyFile << "...";
  cs.savePointCloud(plyFile, numFrames / 2);
  cout << " done!" << endl;

  // write sensor
  cout << "Dumping to " << sensorFile << "...";
  ml::BinaryDataStreamFile outStream(sensorFile, true);
  outStream << cs;
  cout << " done!" << endl;
}

void projectOrtho(const ml::D3D11TriMesh& mesh, ml::GraphicsDevice* graphics,
                  ml::D3D11ConstantBuffer<constants::BasicLightShaderConstantBuffer>* d3d11ConstantBuffer,
                  ml::D3D11RenderTarget* renderTarget, ml::ColorImageR8G8B8A8* outBitmap) {
  std::vector<std::reference_wrapper<const ml::D3D11TriMesh>> meshes;
  meshes.emplace_back(mesh);
  projectOrtho(meshes, graphics, d3d11ConstantBuffer, renderTarget, outBitmap);
}

void projectOrtho(std::vector<std::reference_wrapper<const ml::D3D11TriMesh>> meshes, ml::GraphicsDevice* graphics,
                  ml::D3D11ConstantBuffer<constants::BasicLightShaderConstantBuffer>* d3d11ConstantBuffer,
                  ml::D3D11RenderTarget* renderTarget, ml::ColorImageR8G8B8A8* outBitmap) {
  SG_LOG_ERROR << "[projectOrtho] Unreliable Camera code";
  // Bind to render target and clear
  renderTarget->bind();
  renderTarget->clear(ml::vec4f(0.7f, 0.8f, 0.9f, 0.0f));

  // Set top-view orthographic camera (Z-up - assumes bounding box is not too large)
  ml::bbox3f bbox; 
  for (const ml::D3D11TriMesh& mesh: meshes)  {
    bbox.include(mesh.computeBoundingBox());
  }
  float aspectRatio = (float) renderTarget->getWidth() / renderTarget->getHeight();
  //todo manolis check: here's the origina -- it's somewhat suspicous with 0.0f near, and 100.0f far
  //const ml::Cameraf camera(ml::vec3f(bbox.getCenter().x, bbox.getCenter().y, 10.0f),-ml::vec3f::eY, bbox.getCenter(), 60.0f, aspectRatio, 0.0f, 1.0f, false);
  const ml::Cameraf camera(
    ml::vec3f(bbox.getCenter().x, bbox.getCenter().y, 10.0f),
    bbox.getCenter(),
    -ml::vec3f::eY,
    60.0f,
    aspectRatio, 0.0001f, 100.0f);

  const float scale = 2.0f / (std::max(bbox.getExtentX(), bbox.getExtentY()) + 0.1f);
  const ml::mat4f proj = ml::mat4f::scale(scale, -scale, 0.01f);
  constants::BasicLightShaderConstantBuffer constants;
  constants.worldViewProj = proj * camera.getCamera();
  d3d11ConstantBuffer->update(constants);
  for (const ml::D3D11TriMesh& mesh: meshes) {
    mesh.render();
  }
  renderTarget->captureColorBuffer(*outBitmap);

  // Reset back to default render targets
  graphics->castD3D11().bindRenderTarget();
}

void updateSegGroupAnnotations(SharedAppState& state, bool segmentGroupsReloaded) {  // NOLINT
  if (segmentGroupsReloaded) {
    state.sendAnnotationUpdate("segGroup", "clear");
    for (int sgIdx = 0; sgIdx < state.currScan->segmentGroups.size(); sgIdx++) {
      const auto segGroupPtr = state.currScan->segmentGroups.at(sgIdx);
      state.sendAnnotationUpdate("segGroup", "add", 
                                 segGroupPtr->id, segGroupPtr->label, segGroupPtr->info(), segGroupPtr->objectId);
    }
  } else {
    // Just an update to existing segment groups
    for (int sgIdx = 0; sgIdx < state.currScan->segmentGroups.size(); sgIdx++) {
      const auto segGroupPtr = state.currScan->segmentGroups.at(sgIdx);
      state.sendAnnotationUpdate("segGroup", "update", 
                                 segGroupPtr->id, segGroupPtr->label, segGroupPtr->info(), segGroupPtr->objectId);
    }
  }
}

void sampleMesh(ml::GraphicsDevice& g, const ml::TriMeshf& mesh, size_t numSamples, // NOLINT
                ml::D3D11TriMesh* pointsMesh, const ml::vec4f color /*= ml::vec4f(1, 0, 0, 1)*/) {
  vec<geo::Vec3f> sampled;
  mesh::sampling::sampleTris(mesh, numSamples, &sampled);
  const ml::TriMeshf box = ml::Shapesf::box(0.01f, color);
  vec<ml::vec3f> sampled2(sampled.size());
  for (int i = 0; i < sampled.size(); ++i) {
    sampled2[i] = geo::to<ml::vec3f>(sampled[i]);
  }
  pointsMesh->load(g, ml::meshutil::createPointCloudTemplate(box, sampled2));
}

Skeleton::SegmentsByJointPlusGaze getCurrentActiveSegs(const Skeleton& skel, const SharedAppState& state) {
  const SkelRange skelRange(skel);
  return state.database->interactionFactory.getActiveSegments(*state.currScan, skelRange);
}

void testWritingPIGGraphML(const Skeleton& skel, const SharedAppState& state) {
  std::shared_ptr<IG> pIG =
    state.database->interactionFactory.createInteractionGraph(*state.currScan, skel, state.partType);
  IG& ig = *pIG;

  ofstream igofs("test.graphml");
  ig.writeGraphML(igofs);
  igofs.close();
  interaction::ProtoInteractionGraphAggregator pigAgg;
  interaction::ProtoInteractionGraph pig("test");
  pigAgg.addInteractionGraph(ig, &pig);
  ofstream pigofs("test-proto.graphml");
  pig.writeGraphML(pigofs);
  pigofs.close();
  pig.saveBinary("test-proto.pigbin");
  cout << "Wrote out test IG+PIG GraphML files" << endl;

  interaction::ProtoInteractionGraph pig2("test2");
  pig2.loadBinary("test-proto.pigbin");
  pig2.saveBinary("test-proto2.pigbin");
}

bool moveModelInstance(UINT key, ModelInstance* mInst) {
  const float dStep = 0.05f, tStep = math::constants::PIf / 16.f;
  int dX = 0, dY = 0, dZ = 0, dT = 0; float dS = 0.f;
  switch (key) {
    case KEY_LEFT   : dX =   -1;     break;
    case KEY_RIGHT  : dX =    1;     break;
    case KEY_UP     : dY =    1;     break;
    case KEY_DOWN   : dY =   -1;     break;
    case KEY_PGUP   : dT =    1;     break;
    case KEY_PGDN   : dT =   -1;     break;
    case KEY_HOME   : dZ =    1;     break;
    case KEY_END    : dZ =   -1;     break;
    case KEY_9      : dS =  -.1f;    break;
    case KEY_0      : dS =   .1f;    break;
    default         : return false;  // Bail out since no updates
  }

  const geo::Transform S(Eigen::Scaling(1 + dS)), R(Eigen::AngleAxisf(dT * tStep, geo::Vec3f::UnitZ()));
  const geo::Matrix3f SR = (S * R).linear();
  const geo::Vec3f t(dX * dStep, dY * dStep, dZ * dStep);
  mInst->updateLocalScaleRotateBy(SR);
  mInst->updateLocalTranslationBy(t);
  return true;
}

bool moveProbePoint(UINT key, ml::vec3f& p) { // NOLINT
  const float d = 0.05f;
  switch (key) {
    case KEY_Y : p.x -= d;      break;
    case KEY_U : p.x += d;      break;
    case KEY_H : p.y -= d;      break;
    case KEY_J : p.y += d;      break;
    case KEY_K : p.z += d;      break;
    case KEY_L : p.z -= d;      break;
    default    : return false;  // Bail out since no updates
  }
  return true;
}

void projectSamplesToMeshVerts(const vec<vis::PoseHeatMap::Sample>& samples, const vec<ml::TriMeshf::Vertex>& V,
                               float maxRadius, vec<float>* vertexWeights) {
  // Project mesh vertices
  const int numVerts = static_cast<int>(V.size());
  geo::VecVec2f verts2D;
  verts2D.reserve(numVerts);
  for (int iV = 0; iV < numVerts; ++iV) {
    verts2D.emplace_back(V[iV].position.x, V[iV].position.y);
  }

  // Iterate over all heatmap samples and project weight into all vertices within threshold distance
  const float maxR2 = maxRadius * maxRadius;
  const float sigma = 1.0f/(0.4f*maxR2);
  vec<int> counts(numVerts, 0);
  auto& W = *vertexWeights;
  W.clear();
  W.resize(numVerts, 0.f);
  for (const auto& s : samples) {
    const auto& pSample = s.pos();
#pragma omp parallel for
    for (int iV = 0; iV < numVerts; ++iV) {  // TODO(MS): Replace this with proper kNN search
      const geo::Vec2f& pVert = verts2D[iV];
      const float distSq = (pSample - pVert).squaredNorm();
      if (distSq < maxR2) {
        //const float w = 1.f; // - sqrtf(distSq) / maxR;  // [0,1] weight
        const float w = std::expf(-distSq*sigma);
        if (w > W[iV]) {
          W[iV] = static_cast<float>(s.likelihood)*w;
        }
        //W[iV] += static_cast<float>(s.likelihood) * w;
        counts[iV]++;
      }
    }
  }
  // Normalize by count
  //for (size_t iV = 0; iV < numVerts; ++iV) {
  //  W[iV] = (W[iV] == 0.f) ? 0.f : W[iV] / counts[iV];
  //}
}

void reportInteractionSetKNNIGSim(const Skeleton& skel,
                                  const SharedAppState& sas,
                                  const util::Params& p) {

  const float igSimNNpercent = p.get<float>("Interaction.igSimNNpercent");
  const int igSimMaxNumNNs = p.get<int>("Interaction.igSimNNmaxNumNNs");
  // Create IG for skel and segments
  const std::shared_ptr<IG> pIG = 
    sas.database->interactionFactory.createInteractionGraph(*sas.currScan, skel, sas.partType);
  const IG& ig = *pIG;

  // Sort all other IGs by similarity to current IG and print out top
  struct SimResult { float sim; std::string vSetId; };
  vec<SimResult> results;
  // TODO: Consider more atomic interaction sets
  for (const auto& pair : sas.database->interactions.getInteractionSets()) {
    const interaction::InteractionSet& vSet = pair.second;
    float sim = vSet.similarity(ig, igSimNNpercent, igSimMaxNumNNs);
    results.push_back({sim, vSet.id});
  }
  std::sort(results.begin(), results.end(), [ ] (const SimResult& a, const SimResult& b) {
    return a.sim > b.sim;
  });

  // Print sorted vSet similarities
  using std::cout; using std::endl; const auto oldPrecision = std::cout.precision();
  cout << "KNNIGSim: { ";
  cout << std::setprecision(4);
  for (int i = 0; i < results.size(); ++i) {
    const SimResult& sr = results[i];
    cout << sr.vSetId << ":" << sr.sim << " ";
  }
  cout << "}" << endl;
  cout << std::setprecision(oldPrecision);
}

void reportInteractionSetPIGSim(const Skeleton& skel, 
                                const SharedAppState& sas,
                                const util::Params& p) {
  // Create IG for skel and segments
  const std::shared_ptr<IG> pIG =
    sas.database->interactionFactory.createInteractionGraph(*sas.currScan, skel, sas.partType);
  const IG& ig = *pIG;

  // Sort PIGs by similarity to current IG and selet the top PIG
  struct SimResult { float sim; string vSetId; };
  vec<SimResult> results;
  // TODO: Consider more atomic interaction sets
  for (const auto& pair : sas.database->interactions.getInteractionSets()) {
    const interaction::InteractionSet& vSet = pair.second;
    results.push_back({vSet.protoSimilarity(ig, false), vSet.id});
  }
  std::sort(results.begin(), results.end(), [ ] (const SimResult& a, const SimResult& b) {
    return a.sim > b.sim;
  });

  // Print sorted vSet similarities
  const auto oldPrecision = cout.precision();
  cout << "PIGSim: { " << endl;
  cout << std::setprecision(4);
  for (int i = 0; i < results.size(); ++i) {
    const SimResult& sr = results[i];
    cout << sr.vSetId << ":" << sr.sim << " " << endl;
  }
  cout << "}" << endl;
  cout << std::setprecision(oldPrecision);
}

// Greedily mesh binary voxel grid in volume into TriMesh and return
ml::TriMeshf greedyMesher(const ml::BinaryGrid3& volume, const ml::mat4f& voxel2world) {
  vec<int> mask;
  mask.resize(4096);

  const auto dims = volume.getDimensions();
  vec<ml::vec3f> vertices;
  vec<unsigned int> faces;
  // int dimsX = static_cast<int>(dims[0]);
  // int dimsY = static_cast<int>(dims[1]);
  // int dimsXY = dimsX * dimsY;

  //Sweep over 3-axes
  for (int d = 0; d < 3; ++d) {
    int i, j, k, l, w, W, h, n, c;
    int u = (d + 1) % 3;
    int v = (d + 2) % 3;
    int x[3] = {0, 0, 0};
    int q[3] = {0, 0, 0};
    int du[3] = {0, 0, 0};
    int dv[3] = {0, 0, 0};
    int dimsD = static_cast<int>(dims[d]);
    int dimsU = static_cast<int>(dims[u]);
    int dimsV = static_cast<int>(dims[v]);
    int xd;

    if (mask.size() < dimsU * dimsV) {
      mask.resize(dimsU * dimsV);
    }

    q[d] = 1;
    x[d] = -1;

    // Compute mask
    while (x[d] < dimsD) {
      xd = x[d];
      n = 0;

      for (x[v] = 0; x[v] < dimsV; ++x[v]) {
        for (x[u] = 0; x[u] < dimsU; ++x[u], ++n) {
          bool a = (xd >= 0)        && volume.isVoxelSet(x[0], x[1], x[2]);
          bool b = (xd < dimsD - 1) && volume.isVoxelSet(x[0] + q[0], x[1] + q[1], x[2] + q[2]);
          if (a ? b : !b) {
            mask[n] = 0;
            continue;
          }
          mask[n] = a ? 1 : (b ? -1 : 0);
        }
      }

      ++x[d];

      // Generate mesh for mask using lexicographic ordering
      n = 0;
      for (j = 0; j < dimsV; ++j) {
        for (i = 0; i < dimsU;) {
          c = mask[n];
          if (!c) {
            i++;
            n++;
            continue;
          }

          //Compute width
          w = 1;
          while (c == mask[n + w] && i + w < dimsU) {
            w++;
          }

          //Compute height (this is slightly awkward)
          for (h = 1; j + h < dimsV; ++h) {
            k = 0;
            while (k < w && c == mask[n + k + h*dimsU]) {
              k++;
            }
            if (k < w) {
              break;
            }
          }

          // Add quad
          // The du/dv arrays are reused/reset
          // for each iteration.
          du[d] = 0; dv[d] = 0;
          x[u] = i;
          x[v] = j;

          if (c > 0) {
            dv[v] = h; dv[u] = 0;
            du[u] = w; du[v] = 0;
          } else {
            // ReSharper disable once CppAssignedValueIsNeverUsed
            c = -c;
            du[v] = h; du[u] = 0;
            dv[u] = w; dv[v] = 0;
          }
          unsigned int numVerts = static_cast<unsigned int>(vertices.size());
          const ml::vec3f vx(static_cast<float>(x[0]), static_cast<float>(x[1]), static_cast<float>(x[2]));
          vertices.push_back(vx);
          vertices.push_back(ml::vec3f(vx[0] + du[0], vx[1] + du[1], vx[2] + du[2]));
          vertices.push_back(ml::vec3f(vx[0] + du[0] + dv[0], vx[1] + du[1] + dv[1], vx[2] + du[2] + dv[2]));
          vertices.push_back(ml::vec3f(vx[0] + dv[0], vx[1] + dv[1], vx[2] + dv[2]));
          faces.push_back(numVerts);
          faces.push_back(numVerts + 1);
          faces.push_back(numVerts + 3);
          faces.push_back(numVerts + 1);
          faces.push_back(numVerts + 2);
          faces.push_back(numVerts + 3);

          //Zero-out mask
          W = n + w;
          for (l = 0; l < h; ++l) {
            for (k = n; k < W; ++k) {
              mask[k + l*dimsU] = 0;
            }
          }

          //Increment counters and continue
          i += w;
          n += w;
        }
      }
    }
  }
  ml::TriMeshf mesh(vertices, faces);
  mesh.transform(voxel2world);
  mesh.computeNormals();
  mesh.setColor(ml::vec4f(1, 1, 1, .5));
  return mesh;
}

//! Create scan voxels trimesh
ml::TriMeshf scanVoxelsToTriMesh(const Scan& scene, const OccupancyGrid::OccupancyType occType) {
  const ml::mat4f grid2world = scene.getOccupancyGrid().gridToWorld();
  ml::BinaryGrid3 occ;
  switch (occType) {
    case OccupancyGrid::OccupancyType_Free:
      occ = scene.getOccupancyGrid().free();
      break;
    case OccupancyGrid::OccupancyType_Unknown:
      occ = scene.getOccupancyGrid().unknown();
      break;
    case OccupancyGrid::OccupancyType_UnknownOrOccupied:
      occ = scene.getOccupancyGrid().unknownOrOccupied();
      break;
    case OccupancyGrid::OccupancyType_Occupied:
      occ = scene.getOccupancyGrid().occupied();
      break;
    case OccupancyGrid::OccupancyType_Count: break;
    default: break;
  }
  // cout << occ.getNumOccupiedEntries() << endl;
  //const ml::TriMeshf occMesh(occ, grid2world, true, ml::vec4f(1, 1, 1, .05));
  const ml::TriMeshf occMesh = greedyMesher(occ, grid2world);
  return occMesh;
}

//! Get vector of colors
std::vector<ml::vec4f> getColors(const core::LabeledGrid& labeledGrid,
                                 vis::ColorIndex* pColorIndex) {
  int iUnknown = labeledGrid.getLabelIndex().indexOf(core::CATEGORY_UNKNOWN);

  std::vector<ml::vec4f> colors;
  if (pColorIndex != nullptr) {
    for (int i = 0; i < labeledGrid.getNumLabels(); ++i) {
      const string& label = labeledGrid.getLabelIndex()[i];
      int iColor;
      if (i == iUnknown) {
        iColor = pColorIndex->addLabel(label, ml::RGBColor(ml::vec4f(0.5f, 0.5f, 0.5f, 0.5f)));
      } else {
        iColor = pColorIndex->addLabel(label);
      }
      colors.push_back(pColorIndex->color(iColor));
    }
  } else {
    for (int i = 0; i < labeledGrid.getNumLabels(); ++i) {
      if (i == iUnknown) {
        colors.push_back(ml::vec4f(0.5f, 0.5f, 0.5f, 0.5f));
      } else {
        colors.push_back(ml::ColorUtils::colorById<ml::vec4f>(i));
      }
    }
  }
  return colors;
}

//! Create labeled scan voxels trimesh
ml::TriMeshf labeledScanVoxelsToTriMesh(const core::LabeledGrid& labeledGrid,
                                        vis::ColorIndex* pColorIndex /* = nullptr */) {
  const ml::mat4f grid2world = labeledGrid.worldToGrid().getInverse();
  std::vector<ml::vec4f> colors = getColors(labeledGrid, pColorIndex);

  ml::vec2f texCoord;
  size_t n = labeledGrid.getGrid().size();
  std::vector<ml::TriMeshf::Vertex> verts;
  verts.reserve(n*24);
  std::vector<ml::vec3ui> indices;
  indices.reserve(n * 12);
  for (auto& it : labeledGrid.getGrid()) {
    ml::vec3f p = it.first;
    ml::vec3f pMin = p - 0.5f;
    ml::vec3f pMax = p + 0.5f;

    ml::BoundingBox3f bb;
    bb.include(pMin);
    bb.include(pMax);

    ml::TriMeshf triMesh(bb);
    triMesh.transform(grid2world);
    
    const ml::vec3i pos(static_cast<int>(p.x), static_cast<int>(p.y), static_cast<int>(p.z));
    int i = labeledGrid.getLabelIndex(pos);
    if (i < 0) { continue; }
    const ml::vec4f& color = colors.at(i);
   
    unsigned int vertIdxBase = static_cast<unsigned int>(verts.size());
    for (const auto& vert : triMesh.getVertices()) {
      verts.emplace_back(vert.position, vert.normal, color, texCoord);
    }
    for (const auto& index : triMesh.getIndices()) {
      indices.push_back(vertIdxBase + index);
    }
  }
  
  const ml::TriMeshf mesh(verts, indices, true, false, true);
  return mesh;
}

//! Create colored grid from labeled scan voxels
ml::Grid3<ml::RGBColor> labeledScanVoxelsToColoredGrid(const Scan& scene,
                                                       const core::LabeledGrid& labeledGrid,
                                                       vis::ColorIndex* pColorIndex /* = nullptr */) {
  const OccupancyGrid& occupancyGrid = scene.getOccupancyGrid();
  const ml::vec3ul dims = occupancyGrid.getDimensions();
  std::vector<ml::vec4f> colorsf = getColors(labeledGrid, pColorIndex);
  std::vector<ml::RGBColor> colors;
  for (int i = 0; i < labeledGrid.getNumLabels(); ++i) {
    colors.push_back(ml::RGBColor(colorsf[i]));
  }
  ml::Grid3<ml::RGBColor> volumeTextureData(dims);
  volumeTextureData.fill([&] (size_t x, size_t y, size_t z) -> ml::RGBColor {
    const ml::vec3i pos(static_cast<int>(x), static_cast<int>(y), static_cast<int>(z));
    int i = labeledGrid.getLabelIndex(pos);
    if (i >= 0) {
      return colors.at(i);
    } else {
      return ml::RGBColor::Black;
    }
  });
  return volumeTextureData;
}

ml::TriMeshf visualizeSkelCollisions(const Skeleton& skel, const Scan& scene, int numPts /*= 100*/) {
  const auto& skelPts = skel.getPointsInOBBs(numPts);
  const ml::BinaryGrid3 free = scene.getOccupancyGrid().free();
  const ml::mat4f worldToVoxel = scene.getOccupancyGrid().worldToGrid();
  vec<ml::TriMeshf> skelCloud(numPts);
  auto protoBox = ml::Shapesf::box(.05f, .05f, 0.05f);
  protoBox.computeNormals();
  protoBox.setHasColors(true);
  for (int iPt = 0; iPt < numPts; ++iPt) {
    const auto& skelPt = skelPts[iPt];
    auto& box = skelCloud[iPt];
    box = protoBox;
    box.transform(ml::mat4f::translation(skelPt));
    const ml::vec3ui skelPtVoxelCoords = worldToVoxel * skelPt;
    if (free.isValidCoordinate(skelPtVoxelCoords) && free.isVoxelSet(skelPtVoxelCoords)) {
      box.setColor(ml::vec4f(0, 0, 1, 1));
    } else {
      box.setColor(ml::vec4f(1, 0, 0, 1));
    }
  }
  auto skelPtMesh = ml::meshutil::createUnifiedMesh(skelCloud);
  return skelPtMesh;
}

ml::TriMeshf createPointCloudMesh(const vec<std::pair<geo::Vec3f, float>>& scoredPoints, 
                                  const ml::ColorGradient& colorGradient,
                                  const ml::vec4f& defaultColor,
                                  const float boxSize /* = 0.02 */) {
  const auto box = ml::Shapesf::box(boxSize, defaultColor);
  const size_t numPts = scoredPoints.size();
  std::vector<ml::vec3f> pts(numPts);  std::vector<ml::vec4f> colors(numPts);
  for (size_t iP = 0; iP < numPts; iP++) {
    const auto& pair = scoredPoints.at(iP);
    const geo::Vec3f& p = pair.first;
    pts[iP] = geo::to<ml::vec3f>(p);
    const auto cv = colorGradient.value(pair.second);
    colors[iP] = ml::vec4f(cv.r / 255.f, cv.g / 255.f, cv.b / 255.f, 1.f);
  }
  return ml::meshutil::createPointCloudTemplate(box, pts, colors);
};

ml::TriMeshf skelToMeshWithJointWeights(const Skeleton& skel,
                                        const std::array<double, Skeleton::kNumJointGroups>& jointWeights,
                                        const ml::ColorGradient& colorGradient) {
  vec<ml::vec4f> jointColors(Skeleton::kNumJoints);
  for (int i = 0; i < Skeleton::kNumJoints; ++i) {
    int iJointGroup = (i == Skeleton::JointType_Head)
      ? Skeleton::kNumJointGroups-1
      : Skeleton::kJointToJointGroup[i];
    const auto rgb = colorGradient.value(jointWeights[iJointGroup]);
    jointColors[i].x = rgb.r / 255.f;
    jointColors[i].y = rgb.g / 255.f;
    jointColors[i].z = rgb.b / 255.f;
    jointColors[i].w = 1.f;
  }
  const ml::TriMeshf mesh = vis::toTriMesh(skel, true, true, false, false, false, false, &jointColors);
  return mesh;
}

} // namespace vizutil
