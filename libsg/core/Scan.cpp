#include "common.h"  // NOLINT

#include "core/Scan.h"

#include <functional>
#include <limits>

#include <boost/filesystem.hpp>
BOOST_CLASS_VERSION(ml::TriMeshf, 1)
#include <core-mesh/triMesh.h>
#include <ext-boost/serialization.h>
//#include <mLibOpenMesh.h>

#include "core/OccupancyGrid.h"
#include "core/Skeleton.h"
#include "io/io.h"
#include "geo/OBB.h"
#include "math/math.h"
#include "mesh/mesh_nn.h"
#include "segmentation/MeshSegment.h"
#include "segmentation/segmentation.h"
#include "segmentation/Segmentator.h"
#include "util/Params.h"
#include "vdb.h"

namespace sg {
namespace core {

const string Scan::kCachedExt = ".cached";

const string Scan::kGridFileExt = ".grid.gz";

//! Remove vertices close to ground level (XY plane at minimum Z) and add a new subdivided plane
void addGroundPlane(ml::MeshDataf* meshData) {
  cout << "[Scene] Adding ground plane...";
  // Remove vertices at floor level
  meshData->removeVerticesInFrontOfPlane(-ml::Planef::xyPlane(), -0.1f);

  // Create ground rectangle
  const ml::BoundingBox3f bb = meshData->computeBoundingBox();
  const ml::TriMeshf groundRect = ml::Shapesf::rectangleZ(ml::vec2f(bb.getMinX(), bb.getMinY()),
                                                          ml::vec2f(bb.getMaxX(), bb.getMaxY()),
                                                          0.f,
                                                          ml::vec4f(0.5f, 0.5f, 0.5f, 1));
  ml::MeshDataf groundMeshData = groundRect.getMeshData();

  // Subdivide ground until it meets min edge length threshold for segmentation purposes
  while (true) {
    const float edgeLengthThreshold = 0.05f;
    float maxEdgeLength = groundMeshData.subdivideFacesLoop(edgeLengthThreshold);
    if (maxEdgeLength <= edgeLengthThreshold) { break; }
  }

  // Merge ground into mesh
  meshData->merge(groundMeshData);
  cout << "done." << endl;
}

void Scan::finalizeLoad(const util::Params& params) {
  if (m_meshHelper != nullptr) {
    delete m_meshHelper;
    m_meshHelper = nullptr;
  }
  m_meshHelper = new mesh::MeshHelper(mesh);

  // Get segmentation params
  const segmentation::SegmentationParams segParams(params);

  // Clear current segment groups
  segmentGroups.clear();
  segmentGroups.params = params.extract("Segmentation");
  params.get("Segmentation.segGroupAbsorbThreshold", &segmentGroups.absorbThreshold);
  segmentGroups.constrainZup = segParams.constrainZup;

  // Helper for updating segmentation
  const std::function<void()> segmentatorFun = [this, &segParams] () { updateSegmentation(segParams); };

  // Load annotated segment groups
  if (params.get<bool>("Segmentation.loadAnnotations") && !annotationFile.empty()) {
    const string annFile = params.get<string>("dataDir") + params.get<string>("scanDir") + annotationFile;
    const bool loadSegments = params.get<bool>("SegmentGroup.loadSegments"),
               recomputeOBBs = params.get<bool>("SegmentGroup.recomputeOBBs");

    const bool loadedSegGroups = loadSegmentGroups(annFile, segmentGroups.absorbThreshold,
                                                   segParams.constrainZup, loadSegments, recomputeOBBs, &segmentatorFun);
    if (!loadedSegGroups) {
      cerr << "Failed loading segment groups for scan " << id << ". Resegmenting..." << endl;
      segmentatorFun();
    }
  } else {
    segmentatorFun();
  }

  m_isLoaded = true;
}

void Scan::load(const util::Params& params) {
  const bool loadDebugScenes = params.get<bool>("Dataset.loadDebugScenes");
  const string baseDir          = (id[0] == '.') ? "" : params.get<string>("dataDir") + params.get<string>("scanDir"),
               filename         = loadDebugScenes ? meshFileDebug : meshFile,
               scanId           = io::basename(filename),
               cachedScanFile   = params.get<string>("dataCacheDir") + params.get<string>("scanDir")
                                + scanId + ".cached";
  cout << "[Scan:" + scanId + "...";

  id = scanId;
  m_isDebugMesh = loadDebugScenes;

  if (loadCachedBinary(cachedScanFile, params)) { return; }  // Bail out if can load cached

  //mesh = ml::OpenMeshLoader::load(filename); ml::MeshDataf meshData; mesh.getMeshData(meshData);
  loadedMeshFileFullPath = baseDir + filename;
  ml::MeshDataf meshData; ml::MeshIOf::loadFromFile(loadedMeshFileFullPath, meshData);

  const bool addGround = params.get<bool>("Segmentation.addGround");
  if (loadedMeshFileFullPath[0] != '.' && addGround && !m_hasGround) {
    addGroundPlane(&meshData);
    m_hasGround = true;
  }

  mesh = ml::TriMeshf(meshData);
  bbox = mesh.computeBoundingBox();

  finalizeLoad(params);

  // Save cached binary if it does not already exist
  if (!io::fileExists(cachedScanFile)) {
    saveCachedBinary(cachedScanFile);
  }
}

ml::TriMeshf loadCornellPLY(const string& plyfile) {
  ifstream ifs(plyfile);
  if (!ifs) { SG_LOG_ERROR << "Unable to open: " << plyfile; return ml::TriMeshf(); }
  string line;
  getline(ifs, line);
  if (line != "ply") { SG_LOG_ERROR << "Invalid first line: " << line; return ml::TriMeshf(); }
  getline(ifs, line);  // format ascii 1.0
  getline(ifs, line);  // comment author: Hema
  getline(ifs, line);  // element vertex N
  const size_t numVertices = stoul(util::tokenize(line, " ")[2]);
  while (line != "end_header") {  // skip rest of header since it's fixed
    getline(ifs, line);
  }
  // float x, float y, float z, uchar r, uchar g, uchar b, uchar cameraIdx, float distance, uchar segment, uchar label
  ml::TriMeshf mesh;
  mesh.m_vertices.resize(numVertices);
  vdb_frame();
  for (ml::TriMeshf::Vertex& v : mesh.m_vertices) {
    getline(ifs, line);
    const vec<string> tokens = util::tokenize(line, " ");
    v.position.x = stof(tokens[0]);
    v.position.y = stof(tokens[1]);
    v.position.z = stof(tokens[2]);
    v.normal.x = 0.f;
    v.normal.y = 0.f;
    v.normal.z = 1.f;
    v.color.r = stof(tokens[3]) / 255.f;
    v.color.g = stof(tokens[4]) / 255.f;
    v.color.b = stof(tokens[5]) / 255.f;
    v.color.a = 1.f;
    // int cameraIdx = stoi(tokens[6]);
    // float distance = stof(tokens[7]);
    v.texCoord[0] = stof(tokens[8]);  // segment index
    v.texCoord[1] = stof(tokens[9]);  // label index;
    vdb_color(v.color.r, v.color.g, v.color.b);
    vdb_label_i(static_cast<int>(v.texCoord[0]));
    vdb_point(v.position.x, v.position.y, v.position.z);
  }
  // Create dummy tris since mLib acceleration structs require non-empty triangle TriMeshes
  mesh.m_indices.resize(1000);
  for (unsigned int i = 0; i < 1000; i = i + 3) {
    mesh.m_indices[i] = {i, i+1, i+2};
  }
  return mesh;
}

void Scan::loadFromCornellPLY(const util::Params& params, const string& plyfile) {
  const string baseDir          = params.get<string>("dataDir") + params.get<string>("cornellScanDir"),
               filename         = plyfile,
               scanId           = io::basename(filename),
               cachedScanFile   = params.get<string>("dataCacheDir") + params.get<string>("cornellScanDir")
                                + scanId + ".cached";

  if (loadCachedBinary(cachedScanFile, params)) { return; }  // Bail out if can load cached

  //mesh = ml::OpenMeshLoader::load(filename); ml::MeshDataf meshData; mesh.getMeshData(meshData);
  loadedMeshFileFullPath = baseDir + filename;

  mesh = loadCornellPLY(loadedMeshFileFullPath);
  bbox = mesh.computeBoundingBox();

  finalizeLoad(params);

  // Save cached binary if it does not already exist
  if (!io::fileExists(cachedScanFile)) {
    saveCachedBinary(cachedScanFile);
  }
}

namespace ba = boost::archive;

bool Scan::loadCachedBinary(const string& file, const util::Params& params) {
  if (!io::fileExists(file)) { return false; }
  ifstream ifs(file, std::ios::binary);
  {
    ba::binary_iarchive ar(ifs);
    ar >> id;
    ar >> loadedMeshFileFullPath;
    ar >> mesh;
    ar >> bbox;
  }
  ifs.close();

  finalizeLoad(params);

  return true;
}

void Scan::saveCachedBinary(const string& file) {
  io::ensurePathToFileExists(file);
  ofstream ofs(file, std::ios::binary);
  {
    ba::binary_oarchive ar(ofs);
    ar << id;
    ar << loadedMeshFileFullPath;
    ar << mesh;
    ar << bbox;
  }
  ofs.close();
  cout << "Created cached scene: " << file << endl;
}

void Scan::updateSegmentation(const segmentation::SegmentationParams& params) {
  segmentation::SegmentatorFelzenswalb segtor(params);
  const segmentation::SegmentFilter segFilter = [&] (const segmentation::MeshSegment & s) {
    if (s.obb() == nullptr) { return false; }

    auto extents = s.obb()->axesLengths();
    std::sort(&extents[0], &extents[0] + 3);

    bool extrusion = (extents[2] / extents[0] >= params.segAspectCutoff &&
                      extents[2] / extents[1] >= params.segAspectCutoff);

    return s.diagonalLength() >= params.minSegDiag && s.elements.size() >= params.minSegVerts && !extrusion;
  };

  vec<int> vertexSegIndices;
  SG_LOG_INFO << id;
  if (util::startsWith(id, "cornell")) {  // segmentation provided with point cloud vertices
    SG_LOG_INFO << "Loading segmentation from Cornell PLY data";
    const auto& V = mesh.getVertices();
    vertexSegIndices.resize(V.size());
    for (int iV = 0; iV < V.size(); ++iV) {
      vertexSegIndices[iV] = static_cast<int>(V[iV].texCoord[0]);
    }
  } else {
    vertexSegIndices = segtor.segment(mesh);
  }
  createSegments(mesh, vertexSegIndices, segFilter, params.constrainZup,
                                   segments.get(), rejectedSegments.get());
  storeSegmentIndices(mesh, *segments, *rejectedSegments);

  // Hacky code to make sure our params are in sync
  segmentGroups.params.set("kThresh", params.kthresh);
  segmentGroups.params.set("segMinVerts", params.minSegVerts);
  segmentGroups.params.set("segMinDiag", params.minSegDiag);
  segmentGroups.params.set("colorWeight", params.colorWeight);
  segmentGroups.params.set("segAspectCutoff", params.segAspectCutoff);
  segmentGroups.params.set("constrainZup", params.constrainZup);
  segmentGroups.constrainZup = params.constrainZup;
  segmentGroups.updateSegmentation(*segments, *rejectedSegments, true);
}

void Scan::loadSegmentation(const string& filename, 
                             bool constrainZup) {
  vec<segmentation::SegIndices> vertexSegIndices;
  // TODO: Pass in mesh so we can verify loaded segments are for scene
  segmentation::loadSegIndicesCsv(filename, &vertexSegIndices);
  // Take the first mesh as that of the scene
  createSegments(mesh, vertexSegIndices[0], constrainZup,
                                   segments.get(), rejectedSegments.get());
  storeSegmentIndices(mesh, *segments, *rejectedSegments);
  // Hacky code to make sure our params are in sync
  segmentGroups.params.set("constrainZup", constrainZup);
  segmentGroups.constrainZup = constrainZup;
  segmentGroups.updateSegmentation(*segments, *rejectedSegments, true);
}

void Scan::getActiveSegments(const Skeleton& skel, bool ignoreInferredJoints,
                             unsigned int kNearestSegsPerJoint, float maxContactDist,
                             float maxDistGaze, float maxSegmentSizeRatio,
                             Skeleton::SegmentsByJointPlusGaze* activeSegs,
                             bool doAccumulate /* = false */) const {
  if (!segments) { return; }  // We don't have a segmentation yet

  // Retrieve active segments using helper
  const mesh::MeshHelper& helper = getMeshHelper();
  const float maxSegmentSize = (maxSegmentSizeRatio > 0)? 
    maxSegmentSizeRatio * bbox.getExtent().length() : 0.0f;
  helper.getActiveSegments(*segments, skel, ignoreInferredJoints, kNearestSegsPerJoint, 
                           maxContactDist, maxDistGaze, maxSegmentSize,
                           activeSegs, doAccumulate);
}

void Scan::getActiveSegments(const SkelRange& skelRange, bool ignoreInferredJoints, unsigned int kNearestSegsPerJoint,
                             float maxContactDist, float maxDistGaze, float maxSegmentSizeRatio,
                             Skeleton::SegmentsByJointPlusGaze* segsByJoint) const {
  for (auto& segs : *segsByJoint) {
    segs.clear();
  }
  for (const Skeleton& skel : skelRange) {
    getActiveSegments(skel, ignoreInferredJoints, kNearestSegsPerJoint, maxContactDist, maxDistGaze,
                      maxSegmentSizeRatio, segsByJoint, true);
  }
}

bool Scan::intersect(const ml::Cameraf& cam, const ml::vec2f& imagePlaneCoords, mesh::IntersectionRecord* rec) const {
  const ml::mat4f projToCam = cam.getPerspective().getInverse();
  const ml::mat4f camToWorld = cam.getCamera().getInverse();
  const ml::mat4f trans =  camToWorld * projToCam;

  ml::vec4f p(imagePlaneCoords.x, imagePlaneCoords.y, 0.01f, 1.0f);
  p.x = 2.0f * p.x - 1.0f;
  p.y = 1.0f - 2.0f * p.y;
  p = trans * p;
  p /= p.w;
  ml::Rayf r(cam.getEye(), (ml::vec3f(p.x, p.y, p.z) - cam.getEye()).getNormalized());

  return getMeshHelper().intersect(r, rec);
}

Scan::Scan()
  : id("")
  , meshFile("")
  , meshFileDebug("")
  , annotationFile("")
  , sceneType("")
  , mesh()
  , bbox()
  , segments(new VecSegPtr())
  , rejectedSegments(new VecSegPtr())
  , m_isLoaded(false)
  , m_isDebugMesh(false)
  , m_hasGround(false)
  , m_meshHelper(nullptr)
  , m_vStackGrid(nullptr)
  , m_occGrid(nullptr) { }

void Scan::saveSegmentGroups(const string& filename) {
  // Just pull out the segmentation part of the params
  segmentation::SegmentGroupsRecord rec(id, &segmentGroups);
  // Populate the segment indices
  const auto& V = mesh.getVertices();
  rec.segIndices.resize(V.size());
  for (size_t i = 0; i < V.size(); i++) {
    rec.segIndices[i] = static_cast<int>(V[i].texCoord[0]);
  }
  // Save record to file
  rec.save(filename);
}

bool Scan::loadSegmentGroups(const string& filename, 
                              float absorbThreshold, bool constrainZUp,
                              bool loadSegments, bool recomputeOBBs,
                              const std::function<void()>* segmentatorFun) {
  // Load record from file
  segmentation::SegmentGroupsRecord rec(id, &segmentGroups);
  bool loaded = rec.load(filename);
  // Exit if there was error loading
  if (!loaded) return false;

  if (rec.scanId != id) {
    SG_LOG_WARN << "Segment group is for scan " << rec.scanId << ", not the current scan " << id;
    if (rec.scanId + "d" == id || rec.scanId == id + "d") {
      // Debug vs non-debug version of the same scene
      // Segment groups are probably okay, but we will need to use the current segmentation
      if (loadSegments) {
        SG_LOG_WARN << "Cannot use stored segmentation, will recompute segment groups";
        loadSegments = false;
      }
    } else {
      return false;
    }
  }

  // Load segmentation or recompute segment groups
  if (loadSegments && rec.segIndices.size() > 0) {
    SG_LOG_INFO << "segments from " << io::basename(filename) << "...";
    createSegments(mesh, rec.segIndices, segmentGroups.constrainZup,
                                     segments.get(), rejectedSegments.get());
    storeSegmentIndices(mesh, *segments, *rejectedSegments);
    segmentGroups.params = rec.params;
    // Populate some params
    if (rec.params.exists("constrainZup")) {
      segmentGroups.constrainZup = rec.params.get<bool>("constrainZup");
    }
    if (rec.params.exists("segGroupAbsorbThreshold")) {
      segmentGroups.absorbThreshold = rec.params.get<float>("segGroupAbsorbThreshold");
    }
    segmentGroups.updateSegmentation(*segments, *rejectedSegments, false);
  } else {
    if (segmentatorFun != nullptr) {
      SG_LOG_INFO << "Resegmenting scan for " << filename;
      (*segmentatorFun)();
    } else {
      SG_LOG_INFO << "Using current segmentation to recompute segment groups based on OBBs from " << filename;
    }

    // Hacky code to make sure our params are in sync
    segmentGroups.params.set("constrainZup", constrainZUp);
    segmentGroups.params.set("segGroupAbsorbThreshold", absorbThreshold);
    segmentGroups.absorbThreshold = absorbThreshold;
    segmentGroups.constrainZup = constrainZUp;

    // Recompute segments that belong to each segment group using current segmentation
    segmentGroups.updateSegmentation(*segments, *rejectedSegments, true);
  }
  if (recomputeOBBs) {
    segmentGroups.computeOBBs();
  }
  return true;
}

Scan::~Scan() {
  if (m_meshHelper != nullptr) { delete m_meshHelper; }
  if (m_occGrid != nullptr) { delete m_occGrid; }
  if (m_vStackGrid != nullptr) { delete m_vStackGrid; }
}

const OccupancyGrid& Scan::getOccupancyGrid() const {
  if (m_occGrid == nullptr) {
    m_occGrid = new OccupancyGrid();
    const string gridFile = io::parentPath(loadedMeshFileFullPath) + "/"
                          + io::replaceExtension(meshFile, "") + kGridFileExt;  // This is an ugly hack!
    if (io::fileExists(gridFile)) {
      m_occGrid->readFromGridFile(gridFile);
    } else {
      SG_LOG_WARN << "Cannot load occupancy grid from " << gridFile; 
    }
  }
  return *m_occGrid;
}

const mesh::projection::VertexStackGrid& Scan::getVertexStackGrid() const {
  if (m_vStackGrid != nullptr) {
    return *m_vStackGrid;
  }
  m_vStackGrid = new mesh::projection::VertexStackGrid;
  projectMeshXY(mesh, 0.1f, m_vStackGrid);  // TODO(MS): Paramerize grid cell size
  return *m_vStackGrid;
}

//! Return the canonical scanId (without the "d")
string Scan::getCanonicalId() const {
  if (m_isDebugMesh) {
    char lastChar = id.back();
    if (lastChar == 'd') {
      return id.substr(0, id.length() - 1);
    }
  }
  return id;
}

}  // namespace core
}  // namespace sg

