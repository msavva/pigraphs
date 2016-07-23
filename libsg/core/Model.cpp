#include "common.h"  // NOLINT

#include "core/Model.h"

#include <assimp/Importer.hpp>
#include <assimp/mesh.h>
#include <assimp/scene.h>
#include <mLibCore.h>

#include "geo/OBB.h"
#include "io/json.h"
#include "math/math.h"
#include "segmentation/MeshSegment.h"
#include "segmentation/segmentation.h"
#include "segmentation/Segmentator.h"

namespace sg {
namespace core {

Model::Model()
  : id("")
  , categories()
  , scaleUnit(-1)
  , up(0, 0, 1)
  , front(0, -1, 0)
  , flattenedMesh()
  , allMeshes()
  , segments(new VecSegPtr())
  , surfaceVoxels()
  , solidVoxels()
  , modelToVoxel()
  , voxelToModel()
  , isLoaded(false)
  , isSubdivided(false)
  , m_obb(nullptr) { }

Model::~Model() {
  if (m_obb != nullptr) {
    delete m_obb;
  }
}

void aiMesh2mlMesh(const aiMesh* mesh, ml::TriMeshf* trimesh) {
  auto& V = trimesh->getVertices();
  V.resize(mesh->mNumVertices);
  if (mesh->HasPositions()) {
    for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
      const auto& pIn = mesh->mVertices[i];
      auto& pOut = V[i].position;
      pOut.x = pIn.x;  pOut.y = pIn.y;  pOut.z = pIn.z;
    }
  }

  if (mesh->HasTextureCoords(0)) {
    for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
      V[i].texCoord.x = mesh->mTextureCoords[0][i].x;
      V[i].texCoord.y = mesh->mTextureCoords[0][i].y;
    }
  }

  if (mesh->HasNormals()) {
    for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
      const auto& nIn = mesh->mNormals[i];
      auto& nOut = V[i].normal;
      nOut.x = nIn.x;  nOut.y = nIn.y;  nOut.z = nIn.z;
    }
  }

  if (mesh->HasFaces()) {
    auto& I = trimesh->getIndices();
    I.resize(mesh->mNumFaces);
    for (unsigned int i = 0; i < mesh->mNumFaces; ++i) {
      if (mesh->mFaces[i].mNumIndices > 3) { SG_LOG_ERROR << "Untriangulated face idx=" << i; }
      const auto& nIn = mesh->mFaces[i].mIndices;
      I[i][0] = nIn[0];
      I[i][1] = nIn[1];
      I[i][2] = nIn[2];
    }
  }
};

void Model::load(const string& file) {
  Assimp::Importer importer;
  const aiScene* scene = importer.ReadFile(file, NULL);  // TODO(MS): Define post-process flags
  if (!scene) {
    SG_LOG_ERROR << "Unable to load mesh model: " << file << ". ERROR: "<< importer.GetErrorString();
  }

  vec<ml::TriMeshf> triMeshes(scene->mNumMeshes);
  allMeshes.resize(scene->mNumMeshes);
  for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {
    aiMesh2mlMesh(scene->mMeshes[i], &triMeshes[i]);
    allMeshes[i].mesh = triMeshes[i];
    // TODO(MS): Handle material loading
  }

  flattenedMesh = ml::meshutil::createUnifiedMesh(triMeshes);
  flattenedMesh.computeNormals();
  const ml::vec4f gray(0.78f, 0.78f, 0.78f, 1.f);
  flattenedMesh.setColor(gray);
  const float scale = (scaleUnit < 0) ? 0.0254f : scaleUnit;  // guess inches if no scale metadata  // TODO(MS): Use default scale value from params
  modelTransform = ml::mat4f::scale(scale) *
                   ml::mat4f::translation(-flattenedMesh.computeBoundingBox().getCenter());
  flattenedMesh.transform(modelTransform);
  bbox = flattenedMesh.computeBoundingBox();
  isLoaded = true;
}

void Model::load(const string& objFile, const string& mtlFile, float maxEdgeLength /*= 0.f*/) {
  load(objFile);

  if (maxEdgeLength > 0.f) {
    auto meshData = flattenedMesh.getMeshData();
    const float edgeLengthThreshold = 0.1f;
    while (true) {
      maxEdgeLength = meshData.subdivideFacesLoop(edgeLengthThreshold);
      if (maxEdgeLength <= edgeLengthThreshold) { break; }
    }
    flattenedMesh = ml::TriMeshf(meshData);
    isSubdivided = true;
  }

  //const ml::vec4f gray(0.78f, 0.78f, 0.78f, 1.f);
  //const ml::MeshDataf geometry = ml::MeshIO<float>::loadFromFile(objFile);
  //auto triMeshPairs = geometry.splitByMaterial(mtlFile);

  //vec<ml::TriMeshf> triMeshes(triMeshPairs.size());

  //allMeshes.resize(triMeshPairs.size());
  //for (UINT meshIndex = 0; meshIndex < triMeshPairs.size(); meshIndex++) {
  //  allMeshes[meshIndex].mesh = triMeshPairs[meshIndex].first;
  //  allMeshes[meshIndex].mesh.computeNormals();
  //  allMeshes[meshIndex].mesh.setColor(gray);
  //  allMeshes[meshIndex].material = triMeshPairs[meshIndex].second;

  //  triMeshes[meshIndex] = triMeshPairs[meshIndex].first;
  //}

  //flattenedMesh = ml::meshutil::createUnifiedMesh(triMeshes);
  //flattenedMesh.computeNormals();
  //flattenedMesh.setColor(gray);
  //// Translate centroid to origin and rescale to meters
  //flattenedMesh.transform(ml::mat4f::scale(scaleUnit) *
  //                        ml::mat4f::translation(-flattenedMesh.getBoundingBox().getCenter()));

  //if (maxEdgeLength > 0.f) {
  //  auto meshData = flattenedMesh.getMeshData();
  //  const float edgeLengthThreshold = 0.1f;
  //  while (true) {
  //    float maxEdgeLength = meshData.subdivideFacesLoop(edgeLengthThreshold);
  //    if (maxEdgeLength <= edgeLengthThreshold) { break; }
  //  }
  //  flattenedMesh = ml::TriMeshf(meshData);
  //  isSubdivided = true;
  //}

  //bbox = flattenedMesh.getBoundingBox();
  //isLoaded = true;
}

void Model::clearSegmentation() {
  segments->clear();
}

void Model::segment(const segmentation::SegmentationParams& params) {
  segmentation::SegmentatorFelzenswalb segtor(params);
  const segmentation::SegmentFilter segFilter = [&] (const segmentation::MeshSegment & s) {
    return true;
  };

  VecSegPtr rejected;
  const vec<int> vertexSegIndices = segtor.segment(flattenedMesh);
  createSegments(flattenedMesh, vertexSegIndices, segFilter, params.constrainZup,
                                   segments.get(), &rejected);
  storeSegmentIndices(flattenedMesh, *segments, rejected);
}

bool loadModelSegsJson(const string& filename, bool useFlattenedMesh, bool constrainZup, Model* model);

bool Model::loadSegmentation(const string& filename, bool constrainZup, const string& format) {
  if (isSubdivided) {  // Segmentations are not valid for subdivided mesh
    SG_LOG_ERROR << "Cannot load segmentation for subdivided mesh from " << filename;
    return false;
  }
  bool loaded = loadModelSegsJson(filename, /*useFlattenedMesh =*/ true, constrainZup, this);
  if (loaded) {
    VecSegPtr rejected;
    storeSegmentIndices(flattenedMesh, *segments, rejected);
    return true;
  } else {
    SG_LOG_ERROR << "Error load segmentation from " << filename;
    return false;
  }
}

size_t Model::numSupportingVoxels(size_t zSupport /* = 0 */) const {
  const auto& V = surfaceVoxels;
  const size_t dimX = V.getDimX();
  const size_t dimY = V.getDimY();
  size_t iZ = zSupport;
  size_t numVoxels = 0;
  while (numVoxels == 0) {
    for (size_t iX = 0; iX < dimX; ++iX) {
      for (size_t iY = 0; iY < dimY; ++iY) {
        if (V.isVoxelSet(iX, iY, iZ)) {
          numVoxels++;
        }
      }
    }
    if (numVoxels < 5) { iZ++; }  // TODO(MS): This is a heuristic hack to avoid "noise" voxels at the floor
  }
  return numVoxels;
}

const geo::OBB& Model::getOBB() const {
  if (m_obb != nullptr) {
    return *m_obb;
  }
  assert(hasSurfaceVoxelsLoaded());
  m_obb = new geo::OBB();
  pointSetToOBB(voxelCenters, m_obb);
  return *m_obb;
}

bool loadModelSegsJson(const string& filename, bool useFlattenedMesh, bool constrainZup, Model* model) {
  if (!ml::util::fileExists(filename)) { return false; }

  // Parse JSON document
  rapidjson::Document d;
  if (!io::parseRapidJSONDocument(filename, &d)) {
    cerr << "Parse error reading " << filename << endl
         << "Error code " << d.GetParseError() << " at " << d.GetErrorOffset() << endl;
    return false;
  }

  // Parse out elements

  // Parse modelId
  //const string& modelId = d["modelId"].GetString();

  // Figure out where the vertices of the different meshes start in the flattened mesh
  // Assumes flattening just concatenates
  size_t nMeshes = model->allMeshes.size();
  vec<size_t> meshTriBase(nMeshes);
  if (useFlattenedMesh && nMeshes > 0) {
    meshTriBase[0] = 0;
    //SG_LOG_INFO << "Model " << model->id;
    //for (size_t i = 0; i < nMeshes; i++) {
    //  SG_LOG_INFO << "Mesh " << i << " " << model->allMeshes.at(i).mesh.getIndices().size();
    //}
    for (size_t i = 1; i < nMeshes; i++) {
      meshTriBase[i] = meshTriBase[i - 1] + model->allMeshes.at(i - 1).mesh.getIndices().size();
    }
  }

  // Parse out tri indices
  //int defaultSegIndex = std::numeric_limits<int>::lowest();
  const auto& segArr = d["surface"];
  const unsigned numSegs = segArr.Size();
  model->segments->clear();
  model->segments->resize(numSegs);
  vec<int> segTriIndices;
  for (unsigned i = 0; i < numSegs; i++) {
    // Parse segment group
    const auto& sIn = segArr[i];
    int segIndex = sIn["surfaceIndex"].GetInt();
    int meshIndex = sIn["meshIndex"].GetInt();
    io::toIntVector(sIn["triIndex"], &segTriIndices);

    // Makes sure our vector is large enough
    if (segIndex >= model->segments->size()) {
      model->segments->resize(segIndex + 1);
    }
    const ml::TriMeshf& mesh = (useFlattenedMesh) ? model->flattenedMesh : model->allMeshes.at(meshIndex).mesh;
    const std::vector<ml::vec3ui>& meshIndices = mesh.getIndices();
    size_t triBase = (useFlattenedMesh) ? meshTriBase[meshIndex] : 0;
    size_t numTris = segTriIndices.size();
    vec<size_t> vertIndices;
    vec<size_t> triIndices;
    vertIndices.reserve(numTris * 3);
    triIndices.reserve(numTris);
    for (size_t j = 0; j < numTris; j++) {
      size_t triIndex = triBase + segTriIndices[j];
      triIndices.push_back(triIndex);
      const ml::vec3ui triVertIndices = meshIndices.at(triIndex);
      vertIndices.push_back(triVertIndices[0]);
      vertIndices.push_back(triVertIndices[1]);
      vertIndices.push_back(triVertIndices[2]);
    }
    segmentation::SegPtr segPtr(new segmentation::MeshSegment(
      mesh, vertIndices, segIndex, constrainZup, 0, geo::Transform::Identity(), triIndices));
    (*model->segments)[segIndex] = segPtr;
  }
  return true;
}

}  // namespace core
}  // namespace sg
