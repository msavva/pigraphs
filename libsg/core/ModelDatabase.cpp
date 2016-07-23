#include "common.h"  // NOLINT

#include "core/ModelDatabase.h"

#include <boost/algorithm/string.hpp>
#include <mLibCore.h>

#include "core/Model.h"
#include "geo/OBB.h"
#include "io/binvox.h"
#include "io/io.h"
#include "io/json.h"
#include "math/math.h"
#include "segmentation/Segmentator.h"
#include "util/util.h"
#include "util/Params.h"
#include "util/timer.h"

namespace sg {
namespace core {

using std::stof;

//! Helper to parse vec3f from model metadata tsv entries
inline void str2vec3f(const string& s, vec<string>& tmp, ml::vec3f& out) {  // NOLINT
  util::tokenize(s, ",", &tmp);
  if (tmp.size() > 1) {
    out[0] = stof(tmp[0]);
    out[1] = stof(tmp[1]);
    out[2] = stof(tmp[2]);
  }
}

ModelDatabase::ModelDatabase(const util::Params& params) : m_params(params) {
  util::Timer t("initModelDatabase");
  const string cacheDir = params.get<string>("dataCacheDir"),
               dataDir = params.get<string>("dataDir"),
               modelsDir = dataDir + params.get<string>("modelsDir"),
               modelsMetadataFile = dataDir + params.get<string>("modelsMetadataFile");

  // If there is a model white list, load it in
  set<string> modelWhitelist;
  if (params.exists("modelWhiteListFile")) {
    const auto vecModels = io::getLines(params.get<string>("dataDir") + params.get<string>("modelWhiteListFile"));
    modelWhitelist.insert(vecModels.begin(), vecModels.end());
  }
  m_modelWhitelist = modelWhitelist;

  io::ensureDirExists(cacheDir);

  if (!io::dirExists(modelsDir)) {
    cerr << "[ModelDatabase] modelsDir does not exist: " << modelsDir << endl;
  }

  // Parse model metadata
  if (io::fileExists(modelsMetadataFile)) {
    int numModels = 0;
    rapidjson::Document d;
    io::parseRapidJSONDocument(modelsMetadataFile, &d);
    const int numModelEntries = d["response"]["numFound"].GetInt();
    const auto& modelEntries = d["response"]["docs"];
    vec<string> tmp;
    for (int iModel = 0; iModel < numModelEntries; ++iModel) {
      const auto& modelEntry = modelEntries[iModel];
      const string fullId = modelEntry["fullId"].GetString();

      const bool inWhiteList = modelWhitelist.count(fullId) > 0;

      // Id
      const string id = fullId.substr(4);  // Drops wss. prefix
      Model& m = m_models[id];
      m.isWhitelisted = inWhiteList;
      m.id = id;

      // Categories
      if (modelEntry.HasMember("category")) {
        const auto& cats = modelEntry["category"];
        const unsigned numCats = cats.Size();
        for (unsigned iCat = 0; iCat < numCats; ++iCat) {
          m.categories.push_back(cats[iCat].GetString());
        }
      }

      // Scale
      if (modelEntry.HasMember("unit")) {
        const double unit = modelEntry["unit"].GetDouble();
        m.scaleUnit = static_cast<float>(unit);
      } else {
        m.scaleUnit = 0.0254f;  // guess inches are the units and convert to meters
      }

      // Up and front
      if (modelEntry.HasMember("up")) {  // assume also has front
        const string up = modelEntry["up"].GetString();
        str2vec3f(up, tmp, m.up);
        const string front = modelEntry["front"].GetString();
        str2vec3f(front, tmp, m.front);
      }

      // Aligned dims
      if (modelEntry.HasMember("aligned.dims")) {  // assume also has front
        const string dims = modelEntry["aligned.dims"].GetString();
        str2vec3f(dims, tmp, m.dims);
        m.dims *= 0.01f;  // dims are given in cm so convert to m here
      }

      numModels++;
    }
    cout << "[ModelDatabase] loaded metadata for " << numModels << " models in ";  // timer prints after this
  } else {
    cerr << "[ModelDatabase] modelsMetadataFile does not exist at: " << modelsMetadataFile << endl;
  }

  // Create map from category to all model ids in that category NOTE: this ignores all uncategorized models
  for (const auto& modelPair : m_models) {
    const Model& m = modelPair.second;
    for (const string& cat : m.categories) {
      m_categoryToModelIds[cat].push_back(m.id);
      if (m.isWhitelisted) {
        m_categoryToModelIdsWhitelist[cat].push_back(m.id);
      }
    }
  }

  // Further restricted list of models per category (for debugging)
  map<string, vec<string>> restrictedCategoryToModelIds;
  if (params.exists("ModelDatabase.restrictModels")) {
    vec<string> restricted = params.getVec("ModelDatabase.restrictModels");
    for (const string& r : restricted) {
      vec<string> fields = util::tokenize(r, ":");
      restrictedCategoryToModelIds[fields[0]].push_back(fields[1]);
    }
  }
  // Make sure m_categoryToModelsIdWhiteList has same restricted set of models per category
  for (const auto& p : restrictedCategoryToModelIds) {
    SG_LOG_INFO << "Restricting " << p.first << " to " << util::join(p.second);
    m_categoryToModelIdsWhitelist[p.first] = p.second;
  }
}

void ModelDatabase::ensureAllModelsCached() const {
  for (const auto& pair : m_models) {
    const string& id = pair.first;
    ensureModelIsCached(id);
    cout << id << endl;
  }
}

void ModelDatabase::computeModelStats(const string& filename) {
  vec<string> ids;
  util::mapToKeyVec(m_models, ids);
  const int numModels = static_cast<int>(ids.size());
  vec<string> lines(numModels);
  const string sep = ",";

//#pragma omp parallel for
  for (int i = 0; i < numModels; ++i) {
    const string& id = ids[i];
    Model& model = m_models.at(id);

//    if (model.categories.empty()) {
//      SG_LOG_INFO << "Skipping uncategorized model with id=" << isetId;
//      continue;
//    }

    bool ok = ensureVoxelization(id);
    if (!ok) {
      SG_LOG_WARN << "Skipping model " << id;
      continue;
    }

    const ml::vec3f voxelDimsWorld = model.voxelDims();
    float
      voxelVolWorld = voxelDimsWorld[0] * voxelDimsWorld[1] * voxelDimsWorld[2],
      solidVol = voxelVolWorld * model.occupiedSolidVoxels(),
      surfaceVol = voxelVolWorld * model.occupiedSurfaceVoxels(),
      supportSurf = model.numSupportingVoxels(0) * voxelDimsWorld[0] * voxelDimsWorld[1];
    if (solidVol == 0.f) { solidVol = surfaceVol; }  // TODO(MS)

    lines[i] = id + sep + to_string(solidVol) + sep + to_string(surfaceVol) + sep + to_string(supportSurf);
    SG_LOG_INFO << lines[i];

    m_models.erase(id);
  }

  // write lines
  ofstream ofs(filename);
  const string header = "id,solidVol,surfaceVol,supportSurfArea";
  ofs << header << endl;
  for (int i = 0; i < numModels; ++i) {
    ofs << lines[i] << endl;
  }
  ofs.close();
  SG_LOG_INFO << "Saved stats to " << filename;
}

void ModelDatabase::ensureModelIsCached(const string& id) const {
  const string cachedModelBase = baseFilename(id),
               cachedOBJFile = cachedModelBase + ".obj",
               cachedMTLFile = cachedModelBase + ".mtl";
  if (io::fileExists(cachedOBJFile) && io::fileExists(cachedMTLFile)) {
    return;
  }

  io::ensurePathToFileExists(cachedOBJFile);
  io::ensurePathToFileExists(cachedMTLFile);
  cout << "Copying to local cache: " << id << "...";

  const string remoteModelBase = m_params.get<string>("dataDir") + m_params.get<string>("modelsDir") + id,
               remoteOBJFile = remoteModelBase + ".obj.gz",
               remoteMTLFile = remoteModelBase + ".mtl";

  util::decompressedGZipFile(remoteOBJFile, cachedOBJFile);
  io::copyFile(remoteMTLFile, cachedMTLFile, true);

  cout << "done." << endl;
}

void ModelDatabase::ensureModelIsLoaded(const string& id) {
  if (!m_models[id].isLoaded) {
    ensureModelIsCached(id);
    if (m_models[id].id == "") {
      SG_LOG_WARN << "[ModelDatabase] Loaded model with no metadata, id=" << id;
      m_models[id].id = id;
      m_models[id].scaleUnit = m_params.get<float>("ModelDatabase.defaultModelScaleUnit");
    }
    const string base = baseFilename(id);
    const float maxEdgeLength = m_params.get<float>("ModelDatabase.subDivideMaxEdgeLength");
    m_models[id].load(base + ".obj", base + ".mtl", maxEdgeLength);
  }
}

bool ModelDatabase::modelExists(const string& id) const {
  return m_models.count(id) > 0;
}

boost::optional<const Model&> ModelDatabase::getModel(const string& id) {
  ensureModelIsLoaded(id);
  if (m_models.count(id) > 0) {
    return m_models.at(id);
  } else {
    SG_LOG_WARN << "Model not found id=" << id;
    return boost::none;    
  }
}

boost::optional<const Model&> ModelDatabase::getFirstModelWithCategory(const string& cat, bool restrictToWhitelist) {
  const auto& catToModelIds = restrictToWhitelist? m_categoryToModelIdsWhitelist : m_categoryToModelIds; 
  if (catToModelIds.count(cat) > 0) {
    return getModel(catToModelIds.at(cat)[0]);
  } else {
    SG_LOG_WARN << "No models with category=" + cat;
    return boost::none;
  }
}

boost::optional<const Model&> ModelDatabase::getRandomModelWithCategory(const string& cat, bool restrictToWhitelist) {
  const auto& catToModelIds = restrictToWhitelist? m_categoryToModelIdsWhitelist : m_categoryToModelIds; 
  if (catToModelIds.count(cat) > 0) {
    int index = math::DEF_RAND.uniform_int(0, static_cast<int>(catToModelIds.at(cat).size()));
    return getModel(catToModelIds.at(cat)[index]);
  } else {
    SG_LOG_WARN << "No models with category=" + cat;
    return boost::none;
  }
}

boost::optional<const Model&> ModelDatabase::getModelMatchingOBB(const string& cat, const geo::OBB& obb, bool restrictToWhitelist) {
  const auto& catToModelIds = restrictToWhitelist? m_categoryToModelIdsWhitelist : m_categoryToModelIds; 
  if (catToModelIds.count(cat) == 0) {
    SG_LOG_WARN << "No models with category=" + cat;
    return boost::none;
  }
  const float
    xLength           = obb.axesLengths().x(),
    yLength           = obb.axesLengths().y(),
    xyArea            = xLength * yLength,
    minLength         = std::min(xLength, yLength),
    maxLength         = std::max(xLength, yLength),
    minOverMax        = minLength / maxLength,
    minOverMaxThresh  = 0.1f;

  // Candidate matched model encapsulator
  struct ModelCandidate { string id; float minOverMaxXY, area; };

  // Get list of candidate models
  vec<ModelCandidate> candidates;
  for (const string& mId : catToModelIds.at(cat)) {
    const Model& mStub = m_models.at(mId);
    const ml::vec3f& dims = mStub.dims;  // NOTE: This is y-up
    const float
      mMinOverMax = std::min(dims.x, dims.z) / std::max(dims.x, dims.z),
      mAreaXY     = dims.x * dims.z;
    candidates.push_back({mId, mMinOverMax, mAreaXY});
  }

  // Order candidates by aspect ratio delta
  std::sort(candidates.begin(), candidates.end(), [&] (const ModelCandidate& l, const ModelCandidate& r) {
    return fabsf(l.minOverMaxXY - minOverMax) < fabsf(r.minOverMaxXY - minOverMax);
  });

  // Retain only candidates with aspect ratio within threshold of target's (or only first one)
  const float bestMinOverMax = candidates[0].minOverMaxXY;
  for (size_t i = 0; i < candidates.size(); i++) {
    const float deltaAR = fabsf(candidates[i].minOverMaxXY - bestMinOverMax);
    if (deltaAR > minOverMaxThresh) {
      candidates.resize(i);
      break;
    }
  }

  // Now order remaining candidates by area delta from target
  std::sort(candidates.begin(), candidates.end(), [&] (const ModelCandidate& l, const ModelCandidate& r) {
    return fabsf(l.area - xyArea) < fabsf(r.area - xyArea);
  });

  const ModelCandidate& top = *candidates.begin();

  //SG_LOG_INFO << "t" << "\t" << minOverMax << "\t" << xyArea;
  //for (int i = 0; i < candidates.size(); ++i) {
  //  SG_LOG_INFO << i << "\t" << candidates[i].minOverMaxXY << "\t" << candidates[i].area;
  //}

  return getModel(top.id);
}

size_t ModelDatabase::modelCountForCategory(const string& cat, bool restrictToWhitelist) const {
  const auto& catToModelIds = restrictToWhitelist? m_categoryToModelIdsWhitelist : m_categoryToModelIds; 
  if (catToModelIds.count(cat) > 0) {
    return catToModelIds.at(cat).size();
  } else {
    return 0;
  }
}

string ModelDatabase::baseFilename(const string& id) const {
  return m_params.get<string>("dataCacheDir") + "models/" + id;
}

void ModelDatabase::clearSegmentation() {
  for (auto& it : m_models) {
    it.second.clearSegmentation();
  }
}

bool ModelDatabase::ensureSegmentation(const string& id, const segmentation::SegmentationParams& params, bool load) {
  ensureModelIsLoaded(id);
  MLIB_ASSERT_STR(m_models.count(id) != 0, "Model not found id=" + id);
  auto& model = m_models[id];

  if (!model.hasSegmentsLoaded()) {
    if (load) {
      // Use segmentation file
      const string segmentationFilename = m_params.get<string>("dataDir") + m_params.get<string>("modelSegmentsDir") + id + "/" + id + "-surfaces.json";
      const string format = "json";
      SG_LOG_INFO << "Load segmentation for model from " << segmentationFilename;
      return model.loadSegmentation(segmentationFilename, params.constrainZup, format);
    } else {
      // Segment using params
      SG_LOG_INFO << "Segmenting model " << id;
      model.segment(params);
      return true;
    }
  } else {
    return false;
  }
}

//! Returns an [3 x n] matrix with the coordinates of all occupied voxels in model space
geo::Matrix3Xf voxelsInModelCoords(const Model& model) {
  assert(model.hasSolidVoxelsLoaded());
  const auto& V = model.solidVoxels;
  const size_t
    dimX = V.getDimX(),
    dimY = V.getDimY(),
    dimZ = V.getDimZ();
  // Get voxel center coords
  vec<float> voxelCenters;
  voxelCenters.reserve(dimX * dimY * dimZ / 20);  // heuristically set proportion of occupied voxels to 5%
  for (size_t iX = 0; iX < dimX; ++iX) {
    for (size_t iZ = 0; iZ < dimZ; ++iZ) {
      for (size_t iY = 0; iY < dimY; ++iY) {
        if (V.isVoxelSet(iX, iY, iZ)) {
          voxelCenters.emplace_back(static_cast<float>(iX));
          voxelCenters.emplace_back(static_cast<float>(iY));
          voxelCenters.emplace_back(static_cast<float>(iZ));
        }
      }
    }
  }
  // Map to matrix and transform to model space
  geo::Matrix3Xf M = geo::Matrix3Xf::Map(voxelCenters.data(), 3, voxelCenters.size() / 3);
  M = geo::from(model.voxelToModel) * M.colwise().homogeneous();
  // Permute columns so that we can later pick first k columns if we want to subsample
  Eigen::PermutationMatrix<Eigen::Dynamic> P;
  P.setIdentity(M.cols());
  math::DEF_RAND.shuffle(P.indices().data(), P.indices().data() + P.indices().size());
  M = M * P;
  return M;
}

// Helper to find voxel coord BBox around occupied voxels in V and set into bbox.
// Returns number of voxels found (0 if none and bbox is invalid)
int occupiedVoxelsBBox(const ml::BinaryGrid3& V, geo::BBox* bbox) {
  const size_t
    dimX = V.getDimX(),
    dimY = V.getDimY(),
    dimZ = V.getDimZ();
  auto& m = bbox->min();
  auto& M = bbox->max();
  m.setConstant(math::constants::POSINFf);
  M.setConstant(-math::constants::POSINFf);
  int numVoxels = 0;
  for (size_t iZ = 0; iZ < dimZ; ++iZ) {
    for (size_t iX = 0; iX < dimX; ++iX) {
      for (size_t iY = 0; iY < dimY; ++iY) {
        if (V.isVoxelSet(iX, iY, iZ)) {
          if (m.x() > iX) { m.x() = iX; }
          if (m.y() > iY) { m.y() = iY; }
          if (m.z() > iZ) { m.z() = iZ; }
          if (M.x() < iX) { M.x() = iX; }
          if (M.y() < iY) { M.y() = iY; }
          if (M.z() < iZ) { M.z() = iZ; }
          numVoxels++;
        }
      }
    }
  }
  return numVoxels;
}

// Unsets all voxels in V falling outside given bbox
int clearVoxelsOutsideBBox(const geo::BBox& bbox, ml::BinaryGrid3* V) {
  const size_t
    dimX = V->getDimX(),
    dimY = V->getDimY(),
    dimZ = V->getDimZ();
  auto& m = bbox.min();
  auto& M = bbox.max();
  int numVoxelsCleared = 0;
  // this is a dumb inefficient loop over all coords
  for (size_t iZ = 0; iZ < dimZ; ++iZ) {
    for (size_t iX = 0; iX < dimX; ++iX) {
      for (size_t iY = 0; iY < dimY; ++iY) {
        if (iX < m.x() || iY < m.y() || iZ < m.z() || iX > M.x() || iY > M.y() || iZ > M.z()) {  // outside bbox
          if (V->isVoxelSet(iX, iY, iZ)) {  // and set voxel
            V->clearVoxel(iX, iY, iZ);
            numVoxelsCleared++;
          }
        }
      }
    }
  }
  return numVoxelsCleared;
}

bool ModelDatabase::ensureVoxelization(const string& id) {
  ensureModelIsLoaded(id);  // necessary to get model bbox information populated for code below
  MLIB_ASSERT_STR(m_models.count(id) != 0, "Model not found id=" + id);
  auto& model = m_models[id];

  if (!model.hasSurfaceVoxelsLoaded()) {
    // No voxelization yet, so load it
    const string
      dataDir           = m_params.get<string>("dataDir"),
      voxFilename       = dataDir + m_params.get<string>("modelVoxelsDir") + id + ".binvox",
      solidVoxFilename  = dataDir + m_params.get<string>("modelVoxelsSolidDir") + id + ".binvox";
    int ok = 0;
    ok = io::read_binvox(voxFilename, &model.surfaceVoxels, nullptr);
    if (!io::fileExists(solidVoxFilename)) {
      SG_LOG_WARN << "Using non-solid voxelization for " << id;
      ok += io::read_binvox(voxFilename, &model.solidVoxels, nullptr);
    } else {
      ok += io::read_binvox(solidVoxFilename, &model.solidVoxels, nullptr);
      const size_t numSolidVoxels = model.occupiedSolidVoxels();
      if (numSolidVoxels == 0) {  // empty solid voxelization
        SG_LOG_WARN << "Empty solid voxelization for " << id << " . Using default voxelization instead";
        model.solidVoxels = model.surfaceVoxels;
      } else {  // valid solid voxelization
        geo::BBox surfOccBBox;
        const int numVoxelsOutside = occupiedVoxelsBBox(model.surfaceVoxels, &surfOccBBox);
        if (numVoxelsOutside) {
          clearVoxelsOutsideBBox(surfOccBBox, &model.solidVoxels);
        }
      }
    }

    if (ok) {
      // Compute model to voxel transform (due to model transform)
      // Assume model is just translated and scaled, and compute transform  based on the maximum bb dim
      // Assume equal number of voxels in all dimensions (unit cube voxelization)
      const float
        maxExtent = model.bbox.getMaxExtent(),
        scale     = model.surfaceVoxels.getDimX() / maxExtent;
      const ml::vec3f translate = -model.bbox.getMin() * scale;
      model.modelToVoxel.setScale(scale);  // set to scale matrix
      model.modelToVoxel.setTranslationVector(translate);  // set just translation vector
      model.voxelToModel = model.modelToVoxel.getInverse();
      model.voxelCenters = voxelsInModelCoords(model);
    }
    return ok > 0;
  } else {
    return false;
  }
}

void ModelDatabase::addParentCategories(const CategoryHierarchy& h) {
  for (auto& it : m_models) {
    Model& m = it.second;
    set<string> cats;
    for (auto& c : m.categories) {
      vec<string> v = h.getAncestors(c);
      cats.insert(v.begin(), v.end());
    }
    for (const string& c : cats) {
      if (find(m.categories.begin(), m.categories.end(), c) == m.categories.end()){
        m.categories.push_back(c);
        m_categoryToModelIds[c].push_back(m.id);
      }
    }
  }

}

}  // namespace core
}  // namespace sg
