#include "common.h"  // NOLINT

#include "core/ScanDatabase.h"

#include <mLibCore.h>

#include "core/LabeledGrid.h"
#include "core/synth/ObjectLabeler.h"
#include "core/OccupancyGrid.h"
#include "core/RecordingDatabase.h"
#include "io/io.h"
#include "io/binvox.h"
#include "io/json.h"
#include "segmentation/MeshSegment.h"
#include "util/Params.h"

namespace sg {
namespace core {

bool ScanDatabase::loadFromJSON(const string& file) {
  if (!io::fileExists(file)) { return false; }

  rapidjson::Document d;
  if (!io::parseRapidJSONDocument(file, &d)) {
    return false;
  }

  m_id = d["id"].GetString();
  const auto& scansArr = d["scenes"];
  const unsigned numScenes = scansArr.Size();

  // Scene loading helper function
  auto loadScene = [&] (unsigned iScene) {
    const auto& sIn = scansArr[iScene];
    const string id = sIn["id"].GetString();

    auto& s         = m_scans[id];
    s.id            = id;
    s.meshFile      = sIn["mesh"].GetString();
    if (sIn.HasMember("mesh-debug")) {
      s.meshFileDebug = sIn["mesh-debug"].GetString();
    } else {
      s.meshFileDebug = s.meshFile;
    }

    // Save annotations file if available
    if (sIn.HasMember("annotations")) {
      s.annotationFile = sIn["annotations"].GetString();
    }

    // Scene category
    if (sIn.HasMember("sceneType")) {
      s.sceneType     = sIn["sceneType"].GetString();
    }

    // Set hasGround bit
    if (sIn.HasMember("hasGround")) {
      s.m_hasGround = sIn["hasGround"].GetBool();
    }

  };

  // Load scans
  for (unsigned i = 0; i < numScenes; ++i) {
    loadScene(i);
  }

  cout << "[ScanDatabase] " << numScenes << " scans loaded from " << file << endl;

  return true;
}

ScanDatabase::ScanDatabase(const util::Params& params, const string& scansJsonFile) : m_params(params) {
  const string cacheDir = params.get<string>("dataCacheDir");
  io::ensurePathToFileExists(cacheDir);
  const string scansFile = params.get<string>("dataDir") + scansJsonFile;
  if (!loadFromJSON(scansFile)) {
    cerr << "[ScanDatabase] Error loading from " << scansFile << endl;
  }
}

vec<string> ScanDatabase::scanIds() const {
  vec<string> ids;
  for (const auto& it : m_scans) { ids.push_back(it.first); }
  return ids;
}

void ScanDatabase::loadScan(const string& id) const {
  Scan& s = m_scans.at(id);
  if (s.isLoaded()) { return; }  // Scene already loaded
  if (m_id == "scans-cornell") {
    s.loadFromCornellPLY(m_params, s.meshFile);
  } else {
    s.load(m_params);
  }
}

void ScanDatabase::loadAllScansWithRecordings(const RecordingDatabase& recordingDatabase) {
  cout << "Loading all scans with recordings...";
  const vec<string> recIds = recordingDatabase.getRecordingIds();
  for (const string& recId : recIds) {
    const string scanId = recordingIdtoBaseScanId(recId);
    loadScan(scanId);
  }
  cout << " Done." << endl;
}

bool ScanDatabase::scanExists(const string& id) const {
  return m_scans.count(id) > 0;
}

const Scan& ScanDatabase::getScan(const string& id) const {
  if (id[0] == '.') {  // manual load override
    Scan& s = m_scans[id];
    s.meshFile = id;
    s.meshFileDebug = id;
    s.id = id;
    s.load(m_params);
  }
  if (!m_scans.at(id).isLoaded()) {
    loadScan(id);
  }
  return m_scans.find(id)->second;
}

string ScanDatabase::recordingIdtoBaseScanId(const string& recId) {
  return ml::util::split(recId, '_')[0];
}

void ScanDatabase::saveScansAsPLYs(const vec<string>& scanIds) const {
  for (const string& sId : scanIds) {
    loadScan(sId);
    const Scan& s = getScan(sId);
    const string
      workDir = m_params.get<string>("workDir"),
      meshfile = (s.m_isDebugMesh) ? s.meshFileDebug : s.meshFile,
      filename = workDir + "scanPLYs/" + meshfile;
    io::ensurePathToFileExists(filename);
    ml::MeshIOf::saveToPLY(filename, s.mesh.getMeshData());
    SG_LOG_INFO << "\tSaved Scan " << sId << " to " << filename;
  }
}

void ScanDatabase::saveScanBinvoxes(const vec<string>& scanIds) const {
  for (const string& sId : scanIds) {
    loadScan(sId);
    const Scan& s = getScan(sId);
    const OccupancyGrid& ocGrid = s.getOccupancyGrid();
    const string
      workDir = m_params.get<string>("workDir"),
      basename = workDir + "scanBinvoxes/" + io::removeExtension(s.meshFile);
    io::ensurePathToFileExists(basename);
    const ml::mat4f grid2world = ocGrid.gridToWorld();
    const ml::vec3f normTrans = grid2world.getTranslation();
    const ml::vec3ul& voxelDims = ocGrid.getDimensions();
    const float maxVoxelDim = std::max(voxelDims.x, std::max(voxelDims.y, voxelDims.z));
    const float normScale = grid2world.matrix[0] * maxVoxelDim;
    cout << ocGrid.occupied().getNumOccupiedEntries() << endl;
    io::write_binvox(ocGrid.occupied(), basename + ".surface.binvox", &normTrans, &normScale);
    io::write_binvox(ocGrid.unknownOrOccupied(), basename + ".solid.binvox", &normTrans, &normScale);
    SG_LOG_INFO << "\tSaved Scan binvoxes " << sId << " to " << basename;
  }
}

void ScanDatabase::saveScanLabeledVoxels(const vec<string>& scanIds, const synth::ObjectLabeler& labeler,
                                         const synth::LabelOpts& labelOpts, const string& runId) const {
  const string baseDir = m_params.get<string>("workDir") + "voxels/" + runId + "/";
  io::ensurePathToFileExists(baseDir);
  for (const string& sId : scanIds) {
    loadScan(sId);
    const Scan& s = getScan(sId);
    const OccupancyGrid& ocGrid = s.getOccupancyGrid();
    const string basename = baseDir + io::removeExtension(s.meshFile);
    const ml::mat4f grid2world = ocGrid.gridToWorld();
    const ml::vec3ul& voxelDims = ocGrid.getDimensions();
    LabeledGrid labeledGrid(ocGrid.voxelSize(), ocGrid.getDimensions(), ocGrid.worldToGrid());
    labeler.labelVoxels(s, nullptr, nullptr, labelOpts, &labeledGrid);
    labeledGrid.save(basename + ".vox");
    SG_LOG_INFO << "\tSaved Scan labeled voxels " << sId << " to " << basename;
  }
}

void ScanDatabase::getAnnotationLabels(util::Index<string>* pIndex) const {
  for (const auto& it : m_scans) {
    const string& sId = it.first;
    const Scan& s = getScan(sId);
    for (const auto& segGroup : s.segmentGroups) {
      pIndex->add(segGroup.label);
    }
  }
}

void ScanDatabase::saveSegmentFeatures(const string& dir) const {
  SegmentFeatureGeneratorWolf2016 gen;
  const string header = "scanId,segId,label," + util::join(gen.fieldNames(), ",");

  io::ensureDirExists(dir);
//  io::ensureDirExists(dir + "/segs/");
  ofstream ofs(dir + "/ALL.seg-feats.csv");
  ofs << header << endl;
  for (const auto& it : m_scans) {
    const string& sId = it.first;
    const Scan& s = getScan(sId);
    if (!s.segments) { continue; }

//    ofstream ofs(dir + "/segs/" + sId + ".seg-feats.csv");
//    ofs << header << endl;
    const auto writeFeats = [&] (const segmentation::VecSegPtr& segs) {
      for (const segmentation::SegPtr seg : segs) {
        ofs << sId << "," << seg->id << "," << seg->label;
//        ofsAll << sId << "," << seg->id << "," << seg->label;
        for (float f : gen.generate(*seg)) {
          ofs << "," << f;
//          ofsAll << "," << f;
        }
        ofs << endl;
//        ofsAll << endl;
      }
    };
    writeFeats(*s.segments);
    writeFeats(*s.rejectedSegments);
  }
}

}  // namespace core
}  // namespace sg
