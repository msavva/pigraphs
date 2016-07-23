#include "common.h"  // NOLINT

#include "core/RecordingDatabase.h"

#include <mLibCore.h>

//#include "core/features.h"
#include "core/Recording.h"
#include "core/ScanDatabase.h"
#include "interaction/Interaction.h"
#include "interaction/InteractionDatabase.h"
#include "io/io.h"
#include "util/Params.h"

namespace sg {
namespace core {

string recId2scanId(const string& recId) {
  return ml::util::split(recId, '_')[0];
}

RecordingDatabase::RecordingDatabase(const ScanDatabase& scans, const util::Params& params)
  : m_scans(scans)
  , m_params(params) { 
  m_recordingIds = getRecordingIds(m_params);
}

vec<string> RecordingDatabase::getLoadedRecordingIds() const {
  vec<string> ids;
  for (const auto& it : m_recordings) { ids.push_back(it.first); }
  return ids;
}

Recording& RecordingDatabase::loadRecording(const string& recId) const {
  const string ext = m_params.get<string>("recFileExtension");
  const string recDir = m_params.get<string>("dataDir") + m_params.get<string>("recDir");
  Recording& rec = (*const_cast<map<string, Recording>*>(&m_recordings))[recId];
  if (ext == ".rec") {
    SG_LOG_ERROR << "Deprecated .rec format is unreliable. Please convert to .json format";
    rec.loadFromArchive(recDir + recId + ".rec", nullptr);
  } else if (ext == ".json") {
    rec.loadFromJSON(recDir + recId + ".json");
  } else {
    SG_LOG_ERROR << "Failure loading recording due to unrecognized extension: " << ext;
  }
  // Update map of scan id to recordings
  vec<Recording*>& recs = (*const_cast<map<string, vec<Recording*>>*>(&m_recordingsByScan))[rec.baseScanId()];
  recs.push_back(&rec);
  return rec;
}

void RecordingDatabase::loadAllRecordings(bool computeActiveSegments) {
  const float maxDistToSegment = m_params.get<float>("Interaction.maxDistToSegment");
  const float maxDistGaze = m_params.get<float>("Interaction.maxDistGaze");
  const vec<string> recIds = getRecordingIds();
  for (const string& recId : recIds) {
    // Load recording and interactions
    Recording& rec = loadRecording(recId);
    rec.loadInteractions(m_params);
    const Scan* pScan = &m_scans.getScan(rec.baseScanId());
    for (interaction::Interaction* inter : rec.interactions) {
      inter->scan = pScan;
    }
    if (computeActiveSegments) {
      rec.computeActiveSegments(m_params, m_scans.getScan(rec.baseScanId()),
                                maxDistToSegment, maxDistGaze, /* forceRecompute=*/ true);
    }
  }
}

bool RecordingDatabase::addRecording(const Recording& rec, const string& id) {
  if (m_recordings.count(id) > 0) {
    SG_LOG_WARN << "[RecordingDatabase] Tried to add already added Recording with id " << id;
    return false;
  }
  m_recordings[id] = rec;
  m_allRecordingIds.push_back(id);
  m_recordingIds.push_back(id);
  return true;
}

const Recording& RecordingDatabase::getRecording(const string& id, bool withInteractions,
                                                 bool withActiveSegments) const {
  if (m_recordings.count(id) == 0) {
    loadRecording(id);
  }
  MLIB_ASSERT_STR(m_recordings.count(id) != 0, "Recording not found id=" + id);
  Recording& rec = (*const_cast<map<string, Recording>*>(&m_recordings)).at(id);
  const string scanId = recId2scanId(id);
  if (withInteractions && !rec.hasInteractions) {
    const string recDir = m_params.get<string>("dataDir") + m_params.get<string>("recDir");
    rec.loadInteractions(m_params);
    if (m_scans.scanExists(scanId)) {
      const Scan* pScan = &m_scans.getScan(rec.baseScanId());
      for (interaction::Interaction* inter : rec.interactions) {
        inter->scan = pScan;
      }
    }
  }
  // Ensure active segments are available
  if (rec.hasInteractions && withActiveSegments && !m_recordings.at(id).hasActiveSegments) {
    if (m_scans.scanExists(scanId)) {
      rec.computeActiveSegments(m_params, m_scans.getScan(scanId),
                                m_params.get<float>("Interaction.maxDistToSegment"),
                                m_params.get<float>("Interaction.maxDistGaze"), true);
    }
  }
  return rec;
}

vec<Recording*> RecordingDatabase::getLoadedRecordingsForScan(const string& scanId) const {
  if (m_recordingsByScan.count(scanId) > 0) {
    return m_recordingsByScan.at(scanId);
  }
  return vec<Recording*>();
}

vec<string> RecordingDatabase::getRecordingIdsForScan(const string& scanId) const {
  if (m_recordingIdsByScan.count(scanId) > 0) {
    return m_recordingIdsByScan.at(scanId);
  }
  return vec<string>();
}

vec<string> RecordingDatabase::listRecordingIds(const string& dirname, const string& ext) {
  vec<string> recFiles;
  const ml::Directory dir(dirname);
  recFiles = dir.getFilesWithSuffix(ext);
  recFiles = ml::util::map(recFiles, [&] (const string& s) { return io::basename(s); });
  return recFiles;
}

vec<string> RecordingDatabase::filterRecordingIds(const vec<string>& recIds, bool skipRecordingsWithoutScenes) const {
  if (skipRecordingsWithoutScenes) {
    vec<string> okayRecIds;
    for (const string& recId : recIds) {
      string scanId = recId2scanId(recId);
      bool keep = m_scans.scanExists(scanId);
      if (keep) {
        okayRecIds.push_back(recId);
      } else {
        SG_LOG_ERROR << "[RecordingDatabase] Cannot find scan " << scanId
          << " for recording " << recId;
      }
    }
    return okayRecIds;
  } else {
    return recIds;
  }
}

vec<string> RecordingDatabase::getRecordingIds(const util::Params& params) {
  vec<string> recFiles;
  const bool skipRecordingsWithoutScenes = params.get<bool>("Dataset.skipRecordingsWithoutScenes");
  const bool debugDatabase = params.get<bool>("Dataset.debugDatabase");
  const int maxDebugRecordings = params.get<int>("Dataset.maxDebugRecordings");
  const string recDir = params.get<string>(".dataDir") + params.get<string>(".recDir");
  const string ext = params.get<string>(".recFileExtension");
  // All recordings in the recDir
  m_allRecordingIds = listRecordingIds(recDir, ext);
  for (const string& recId : m_allRecordingIds) {
    m_recordingIdsByScan[recId2scanId(recId)].push_back(recId);
  }
  if (debugDatabase) {  // Just the recording defined by user
    if (params.exists("rec")) { recFiles.push_back(params.get<string>("rec")); }
    int numRemaining = maxDebugRecordings - static_cast<int>(recFiles.size());
    if (numRemaining > 0) {
      vec<string> allRecFiles = filterRecordingIds(m_allRecordingIds, skipRecordingsWithoutScenes);
      numRemaining = std::min(numRemaining, static_cast<int>(allRecFiles.size()));
      for (int i = 0; i < numRemaining; i++) {
        recFiles.push_back(allRecFiles.at(i));
      }
    }
  } else { 
    recFiles = filterRecordingIds(m_allRecordingIds, skipRecordingsWithoutScenes);
  }
  return recFiles;
}

ml::mat4f readAlignmentMatrixFromMeshLabALN(const string& alnFilename) {
  string tag = ml::util::replace(alnFilename, '/', '\\');
  tag = ml::util::split(ml::util::dropExtension(tag), '\\').back();

  const auto lines = ml::util::getFileLines(alnFilename);
  int iLine;
  for (iLine = 0; iLine < lines.size(); iLine++) {
    if (ml::util::rtrim(lines[iLine]).compare(0, tag.size(), tag) == 0) { break; }
  }
  iLine = iLine + 2;  // Skip tag line and "#" line
  const auto r0 = ml::util::split(lines[iLine++], ' ');
  const auto r1 = ml::util::split(lines[iLine++], ' ');
  const auto r2 = ml::util::split(lines[iLine++], ' ');
  const auto r3 = ml::util::split(lines[iLine++], ' ');
  using std::stof;
  return ml::mat4f(
           stof(r0[0]), stof(r0[1]), stof(r0[2]), stof(r0[3]),
           stof(r1[0]), stof(r1[1]), stof(r1[2]), stof(r1[3]),
           stof(r2[0]), stof(r2[1]), stof(r2[2]), stof(r2[3]),
           stof(r3[0]), stof(r3[1]), stof(r3[2]), stof(r3[3]));
}

void RecordingDatabase::batchConvertRecToJSON(const string& dirPath, bool skipExisting) {
  const ml::Directory dir(dirPath);
  vec<string> recFiles = dir.getFilesWithSuffix(".rec");
  for (string recFile : recFiles) {
    const string jsonFile = dir.getPath() + "/" + io::basename(recFile) + ".json";
    if (skipExisting && ml::util::fileExists(jsonFile)) {
      cout << "Skipping rec-to-json conversion of " << recFile
           << " since " << jsonFile << " already exists." << endl;
      continue;
    }

    vec<string> filenameParts = ml::util::split(recFile, '_');
    string aln;
    for (size_t i = 0; i < filenameParts.size() - 1; i++) {
      if (i != 0) { aln = aln + "_"; }
      aln = aln + filenameParts[i];
    }
    aln = dir.getPath() + "/" + aln + ".aln";
    if (ml::util::fileExists(aln)) {
      Recording rec;
      if (!rec.loadFromArchive(dir.getPath() + "/" + recFile)) {
        throw MLIB_EXCEPTION("recording not found");
      }
      rec.camera = readAlignmentMatrixFromMeshLabALN(aln);
      ofstream jsonOut(jsonFile);
      rec.toJSON(jsonOut, true);
      jsonOut.close();
      cout << "Converted " << recFile << " to " << jsonFile << endl;
    } else {
      cerr << "Warning: skipped rec-to-json conversion of " << recFile
           << " since no matching .aln file." << endl;
    }
  }
}

void RecordingDatabase::loadAllRecordingsWithInteractions(interaction::InteractionDatabase& interactionDb) const {
  for (const string recId : getRecordingIds()) {
    const Recording& rec = getRecording(recId, true, true);
    for (interaction::Interaction* pInter : rec.interactions) {
      interactionDb.registerAndPopulateISets(pInter);
    }
  }
}

}  // namespace core
}  // namespace sg
