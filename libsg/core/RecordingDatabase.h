#pragma once

#include "libsg.h"  // NOLINT

namespace sg {
namespace core {

class RecordingDatabase {
 public:
  RecordingDatabase(const ScanDatabase& scans, const util::Params& params);

  //! Batch loads all recordings in the given path and optionally precomputes
  //! active segments for all the observed interactions
  void loadAllRecordings(bool computeActiveSegments);

  //! Add Recording to this RecordingDatabase under given id
  //! Return whether added, or recording with given id already existed
  bool addRecording(const Recording& rec, const string& id);

  //! Returns Recording with given id (format: scanId_deviceId_timestamp).
  //! Optional parameters ensure presence of interactions and active segments in Recording
  const Recording& getRecording(const string& id, bool withInteractions, bool withActiveSegments) const;

  //! Returns currently loaded recordings for given scanId
  vec<Recording*> getLoadedRecordingsForScan(const string& scanId) const;

  // Returns all recording ids for given scanId
  vec<string> getRecordingIdsForScan(const string& scanId) const;

  //! Return ids of all Recordings currently loaded in this RecordingDatabase
  vec<string> getLoadedRecordingIds() const;

  //! Return ids of all Recordings to load for this RecordingDatabase
  vec<string> getRecordingIds() const { return m_recordingIds; }

  //! Batch converts all .rec recordings in dir to equivalent .json files
  static void batchConvertRecToJSON(const string& dir, bool skipExisting = true);

  //! Loads all recordings with interactions
  void loadAllRecordingsWithInteractions(interaction::InteractionDatabase& interactionDb) const;

 private:
  //! Load single recording from recFile and return by reference
  Recording& loadRecording(const string& recFile) const;

  //! Return ids of all Recordings to load based on the params 
  vec<string> getRecordingIds(const util::Params& params);
  vec<string> filterRecordingIds(const vec<string>& recIds, bool skipRecordingsWithoutScenes) const;

  //! Return ids of all Recordings in the specified directory
  static vec<string> listRecordingIds(const string& dirname, const string& ext);

  const ScanDatabase& m_scans;
  const util::Params& m_params;
  map<string, Recording> m_recordings;
  map<string, vec<Recording*>> m_recordingsByScan;
  map<string, vec<string>> m_recordingIdsByScan;  // list of ALL recordings ids (not necessarily loaded) by scan id
  vec<string> m_allRecordingIds;  // List of ALL recording ids (not necessarily loaded) 
  vec<string> m_recordingIds;     // List of recording ids of interest (not necessarily loaded) 
};

}  // namespace core
}  // namespace sg


