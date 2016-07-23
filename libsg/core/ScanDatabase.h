#pragma once

#include "libsg.h"  // NOLINT
#include "core/Scan.h"

namespace sg {
namespace core {

class RecordingDatabase;

class ScanDatabase {
 public:
  explicit ScanDatabase(const util::Params& params, const string& scansJsonFile);
  
  //! Returns scan with given id
  const Scan& getScan(const string& id) const;

  //! Returns true if scan with given id exists, false otherwise
  bool scanExists(const string& id) const;
  
  //! Returns list of current scanIds
  vec<string> scanIds() const;
  
  //! Batch loads all scans that have recordings for them
  void loadAllScansWithRecordings(const RecordingDatabase& recordingDatabase);

  //! Saves scans as PLY files (useful for saving away added ground planes)
  void saveScansAsPLYs(const vec<string>& scanIds) const;

  //! Saves scan occupied and unobserved space as binvox format files
  void saveScanBinvoxes(const vec<string>& scanIds) const;

  //! Save scan labeled voxel grids
  void saveScanLabeledVoxels(const vec<string>& scanIds, const synth::ObjectLabeler& labeler,
                             const synth::LabelOpts& opts, const string& runId) const;

  //! Get annotation labels across all scans
  void getAnnotationLabels(util::Index<string>* pIndex) const;

  //! Save segment features for each Scan in this database in csv format file within dir
  void saveSegmentFeatures(const string& dir) const;

 private:
  //! Load SceneDatabase metadata from JSON file. Returns true if succeeded, false otherwise.
  bool loadFromJSON(const string& file);

  //! Load a single scan with given id
  void loadScan(const string& id) const;

  //! Returns scan id in which recording with recId was recorded
  static string recordingIdtoBaseScanId(const string& recId);

  const util::Params& m_params;
  string m_id;
  mutable map<string, Scan> m_scans;
};

}  // namespace core
}  // namespace sg


