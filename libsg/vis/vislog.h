#pragma once

#include "libsg.h"  // NOLINT

namespace sg {
namespace vis {

struct VisImage {
  string imagePath;
  string description;
};

struct VisLogRecord {
  uint64_t timestamp;
  string description;
  vec<VisImage> images;
};

//! Special log with images and stuff
//! Stored in directory with structure
//!    basedir/images/...
//!    basedir/summary.tsv  (tab delimited records)
//!    basedir/summary.html (html file)
class VisLog {
 public:
  VisLog() {};
  explicit VisLog(const string& basedir);
  void init(const string& basedir);
  void close();
  //! Add image to log
  void log(const string& desc, const VisImage& image);
  //! Add images to log
  void log(const string& desc, const vec<VisImage>& images);
  //! Saves html summary of records
  void saveHtml(const string& filename) const;
  //! Returns relative path of next image
  string getNextImageName() const;
  //! Returns absolute path to use given relative path
  string getFilename(const string& relPath) const;
 private:
  //! Directory where the log records are stored
  string m_basedir;
  //! TSV file of log records (appended to as log records are added)
  ofstream m_tsv;
  //! Records
  vec<VisLogRecord> m_records;
};

}  // namespace vis
}  // namespace sg


