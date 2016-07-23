#include "common.h"  // NOLINT

#include "vis/vislog.h"

#include "io/io.h"

namespace sg {
namespace vis {

VisLog::VisLog(const string& basedir) {
  init(basedir);
}

void VisLog::init(const string& basedir) {
  if (m_tsv.is_open()) {
    SG_LOG_WARN << "VisLog is already initialized with " << m_basedir;
  }
  m_basedir = basedir;
  if (m_basedir.length() > 0 && m_basedir.at(m_basedir.length()-1) != '/') {
    m_basedir = m_basedir + "/";
  }
  SG_LOG_INFO << "Starting VisLog to " << m_basedir;
  io::ensureDirExists(m_basedir);
  m_tsv.open(m_basedir + "summary.tsv", std::ofstream::out);
}

void VisLog::close() {
  if (m_tsv.is_open()) {
    m_tsv.close();
    if (m_records.size() > 0) {
      saveHtml(m_basedir + "summary.html");
    } else {  // also get rid of empty tsv file
      io::deleteFile(m_basedir + "summary.tsv");
    }
  }
}

void VisLog::log(const string& desc, const VisImage& image) {
  vec<VisImage> images(1);
  images[0] = image;
  log(desc, images);
}

void VisLog::log(const string& desc, const vec<VisImage>& images) {
  time_t now = time(0);
  size_t index = m_records.size();
  m_records.push_back({now, desc, images});
  if (m_tsv.is_open()) {
    if (images.empty()) {
      m_tsv << index << "\t" << now << "\t" << desc << "\t\t" << endl;
    } else {
      for (const VisImage& image : images) {
        m_tsv << index << "\t" << now << "\t" << desc << "\t" 
          << image.imagePath << "\t" << image.description << endl;
      }
    }
  }
}

void VisLog::saveHtml(const string& filename) const {
  // Create html for records
  sg::io::ensurePathToFileExists(filename);
  ofstream os(filename);
  os << "<!DOCTYPE html><html><body>" << endl;

  // Create a html table
  // TODO: Proper HTML escaping...
  os << "<table>" << endl;
  for (const VisLogRecord rec : m_records) {
    os << "<tr>";
    os << "<td>" << rec.description << "</td>";
    for (const VisImage image : rec.images) {
      os << "<td>" << "<img src=\"" << image.imagePath << "\"/>" 
         << "<br/>" << image.description << "</td>";
    }
    os << "<tr>" << endl;
  }
  os << "</table>" << endl;

  // End html and close
  os << "</body></html>" << endl;
  os.close();
}

string VisLog::getNextImageName() const {
  size_t index = m_records.size();
  return "images/image_ " + to_string(index) + ".png";
}

string VisLog::getFilename(const string& relPath) const {
  // TODO: make sure "." and ".." are collapsed
  return m_basedir + relPath;
}

}  // namespace vis
}  // namespace sg
