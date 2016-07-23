#include "common.h"  // NOLINT

#include "io/io.h"
#include "util/util.h"

#include <sstream>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filter/bzip2.hpp>

namespace sg {
namespace io {

void readFile(const string& file, std::function<void(istream&)> reader, std::ios::openmode mode /* = 0 */) {
  namespace bio = boost::iostreams;
  const string ext = io::extension(file);
  if (ext == ".gz") {
    bio::filtering_istream in;
    in.push(bio::gzip_decompressor());
    in.push(bio::file_source(file, std::ios::in | std::ios::binary));
    reader(in);
  } else if (ext == ".bz2") {
    bio::filtering_istream in;
    in.push(bio::bzip2_decompressor());
    in.push(bio::file_source(file, std::ios::in | std::ios::binary));
    reader(in);
  } else {
    reader(ifstream(file, mode));
  }
}

string slurp(const istream& in) {
  std::stringstream sstr;
  sstr << in.rdbuf();
  return sstr.str();
}

string slurp(const string& file) {
  string out;
  readFile(file, [&out] (istream& is) {  // NOLINT
    out = slurp(is);
  });
  return out;
}

vec<string> getLines(const string& file) {
  vec<string> lines;
  readFile(file, [&lines] (istream& is) {  // NOLINT
    string line;
    while (std::getline(is, line)) {
      lines.push_back(line);
    }
  });
  return lines;
}

vec<vec<string>> getTokenizedLines(const string& file, const string& delimiter) {
  vec<string> lines = getLines(file);
  vec<vec<string>> tokenizedLines(lines.size());
  for (size_t i = 0; i < lines.size(); ++i) {
    boost::split(tokenizedLines[i], lines[i], boost::is_any_of(delimiter));
  }
  return tokenizedLines;
}

namespace bf = boost::filesystem;

bool dirExists(const string& path) {
  return bf::is_directory(path);
}

bool fileExists(const string& path) {
  return bf::exists(path);
}

void ensureDirExists(const string& path) {
  if (path.empty()) return;  // No path, nothing to do
  bf::create_directories(path);
}

void ensurePathToFileExists(const string& filename) {
  const bf::path p(filename);
  const bf::path dir = p.parent_path();
  if (dir.string().empty()) return;  // No path, nothing to do
  if (!bf::exists(dir)) {
    bf::create_directories(dir);
  } else if (!bf::is_directory(dir)) {
    // NOTE+TODO: On Windows, junctions do not evaluate to true for boost < 1.57
    // (see https://svn.boost.org/trac/boost/ticket/9016)
    //SG_LOG_ERROR << "[io] File with same name as desired directory name exists" << endl;
  }
}

void copyFile(const string& fromPath, const string& toPath, bool allowOverwrite /* = false */) {
  ensurePathToFileExists(toPath);
  const auto opt = allowOverwrite ? bf::copy_option::overwrite_if_exists : bf::copy_option::fail_if_exists;
  bf::copy_file(fromPath, toPath, opt);
}

void moveFile(const string& fromPath, const string& toPath, bool allowOverwrite /* = false */) {
  ensurePathToFileExists(toPath);
  bool toFileExists = io::fileExists(toPath);
  if (allowOverwrite || !toFileExists) {
    bf::rename(fromPath, toPath);
  } else {
    SG_LOG_ERROR << "Cannot move " << fromPath << " to " << toPath << ": file exisits";
  }
}

string basename(const string& path) {
  return bf::basename(path);
}

string filename(const string& path) {
  return bf::path(path).filename().string();
}

string parentPath(const string& path) {
  return bf::path(path).parent_path().string();
}

string extension(const string& path) {
  return bf::extension(path);
}

string replaceExtension(const string& path, const string& ext) {
  return bf::path(path).replace_extension(ext).string();
}

sg::string removeExtension(const string& path) {
  return replaceExtension(path, "");
}

bool deleteFile(const string& path) {
  return bf::remove(path);
}

unsigned long long deleteRecursively(const string& path) {
  return bf::remove_all(path);
}

string getAbsolutePath(const string& path) {
  return bf::path(path).string();
}

bool isDirectory(const string& path) {
  return is_directory(bf::path(path));  
}

void listFiles(const bf::path& p, std::function<bool(const bf::path&)> filter, const bool recursive,
               vec<string>* pFiles)
{
  if (is_regular_file(p)) {
    if (filter(p)) {
      pFiles->push_back(p.string());
    }
  } else if (is_directory(p)) {
    if (recursive) {
      for (auto&& f : bf::directory_iterator(p)) {
        listFiles(f, filter, recursive, pFiles);
      }
    } else {
      for (auto&& f : bf::directory_iterator(p)) {
        const bf::path fp(f);
        if (is_regular_file(fp) && filter(fp)) {
          pFiles->push_back(fp.string());
        }
      }      
    }
  }
}

vec<string> listFilesWithSuffix(const string& path, const string& suffix, const bool recursive)
{
  bf::path rootpath(path);
  vec<string> filenames;
  listFiles(rootpath, [&](const bf::path& p) { return util::endsWith(p.string(), suffix); }, recursive, &filenames);
  return filenames;
}

vec<string> listFilesWithName(const string& path, const string& name, const bool recursive)
{
  bf::path rootpath(path);
  vec<string> filenames;
  listFiles(rootpath, [&](const bf::path& p) { return p.filename().string() == name; }, recursive, &filenames);
  return filenames;
}

}  // namespace io
}  // namespace sg
