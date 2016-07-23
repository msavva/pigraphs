#pragma once

#include "libsg.h"  // NOLINT

namespace sg {
namespace io {

//! Handles uncompression of target file on the fly and calls reader on uncompressed istream.
//! Can pass file open mode through mode variable (mostly for ios::binary reading).
void readFile(const string& file, std::function<void(istream&)> reader, std::ios::openmode mode = 0);

//! Slurp contents of entire file underlying ifstream is and return as string
string slurp(const istream& is);

//! Slurp contents of entire file and return as string (also uncompresses .gz and .bzip on-the-fly)
string slurp(const string& file);

//! Return a vector of strings for each line in the given file (.gz,.bzip supported)
vec<string> getLines(const string& file);

//! Return a vector of vec<string> for each line, splitting by the given tokenizer (.gz,.bzip supported)
vec<vec<string>> getTokenizedLines(const string& file, const string& delimiter);

//! Return whether directory exists at the given path
bool dirExists(const string& path);

//! Returns whether file exists at the given location specified by path
bool fileExists(const string& path);

//! Ensure that the path in the string exists as a directory. Creates all directories in path if necessary.
//! Returns whether the path already existed.
void ensureDirExists(const string& path);

//! Ensures that path to given filename exists by recursively creating all directories up to filename's parent.
void ensurePathToFileExists(const string& filename);

//! Copy file fromPath -> toPath
void copyFile(const string& fromPath, const string& toPath, bool allowOverwrite = false);

//! Copy file fromPath -> toPath
void moveFile(const string& fromPath, const string& toPath, bool allowOverwrite = false);

//! Returns the basename of the file or directory at the path (excludes extension)
string basename(const string& path);

//! Returns the filename of the file or directory at the path (includes extension)
string filename(const string& path);

//! Returns the parent path of the given file or directory
string parentPath(const string& path);

//! Returns the extension of the file
string extension(const string& path);

//! Replaces the current extension of the given path with ext and returns the new path
string replaceExtension(const string& path, const string& ext);

//! Removes the extension from the given path and returns the new path
string removeExtension(const string& path);

//! Delete file at path. Return false if did not exist, otherwise true
bool deleteFile(const string& path);

//! Recursively delete contents of path if it exists, then delete path itself.
//! Returns number of files deleted.
unsigned long long deleteRecursively(const string& path);

//! Returns whether a path is absolute or relative
inline bool isAbsolutePath(const string& path) {
  if (path.length() > 0) {
    if (path.at(0) == '/') return true;
    else if (path.length() > 1) {
      if (path.at(1) == ':') return true;
    }
  }
  return false;
}

//! Return absolute path given path
string getAbsolutePath(const string& path);

//! Returns whether a path is a directory or not
bool isDirectory(const string& path);

//! Returns a list of files ending with the given suffix
vec<string> listFilesWithSuffix(const string& path, const string& suffix, const bool recursive);

//! Returns a list of files in directory with given name
vec<string> listFilesWithName(const string& path, const string& name, const bool recursive);

//! Return a function that outputs string s and optionally endline into ostream os on every call
inline std::function<void(void)> put(ostream& os, const string& s, bool endlines = false) {
  return ([&os, s, endlines] () {
    os << s;
    if (endlines) {
      os << endl;
    }
  });
}

template <typename T>
inline void saveToFile(const string& filename, const T& x) {
  ofstream ofs(filename);
  ensurePathToFileExists(filename);
  ofs << x;
  ofs.close();
}

//! Templated boost binary serialization helper
namespace serialization {

//! Load binary format from file and return whether successful
template<typename T>
bool loadBinary(T& x, const string& file) {
  if (!fileExists(file)) {
    SG_LOG_ERROR << "File not found: " << file;
    return false;
  }
  ifstream ifs(file, std::ios::binary);
  {
    boost::archive::binary_iarchive ar(ifs);
    ar >> x;
  }
  ifs.close();
  return true;
}

//! Save in binary format to file and return whether successful
template<typename T>
bool saveBinary(const T& x, const string& file) {
  ensurePathToFileExists(file);
  ofstream ofs(file, std::ios::binary);
  {
    boost::archive::binary_oarchive ar(ofs);
    ar << x;
  }
  ofs.close();
  return true;
}

}  // namespace serialization

//! Interface for serialization. Default implementation uses binary
class Serializable {
  friend class boost::serialization::access;
 public:
  virtual ~Serializable() { }
  virtual bool load(const string& file) { return loadBinary(file); }
  virtual bool save(const string& file) const { return saveBinary(file); }
  virtual bool loadBinary(const string& file) = 0;
  virtual bool saveBinary(const string& file) const= 0;
};

//! Include macro in cpp to provide default boost binary serialization
#define BOOST_BINARY_SERIALIZABLE_FUNCS                     \
  virtual bool loadBinary(const string& f) override;        \
  virtual bool saveBinary(const string& f) const override;  \

//! Include macro in cpp to provide default boost binary serialization
#define BOOST_BINARY_SERIALIZABLE_IMPL(T)                                                            \
  bool T::loadBinary(const string& f) { return sg::io::serialization::loadBinary(*this, f); }        \
  bool T::saveBinary(const string& f) const { return sg::io::serialization::saveBinary(*this, f); }  \

}  // namespace io
}  // namespace sg


