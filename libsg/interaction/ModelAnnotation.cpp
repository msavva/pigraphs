#include "common.h"  // NOLINT

#include "interaction/ModelAnnotation.h"

#include <ext-boost/serialization.h>

#include "io/io.h"
#include "util/timer.h"

namespace sg {
namespace interaction {

namespace fs = boost::filesystem;
namespace bar = boost::archive;

void ModelAnnotation::load(const string& file) {
  ifstream ifs(file, std::ios::binary);
  bar::binary_iarchive bia(ifs);
  bia >> entries;
}

const string ModelAnnotationDatabase::kCachedFile = "ModelAnnotationDatabase.cached";

void ModelAnnotationDatabase::loadAnnotations(const util::Params& p) {
  util::Timer timer("initModelAnnotationDatabase");
  const string cacheFile = p.get<string>("dataCacheDir") + kCachedFile;
  if (io::fileExists(cacheFile)) {
    ifstream ifs(cacheFile, std::ios::binary);
    bar::binary_iarchive bia(ifs);
    bia >> m_modelId2annotation;
  } else {  // Load annotations from scratch and store away in cache file
    fs::path dir(p.get<string>("dataDir") + p.get<string>("modelAnnotationsDir"));
    if (is_directory(dir)) {
      for (auto& entry : make_iterator_range(fs::directory_iterator(dir))) {
        const fs::path& filePath = entry.path();
        if (filePath.extension() == ".arv") {
          const string modelId = basename(filePath);
          m_modelId2annotation[modelId].load(filePath.string());
        }
      }
      ofstream ofs(cacheFile, std::ios::binary);
      bar::binary_oarchive boa(ofs);
      boa << m_modelId2annotation;
      cout << "[ModelAnnotationDatabase] cached in " << cacheFile << endl;
    } else {
      cerr << "[ModelAnnotationDatabase] directory not found: " << dir;
    }
  }
  cout << "[ModelAnnotationDatabase] loaded " << m_modelId2annotation.size() << " annotations in ";
}

set<string> ModelAnnotationDatabase::getAnnotatedModelIds() const {
  set<string> ids;
  for (const auto& entry : m_modelId2annotation) {
    ids.insert(entry.first);
  }
  return ids;
}

const ModelAnnotation& ModelAnnotationDatabase::getAnnotation(const string& modelId) const {
  return m_modelId2annotation.at(modelId);
}

}  // namespace interaction
}  // namespace sg
