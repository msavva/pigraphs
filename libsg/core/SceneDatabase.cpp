#include "common.h"  // NOLINT

#include "core/SceneDatabase.h"
#include "core/Scene.h"
#include "core/SceneSerialized.h"
#include "io/csv.h"
#include "io/io.h"

namespace sg {
namespace core {

using sg::io::csv::CSVReader;

SceneDatabase::SceneDatabase(const util::Params& params, ModelDatabase& modelDatabase) 
  : m_params(params) 
  , m_modelDatabase(modelDatabase) {
  const string scenesFile = params.get<string>("dataDir") + "recon2scene-items.csv";
  if (!loadFromCsv(scenesFile)) {
    SG_LOG_ERROR << "[SceneDatabase] Error loading from " << scenesFile;
  }
}

bool SceneDatabase::sceneExists(const string& id) const {
  return m_scenesSerialized.count(id) > 0;
}

const Scene& SceneDatabase::getScene(const string& id) const {
  ensureSceneIsLoaded(id);
  MLIB_ASSERT_STR(m_scenes.count(id) != 0, "Scene not found id=" + id);
  return m_scenes.at(id);
}

bool SceneDatabase::getScene(const string& id, Scene* pScene) const {
  if (m_scenesSerialized.count(id) > 0) {
    const SceneSerialized& ss = m_scenesSerialized.at(id);
    return pScene->load(ss, &m_modelDatabase);
  } else {
    return false;
  }
}

const SceneSerialized& SceneDatabase::getSceneSerialized(const string& id) const {
  MLIB_ASSERT_STR(m_scenesSerialized.count(id) != 0, "Scene not found id=" + id);
  return m_scenesSerialized.at(id);
}

void SceneDatabase::ensureSceneIsLoaded(const string& id) const {
  if (m_scenes.count(id) == 0) {
    if (m_scenesSerialized.count(id) > 0) {
      const SceneSerialized& ss = m_scenesSerialized.at(id);
      m_scenes[id].load(ss, &m_modelDatabase);
    }
  }
}

vec<string> SceneDatabase::sceneIds() const {
  vec<string> ids;
  for (const auto& it : m_scenesSerialized) { ids.push_back(it.first); }
  return ids;
}

bool SceneDatabase::loadFromCsv(const string& file) {
  if (!io::fileExists(file)) { return false; }

  CSVReader<5, io::csv::trim_chars<' '>, io::csv::double_quote_escape<',','\"'> > csv(file);
  csv.read_header(io::csv::ignore_extra_column, "id", "condition", "item", "data", "workerId");
  std::string id, condition, item, data, workerId;
  while(csv.read_row(id, condition, item, data, workerId)){
    // do stuff with the data
    bool parseOkay = m_scenesSerialized[id].parse(data);
    if (parseOkay) {
      m_scenesSerialized[id].id = id;
      m_scenesSerialized[id].name = item;
    } else {
      SG_LOG_ERROR << "Error parsing scene " << id;
    }
  }
  return true;
}


}  // namespace core
}  // namespace sg
