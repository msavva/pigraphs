#pragma once

#include "./mLibInclude.h"

struct ModelAssetData {
  ml::D3D11TriMesh mesh;
};

namespace sg {
namespace core {
class ModelDatabase;
}
}

class AssetManager {
 public:
  AssetManager() : m_models(), m_database(nullptr) { }

  void init(sg::core::ModelDatabase& _database);

  const ModelAssetData& loadModel(ml::GraphicsDevice& g, const std::string& modelId);

  const std::map<std::string, ModelAssetData>& getModelsMap() {
    return m_models;
  }

 private:
  std::map<std::string, ModelAssetData> m_models;
  sg::core::ModelDatabase* m_database;
};


