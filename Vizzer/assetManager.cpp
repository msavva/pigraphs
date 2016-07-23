#include "common.h"  // NOLINT
#include "assetManager.h"

#include <core/Database.h>

void AssetManager::init(sg::core::ModelDatabase& database) {
  m_database = &database;
}

const ModelAssetData& AssetManager::loadModel(ml::GraphicsDevice& g, const std::string& modelId) {
  if (m_models.find(modelId) == m_models.end()) {
    ModelAssetData& assetData = m_models[modelId];

    const sg::core::Model& model = m_database->getModel(modelId).get();
    assetData.mesh.load(g, model.flattenedMesh);
  }
  return m_models[modelId];
}
