#include "common.h"  // NOLINT

#include "core/synth/ModelRetriever.h"

#include <mLibCore.h>

#include "core/Database.h"
#include "core/Model.h"
#include "core/synth/ObjectLabeler.h"
#include "core/synth/synth.h"

namespace sg {
namespace core {
namespace synth {

bool ModelRetriever::retrieve(const SelectModelStrategy selectModelStrategy,
                              const string& category,
                              bool restrictToWhitelist,
                              vec<ModelInstance>* pModels) const {
  return retrieve(selectModelStrategy, category, category, restrictToWhitelist, pModels);
}

bool ModelRetriever::retrieve(const SelectModelStrategy selectModelStrategy,
                              const string& retrieveCategory,
                              const string& category,
                              bool restrictToWhitelist,
                              vec<ModelInstance>* pModels) const {
  string catstr = (retrieveCategory == category)? category : category + " (" + retrieveCategory + ")";
  if (m_pDatabase->models.modelCountForCategory(retrieveCategory, restrictToWhitelist) > 0) {
    const Model& model = (selectModelStrategy == SelectModelStrategy::kFirst) ?
      m_pDatabase->models.getFirstModelWithCategory(retrieveCategory, restrictToWhitelist).get() :
      m_pDatabase->models.getRandomModelWithCategory(retrieveCategory, restrictToWhitelist).get();
    m_pDatabase->models.ensureVoxelization(model.id);
    ModelInstance mInst(model);
    int colorId = m_pDatabase->getLabeler().getCategoryIndex().indexOf(category);
    mInst.color = ml::ColorUtils::colorById<ml::vec4f>(colorId);
    mInst.category = category;
    SG_LOG_INFO << "[ModelRetriever] Retrieve " << model.id << " for category "
      << catstr << " with color index " << colorId << ", color " << mInst.color;
    pModels->push_back(mInst);
    return true;
  } else {
    SG_LOG_WARN << "[ModelRetriever] Skipping category " << catstr << " (no models)";
    return false;
  }
}

}  // namespace synth
}  // namespace core
}  // namespace sg

