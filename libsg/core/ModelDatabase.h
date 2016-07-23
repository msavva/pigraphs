#pragma once

#include "libsg.h"  // NOLINT
#include "core/Model.h"
#include "core/ScenePriors.h"
#include <boost/optional.hpp>

namespace sg {
namespace core {

class ModelDatabase {
 public:
  explicit ModelDatabase(const util::Params& params);

  //! Returns model with given modelId
  boost::optional<const Model&> getModel(const string& id);

  //! Return first model in the given category
  boost::optional<const Model&> getFirstModelWithCategory(const string& cat, bool restrictToWhitelist);

  //! Return random model in the given category
  boost::optional<const Model&> getRandomModelWithCategory(const string& cat, bool restrictToWhitelist);

  //! Return model in the given category with closest minXY/maxXY dimension aspect ratio and overall XY area
  boost::optional<const Model&> getModelMatchingOBB(const string& cat, const geo::OBB& obb, bool restrictToWhitelist);

  //! Returns true if model with given id exists, false otherwise
  bool modelExists(const string& id) const;

  //! Return true if there are models with the given categories
  size_t modelCountForCategory(const string& cat, bool restrictToWhitelist) const; 

  //! Ensure segmentation for specified model
  bool ensureSegmentation(const string& id, const segmentation::SegmentationParams& params, bool load);

  //! Clear segmentation for loaded models
  void clearSegmentation();

  //! Ensure voxelization for the specified model
  bool ensureVoxelization(const string& id);

  //! Ensure mesh data for all currently known models is cached locally
  void ensureAllModelsCached() const;

  //! Computes and writes out model stats to given filename in csv format
  void computeModelStats(const string& filename);

  //! add categories
  void addParentCategories(const CategoryHierarchy& db);

 private:
  //! Ensure mesh data for model with given modelId is loaded
  void ensureModelIsLoaded(const string& id);

  //! Helper function that ensures model with given id is resident in local cache and returns path
  void ModelDatabase::ensureModelIsCached(const string& id) const;

  //! Returns path to locally cached filename for model given its id
  string ModelDatabase::baseFilename(const string& id) const;

  const util::Params& m_params;
  //! Map of string to model
  map<string, Model> m_models;
  //! Model ids by category
  map<string, vec<string>> m_categoryToModelIds;

  //! Whitelisted models
  map<string, vec<string>> m_categoryToModelIdsWhitelist;
  set<string> m_modelWhitelist;
};

}  // namespace core
}  // namespace sg


