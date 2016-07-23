#pragma once

#include "libsg.h"  // NOLINT
#include "core/synth/synth.h"

namespace sg {
namespace core {
namespace synth {

struct ObjectJointInteractions;

struct ModelRetrieverParams {
  //! Select model parameter
  SelectModelStrategy selectModelStrategy = SelectModelStrategy::kFirst;  
};

//! Retrieves models in a scene
class ModelRetriever {
 public:
  void init(Database* db) { m_pDatabase = db; }

  //! Retrieves a model for the specified category and adds it to the vector of models
  //! Returns true if retrieval successful
  bool retrieve(const SelectModelStrategy selectModelStrategy,
                const string& category,
                bool restrictToWhitelist, 
                vec<ModelInstance>* pModels) const;

  //! Retrieves a model for the specified category and adds it to the vector of models
  //! Returns true if retrieval successful
  bool retrieve(const SelectModelStrategy selectModelStrategy,
                const string& retrieveCategory,
                const string& category,
                bool restrictToWhitelist, 
                vec<ModelInstance>* pModels) const;

 private:
  //! Database from which we search for models
  Database* m_pDatabase = nullptr;
};

}  // namespace synth
}  // namespace core
}  // namespace sg
