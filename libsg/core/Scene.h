#pragma once

#include "libsg.h"  // NOLINT

#include "core/ModelInstance.h"
#include "core/Skeleton.h"

namespace sg {
namespace core {

class Scene {
 public:
  //! Default ctor
  Scene() : m_id(), m_name(), m_up(0, 0, 1), m_front(0, -1, 0), m_unit(1), m_models() { }
  //! Creates Scene from serialized description in file, retrieving models from pModelDb
  Scene(const string& file, ModelDatabase* pModelDb);
  //! Creates Scene from serialized description ss, retrieving models from pModelDb
  Scene(const SceneSerialized& ss, ModelDatabase* pModelDb);

  //! Loaders
  bool load(const SceneSerialized& ss, ModelDatabase* pModelDb);
  bool load(const string& file, ModelDatabase* pModelDb);

  //! Convert to serialized representation in pSs
  bool serialize(SceneSerialized* pSs) const;
  //! Write out serialized representation of this Scene to file
  bool serialize(const string& file) const;

  //! Add ModelInstance to this Scene
  void add(const ModelInstance& mInst) {
    m_models.push_back(mInst);
  }
  //! Add Skeleton to this Scene
  void add(const Skeleton& skel) {
    m_skels.push_back(skel);
  }
  //! Clear all models in this scene
  void clear() {
    m_models.clear();
    m_skels.clear();
  }
  //! Return number of models in scene
  size_t size() const {
    return m_models.size();
  }

  //! Return model instances in this scene
  const vec<ModelInstance>& getModelInstances() const {
    return m_models;
  }

  //! Return skeletons in this scene
  const vec<Skeleton>& getSkels() const {
    return m_skels;
  }

  void setScore(double s) { m_score = s; }
  double getScore() const { return m_score; }

 private:
  string             m_id;       //! Identifier
  string             m_name;     //! Human-readable name
  geo::Vec3f         m_up;       //! Semantic up direction
  geo::Vec3f         m_front;    //! Semantic front direction
  float              m_unit;     //! Unit scale (virtual unit --> meters)
  vec<ModelInstance> m_models;   //! Model entries in this scene
  vec<Skeleton>      m_skels;    //! Skeletons in this scene
  double             m_score;    //! Score we gave to this scene;
};

}  // namespace core
}  // namespace sg


