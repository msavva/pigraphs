#pragma once

#include "libsg.h"  // NOLINT

#include <mLibCore.h>

namespace sg {
namespace interaction {

struct ModelAnnotation {
  //! Point-based annotation of a given interaction type over the surface of a model
  struct Annotation {
    //! Annotated point in model space
    struct Point {
      ml::vec3f pos; int meshIdx; int triIdx; ml::vec2f uv;
      template<class Archive>
      void serialize(Archive& ar, unsigned version) {  // NOLINT
        ar & pos & meshIdx & triIdx & uv;
      }
    };
    string type;                  //! interaction type: gaze, feet, fingertip, backSupport, hips
    vec<Point> points;            //! set of annotated points in this annotation
    vec<vec<float>> faceValues;   //! cache of values indexed with [meshIdx][triangleIdx]
    template<class Archive>
    void serialize(Archive& ar, unsigned version) {  // NOLINT
      ar & type & points & faceValues;
    }
  };

  //! Loading function for loading from libsynth output .arv files
  void load(const string& file);

  //! map from interaction type -> Annotation containing all points with type
  map<string, Annotation> entries;

 private:
  //! boost serialization helper
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive& ar, unsigned version) {  // NOLINT
    ar & entries;
  }
};

class ModelAnnotationDatabase {
 public:
  //! Load all model annotations in given directory
  void loadAnnotations(const util::Params& params);

  //! Return set of ids for all models with annotations
  set<string> getAnnotatedModelIds() const;

  //! Return Annotation for model with given id
  const ModelAnnotation& getAnnotation(const string& modelId) const;

  //! File to which database is cached
  static const string kCachedFile;

 private:
  map<string, ModelAnnotation> m_modelId2annotation;
};

}  // namespace interaction
}  // namespace sg


