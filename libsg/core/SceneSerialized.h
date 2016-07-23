#pragma once

#include "libsg.h"  // NOLINT

namespace sg {
namespace core {

//! Handles loading and saving to serialized Scene data
class SceneSerialized {
public:
  //! Default ctor
  SceneSerialized() = default;
  //! Load from file
  explicit SceneSerialized(const string& file) : SceneSerialized() {
    load(file);
  }
  //! Load Scene from file and return whether successful
  bool load(const string& file);
  //! Parse Scene from string and return whether successful
  bool parse(const string& jsonString);
  //! Save Scene to file and return whether successful
  bool save(const string& file) const;
  //! Writes out JSON representation of this Scene to os and returns os
  ostream& toJSON(ostream& os, bool endlines) const;

  //! Stub for loading ModelInstance info
  struct ModelEntry {
    string id;
    int index;
    int parentIndex;
    ml::mat4f transform;
  };

  //! Stub for loading Camera info
  struct CameraEntry {
    string name;
    ml::vec3f position;
    ml::vec3f up;
    ml::vec3f direction;
  };

  //! Stub for loading Skeleton info
  struct SkelEntry {
    const static int kNumJoints = 25;
    string id;
    ml::vec3f worldPosition;
    ml::vec4f worldOrientation;
    vec<ml::vec3f> jointPositionsKinect;
    vec<ml::vec4f> jointOrientationsKinect;
    vec<float> jointConfidencesKinect;
    vec<ml::vec3f> bonePositions;
    vec<ml::vec4f> boneOrientations;
    vec<float> boneLengths;
  };

  string            id;       //! Identifier
  string            name;     //! Human-readable name
  ml::vec3f         up;       //! Semantic up direction
  ml::vec3f         front;    //! Semantic front direction
  float             unit;     //! Unit scale (virtual unit --> meters)
  vec<ModelEntry>   models;   //! Model entries in this scene
  vec<CameraEntry>  cameras;  //! Cameras defined for this scene
  vec<SkelEntry>    skeletons;  //! Skeletons in this scene
  double            score;      //! Overall scene score
};

}  // namespace core
}  // namespace sg


