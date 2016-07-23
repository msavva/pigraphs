#include "common.h"  // NOLINT

#include "core/SceneSerialized.h"

#include "io/json.h"

namespace sg {
namespace core {

template <typename GV>
bool parseScene(const GV& d, SceneSerialized* pScene) {
  // parse out scene header elements
  if (d.HasMember("sceneId")) {
    pScene->id = d["sceneId"].GetString();
  }
  if (d.HasMember("sceneName")) {
    pScene->name = d["sceneName"].GetString();
  }
  pScene->unit = d["unit"].GetDouble();
  const auto& upj = d["up"];
  pScene->up.x = upj["x"].GetDouble();
  pScene->up.y = upj["y"].GetDouble();
  pScene->up.z = upj["z"].GetDouble();
  const auto& frontj = d["front"];
  pScene->front.x = frontj["x"].GetDouble();
  pScene->front.y = frontj["y"].GetDouble();
  pScene->front.z = frontj["z"].GetDouble();
  if (d.HasMember("score")) {
    pScene->score = d["score"].GetDouble();
  }

  // Parse models: Required
  if (!d.HasMember("object")) {
    SG_LOG_ERROR << "Cannot parse scene: no objects found";
    return false;
  }
  const auto& mArr = d["object"];
  const unsigned numModels = mArr.Size();
  pScene->models.resize(numModels);
  for (unsigned iModel = 0; iModel < numModels; iModel++) {
    const auto& mIn = mArr[iModel];
    SceneSerialized::ModelEntry& m = pScene->models[iModel];
    m.id = mIn["modelId"].GetString();
    m.index = mIn["index"].GetInt();
    m.parentIndex = mIn["parentIndex"].GetInt();
    const auto& xformIn = mIn["transform"]["data"];
    for (unsigned i = 0; i < xformIn.Size(); i++) {
      m.transform.matrix[i] = xformIn[i].GetDouble();
    }
    m.transform.transpose();  // JSON is column-major, we're row major
  }

  // parse out cameras: Optional
  if (d.HasMember("camera")) {
    const auto& camArr = d["camera"];
    const unsigned numCameras = camArr.Size();
    pScene->cameras.resize(numCameras);
    using ml::vec3f;
    for (unsigned iCamera = 0; iCamera < numCameras; iCamera++) {
      const auto& cam = camArr[iCamera];
      const auto& pj = cam["position"];
      const vec3f pos(pj["x"].GetDouble(), pj["y"].GetDouble(), pj["z"].GetDouble());
      // ReSharper disable once CppDeclarationHidesLocal
      const auto& upj = cam["up"];
      const vec3f up(upj["x"].GetDouble(), upj["y"].GetDouble(), upj["z"].GetDouble());
      vec3f dir;
      if (cam.HasMember("direction")) {
        const auto& dj = cam["direction"];
        dir = vec3f(dj["x"].GetDouble(), dj["y"].GetDouble(), dj["z"].GetDouble());
      } else {
        const auto& tj = cam["target"];
        dir = vec3f(tj["x"].GetDouble() - pos.x,
                    tj["y"].GetDouble() - pos.y,
                    tj["z"].GetDouble() - pos.z);
        dir.normalizeIfNonzero();
      }
      const string name = cam["name"].GetString();
      pScene->cameras[iCamera] = {name, pos, up, dir};
    }
  }

  // parse out skeletons: Optional
  if (d.HasMember("skeleton")) {
    const auto& skelArr = d["skeleton"];
    const unsigned numSkels = skelArr.Size();
    pScene->skeletons.resize(numSkels);
    using ml::vec3f;
    const int numJoints = SceneSerialized::SkelEntry::kNumJoints;
    for (unsigned iSkel = 0; iSkel < numSkels; iSkel++) {
      const auto& sIn = skelArr[iSkel];
      const auto& wpArr = sIn["worldPosition"];
      const auto& woArr = sIn["worldOrientation"];
      const auto& pArr = sIn["jointPositionsKinect"];
      const auto& oArr = sIn["jointOrientationsKinect"];
      const auto& cArr = sIn["jointConfidencesKinect"];
      const auto& bpArr = sIn["bonePositions"];
      const auto& boArr = sIn["boneOrientations"];
      const auto& blArr = sIn["boneLengths"];

      SceneSerialized::SkelEntry& s = pScene->skeletons[iSkel];
      s.id = sIn["id"].GetString();
      s.worldPosition.x = wpArr[0].GetDouble();
      s.worldPosition.y = wpArr[1].GetDouble();
      s.worldPosition.z = wpArr[2].GetDouble();
      s.worldOrientation.x = woArr[0].GetDouble();
      s.worldOrientation.y = woArr[1].GetDouble();
      s.worldOrientation.z = woArr[2].GetDouble();
      s.worldOrientation.w = woArr[3].GetDouble();
      s.jointPositionsKinect.resize(numJoints);
      s.jointOrientationsKinect.resize(numJoints);
      s.jointConfidencesKinect.resize(numJoints);
      s.bonePositions.resize(numJoints);
      s.boneOrientations.resize(numJoints);
      s.boneLengths.resize(numJoints);
      for (int i = 0; i < numJoints; ++i) {
        s.jointPositionsKinect[i].x    = pArr[i][0].GetDouble();
        s.jointPositionsKinect[i].y    = pArr[i][1].GetDouble();
        s.jointPositionsKinect[i].z    = pArr[i][2].GetDouble();
        s.jointOrientationsKinect[i].x = oArr[i][0].GetDouble();
        s.jointOrientationsKinect[i].y = oArr[i][1].GetDouble();
        s.jointOrientationsKinect[i].z = oArr[i][2].GetDouble();
        s.jointOrientationsKinect[i].w = oArr[i][3].GetDouble();
        s.jointConfidencesKinect[i]    = cArr[i].GetDouble();
        s.bonePositions[i].x           = bpArr[i][0].GetDouble();
        s.bonePositions[i].y           = bpArr[i][1].GetDouble();
        s.bonePositions[i].z           = bpArr[i][2].GetDouble();
        s.boneOrientations[i].x        = boArr[i][0].GetDouble();
        s.boneOrientations[i].y        = boArr[i][1].GetDouble();
        s.boneOrientations[i].z        = boArr[i][2].GetDouble();
        s.boneOrientations[i].w        = boArr[i][3].GetDouble();
        s.boneLengths[i]               = blArr[i].GetDouble();
      }
    }
  }

  return true;
}

template <typename GV>
bool jsonToScene(const GV& d, SceneSerialized* pScene) {
  // Check what kind of fields this document has...
  // TODO: Check format...
  if (d.HasMember("scene")) {
    // Extract out the scene
    switch (d["scene"].GetType()) {
      case rapidjson::kStringType: {
        rapidjson::Document d2;
        if (!io::parseRapidJSONString(d["scene"].GetString(), &d2)) {
          SG_LOG_ERROR << "Cannot parse scene: invalid json string";
          return false;
        }
        return jsonToScene(d2, pScene);
      }
      case rapidjson::kObjectType: 
        return jsonToScene(d["scene"], pScene);
      default:
        SG_LOG_ERROR << "Cannot parse scene: bad type " << d["scene"].GetType();
        return false;
    }
  } 
  return parseScene(d, pScene);
}

bool SceneSerialized::load(const string& file) {
  // load json
  rapidjson::Document d;
  if (!io::parseRapidJSONDocument(file, &d)) {
    return false;
  }

  return jsonToScene(d, this);
}

bool SceneSerialized::parse(const string& jsonString) {
  // load json
  rapidjson::Document d;
  if (!io::parseRapidJSONString(jsonString, &d)) {
    return false;
  }

  return jsonToScene(d, this);
}

ostream& SceneSerialized::toJSON(ostream& os, bool endlines) const {  // NOLINT(*)
  // make json helper funcs
  const std::function<void(void)> sep = io::put(os, ",", endlines);
  const auto t = [&] (int n) -> ostream& {
    for (int i = 0; i < n; ++i) {
      os << "  ";
    }
    return os;
  };
  const auto k = [&] (const string& id) -> ostream& {
    return os << "\"" + id + "\": ";
  };
  const auto ks = [&] (const string& id, const string& v) -> ostream& {
    k(id);
    return os << "\"" + v + "\"";
  };
  const auto nl = [&] () -> ostream& {
    return endlines ? os << endl : os;
  };
  const auto kvec3f = [&] (const string& id, const ml::vec3f& v) -> ostream& {
    k(id);
    os << "{";
    k("x") << v.x << ", ";
    k("y") << v.y << ", ";
    k("z") << v.z;
    os << "}";
    return os;
  };
  const auto kquat = [&] (const string& id, const ml::vec4f& v) -> ostream& {
    k(id);
    os << "{";
    k("x") << v.x << ", ";
    k("y") << v.y << ", ";
    k("z") << v.z << ", ";
    k("w") << v.w;
    os << "}";
    return os;
  };

  // start scene blob
  os << "{";  nl();

  // header elements
  ks("format", "sceneState");  sep();
  ks("sceneId", id);  sep();
  ks("sceneName", name);  sep();
  kvec3f("up", up);  sep();
  kvec3f("front", front); sep();
  k("unit") << unit; sep();
  k("score") << score; sep();

  // object array
  k("object") << "[";  nl();
  const size_t numModels = models.size();
  for (size_t iModel = 0; iModel < numModels; ++iModel) {
    const auto& m = models[iModel];
    const auto xform = m.transform.getTranspose();  // we're row-major, JSON is column major
    t(1);  os << "{";  nl();
    t(2);  ks("modelId", m.id);  sep();
    t(2);  k("index") << m.index;  sep();
    t(2);  k("parentIndex") << m.parentIndex;  sep();
    t(2);  k("transform") << "{";  nl();
    t(3);  k("rows") << 4;  sep();
    t(3);  k("cols") << 4;  sep();
    t(3);  k("data");  io::toJSON(os, xform); nl();
    t(2);  os << "}";  nl();
    t(1);  os << "}";
    if (iModel == numModels - 1) {
      nl();
    } else {
      sep();
    }
  }
  os << "]";  sep();

  // camera array
  k("camera") << "[";  nl();
  const size_t numCameras = cameras.size();
  for (size_t iCamera = 0; iCamera < numCameras; ++iCamera) {
    const auto& c = cameras[iCamera];
    t(1);  os << "{";  nl();
    t(2);  ks("name", c.name);  sep();
    t(2);  kvec3f("position", c.position);  sep();
    t(2);  kvec3f("up", c.up);  sep();
    t(2);  kvec3f("direction", c.direction);  nl();
    t(1);  os << "}";
    if (iCamera == numCameras - 1) {
      nl();
    } else {
      sep();
    }
  }
  os << "]";  sep();

  // skeleton array
  k("skeleton") << "[";  nl();
  const size_t numSkels = skeletons.size();
  for (size_t iSkel = 0; iSkel < numSkels; ++iSkel) {
    const auto& s = skeletons[iSkel];
    t(1);  os << "{";  nl();
    t(2);  ks("id", s.id);  sep();
    t(2);  k("worldPosition"); io::toJSON(os, s.worldPosition);  sep();
    t(2);  k("worldOrientation"); io::toJSON(os, s.worldOrientation);  sep();
    t(2);  k("jointPositionsKinect"); io::toJSON(os, s.jointPositionsKinect);  sep();
    t(2);  k("jointOrientationsKinect"); io::toJSON(os, s.jointOrientationsKinect);  sep();
    t(2);  k("jointConfidencesKinect"); io::toJSON(os, s.jointConfidencesKinect);  sep();
    t(2);  k("bonePositions"); io::toJSON(os, s.bonePositions);  sep();
    t(2);  k("boneOrientations"); io::toJSON(os, s.boneOrientations);  sep();
    t(2);  k("boneLengths"); io::toJSON(os, s.boneLengths);  nl();
    t(1);  os << "}";
    if (iSkel == numSkels - 1) {
      nl();
    } else {
      sep();
    }
  }
  os << "]";  nl();

  // end scene blob
  os << "}";  nl();

  return os;
}

bool SceneSerialized::save(const string& file) const {
  ofstream os(file);
  toJSON(os, true);
  return true;
}

}  // namespace core
}  // namespace sg

