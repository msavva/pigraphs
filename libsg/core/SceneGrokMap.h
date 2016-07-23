#pragma once

#include <mLibCore.h>

#include "libsg.h"  // NOLINT
#include "core/Skeleton.h"
#include "io/io.h"
#include "vis/Heatmap.h"

namespace sg {
namespace core {

// Struct for serializing out SceneGrok action maps
struct SceneGrokMap : public io::Serializable {
  BOOST_BINARY_SERIALIZABLE_FUNCS
  //! Construct a SceneGrokMap given an input heatMap with sample points, and a set of baseSkeletons generating it
  void create(const vis::PoseHeatMap& heatMap, const vec<TransformedSkeleton>& skels) {
    // Fill out sampled orientation angles
    const int numThetas = heatMap.numThetaDivisions();
    const float thetaNorm = static_cast<float>(ml::PI) * 2.f / numThetas;
    thetas.resize(numThetas);
    for (int iTheta = 0; iTheta < numThetas; ++iTheta) {
      thetas[iTheta] = iTheta * thetaNorm;
    }

    // Fill out baseSkeletons array
    const size_t numSkeletons = skels.size();
    baseSkeletons.resize(numSkeletons);
    for (size_t i = 0; i < numSkeletons; i++) {
      baseSkeletons[i] = skels[i].getSkeleton();
    }

    // Fill out sample points array
    const auto& data = heatMap.points();
    const size_t numPoints = data.size();
    points.resize(numPoints);
    for (size_t iPoint = 0; iPoint < numPoints; ++iPoint) {
      const auto& heatMapPoint = data[iPoint];
      points[iPoint].first.x = heatMapPoint.pos[0];
      points[iPoint].first.y = heatMapPoint.pos[1];
      auto& likelihoods = points[iPoint].second;
      likelihoods.resize(numSkeletons * numThetas);
      for (size_t iSkel = 0; iSkel < numSkeletons; ++iSkel) {
        const size_t baseIdx = iSkel * numThetas;
        for (size_t iTheta = 0; iTheta < numThetas; ++iTheta) {
          likelihoods[baseIdx + iTheta] = static_cast<float>(heatMapPoint.likelihoods[iSkel][iTheta]);
        }
      }
    }
  }

  //! Transforms the points and skeletons of this SceneGrokMap by m
  void transform(const ml::mat4f& m) {
    for (auto& p : points) {
      const ml::vec3f v = m * ml::vec3f(p.first.x, p.first.y, 0.f);
      p.first.x = v.x;  p.first.y = v.y;
    }
    for (Skeleton& s : baseSkeletons) {
      s = TransformedSkeleton(s, m).getSkeleton();
    }
  }

  //! Returns TransformedSkeleton 
  TransformedSkeleton makeTransformedSkeleton(int iPoint, int iSkel, int iTheta) const {
    assert(iPoint >= 0 && iPoint < points.size() &&
           iSkel >= 0 && iSkel < baseSkeletons.size() &&
           iTheta >= 0 && iTheta < thetas.size());
    const auto& p = points[iPoint].first;
    ml::vec3f transXY(p[0], p[1], 0.0f);
    const float rotZ = ml::math::radiansToDegrees(thetas[iTheta]);
    return TransformedSkeleton(baseSkeletons[iSkel], ml::mat4f::translation(transXY) * ml::mat4f::rotationZ(rotZ));
  }

  //! boost serialization function
  template<class Archive>
  void serialize(Archive& ar, const unsigned int version) {  // NOLINT
    ar & baseSkeletons;
    ar & thetas;
    ar & points;
  }

  ////! Write out this SceneGrokMap to os in JSON format
  //ostream& SceneGrokMap::toJSON(ostream& os, bool endlines) const {  // NOLINT(*)
  //  using sg::io::toJSON;
  //  const std::function<void(void)> sep = sg::io::put(os, ",", endlines);
  //  const auto key = [ ] (const string& id) { return "\"" + isetId + "\": "; };

  //  os << "{";                      if (endlines) { os << endl; }
  //  os << key("baseSkeletons");     toJSON(os, baseSkeletons);  sep();
  //  os << key("thetas");            toJSON(os, thetas);         sep();
  //  os << key("samples");           toJSON(os, points);         if (endlines) { os << endl; }
  //  os << "}";                      if (endlines) { os << endl; }

  //  return os;
  //}

  vec<Skeleton> baseSkeletons;        //! skeletons sampled at each point
  vecf thetas;                        //! orientations sampled at each point (in radians)
  vec<pair<ml::vec2f, vecf>> points;  //! pairs of sample (x,y) and likelihood[i = iSkel * numTheta + iTheta] arrays
};

}  // namespace core
}  // namespace sg


