#pragma once

#include "mLibCore.h"

#include "libsg.h"  // NOLINT

namespace sg {
namespace segmentation {

enum PartType {
  kPartObject = 0,
  kPartSegment = 1,
  kPartSegmentGroup = 2,
  kPartTypeCount = 3
};

//! Names for part types
static const arr<string, kPartTypeCount> kPartTypeNames = {
  "Object",
  "Segment",
  "SegmentGroup"
};


inline PartType getPartTypeFromString(const std::string& str) {
  for (int i = 0; i < kPartTypeCount; ++i) {
    if (kPartTypeNames[i] == str) {
      return (PartType) i;
    }
  }
  return PartType::kPartTypeCount;
}

class Part {
 public:
  Part(PartType _pt, int _id, string _label)
    : partType(_pt)
    , id(_id)
    , label(_label)
    , m_obb(nullptr)
    , m_dominantNormal() { }

  //! Part type
  PartType partType;
  //! Id of this part
  int id;
  //! Label of this part
  string label;

  //! OBB that characterizes this part
  std::shared_ptr<geo::OBB> obb() const { return m_obb; }

  //! Returns the dominant normal (i.e. normal to the least squares plane fit through this segment's points)
  const ml::vec3f& dominantNormal() const { return m_dominantNormal; }

  // Wrappers around OBB members
  float diagonalLength() const;
  template <typename Vec3>
  float distance(const Vec3& v) const;

  // Closest point from v to segment OBB
  // The closest point is either on the OBB or inside (if v is inside OBB)
  template <typename Vec3>
  Vec3 closestPoint(const Vec3& v) const;
  // Closest point on OBB surface from v
  template <typename Vec3>
  Vec3 closestSurfacePoint(const Vec3& v) const;
  template <typename Vec3>
  Vec3 centroid() const;
  template <typename Vec3>
  Vec3 axesLengths() const;
  template <typename Mat3x3>
  void normalizedAxes(Mat3x3* m) const;
  ml::mat4f localToWorld() const;

 protected:
  //! Compute the dominant normal for this part
  void computeDominantNormal();

  //! Parse OBB from json
  template <typename GV>
  void parseJsonOBB(const GV& sIn) {
    using namespace sg::eigenutil;

    const auto& obbIn = sIn["obb"];
    vecf centroidArr, axesLengthsArr, normalizedAxesArr;
    sg::io::toFloatVector(obbIn["centroid"], &centroidArr);
    sg::io::toFloatVector(obbIn["axesLengths"], &axesLengthsArr);
    sg::io::toFloatVector(obbIn["normalizedAxes"], &normalizedAxesArr);
    MatrixXf c = vec2matf(centroidArr, 3, 1);
    MatrixXf r = vec2matf(axesLengthsArr, 3, 1) * 0.5f;  // full lengths are double the half-widths (r_ in OBB)
    MatrixXf R = vec2matf(normalizedAxesArr, 3, 3);
    m_obb = std::make_shared<geo::OBB>(c, r, R);

    // Read dominant normal
    if (sIn.HasMember("dominantNormal")) {
      vecf dominantNormalArr;
      sg::io::toFloatVector(sIn["dominantNormal"], &dominantNormalArr);
      m_dominantNormal[0] = dominantNormalArr[0];
      m_dominantNormal[1] = dominantNormalArr[1];
      m_dominantNormal[2] = dominantNormalArr[2];
    } else {
      computeDominantNormal();
    }
  }

  std::shared_ptr<geo::OBB> m_obb;  //! OBB that characterizes this part
  ml::vec3f m_dominantNormal;       //! Dominant normal of OBB
};

}  // namespace segmentation
}  // namespace sg



