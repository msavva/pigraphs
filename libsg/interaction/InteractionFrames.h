#pragma once

#include "libsg.h"  // NOLINT
#include "interaction/InteractFrameSurfSampled.h"
#include "util/smartenum.h"

namespace sg {
namespace interaction {

#define INTERACTION_FRAME_TYPES(m,t)  \
        m(t, Skeleton)                \
        m(t, JointsSurface)           \
        m(t, JointsVolumetric)        \
        m(t, Occupancy)               \
        m(t, ObjectLabel)

SMARTENUMCLASS_DEFINE_ENUM(InteractionFrameType, INTERACTION_FRAME_TYPES)
static SMARTENUMCLASS_DEFINE_NAMES(InteractionFrameType, INTERACTION_FRAME_TYPES)
inline SMARTENUMCLASS_DEFINE_GET_VALUE_FROM_STRING(InteractionFrameType, InteractionFrameType::kCount)
OSTREAMABLE(InteractionFrameType)

// A bunch of interaction frames
class InteractionFrames : public io::Serializable {
public:
  BOOST_BINARY_SERIALIZABLE_FUNCS
  InteractionFrames();
  //! Create an InteractionFrame with the given world halfwidth dimensions and numBinsPerDim along each axis
  explicit InteractionFrames(const geo::Vec3f& halfwidths, int numBinsPerDim);
  InteractionFrames(float h, int numBinsPerDim) : InteractionFrames(geo::Vec3f(h, h , h), numBinsPerDim) { }

  void clear();

  //! Return the total support of these InteractionFrames centered on skel within scene
  //! NOTE: Repositions InteractionFrame at skel before support computation
  double support(const core::Skeleton& skel, const core::Scan* pScan);

  //! Return the support of these InteractionFrames centered on skel wrt to a model instance
  //! NOTE: Repositions InteractionFrame at skel before support computation
  double support(const core::Skeleton& skel, const core::ModelInstance& modelInst);

  //! Return the support of these InteractionFrames centered on skel wrt to a voxel grid with the given world to grid transform
  //! NOTE: Repositions InteractionFrame at skel before support computation
  double support(const core::Skeleton& skel, const ml::BinaryGrid3& voxelGrid, 
                 const ml::mat4f& worldToGrid, bool skipIfOutsideGrid);

  //! Repositions this InteractionFrame so that it contains Skeleton skel and axis Y faces in the body normal direction
  void reposition(const core::Skeleton& skel);
  //! Recenters InteractionFrame so that it is centered at center with front along positive x axis
  void recenter(const geo::Vec3f& center);

  std::shared_ptr<InteractionFrame> getInteractionFrame(InteractionFrameType t) const {
    return m_iframes[static_cast<size_t>(t)];
  }

  vec<std::shared_ptr<InteractionFrame>>& getInteractionFrames() {
    return m_iframes;
  }

private:
  vec<std::shared_ptr<InteractionFrame>> m_iframes;
  vec<double> m_iframeWeights;

private:
  //! boost serialization function
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {  // NOLINT
    boost::serialization::split_member(ar, *this, version);
  }
  template <typename Archive>
  void save(Archive& ar, const unsigned int version) const {  // NOLINT
    size_t nIFrames = m_iframes.size();
    ar & nIFrames;
    for (size_t i = 0; i < m_iframes.size(); ++i) {
      const InteractionFrame& iframe = *m_iframes[i];
      ar & iframe;
    }
    ar & m_iframeWeights;
  }
  template <typename Archive>
  void load(Archive& ar, const unsigned int version) {  // NOLINT
    m_iframes.clear();
    m_iframes.resize(static_cast<size_t>(InteractionFrameType::kCount));
    size_t nIFrames;
    ar & nIFrames;
    for (size_t i = 0; i < nIFrames; ++i) {
      std::shared_ptr<InteractionFrame> pIFrame =
        std::make_shared<InteractionFrame>();
      ar & *pIFrame;
      if (i < m_iframes.size()) {
        m_iframes[i] = pIFrame;
      } else {
        SG_LOG_WARN << "Ignoring unexpected interaction frame of type " << i;
      }
    }
    ar & m_iframeWeights;
  }
};

}  // namespace interaction
}  // namespace sg
