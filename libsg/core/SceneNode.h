#pragma once

#include "libsg.h"  // NOLINT

#include "core/TreeNode.h"
#include "geo/geo.h"

namespace sg {
namespace core {

// Node for use in a scene graph
// Just like a TreeNode but with transform
struct SceneNode : public TreeNode { 
  SceneNode(const geo::Transform& _xform = geo::Transform::Identity())
    : m_localToParent(_xform), parentToLocalNeedUpdate(true) { }
  const geo::Transform& getParentToLocal() const {
    if (parentToLocalNeedUpdate) {
      m_parentToLocal = m_localToParent.inverse();
      parentToLocalNeedUpdate = false;
    }
    return m_parentToLocal;
  }
  const geo::Transform& getLocalToParent() const { return m_localToParent; }
  const geo::Transform& getLocalTransform() const { return m_localToParent; }
  void setLocalToParent(const geo::Transform& xform) {
    m_localToParent = xform;
    parentToLocalNeedUpdate = true;
  }
  void setLocalTransform(const geo::Transform& xform) {
    m_localToParent = xform;
    parentToLocalNeedUpdate = true;
  }
  void setParentToLocal(const geo::Transform& xform) {
    m_parentToLocal = xform;
    m_localToParent = xform.inverse();
    parentToLocalNeedUpdate = false;
  }
  void setLocalTranslation(const geo::Vec3f& t) {
    m_localToParent.translation() = t;
    parentToLocalNeedUpdate = true;
  }
  void updateLocalTransformBy(const geo::Transform& xform) {
    m_localToParent = xform * m_localToParent;
    parentToLocalNeedUpdate = true;
  }
  void updateLocalScaleRotateBy(const geo::Matrix3f& r) {
    m_localToParent.linear() = r * m_localToParent.linear();
    parentToLocalNeedUpdate = true;
  } 
  void updateLocalTranslationBy(const geo::Vec3f& t) {
    m_localToParent.translation() += t;
    parentToLocalNeedUpdate = true;
  } 

protected:
  geo::Transform m_localToParent;
  mutable geo::Transform m_parentToLocal;
  mutable bool parentToLocalNeedUpdate = false;
};

}  // namespace core
}  // namespace sg


