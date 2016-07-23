#pragma once

#include "libsg.h"  // NOLINT

namespace sg {
namespace core {

//! A node struct for creating simple trees
struct TreeNode {
  TreeNode* pParent = nullptr; //! Parent of this node
  vec<TreeNode*> pChildren;    //! Children of this node

  //! Add child to this node
  void add(TreeNode* child) {
    child->pParent = this;
    pChildren.push_back(child);
  }

  //! Remove child from this node
  void remove(TreeNode* child) {
    const auto nodeIt = find(pChildren.begin(), pChildren.end(), child);
    if (nodeIt != pChildren.end()) {
      (*nodeIt)->pParent = nullptr;
      pChildren.erase(nodeIt);
    }
  }

  //! Attach this node to parent
  void attachTo(TreeNode* parent) {
    pParent = parent;
    parent->add(this);
  }

  //! Remove all children from this node
  void removeAllChildren() {
    for (TreeNode* pNode : pChildren) {
      pNode->pParent = nullptr;
    }
    pChildren.clear();
  }
  //! Equality operator
  bool operator==(const TreeNode& o) const {
    return this == &o;
  }
};

}  // namespace core
}  // namespace sg


