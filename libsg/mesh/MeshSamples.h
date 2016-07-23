#pragma once

#include "libsg.h"  // NOLINT
#include "geo/geo.h"
#include "mesh/TriMesh.h"

namespace sg {
namespace mesh {

// Represents a set of samples from a Mesh. Provides storage and iterators for
// sample points, normals, original mesh facehandles, colors, and total area
// of sampled mesh surface
class MeshSamples {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Container typedefs
  typedef TriMesh Mesh;
  typedef vec<typename Mesh::FaceHandle> VecFaceHandle;
  typedef vec<typename Mesh::Color> VecColor;
  // Iterator typedefs
  typedef geo::VecVec3f::iterator pIter;
  typedef geo::VecVec3f::const_iterator cpIter;
  typedef geo::VecVec3f::iterator nIter;
  typedef geo::VecVec3f::const_iterator cnIter;
  typedef VecFaceHandle::iterator fIter;
  typedef VecFaceHandle::const_iterator cfIter;
  typedef VecColor::iterator cIter;
  typedef VecColor::const_iterator ccIter;

  MeshSamples() : p_(0), n_(0), f_(0), c_(0), area_(0) { }
  MeshSamples(const geo::VecVec3f& p, const geo::VecVec3f& n,
              const VecFaceHandle& f, const VecColor& c, float area)
    : p_(p), n_(n), f_(f), c_(c), area_(area) { }

  // Sample uniformly at random points over the mesh surface faces uisng
  // area weighted samples and baryncentric coordinates
  MeshSamples(const Mesh& mesh, cfIter fBegin, cfIter fEnd, size_t nSamples,
              bool addCentroids = false, bool addVertices = false);

  void clear() { p_.clear(); n_.clear(); f_.clear(); c_.clear(); }

  void resize(size_t k) {
    p_.resize(k); n_.resize(k); f_.resize(k); c_.resize(k);
  }

  size_t size() const { return p_.size(); }

  // Accessors
  const geo::VecVec3f& points()  const { return p_; }
  const geo::VecVec3f& normals() const { return n_; }
  const VecFaceHandle& faces()   const { return f_; }
  const VecColor& colors()       const { return c_; }
  float area()                   const { return area_; }

  // Delegate iterators to container classes
  cpIter begin()  const { return p_.begin(); }
  cpIter end()    const { return p_.end();   }
  cpIter pBegin() const { return p_.begin(); }
  cpIter pEnd()   const { return p_.end();   }
  cnIter nBegin() const { return n_.begin(); }
  cnIter nEnd()   const { return n_.end();   }
  cfIter fBegin() const { return f_.begin(); }
  cfIter fEnd()   const { return f_.end();   }
  ccIter cBegin() const { return c_.begin(); }
  ccIter cEnd()   const { return c_.end();   }

 private:
  geo::VecVec3f p_, n_;
  VecFaceHandle f_;
  VecColor c_;
  float area_;
};

}  // namespace mesh
}  // namespace sg


