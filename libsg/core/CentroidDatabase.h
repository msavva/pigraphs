#pragma once

#include <mLibCore.h>

#include "libsg.h"  // NOLINT
#include "io/io.h"
#include "stats/centroidset.h"

namespace sg {
namespace core {

class CentroidDatabase : public io::Serializable {
 public:
  BOOST_BINARY_SERIALIZABLE_FUNCS
  //! Creates centroids by running K-means on set of all active segments
  void create(const Database& database);

  //! Compute activation of segment features in segFeatures (row-major order)
  //! against current centroid set and return as row-major vector
  vecd computeCentroidActivations(const vecd& segFeats, size_t numSegs) const;

  //! Return CentroidSet for given jointGroupIdx or nullptr if none is available
  const stats::CentroidSet* getCentroidSetForJointGroup(size_t jointGroupIdx) const;

  //! Return CentroidSet over all joint groups or nullptr if none is available
  const stats::CentroidSet* getCentroidSetForAllJoints() const;

 private:
  //! boost serialization function
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive& ar, const unsigned int) {  // NOLINT
    ar & _centroidSets;
  }

  //! Map from string id of verb+jointGroupIndex to CentroidSet
  map<string, stats::CentroidSet> _centroidSets;
};

}  // namespace core
}  // namespace sg


