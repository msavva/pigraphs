#pragma once

#include <mLibCore.h>

#include <string>
#include <vector>
#include <set>

#include "../stats/stats.h"

namespace sg {

namespace segmentation { struct MeshSegment; }

namespace core {

class SceneDatabase;
struct Database;
struct VerbCluster;
class FeatureTransform;

struct Centroid
{
    // each centroid considers only features from a single joint
    UINT jointIndex;

    double activationSigma;

    // ID that will be spit out when fieldNames is called. Could just be an integer.
    std::string id;

    // transformedFeatures are the geometric features after FeatureTransform has been
    // applied to each dimensions.
    std::vector<double> transformedFeatures;

    void recordVerb(const VerbCluster *v, float weight);

    std::map<const std::string,float> verbWeights;

    //! boost serialization function
    template<class Archive>
    inline void serialize(Archive& ar, const unsigned int version) {
      ar & jointIndex & activationSigma & id & transformedFeatures & verbWeights;
    }
};

class CentroidDatabase {
public:
    bool save(const std::string &filename) const;
    bool load(const Database& database, const std::string &filename);
    void create(const Database &database);
    
    //! Creates centroids by running K-means on set of all active segments
    void createCentroidSetWithWekaKmeans(const Database& database, const size_t numCentroids);

    //! Compute activation of segment features in segFeatures (row-major order) against current centroid set and return as row-major vector
    std::vector<double> CentroidDatabase::computeCentroidActivations(const std::vector<double>& segFeats, const size_t numSegs) const;
    
    //! Return CentroidSet
    const sg::stats::CentroidSet& centroidSet() const { return _centroidSet; }

    //! Loads a set of segment centroids stored as rows in csvFile and returns them
    static std::vector<std::vector<double>> loadSegmentCentroids(const std::string& csvFile);
    
    double dist(const Centroid &c, const sg::segmentation::MeshSegment &seg) const;
    static double dist(const std::vector<double> &transformedFeaturesA, const std::vector<double> &transformedFeaturesB);
    void computeSegmentFeatures(const sg::segmentation::MeshSegment& seg) const;

    const std::vector< Centroid >& centroids() const
    {
        return _centroids;
    }

    const std::vector< Centroid >& filteredCentroids(const VerbCluster &v) const;

    //! boost serialization function
    template<class Archive>
    inline void serialize(Archive& ar, const unsigned int version) {
      ar & _centroids & _filteredCentroids & _centroidSet;
    }

private:
    static std::vector<sg::core::FeatureTransform*> computeTransforms(const sg::util::Params& params); //const std::vector< ml::MathVector<double> > &segments);
    //void computeTransforms(const SceneDatabase &scenes);
    std::vector< ml::MathVector<double> > makeTransformedFeatureVector(const Database &database, const std::set<const sg::segmentation::MeshSegment*> &segs);
    std::set<const sg::segmentation::MeshSegment*> accumulateJointSegments(const Database& database, UINT jointIndex);
    void computeClusterUsage();

    std::vector< Centroid > _centroids;
    std::map< const std::string, std::vector<Centroid> > _filteredCentroids;
    sg::stats::CentroidSet _centroidSet;
};

}  // namespace core
}  // namespace sg


