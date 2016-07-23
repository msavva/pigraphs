#pragma once

#include "jace/proxy/java/lang/String.h"
#include "jace/proxy/edu/stanford/graphics/wekautils/Classer.h"
#include "jace/proxy/edu/stanford/graphics/wekautils/Common.h"
#include "jace/proxy/edu/stanford/graphics/wekautils/Clustering.h"

namespace wekautil {

typedef jace::proxy::java::lang::String String;
typedef jace::proxy::edu::stanford::graphics::wekautils::Classer Classifier;
typedef jace::proxy::edu::stanford::graphics::wekautils::Common common;
typedef jace::proxy::edu::stanford::graphics::wekautils::Clustering Clustering;

//! Initializes JVM through jace interface (needs to be called before any other interface operations
int initJVM();

//! Checks exceptions running with the JVM and convert it into a error code
int handleJaceExceptions();

//! Returns classifier weights
bool getClassifierWeights(Classifier* pClassifier, std::vector<double>* pWeights);

//! Returns a flat array of numCentroids*numFeats after performing kmeans clustering on instances in csvFile
std::vector<double> getKmeansCentroids(const std::string& csvFile, const size_t numCentroids);

//! Same above except input is flat array of row-major instance features
std::vector<double> getKmeansCentroids(const std::vector<double>& rawFeats, const size_t numFeats, const size_t numInstances, const size_t numCentroids);

}  // namespace wekautil



