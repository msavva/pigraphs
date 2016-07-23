#pragma once

#include <mLibCore.h>

#include "libsg.h"  // NOLINT

namespace sg {
namespace core {

struct TrainOpts;

//! Dump training datasets indicated by ids in "Dataset.datasets" vector
void dumpDatasets(const Database& database);

//! Dump training datasets for per joint verb prediction from segment+joint features
void makeSegmentPerJointDataset(const Database& database, const TrainOpts& trainOpts, bool useJointFeatures);

//! Dump training datasets for verb prediction from scene+pose features
void makeScenePoseBoWDataset(const Database& database);

//! Dump features for all segment-pose observations into verb.csv files
void makeFeaturesFiles(const Database& database);

//! Dump training datasets for aggregated per-joint segment presence+features classifiers
void makeSegmentJointsAggregatedDataset(const Database& database, const TrainOpts& trainOpts);

//! Dump datasets using centroid activation features
void makeSegmentCentroidActivationDataset(const Database& database, const TrainOpts& trainOpts);

}  // namespace core
}  // namespace sg


