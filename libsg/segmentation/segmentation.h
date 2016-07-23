#pragma once

#include <mLibCore.h>

#include "libsg.h"  // NOLINT

#include "geo/geo.h"
#include "segmentation/Segmentator.h"

// Utilty functions for segmentation
namespace sg {
namespace segmentation {

//! Stores segment indices into a mesh for later retrieval
void storeSegmentIndices(ml::TriMeshf& mesh, const VecSegPtr& segments, const VecSegPtr& rejectedSegments);

//! Retrieves segment index to which the vertex with the given index belongs
int vIdxToSegIdx(const ml::TriMeshf& mesh, int vIdx);

//! Helper function to compute colors for current segmentation
//! @param[in] selectedSegIdx - index of selected segment
//! @param[out] colors - pointer to vector that holds colors
void getSegmentationColors(const ml::TriMeshf& mesh, int selectedSegIdx, vec<ml::vec4f>* colors);

//! Helper function to compute colors for current segmentation
//! @param[in] selectedSegIdx - vector of indices of selected segments
//! @param[out] colors - pointer to vector that holds colors
void getSegmentationColors(const ml::TriMeshf& mesh, vec<int> selectedSegIdx, vec<ml::vec4f>* colors);

//! Helper function to compute colors for current segmentation based on scores and a color gradient
//! @param[in] scores - scores corresponding to each segment
//! @param[in] colorGradient - colorGradient to use to convert scores to colors
//! @param[out] colors - pointer to vector that holds colors
void getSegmentationColors(const ml::TriMeshf& mesh, const vec<double>& scores,
                           const ml::ColorGradient& colorGradient, vec<ml::vec4f>* colors);


//! Output the results of a segmentation to a CSV file
void saveSegIndicesCsv(const string& file, const vec<SegIndices>& segIndices);
void saveSegIndicesCsv(const string& file, const SegIndices& segIndices);

//! Load a segmentation from a CSV file
bool loadSegIndicesCsv(const string& file, vec<SegIndices>* segIndices);

//! Compute OBB around segs
geo::OBB computeSegsOBB(const VecConstSegPtr& segs);

// Compute AABB around segs
geo::BBox computeSegsBBox(const VecConstSegPtr& segs);

}  // namespace segmentation
}  // namespace sg


