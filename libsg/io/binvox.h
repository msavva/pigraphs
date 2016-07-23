#pragma once

#include "libsg.h"  // NOLINT

namespace sg {
namespace io {

//! Read a file in the binvox format (see http://www.cs.princeton.edu/~min/binvox/binvox.html)
//! Sets model-to-voxel space transform in model2voxel.
//! Return 1 if success, 0 if error
int read_binvox(const string& file, ml::BinaryGrid3* grid,
                ml::mat4f* model2voxel = nullptr,
                ml::vec3f* normTrans = nullptr,
                float* normScale = nullptr);

//! Write a BinaryGrid to binvox format (see http://www.cs.princeton.edu/~min/binvox/binvox.html)
//! Records voxel-to-model normalization translation and scale if given in normTrans and normScale.
//! Return 1 if success, 0 if error
int write_binvox(const ml::BinaryGrid3& grid, const string& file,
                 const ml::vec3f* normTrans = nullptr,
                 const float* normScale = nullptr);

//! Combines two binvox files into one!
//! Return 1 if success, 0 if error
int merge_binvox(const string& file1, const string& file2, const string& outfile);
int merge_binvox_dir(const string& dir1, const string& dir2, const string& outdir);
int merge_binvox_dir(const string& dir, const string& file1, const string& file2, const string& outfile);

//! Batch merging of binvox
int merge_binvox_batch(const string& file);
int merge_binvox_batch(const vec<vec<string>>& mergeFiles, const string& filename = "??");
}  // namespace io
}  // namespace sg


