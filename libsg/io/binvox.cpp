#include "common.h"  // NOLINT

#include "io/binvox.h"
#include "io/io.h"
#include "geo/gridutils.h"
#include "util/util.h"

#include <string>
#include <fstream>
#include <iostream>

namespace sg {
namespace io {

typedef unsigned char byte;

//! Read a file in the binvox format (see http://www.cs.princeton.edu/~min/binvox/binvox.html)
int read_binvox(const string& file, ml::BinaryGrid3* grid,
                ml::mat4f* toVoxel /* = nullptr */,
                ml::vec3f* normTrans /*= nullptr */,
                float* normScale /* = nullptr */ ) {
  ifstream input(file, std::ios::in | std::ios::binary);

  //
  // read header
  //
  string line;
  input >> line;  // #binvox
  if (line.compare("#binvox") != 0) {
    SG_LOG_ERROR << "Error: first line reads [" << line << "] instead of [#binvox]";
    return 0;
  }
  int version;
  input >> version;
  SG_LOG_INFO << "reading " << file << " with binvox version " << version;

  int depth, height, width;
  float tx, ty, tz;
  float scale;
  depth = -1;
  int done = 0;
  while (input.good() && !done) {
    input >> line;
    if (line.compare("data") == 0) {
      done = 1;
    } else if (line.compare("dim") == 0) {
      input >> depth >> height >> width;
    } else if (line.compare("translate") == 0) {
      input >> tx >> ty >> tz;
    } else if (line.compare("scale") == 0) {
      input >> scale;
    } else {
      SG_LOG_ERROR << "  unrecognized keyword [" << line << "], skipping";
      char c;
      do {  // skip until end of line
        c = input.get();
      } while (input.good() && (c != '\n'));
    }
  }
  if (!done) {
    SG_LOG_ERROR << "  error reading header";
    return 0;
  }
  if (depth == -1) {
    SG_LOG_ERROR << "  missing dimensions in header";
    return 0;
  }
  //if (width != depth || height != depth || width != height) {
  //  SG_LOG_ERROR << " width/depth/height should all be the same";
  //  return 0;
  //}

  grid->allocate(depth, width, height);
  grid->clearVoxels();

  if (normScale != nullptr) {
    (*normScale) =  scale;
  }

  if (normTrans != nullptr) {
    normTrans->x = tx;
    normTrans->y = ty;
    normTrans->z = tz;
  }

  // Translation and scale specifies transform to take model space to
  // 1x1x1 cube with center at 0,0,0
  // binvox translate to 0,0,0 and then multiplies by the scale
  // to go back to original mesh space, scale and then translate
  // Translate*Scale
  if (toVoxel != nullptr) {
    ml::vec3f minMeshBB(-0.5f, -0.5f, -0.5f);
    minMeshBB *= 1.0f / scale;
    minMeshBB += ml::vec3f(tx, ty, tz);

    int maxDim = std::max(depth, std::max(width, height));
    ml::vec3f toVoxelTranslation = minMeshBB * (-maxDim * scale);
    // Set transform
    toVoxel->setScale(maxDim * scale); // set to scale matrix
    toVoxel->setTranslationVector(toVoxelTranslation);  // set just translation vector
  }

  //
  // read voxel data
  //
  byte value;
  byte count;
  size_t index = 0;
  size_t end_index = 0;
  size_t nr_voxels = 0;

  input.unsetf(std::ios::skipws);  // need to read every byte now (!)
  input >> value;  // read the linefeed char

  size_t x, y, z, r;
  size_t wxh = width * height;
  size_t size = width * height * depth;
  while ((end_index < size) && input.good()) {
    input >> value >> count;

    if (input.good()) {
      end_index = index + count;
      if (end_index > size) { return 0; }
      if (value) {
        for (size_t i = index; i < end_index; i++) {
          x = i / wxh;
          r = i % wxh;
          z = r / width;
          y = r % width;
          grid->setVoxel(x, y, z);
          //voxels[i] = value;
        }
      }

      if (value) { nr_voxels += count; }
      index = end_index;
    }  // if file still ok
  }  // while

  input.close();
  SG_LOG_INFO << "  read " << nr_voxels << " voxels";

  return 1;
}

int write_binvox(const ml::BinaryGrid3& grid, const string& file,
                 const ml::vec3f* normTrans /*= nullptr*/,
                 const float* normScale /*= nullptr*/) {
  io::ensurePathToFileExists(file);
  ofstream out(file, std::ios::out | std::ios::binary);
  SG_LOG_INFO << "Writing binvox file to " << file;

  const ml::vec3f& trans = normTrans ? *normTrans : ml::vec3f(0, 0, 0);
  const float scale = normScale ? *normScale : 1.f;
  SG_LOG_INFO << "scale=" << scale << ", trans=" << trans;

  const size_t depth   = grid.getDimX();
  const size_t width   = grid.getDimY();
  const size_t height  = grid.getDimZ();
  const size_t wxh     = width * height;
  const size_t size    = wxh * depth;

  //
  // write header
  //
  out << "#binvox 1" << endl;
  out << "dim " << depth << " " << height << " " << width << endl;
  out << "translate " << trans.x << " " << trans.y << " " << trans.z << endl;
  out << "scale " << scale << endl;
  out << "data" << endl;

  // Helper to get voxel value at index as byte
  const std::function<byte(const size_t)> voxelAsByte = [&] (const size_t i) {
    const size_t x = i / wxh;
    const size_t r = i % wxh;
    const size_t z = r / width;
    const size_t y = r % width;
    const bool isSet = grid.isValidCoordinate(x, y, z) && grid.isVoxelSet(x, y, z);
    return static_cast<byte>(isSet);
  };

  //
  // write voxel data
  //
  size_t bytes_written = 0;
  size_t total_ones = 0;
  size_t index = 0;
  while (index < size) {
    byte value = voxelAsByte(index);
    byte count = 0;
    while ((index < size) && (count < 255) && (value == voxelAsByte(index))) {
      index++;
      count++;
    }
    //    value = 1 - value;
    if (value) { total_ones += count; }

    out << value << count;  // inverted...
    bytes_written += 2;
  }  // while

  out.close();

  SG_LOG_INFO << "Wrote " << total_ones << " set voxels out of " << size << ", in "
              << bytes_written << " bytes";
  return 1;
}

//! Combines two binvoxes into one!
int merge_binvox(const string& file1, const string& file2, const string& outfile) {
  ml::BinaryGrid3 grid1;
  ml::BinaryGrid3 grid2;
  ml::vec3f normTrans1;
  ml::vec3f normTrans2;
  float normScale1;
  float normScale2;
  int rv;
  rv = sg::io::read_binvox(file1, &grid1, nullptr, &normTrans1, &normScale1);
  if (!rv) {
    SG_LOG_ERROR << "Error reading voxels from " << file1;
    return rv;
  }
  rv = sg::io::read_binvox(file2, &grid2, nullptr, &normTrans2, &normScale2);
  if (!rv) {
    SG_LOG_ERROR << "Error reading voxels from " << file2;
    return rv;
  }

  ml::BinaryGrid3 combined(grid1);
  size_t added;
  bool ok = sg::geo::addBinaryGrid(grid2, &combined, &added);
  if (!ok) {
    SG_LOG_ERROR << "Error combining grids from " << file1 << " and " << file2;
    return 0;
  }
  
  SG_LOG_INFO << "Added " << added << " voxels";
  rv = sg::io::write_binvox(combined, outfile, &normTrans1, &normScale1);
  if (!rv) {
    SG_LOG_ERROR << "Error writing output file " << outfile;
    return rv;
  }

  return 1;
}

int merge_binvox_dir(const string& dir1, const string& dir2, const string& outdir)
{
  vec<string> files1 = listFilesWithSuffix(dir1, ".binvox", true);
  vec<vec<string>> inputOutputFiles;
  inputOutputFiles.reserve(files1.size());
  const string dir1path = io::getAbsolutePath(dir1);
  for (const string& f : files1) {
    string relpath = f;
    if (util::startsWith(relpath, dir1)) {
      relpath = relpath.replace(0, dir1.length(), "");
    }
    const string f2 = dir2 + relpath;
    const string f3 = outdir + relpath;
    inputOutputFiles.push_back({f,f2,f3}); 
  }
  return merge_binvox_batch(inputOutputFiles);
}

int merge_binvox_dir(const string& dir, const string& file1, const string& file2, const string& outfile)
{
  vec<string> files1 = listFilesWithName(dir, file1, true);
  vec<vec<string>> inputOutputFiles;
  inputOutputFiles.reserve(files1.size());
  for (const string& f : files1) {
    string subdir = io::parentPath(f) + "/";
    const string f2 = subdir + file2;
    const string f3 = subdir + outfile;
    inputOutputFiles.push_back({f,f2,f3}); 
  }
  return merge_binvox_batch(inputOutputFiles);
}

//! Batch merge of binvox (input file with list of file1,file2,output) 
int merge_binvox_batch(const string& file) {
  const auto lines = io::getTokenizedLines(file, ",");
  return merge_binvox_batch(lines, file);
}
int merge_binvox_batch(const vec<vec<string>>& lines, const string& filename) {
  int ok = 0;
  int err = 0;
  int total = 0;
  for (const auto& line : lines) {
    total++;
    if (line.size() >= 3) {
      int rv = merge_binvox(line[0], line[1], line[2]);
      if (rv) { ok++; } else { err++; }
    } else {
      SG_LOG_ERROR << "Not enough fields(" << filename << ":" << total + ")"; 
      err++;
    }
    if (total % 200 == 0) {
      SG_LOG_INFO << "Processed ok=" << ok << ", err=" << err << ", total=" << total << "/" << lines.size();
    }
  }
  SG_LOG_INFO << "Processed ok=" << ok << ", err=" << err << ", total=" << total;
  return ok;
}

}  // namespace io
}  // namespace sg
