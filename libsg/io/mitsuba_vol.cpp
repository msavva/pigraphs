#include "common.h"  // NOLINT

#include "io/mitsuba_vol.h"

#include "interaction/InteractFrameSurfSampled.h"
#include "math/math.h"
#include "vis/vis.h"

namespace sg {
namespace io {

struct Header {
  char magic[3];
  int8_t version = 3;
  int32_t type = 1;  /*encoding=float32*/
  int32_t dimX, dimY, dimZ, dimC;
  float minX, minY, minZ, maxX, maxY, maxZ;
};

using interaction::InteractionFrame;

int write_mitsuba_vol(const InteractionFrame& iframe,
                      const vis::IFVisParams& p,
                      const string& file) {
  SG_LOG_INFO << "Writing mitsuba volume file to " << file;

  const int binsPerDim = iframe.getNumBinsPerDim();
  const int dimX = binsPerDim, dimY = binsPerDim, dimZ = binsPerDim, dimCdensity = 1;
  const auto& bbox = iframe.getWorldOBB().toAABB();
  const auto& bbmin = bbox.min();
  const auto& bbmax = bbox.max();

  Header h;
  h.magic[0] = 'V';  h.magic[1] = 'O';  h.magic[2] = 'L';
  h.dimX = dimX;  h.dimY = dimY;  h.dimZ = dimZ;
  h.minX = bbmin.x();  h.minY = bbmin.y();  h.minZ = bbmin.z();
  h.maxX = bbmax.x();  h.maxY = bbmax.y();  h.maxZ = bbmax.z();

  InteractionFrame::ColoredBinGrid bins;
  iframe.getWeightedColorBins(p, &bins);
  const int numBins = dimZ * dimX * dimY;
  vec<float> density(numBins);
  vec<float> albedo(numBins * 3);
  fill(density.begin(), density.end(), 0.f);
  fill(albedo.begin(), albedo.end(), 0.f);
  double totalDensity = 0;
  int n = 0;
  for (int x = 0; x < dimX; ++x) {
    for (int y = 0; y < dimY; ++y) {
      for (int z = 0; z < dimZ; ++z) {
        const int idx = ((z*dimY + y)*dimX + x);
        const ml::vec3i coords = iframe.binIndexToCoords(x*(dimY*dimZ) + y*dimZ + z);
        if (!bins.exists(coords)) { continue; }
        const auto& vals = bins[coords];
        const auto maxVal = *max_element(vals.begin(), vals.end(), [] (const InteractionFrame::ColoredBin& l,
          const InteractionFrame::ColoredBin& r) { return l.val.w < r.val.w; });
        for (int c = 0; c < 3; ++c) {
          albedo[idx*3 + c] = math::clamp(maxVal.val[c], 0.f, 1.f);
        }
        totalDensity += maxVal.val.w;
        density[idx] = maxVal.val.w;
        n++;
      }
    }
  }
  SG_LOG_INFO << n;
  SG_LOG_INFO << totalDensity;

  // format is little endian (most architectures are little endian so okay)
  ofstream od(file + ".density.vol", std::ios::out | std::ios::binary);
  h.dimC = 1;
  od.write(reinterpret_cast<const char*>(&h), sizeof(h));
  od.write(reinterpret_cast<const char*>(&density[0]), numBins * sizeof(float));
  od.close();

  h.dimC = 3;
  ofstream oa(file + ".albedo.vol", std::ios::out | std::ios::binary);
  oa.write(reinterpret_cast<const char*>(&h), sizeof(h));
  oa.write(reinterpret_cast<const char*>(&albedo[0]), numBins * sizeof(float) * 3);
  oa.close();

  return 1;
}

}  // namespace io
}  // namespace sg
