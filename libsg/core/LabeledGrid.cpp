#include "common.h"  // NOLINT

#include "core/LabeledGrid.h"

#include "io/io.h"
#include "stats/Counter.h"

namespace sg {
namespace core {

struct LabeledVoxel {
  int16_t x, y, z, label;
};

//! Read an occupancy grid file. File starts with ASCII header of following format (excluding comments after #):
//!   labeledgrid\t1
//!   dimensions\tdimX dimY dimZ  # vec3ul
//!   worldToGrid\tfloat00 float01 float02 floa03 float10 ... float33  # mat4f
//!   voxelSize\tfloat
//!   labels\tId1,Id2,Id3,...
//!   numVoxels\tsize_t
//!   data:
//! Binary section follows with 4-tuples of int16_t (x, y, z, label)
bool LabeledGrid::load(const string& file) {
  if (!io::fileExists(file)) { return false; }

  const auto readerFun = [&] (istream& is) {
    // Parse header
    string symbol, line;
    uint32_t dimX, dimY, dimZ, numVoxels;
    getline(is, line);
    if (line.compare("labeledgrid\t1") != 0) {
      SG_LOG_ERROR << "Unknown VoxelGrid format. "
        << "First line should be 'labeledgrid\t1'. "
        << "Line is: " << line << endl;
    }

    while (line.compare("data:") != 0) {
      getline(is, line);
      std::stringstream ss(line);
      ss >> symbol;
      if (symbol == "dimensions") {
        ss >> dimX >> dimY >> dimZ;
        m_gridDims.x = dimX;  m_gridDims.y = dimY;  m_gridDims.z = dimZ;
      } else if (symbol == "worldToGrid") {
        for (int i = 0; i < 16; ++i) {
          ss >> m_world2grid.matrix[i];
        }
      } else if (symbol == "voxelSize") {
        ss >> m_voxelSize;
      } else if (symbol == "numVoxels") {
        ss >> numVoxels;
      } else if (symbol == "labels") {
        string labelsString;
        ss >> labelsString;
        const vec<string> labels = ml::util::split(labelsString, ',');
        for (const string& l : labels) {
          m_labelIndex.add(l);
//          if (l == "numVoxels") {
//            SG_LOG_INFO << "Found numVoxels in " << util::join(labels);
//          }
        }
      }
    }

    // Allocate and read grid binary section
    vec<LabeledVoxel> voxels(numVoxels);
    is.read(reinterpret_cast<char*>(&voxels[0]), sizeof(LabeledVoxel) * numVoxels);
    for (const LabeledVoxel& v : voxels) {
      m_labeledGrid(v.x, v.y, v.z) = v.label;
    }
  };

  io::readFile(file, readerFun, std::ios::binary);
  SG_LOG_INFO << "[LabeledGrid] Loaded from " << file;
  return true;
}

bool LabeledGrid::save(const string& filename) const {
  ofstream ofs(filename, std::ios::binary);
  ofs << "labeledgrid\t" << kFormatVersion << endl;
  ofs << "dimensions\t" << m_gridDims.x << " " << m_gridDims.y << " " << m_gridDims.z << endl;
  ofs << "worldToGrid\t";
  for (int i = 0; i < 15; ++i) {
    ofs << m_world2grid.getData()[i] << " ";
  }
  ofs << m_world2grid.getData()[15] << endl;
  ofs << "voxelSize\t" << m_voxelSize << endl;
  ofs << "labels\t" << util::join(m_labelIndex.labels(), ",") << endl;
  const size_t numVoxels = m_labeledGrid.size();
  ofs << "numVoxels\t" << numVoxels << endl;
  ofs << "data:" << endl;

  vec<LabeledVoxel> voxels(numVoxels);
  int i = 0;
  for (const auto& pair : m_labeledGrid) {
    const ml::vec3i& coords = pair.first;
    const int label = pair.second;
    voxels[i] = { coords.x, coords.y, coords.z, label };
    ++i;
  }

  ofs.write(reinterpret_cast<const char*>(&voxels[0]), sizeof(LabeledVoxel) * numVoxels);

  ofs.close();
  SG_LOG_INFO << "[LabeledGrid] Saved to " << filename;
  return true;
}

set<string> getAllLabels(const string& dir) {
  set<string> labels;
  for (const string& path : io::listFilesWithSuffix(dir, ".vox", true)) {
    const LabeledGrid grid(path);
    for(const string& l : grid.getLabelIndex().labels()) {
      labels.insert(l);
    }
  }
  return labels;
}

LabeledGridComparison::LabeledGridComparison(const string& predictedDir, const string& truthDir) {
  labels.add(CATEGORY_NONE);  // always index 0
  for (const string& l : getAllLabels(truthDir)) {
    labels.add(l);
  }
  for (const string& l : getAllLabels(predictedDir)) {
    labels.add(l);
  }
  confusionMatrix.init(labels);
}

void compare(const LabeledGrid& predicted, const LabeledGrid& groundTruth,
             LabeledGridComparison* pComp) {
  // check dims match
  const ml::vec3ui dimsG = groundTruth.gridDims();
  const ml::vec3ui dimsP = predicted.gridDims();
  if (dimsG != dimsP) {
    SG_LOG_ERROR << "Cannot compare LabeledGrids with unequal dimensions: " << dimsG << " vs " << dimsP;
    return;
  }

  // initialize counters
  stats::Counter<string, int> TP;
  stats::Counter<string, int> numPredicted;  // TP + FP
  stats::Counter<string, int> numRelevant;   // TP + FN
  size_t numNoneInGroundTruth = 0;
  size_t numNoneInPredicted = 0;

  // go over ground truth
  for (const auto& p : groundTruth.getGrid()) {
    const ml::vec3i& position = p.first;
    const string predictedLabel = predicted.getLabel(position);
    const string groundTruthLabel = groundTruth.getLabel(position);

    if (groundTruthLabel == CATEGORY_NONE) { numNoneInGroundTruth++; }  // just for sanity checking
    if (predictedLabel == CATEGORY_NONE) { numNoneInPredicted++; }  // just for sanity checking
    pComp->confusionMatrix.inc(groundTruthLabel, predictedLabel);
    numRelevant.inc(groundTruthLabel);  // relevant since in groundTruth
    if (predictedLabel == groundTruthLabel) { TP.inc(groundTruthLabel); }  // TP
  }

  // go over predicted
  for (const auto& p : predicted.getGrid()) {
    const ml::vec3i& position = p.first;
    const string predictedLabel = predicted.getLabel(position);
    const string groundTruthLabel = groundTruth.getLabel(position);

    if (!groundTruth.contains(position)) {  // not accounted in previous loop
      pComp->confusionMatrix.inc(groundTruthLabel, predictedLabel);
    }

    numPredicted.inc(predictedLabel);  // this was a prediction
    // TP match accounted for in previous loop
  }

  // stuff results into BinaryConfusionMatrixSet
  const int numElements = static_cast<int>(dimsG.x * dimsG.y * dimsG.z);
  for (const string& label : pComp->labels.labels()) {
    const int tp = TP.count(label);
    const int fp = numPredicted.count(label) - tp;
    const int fn = numRelevant.count(label) - tp;
    const int tn = numElements - tp - fn - fp;
    pComp->binaryConfusionPerLabel[label].addTP(tp);
    pComp->binaryConfusionPerLabel[label].addTN(tn);
    pComp->binaryConfusionPerLabel[label].addFP(fp);
    pComp->binaryConfusionPerLabel[label].addFN(fn);  // Overcounts if predicted was not this label
  }

  if (numNoneInGroundTruth) {
    SG_LOG_ERROR << "CATEGORY_NONE in ground truth: " << numNoneInGroundTruth << ", computation unreliable";
  }
  if (numNoneInPredicted) {
    SG_LOG_INFO << "CATEGORY_NONE in predicted: " << numNoneInPredicted;
  }
}

LabeledGridComparison compare(const string& predictedDir, const string& groundTruthDir) {
  LabeledGridComparison comp(predictedDir, groundTruthDir);
  vec<string> pFiles = io::listFilesWithSuffix(predictedDir, ".vox", true);
  for (const string& pFile : pFiles) {
    const string id = io::basename(pFile);
    const string gFile = groundTruthDir + "/" + io::filename(pFile);
    if (!io::fileExists(gFile)) {
      SG_LOG_WARN << "Ground truth for " << pFile << " not found at " << gFile << ". Skipping...";
      continue;
    }
    const LabeledGrid P(pFile);
    const LabeledGrid G(gFile);
    compare(P, G, &comp);
  }
  const string setId = io::filename(predictedDir);
  ofstream ofs1(setId + "_aprfs.csv");
  ofs1 << "category,accuracy,precision,recall,f1" << endl;
  ofs1 << "microAverage," << microAverage(comp.binaryConfusionPerLabel) << endl;
  double ACC = 0, P = 0, R = 0, F1 = 0;
  int N = 0;
  for (const auto& pair : comp.binaryConfusionPerLabel) {
    const string& cat = pair.first;
    if (cat == CATEGORY_NONE) { continue; }  // skip NONEs

    const eval::BinaryConfusionMatrix& m = pair.second;
    ACC += m.accuracy();  P += m.precision();  R += m.recall();  F1 += m.f1();
    N++;
    ofs1 << cat << "," << m << endl;
  }
  ofs1 << "macroAverage," << ACC/N << "," << P/N << "," << R/N << "," << F1/N << endl;
  ofs1.close();
  ofstream ofs2(setId + "_confusion-matrix.csv");
  ofs2 << "gold,predicted,count" << endl;
  toDelimited(ofs2, comp.confusionMatrix);
  ofs2.close();
  return comp;
}

}  // namespace core
}  // namespace sg
