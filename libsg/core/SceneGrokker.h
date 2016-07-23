#pragma once

#include <mLibCore.h>

#include "libsg.h"  // NOLINT
#include "eval/ConfusionMatrix.h"

namespace sg {
namespace core {

class SceneGrokker {
 public:
  //! General initialization given database and specific scene
  SceneGrokker(const Database& _database, const Scan& _scene);

  //! Creates a "heatmap" of verb cluster poses being activated at points within the scan by aggregating likelihoods
  //! of each scan segment as judged by verb cluster classifier.
  void makeHeatMap(const ScenePoseClassifier& classifier,
                   const vec<TransformedSkeleton>& skels,
                   vis::PoseHeatMap* pHeatmap) const;

  static void computeEvaluationDataset(const ml::ColorImageR8G8B8A8& base, const ml::ColorImageR8G8B8A8& mask,
                                       const ml::ColorImageR8G8B8A8& test,
                                       const ml::ColorImageR8G8B8A8& truth, const string& filename,
                                       const string& visFilenameBase);
  static void computeEvaluationDataset(const string& dataDir, const string& sceneName);
  static void computeEvaluationDataset(const string& dataDir);
  static vec<eval::BinaryConfusionMatrix> computeConfusionMatrixSet(const string& dataDir, const string& sceneName,
                                                              const string& verbID, const ml::ColorImageR8G8B8A8& test,
                                                              double thresholdIncrement, const string& hybridImageSave);
  static void computeSceneFeatureDescriptors(const string& dataDir);
  static double computeSceneIntensity(const ml::ColorImageR8G8B8A8& bmp, double cutoff);
  static double computeSceneIntensity(const vis::PoseHeatMap& heatMap, double cutoff);
  static ml::ColorImageR8G8B8A8 makeMask(const ml::ColorImageR8G8B8A8& base, const ml::ColorImageR8G8B8A8& maskOverlay);

 private:
  const Database& database;
  const Scan& scan;
};

}  // namespace core
}  // namespace sg


