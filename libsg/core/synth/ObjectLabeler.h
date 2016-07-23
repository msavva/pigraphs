#pragma once

#include "libsg.h"  // NOLINT
#include "interaction/InteractionSet.h"
#include "core/LabeledGrid.h"
#include "stats/Counter.h"

namespace sg {
namespace core {
namespace synth {

using segmentation::ConstSegPtr;
using segmentation::SegmentGroups;

//! Options for labeling objects
enum LabelStrategy {
  kLabelsAnnotation = 0,             //! Label based on annotated segments
  kLabelsRecording = 1,              //! Label using current recording for a scene
  kLabelsRecordingsAll = 2,          //! Label using all recordings for a scene
  kLabelsSkeleton = 3,               //! Label using the current skeleton
  kLabelsModels = 4,                 //! Label using placed models
  kLabelsPredict = 5,                //! Label by predicting likely actions and locations
  kLabelStrategyCount = 6
};
static const arr<string, kLabelStrategyCount> LabelStrategyNames = { "Annotation", "Recording", "RecordingsAll", "Skeleton", "Models", "Predict" };

enum LabelType {
  kLabelTypeCategory = 0,        //! Label using category
  kLabelTypePart = 1,            //! Label using part
  kLabelTypeObjectId = 2,        //! Label using object id
  kLabelTypeCount = 3
};
static const arr<string, kLabelTypeCount> LabelTypeNames = { "Category", "Part", "ObjectId" };

struct LabelOpts {
  bool includeUnknownOccupancy;  //! Whether to include voxels whose occupancy is unknown
  bool includeUnlabeled;         //! Whether to included unlabeled voxels
  LabelType labelType;           //! Whether to use part, category, or object id labels
  LabelStrategy labelStrategy;   //! What labeling strategy to use
  interaction::InteractionSetType interactionSetType;  //! What interaction set type to use when predicting labels
  string interactionSetId;       //! What interaction set to use when predicting labels
  std::shared_ptr<SkeletonPoserParams> pPoserParams;  //! Skeleton poser params (required if predicting actions/locations)

  LabelOpts() : includeUnknownOccupancy(false)
    , includeUnlabeled(false)
    , labelType(kLabelTypeCategory)
    , labelStrategy(kLabelsAnnotation)
    , interactionSetType(interaction::ISetType_Composite) { }
};

//! Identifies and labels objects in a scene
class ObjectLabeler {
 public:
  typedef map<string, segmentation::VecConstSegPtr> LabelToSegMapT;
  typedef map<ConstSegPtr, stats::CounterSf> SegLabelScoresT;

  //! Initializes this object labeler
  void init(Database* db);

  //! Takes a map of labeled object segments (cat:part)
  //! and convert it to a map of categorized object segments (modelDbCat)
  LabelToSegMapT convertToCategories(const LabelToSegMapT& segsByLabel) const;

  //! Predict interaction labels for each joint group given a PIG
  vec<pair<string, double>> ObjectLabeler::predictInteractingLabels(
    const interaction::ProtoInteractionGraph& pig,
    vec<stats::Counter<string,double>>* pLabelProbs = nullptr) const;

  //! Label object segments using annotations
  LabelToSegMapT labelFromAnnotation(const Scan& scan, const segmentation::VecConstSegPtr segs) const;

  //! Label object segments using PIG similarity
  LabelToSegMapT labelWithPIG(const Scan& scan,
                              const Skeleton& skel,
                              const Skeleton::SegmentsByJointPlusGaze& segsByJoint,
                              const interaction::ProtoInteractionGraph& pig,
                              SegLabelScoresT* pSegLabelScores = nullptr) const;

  //! Label object segments using IG similarity
  LabelToSegMapT labelWithIG(const Scan& scan,
                             const Skeleton& skel,
                             const Skeleton::SegmentsByJointPlusGaze& segsByJoint,
                             const interaction::InteractionGraph& ig,
                             SegLabelScoresT* pSegLabelScores = nullptr) const;

  //! Label object segments using a PIG and SkelRange
  LabelToSegMapT labelWithPIG(const Scan& scan, const SkelRange& skelRange,
                              const interaction::ProtoInteractionGraph& PIG) const;

  //! Label object segments using recordings
  void label(const Scan& scene, const vec<Recording*> recordings, SegmentGroups* pSegGroups) const;

  //! Label object segments using one skeleton
  void ObjectLabeler::label(const Scan& scan, const Skeleton& skel,
                            const interaction::ProtoInteractionGraph& pig,
                            SegmentGroups* pSegGroups) const;

  //! Label object segments using posed skeletons
  void ObjectLabeler::label(const Scan& scan, const vec<TransformedSkeleton>& posedSkels,
                            const interaction::ProtoInteractionGraph& pig,
                            SegmentGroups* pSegGroups) const;
                          
  //! Label voxels in scan based on labelOpts
  void labelVoxels(const Scan& scan, const Skeleton* pSkel,
                   Recording* currRecording, const LabelOpts& labelOpts,
                   LabeledGrid* pLabeledGrid) const;

  //! Label voxels in scan based on the placed model instances
  void labelVoxels(const Scan& scan, const vec<ModelInstance>& modelInsts,
                   const LabelType labelType,
                   LabeledGrid* pLabeledGrid) const;

  //! Returns category from model database given annotation label
  string getCategory(const string& label, bool makeUpperIfUnknown = false) const;

  //! Returns annotation noun given model category
  string getAnnotationNoun(const string& category) const;
  //! Returns annotation noun given multiple model categories
  string getAnnotationNoun(const vec<string>& categories) const;

  //! Returns category index
  util::Index<string> getCategoryIndex() const {
    return m_categoryIndex;
  }

  //! Returns set of known categories
  set<string> getCategories() const {
    vec<string> v = m_categoryIndex.labels();
    return set<string>(v.begin(), v.end());
  }

  void dumpJointCategoryProbs(const interaction::ProtoInteractionGraph& pig,
                              const string& filename) const;
  void dumpJointLabelProbs(const interaction::ProtoInteractionGraph& pig,
                           const string& filename) const;

  static string labelToAnnotationNoun(const string& label) {
    vec<string> parts = ml::util::split(label, ":");
    return parts[0];
  }

 private:
  //! Helper functions for labeling voxels
  //! Label voxels in scan using recordings
  void labelVoxelsFromRecordings(const Scan& scan, const vec<Recording*> recordings,
                                 const LabelOpts& labelOpts, LabeledGrid* pLabeledGrid) const;

  //! Label voxels in scan using one skeleton
  void labelVoxelsFromSkeleton(const Scan& scan, const Skeleton& skel,
                               const interaction::ProtoInteractionGraph& pig,
                               const LabelOpts& labelOpts, LabeledGrid* pLabeledGrid) const;

  //! Label voxels in scan by predicting likely interactions and then labeling 
  void predictInteractionsAndLabelVoxels(const Scan& scan, const LabelOpts& labelOpts, LabeledGrid* pLabeledGrid) const;

  //! Label voxels in scan using annotations
  void labelVoxelsFromAnnotation(const Scan& scan, const SegmentGroups& segGroups,
                                 const LabelOpts& labelOpts, LabeledGrid* pLabeledGrid) const;

  //! Label voxels by their segment group index
  void labelVoxelsBySegmentGroups(const Scan& scan, const SegmentGroups& segGroups,
                                  const bool includeUnknown, ml::SparseGrid3<int>* pGrid) const;

  //! Add unlabeled occupied voxel to the labeled grid
  void addUnlabeledOccupied(const Scan& scene, LabeledGrid* pLabeledGrid) const;

  //! Helper function to take aggregated scores for each segment
  //! and do final assignment of labels
  void scoresToLabels(const Scan& scan,
                      const SegLabelScoresT& aggregatedSegLabelScores,
                      SegmentGroups* pSegGroups) const;

  //! Database from which we get scan recordings and such
  Database* m_pDatabase = nullptr;
  //! Object category index
  util::Index<string> m_categoryIndex;
  //! Map that goes from annotation categories to model categories
  map<string, string> m_annoToModelCatMap;
  //! Map that goes from model categories to annotation categories
  map<string, string> m_modelToAnnoCatMap;
};

ostream& operator<<(ostream& os, const LabelOpts& opts);

}  // namespace synth
}  // namespace core
}  // namespace sg


