#pragma once

#include <array>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// Check MSVC version
#if defined(_MSC_VER) && (_MSC_FULL_VER < 180031101)
#error "MSVC compiler version >= 18.00.31101 (2013 Update 4) required"
#endif // _MSC_VER

namespace sg {

//
// Aliases for commonly used types
//
using std::cin; using std::cout; using std::cerr; using std::endl;
using std::istream; using std::ifstream; using std::ostream; using std::ofstream;
using std::map;
using std::pair;
using std::set;
using std::string;
using std::to_string;

template <typename T, size_t size> using arr = std::array<T, size>;
template <typename T> using vec = std::vector<T>;
template <typename T> using uset = std::unordered_set<T>;
template <typename K, typename V> using umap = std::unordered_map<K, V>;

typedef vec<float> vecf; typedef vec<double> vecd; typedef vec<int> veci;

//
// Forward declarations
//

namespace core {

class   CentroidDatabase;
class   ClassifierDatabase;
struct  Database;
class   InteractionGraphKNNSimilarity;
class   InteractingSkelFeatureGenerator;
class   ISetPoseController;
struct  Model;
class   ModelDatabase;
class   ModelInstance;
class   MotionDatabase;
class   OccupancyGrid;
struct  Recording;
class   RecordingDatabase;
class   Scan;
class   ScanDatabase;
class   Scene;
class   PoseController;
class   ScenePoseClassifier;
class   SceneSerialized;
class   SegmentCentroidActivationAggregatedClassifier;
class   SegmentClassifier;
class   SegmentFeatureGenerator;
class   SegmentJointsAggregatedClassifier;
class   SegmentPoseClassifier;
struct  Skeleton;
class   SkeletonDatabase;
class   SkelPoseSampler;
struct  SkelRange;
struct  SkelState;
struct  TransformedSkeleton;

namespace synth {

struct  AlignmentState;
struct  InteractionSynthParams;
class   InteractionSynth;
struct  LabelOpts;
struct  ModelAlignerParams;
class   ModelAligner;
struct  ModelPlacerParams;
class   ModelPlacer;
struct  ModelRetrieverParams;
class   ModelRetriever;
class   ObjectLabeler;
struct  SkeletonPoserParams;
class   SkeletonPoser;

}  // namespace synth

}  // namespace core

namespace interaction {

enum class InteractionFrameType;
class   InteractionFrame;
struct  Interaction;
class   InteractionDatabase;
class   InteractionFactory;
class   InteractionFrames;
class   InteractionGraph;
class   InteractionMap;
struct  InteractionSet;
struct  ModelAnnotation;
class   ProtoInteractionGraph;
struct  WeightedPIG;
class   SkeletonVolume;
struct  VerbNoun;
struct  VerbNounISetGroup;

}  // namespace interaction

namespace geo {

class   OBB;

} // namespace geo

namespace graph {

}  // namespace graph

namespace mesh {

struct  IntersectionRecord;
struct  MeshActivationRecord;
class   MeshHelper;

}  // namespace mesh

namespace segmentation {

struct  SegmentationParams;
class   SegmentGroup;
class   SegmentGroups;
class   SegmentGroupsRecord;

struct  MeshSegment;
typedef std::shared_ptr<MeshSegment> SegPtr;
typedef std::shared_ptr<const MeshSegment> ConstSegPtr;
typedef vec<SegPtr> VecSegPtr;
typedef vec<ConstSegPtr> VecConstSegPtr;

class   Part;
typedef std::shared_ptr<Part> PartPtr;
typedef std::shared_ptr<const Part> ConstPartPtr;
typedef vec<PartPtr> VecPartPtr;
typedef vec<ConstPartPtr> VecConstPartPtr;
enum    PartType;


}  // namespace segmentation

namespace stats {

class   LDSampler;

}  // namespace stats

namespace util {

template <typename T>
class   Grid;
struct  Params;
class   Timer;

}  // namespace util

namespace vis {

class   PoseHeatMap;
struct  VisImage;
class   VisLog;
struct  VisLogRecord;

}  // namespace vis

}  // namespace sg

// For printing enum classes
#define OSTREAMABLE(T) inline ostream& operator<<(ostream& os, const T& e) { return os << static_cast<std::underlying_type<T>::type>(e); }

/*  Safety Pig: Vanguard of LibSG Integrity
                                _
       _._ _..._ .-',     _.._(`))
      '-. `     '  /-._.-'    ',/
         )         \            '.
        / _    _    |             \
       |  a    a    /              |
       \   .-.                     ;
        '-('' ).-'       ,'       ;
           '-;           |      .'
              \           \    /
              | 7  .__  _.-\   \
              | |  |  ``/  /`  /
             /,_|  |   /,_/   /
                /,_/      '`-'
*/


