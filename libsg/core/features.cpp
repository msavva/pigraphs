#include "common.h"  // NOLINT

#include "core/features.h"

#include <Eigen/Eigenvalues>

#include <mLibCore.h>
#include "core/CentroidDatabase.h"
#include "core/Database.h"
#include "core/Scan.h"
#include "core/Skeleton.h"
#include "segmentation/MeshSegment.h"

namespace sg {
namespace core {

const arr<const string, SegmentFeatureGenerator::kNumFeatureTypes> 
  SegmentFeatureGenerator::kFeatureTypes = {
    "meshsimplistic", // 0
    "meshcentroid",   // 1
};

SegmentFeatureGenerator::FeatureType SegmentFeatureGenerator::getFeatureType(const string& featureTypeName) {
  int featureType = -1;
  for (size_t i = 0; i < kNumFeatureTypes; i++) {
    if (kFeatureTypes[i] == featureTypeName) {
      featureType = static_cast<int>(i);
    }
  }
  if (featureType >= 0) {
    return static_cast<FeatureType>(featureType);
  } else {
    const string message = "Invalid segmentPerJointFeatures " + featureTypeName;
    throw std::invalid_argument(message.c_str());
  }
}

static const int segmentFeatureJointCount = 14;
static const int segmentFeatureJoints[segmentFeatureJointCount] = {
  Skeleton::JointType_SpineBase,
  Skeleton::JointType_SpineMid,
  Skeleton::JointType_Neck,
  Skeleton::JointType_Head,
  Skeleton::JointType_ShoulderLeft,
  Skeleton::JointType_ElbowLeft,
  Skeleton::JointType_HandLeft,
  Skeleton::JointType_ShoulderRight,
  Skeleton::JointType_ElbowRight,
  Skeleton::JointType_HandRight,
  Skeleton::JointType_KneeLeft,
  Skeleton::JointType_FootLeft,
  Skeleton::JointType_KneeRight,
  Skeleton::JointType_FootRight
};

vec<string> SegmentSkelFeatureGeneratorBasic::fieldNames() const {
  vec<string> result;

  for (UINT jointIndex = 0; jointIndex < segmentFeatureJointCount; jointIndex++) {  
    result.push_back(Skeleton::jointName(segmentFeatureJoints[jointIndex]) + "_zDisp");
    result.push_back(Skeleton::jointName(segmentFeatureJoints[jointIndex]) + "_xyDist");
    result.push_back(Skeleton::jointName(segmentFeatureJoints[jointIndex]) + "_enclosed");
  }

  return result;
}

vecd SegmentSkelFeatureGeneratorBasic::generate(const SegmentSkeletonPair& pair) const {
  vecd result;

  //const Skeleton& skel = pair.second->getSkeleton();
  for (UINT jointIndex = 0; jointIndex < segmentFeatureJointCount; jointIndex++) {  
    auto v = pair.second->transformedJoint(segmentFeatureJoints[jointIndex]);
    //ml::vec3f centroid = pair.first->centroid<ml::vec3f>();

    auto closestPt = pair.first->closestPoint(v);
    result.push_back(v.z - closestPt.z);

    float xyDist = ml::vec3f::dist( ml::vec3f(v.x, v.y, 0.0f), ml::vec3f(closestPt.x, closestPt.y, 0.0f));
    if(xyDist < 0.01f) xyDist = 0.0f;
    result.push_back(xyDist);

    result.push_back((pair.first->distance(v) > 0) ? 0 : 1);
  }

  return result;
}

vec<string> SegmentJointFeatureGeneratorBasic::fieldNames() const {
  vec<string> result;

  result.push_back("joint_zDisp");
  result.push_back("joint_xyDist");
  result.push_back("joint_enclosed");

  return result;
}

vecd SegmentJointFeatureGeneratorBasic::generate(const SegmentSkeletonJoint& segJoint) const {
  vecd result;

  auto v = segJoint.skeleton.transformedJoint(segJoint.jointIndex);

  auto closestPt = segJoint.segment->closestPoint(v);
  result.push_back(v.z - closestPt.z);

  float xyDist = ml::vec3f::dist( ml::vec3f(v.x, v.y, 0.0f), ml::vec3f(closestPt.x, closestPt.y, 0.0f));
  if(xyDist < 0.01f) xyDist = 0.0f;
  result.push_back(xyDist);

  result.push_back((segJoint.segment->distance(v) > 0) ? 0 : 1);

  return result;
}

vec<string> SceneSkelJointAggrFeatureGeneratorBasic::fieldNames() const
{
    vec<string> result;
    for (int iJointGroup = 0; iJointGroup < Skeleton::kNumJointGroups; iJointGroup++) {
      const string jointName = Skeleton::jointGroupName(iJointGroup);
      if (m_useActivation) {
        result.push_back(jointName + "_active");
      }
      if (m_useJointSegScores) {
        result.push_back(jointName + "_score");
      }
      if (m_useJointSegFeatures) {

      }
    }
    return result;
}

pair<double, int> SceneSkelJointAggrFeatureGeneratorBasic::jointInteractionScore(const SceneSkel& interSkel, 
                                                                                 size_t iJointGroup) const {
  const Skeleton::SegmentsByJointPlusGaze& segsByJoints = *interSkel.activatedSegsByJoint;
  double scoreSum = 0;
  int scoreCount = 0;
  if (m_useJointSegFeatures) {
    // Use joint joint to mesh score
    if (m_interactionSet && m_interactionSet->segJointPerJointGroupClassifier[iJointGroup]) {
      const vec<size_t>& jointsForJointGroup = Skeleton::kJointGroupToJoints[iJointGroup];
      for (size_t iJoint:jointsForJointGroup) {
        for (const auto& seg : segsByJoints[iJoint]) {
          SegmentSkeletonJoint ssj = {seg.get(), *interSkel.tSkel, iJoint};
          const double score = m_interactionSet->segJointPerJointGroupClassifier[iJointGroup]->classify(ssj);
          scoreSum += score;
          scoreCount++;
        }
      }
    }
  } else {
    // Use just mesh score
    if (m_interactionSet && m_interactionSet->segPerJointGroupClassifier[iJointGroup]) {
      const vec<size_t>& jointsForJointGroup = Skeleton::kJointGroupToJoints[iJointGroup];
      for (size_t iJoint:jointsForJointGroup) {
        for (const auto& seg : segsByJoints[iJoint]) {
          const double score = m_interactionSet->segPerJointGroupClassifier[iJointGroup]->classify(*seg);
          scoreSum += score;
          scoreCount++;
        }
      }
    }
  }
  //if (scoreCount > 0) { scoreSum /= scoreCount; }
  return pair<double,int>(scoreSum, scoreCount);
}

vecd SceneSkelJointAggrFeatureGeneratorBasic::generate(const SceneSkel& interSkel) const
{
    vecd result;
    const Skeleton::SegmentsByJointGroup& segs = Skeleton::groupSegmentsByJointGroup(*interSkel.activatedSegsByJoint);
    
    for (size_t iJointGroup = 0; iJointGroup < Skeleton::kNumJointGroups; iJointGroup++) {
      if (m_useActivation) {
        const double active = (segs[iJointGroup].empty()) ? 0 : 1;  // Indicates presence of any segments
        result.push_back(active);
      }
      if (m_useJointSegScores) {
        pair<double,int> scoreAndCount = jointInteractionScore(interSkel, iJointGroup);
        result.push_back(scoreAndCount.first);
      }
    }
    return result;
}


vec<string> SceneSkelCentroidActivationFeatGen::fieldNames() const {
  vec<string> result;
  if (m_splitCentroidsPerJointGroup) {
    for (int iJointGroup = 0; iJointGroup < Skeleton::kNumJointGroups; iJointGroup++) {
      const stats::CentroidSet* centroids = m_centroidDb.getCentroidSetForJointGroup(iJointGroup);
      if (!centroids) { continue; }  // No fields if no centroids
      const string jointName = Skeleton::jointGroupName(iJointGroup);
      for (int iCentroid = 0; iCentroid < centroids->k; iCentroid++) {
        result.push_back(jointName + "_c" + to_string(iCentroid));
      }
    }
  } else {
    const stats::CentroidSet* centroids = m_centroidDb.getCentroidSetForAllJoints();
    if (centroids != nullptr) { 
      for (int iCentroid = 0; iCentroid < centroids->k; iCentroid++) {
        result.push_back("all_c" + to_string(iCentroid));
      }
    }
  }
  if (m_useActivation) {
    for (size_t iJointGroup = 0; iJointGroup < Skeleton::kNumJointGroups; iJointGroup++) {
      const string jointName = Skeleton::jointGroupName(iJointGroup);
      result.push_back(jointName + "_active");
    }
  }
  return result;
}

vecd SceneSkelCentroidActivationFeatGen::generate(const SceneSkel& sceneSkel) const {
  vecd result;  // summed centroid activations for all joints
  const Skeleton::SegmentsByJointGroup& segsByJointGroup = Skeleton::groupSegmentsByJointGroup(*sceneSkel.activatedSegsByJoint);
  if (m_splitCentroidsPerJointGroup) {
    for (size_t iJointGroup = 0; iJointGroup < Skeleton::kNumJointGroups; iJointGroup++) {
      const stats::CentroidSet* centroids = m_centroidDb.getCentroidSetForJointGroup(iJointGroup);
      if (!centroids) { continue; }

      const auto& segs = segsByJointGroup[iJointGroup];
      if (segs.empty()) {
        const vecd zeroActivation(centroids->k, 0);
        result.insert(result.end(), zeroActivation.begin(), zeroActivation.end());
      } else {
        vecd allSegFeats;
        for (const auto& seg : segs) {
          const vecd segFeats = m_segFeatGen.generate(*seg);
          allSegFeats.insert(allSegFeats.end(), segFeats.begin(), segFeats.end());
        }
        const vecd actSum
          = centroids->computeActivationSummed(allSegFeats, segs.size());  // [1 x k] summed activation for this joint
        result.insert(result.end(), actSum.begin(), actSum.end());
      }
    }
  } else {  // No splitting
    const stats::CentroidSet* centroids = m_centroidDb.getCentroidSetForAllJoints();
    if (centroids == nullptr) { return vecd(); }  // TODO: Make sure this is fine downstream
    
    vecd allSegFeats;
    size_t numSegs = 0;
    for (size_t iJointGroup = 0; iJointGroup < Skeleton::kNumJointGroups; iJointGroup++) {
      for (const auto& seg : segsByJointGroup[iJointGroup]) {
        const vecd segFeats = m_segFeatGen.generate(*seg);
        allSegFeats.insert(allSegFeats.end(), segFeats.begin(), segFeats.end());
        numSegs++;
      }
    }
    result = centroids->computeActivationSummed(allSegFeats, numSegs);  // [1 x k] summed activation summed over all joints
  }

  // Activation indicators
  if (m_useActivation) {
    for (size_t iJointGroup = 0; iJointGroup < Skeleton::kNumJointGroups; iJointGroup++) {
      const double active = (segsByJointGroup[iJointGroup].empty()) ? 0 : 1;  // Indicates presence of any segments
      result.push_back(active);
    }
  }

  return result;
}

vec<string> SegmentCentroidActivationFeatGen::fieldNames() const {
  vec<string> fields;
  if (m_splitCentroidsPerJoint) {
    for (size_t iJointGroup = 0; iJointGroup < Skeleton::kNumJointGroups; iJointGroup++) {
      const string jointGroupName = Skeleton::jointGroupName(iJointGroup);
      const stats::CentroidSet* centroidSet = m_centroidDb.getCentroidSetForJointGroup(iJointGroup);
      if (centroidSet == nullptr) { continue; }
      for (size_t iCentroid = 0; iCentroid < centroidSet->k; iCentroid++) {
        fields.push_back(jointGroupName + "_c" + to_string(iCentroid));
      }
    }
  } else {
    const stats::CentroidSet* centroids = m_centroidDb.getCentroidSetForAllJoints();
    const size_t numCentroids = (centroids == nullptr) ? 0 : centroids->k;
    for (size_t i = 0; i < numCentroids; i++) {
      fields.push_back("c" + to_string(i));
    }
  }
  return fields;
}

vecd SegmentCentroidActivationFeatGen::generate(const MeshSegment& seg) const {
  const vecd segFeats = m_segFeatGen.generate(seg);
  if (m_splitCentroidsPerJoint) {
    vecd allActivations;
    for (size_t iJointGroup = 0; iJointGroup < Skeleton::kNumJointGroups; iJointGroup++) {
      const stats::CentroidSet* centroidSet = m_centroidDb.getCentroidSetForJointGroup(iJointGroup);
      if (centroidSet == nullptr) { continue; }
      const vecd activation = centroidSet->computeActivation(segFeats, 1);  // [1 x k] activation
      allActivations.insert(allActivations.end(), activation.begin(), activation.end());
    }
    return allActivations;
  } else {
    const stats::CentroidSet* centroids = m_centroidDb.getCentroidSetForAllJoints();
    if (centroids == nullptr) { return vecd(); }  // TODO: Check if empty activations cause trouble downstream
    const vecd activation = centroids->computeActivationSummed(segFeats, 1);  // [1 x k] activation
    return activation;
  }
}

vec<string> SegmentFeatureGeneratorSimplistic::fieldNames() const {
  const size_t numFeats = 5;
  vec<string> feats(numFeats);
  feats[0] = "centroidZ";
  feats[1] = "diagXY";
  feats[2] = "sqrtAreaXY";
  feats[3] = "heightZ";
  feats[4] = "normalDotZ";
  return feats;
}

vecd SegmentFeatureGeneratorSimplistic::generate(const MeshSegment& seg) const {
  // TODO + WARNING: This needs to account for caching of incompatible types of features if they come into existence
  const size_t numFeats = 5;
  if (seg.rawFeatureCache.size() == numFeats) { return seg.rawFeatureCache; }
  vecd feats(numFeats);

  const auto& axesLengths = seg.axesLengths<ml::vec3f>();

  feats[0] = seg.centroid<ml::vec3f>().z;                                              // centroidZ
  feats[1] = sqrtf(axesLengths.x * axesLengths.x + axesLengths.y * axesLengths.y);     // diagXY
  feats[2] = sqrtf(axesLengths.x * axesLengths.y);                                     // sqrtAreaXY
  feats[3] = axesLengths.z;                                                            // heightZ
  feats[4] = fabsf(seg.dominantNormal().z);                                            // normalDotZ

  seg.rawFeatureCache = feats;

  return feats;
}

vec<string> SegmentFeatureGeneratorWolf::fieldNames() const {
  const size_t numFeats = 14;
  vec<string> feats(numFeats);
  feats[0]  = "compactness";    // lambda_0
  feats[1]  = "planarity";      // lambda_1 - lambda_0
  feats[2]  = "linearity";      // lambda_2 - lambda_1
  feats[3]  = "angGroundMean";  // z of dominant normal
  feats[4]  = "angGroundStd";   // circularity around mean angle w ground
  feats[5]  = "hTop";           // height of top point
  feats[6]  = "hCentroid";      // height of centroid
  feats[7]  = "hBottom";        // height of bottom point
  feats[8]  = "labLmean";       // CIELAB color mean and standard deviation (below)
  feats[9]  = "labAmean";
  feats[10] = "labBmean";
  feats[11] = "labLstd";
  feats[12] = "labAstd";
  feats[13] = "labBstd";
  return feats;
}

// copied from http://docs.opencv.org/2.4/modules/imgproc/doc/miscellaneous_transformations.html#cvtcolor
// Assumes RGB input in [0,1] and returns CIE L*a*b* with L in [0,100], a and b in [-127,127]
ml::vec3f rgb2cielab(const ml::vec3f& rgb) {
  // first do approximate RGB to sRGB transform
  const auto srgb = [] (float c) { return (c <= 0.04045) ? c / 12.92f : powf((c+0.055f)/1.055f, 2.4f); };
  const float R = srgb(rgb.r);
  const float G = srgb(rgb.g);
  const float B = srgb(rgb.b);
  float X = 0.412453f * R + 0.357580f * G + 0.180423f * B;
  float Y = 0.212671f * R + 0.715160f * G + 0.072169f * B;
  float Z = 0.019334f * R + 0.119193f * G + 0.950227f * B;
  X = X / 0.950456f;
  Y = Y / 1.088754f;

  const auto f = [] (float t) { return (t > 0.008856f) ? powf(t, 1.f/3.f) : 7.787f * t + 16.f/116.f; };

  const float L = (Y > 0.008856f) ? 116.f * powf(Y, 1.f/3.f) - 16.f : 903.3f * Y;
  const float a = 500 * (f(X) - f(Y));
  const float b = 200 * (f(Y) - f(Z));
  return {L, a, b};
}

vecd SegmentFeatureGeneratorWolf::generate(const MeshSegment& seg) const {
  // TODO + WARNING: This needs to account for caching of incompatible types of features if they come into existence
  const size_t numFeats = 14;
  if (seg.rawFeatureCache.size() == numFeats) { return seg.rawFeatureCache; }
  vecd feats(numFeats);

  // Compute centroid and centered matrix (also store lowest and highest point)
  using namespace geo;
  const size_t numPoints = seg.points.size();
  Matrix3Xf A(3, numPoints);
  Vec3f centroid(0, 0, 0);
  float minZ = std::numeric_limits<float>::max();
  float maxZ = -std::numeric_limits<float>::max();
  for (size_t i = 0; i < numPoints; ++i) {
    const Vec3f p = vec3f(seg.points[i]);
    A.col(i) = p;
    centroid += p;
    if (minZ > p.z()) { minZ = p.z(); }
    if (maxZ < p.z()) { maxZ = p.z(); }
  }
  centroid /= static_cast<float>(numPoints);
  A.colwise() -= centroid;

  // get eigenvalues of scatter matrix
  const Matrix3f S = A * A.transpose();
  const Eigen::SelfAdjointEigenSolver<Matrix3f> eig(S);
  const Vec3f lambdas = eig.eigenvalues();

  // compute mean and std of normal cos(angle) with ground
  vec<float> angNormal(numPoints);
  vec<ml::vec3f> colors(numPoints);  // will store CIELAB point colors
  if (seg.elements.size() != numPoints) { SG_LOG_ERROR << "Segment num elements unequal to num points"; }
  const auto& V = seg.mesh->getVertices();
  for (size_t i = 0; i < numPoints; ++i) {
    const size_t iElement = seg.elements[i];
    angNormal[i] = abs(V[iElement].normal.z);
    colors[i] = rgb2cielab(V[iElement].color.getVec3());
  }
  const float angGroundMean = math::mean(angNormal);
  const float angGroundStd = math::stdev(angNormal);

  // compute mean and std of CIELAB colors
  ml::vec3f labMean(0, 0, 0);
  for (size_t i = 0; i < numPoints; ++i) {
    labMean += colors[i];
  }
  labMean /= static_cast<float>(numPoints);
  ml::vec3f labStd(0, 0, 0);
  //return sqrt(accum / (v.size() - 1));
  for (size_t iP = 0; iP < numPoints; ++iP) {
    const ml::vec3f d = (colors[iP] - labMean);
    for (int i = 0; i < 3; ++i) { labStd[i] += d[i] * d[i]; }
  }
  for (int i = 0; i < 3; ++i) { labStd[i] = sqrtf(labStd[i] / (numPoints - 1)); }

  feats[0]  = lambdas[2];                // compactness
  feats[1]  = lambdas[2] - lambdas[1];   // planarity
  feats[2]  = lambdas[1] - lambdas[0];   // linearity
  feats[3]  = angGroundMean;             // angGroundMean
  feats[4]  = angGroundStd;              // angGroundStd
  feats[5]  = maxZ;                      // height of top point
  feats[6]  = centroid.z();              // height of centroid
  feats[7]  = minZ;                      // height of bottom point
  feats[8]  = labMean[0];                // labLmean
  feats[9]  = labMean[1];                // labAmean
  feats[10] = labMean[2];                // labBmean
  feats[11] = labStd[0];                 // labLstd
  feats[12] = labStd[1];                 // labAstd
  feats[13] = labStd[2];                 // labBstd

  seg.rawFeatureCache = feats;

  return feats;
}

vec<string> SegmentFeatureGeneratorWolf2016::fieldNames() const {
  const size_t numFeats = 17;
  vec<string> feats(numFeats);
  feats[0]  = "labLmean";       // CIELAB color mean and standard deviation (below)
  feats[1]  = "labAmean";
  feats[2]  = "labBmean";
  feats[3]  = "labLstd";
  feats[4]  = "labAstd";
  feats[5]  = "labBstd";
  feats[6]  = "angGroundMean";  // z of dominant normal
  feats[7]  = "angGroundStd";   // circularity around mean angle w ground
  feats[8]  = "obbMaxZ";        // height of top point
  feats[9]  = "obbMinZ";        // height of bottom point
  feats[10] = "obbH";           // height of OBB
  feats[11] = "obbXYmin";       // min dim of OBB in XY
  feats[12] = "obbXYmax";       // max dim of OBB in XY
  feats[13] = "obbXYratio";     // max / min dim in XY of OBB
  feats[14] = "obbZXYratio";    // Z / XY of OBB
  feats[15] = "obbXYarea";      // horizontal (XY) plane area of OBB
  feats[16] = "obbZarea";       // vertical (ZX+ZY) plane area of OBB
  return feats;
}

vecd SegmentFeatureGeneratorWolf2016::generate(const MeshSegment& seg) const {
  // TODO + WARNING: This needs to account for caching of incompatible types of features if they come into existence
  const size_t numFeats = 17;
  if (seg.rawFeatureCache.size() == numFeats) { return seg.rawFeatureCache; }
  vecd feats(numFeats);

  // compute mean and std of normal cos(angle) with ground
  const size_t numPoints = seg.points.size();
  vec<float> angNormal(numPoints);
  vec<ml::vec3f> colors(numPoints);  // will store CIELAB point colors
  if (seg.elements.size() != numPoints) { SG_LOG_ERROR << "Segment num elements unequal to num points"; }
  const auto& V = seg.mesh->getVertices();
  float minZ = std::numeric_limits<float>::max();
  float maxZ = -std::numeric_limits<float>::max();
  for (size_t i = 0; i < numPoints; ++i) {
    const size_t iElement = seg.elements[i];
    angNormal[i] = abs(V[iElement].normal.z);
    colors[i] = rgb2cielab(V[iElement].color.getVec3());
    const ml::vec3f& p = V[iElement].position;
    if (minZ > p.z) { minZ = p.z; }
    if (maxZ < p.z) { maxZ = p.z; }
  }
  const float angGroundMean = math::mean(angNormal);
  const float angGroundStd = math::stdev(angNormal);

  // compute mean and std of CIELAB colors
  ml::vec3f labMean(0, 0, 0);
  for (size_t i = 0; i < numPoints; ++i) {
    labMean += colors[i];
  }
  labMean /= static_cast<float>(numPoints);
  ml::vec3f labStd(0, 0, 0);
  //return sqrt(accum / (v.size() - 1));
  for (size_t iP = 0; iP < numPoints; ++iP) {
    const ml::vec3f d = (colors[iP] - labMean);
    for (int i = 0; i < 3; ++i) { labStd[i] += d[i] * d[i]; }
  }
  for (int i = 0; i < 3; ++i) { labStd[i] = sqrtf(labStd[i] / (numPoints - 1)); }

  const geo::Vec3f& obbAxes = seg.obb()->axesLengths();
  const float obbH = obbAxes.z();
  const float obbXY = sqrtf(obbAxes.x()*obbAxes.x() + obbAxes.y()*obbAxes.y());
  const float obbXYmin = std::min(obbAxes.x(), obbAxes.y());
  const float obbXYmax = std::max(obbAxes.x(), obbAxes.y());
  const float obbXYratio = obbXYmax / obbXYmin;
  const float obbZXYratio = obbH / obbXY;
  const float obbXYarea = obbAxes.x() * obbAxes.y();
  const float obbZarea = obbAxes.z() * obbAxes.y() + obbAxes.z() * obbAxes.x();

  feats[0]  = labMean[0];       // labLmean
  feats[1]  = labMean[1];       // labAmean
  feats[2]  = labMean[2];       // labBmean
  feats[3]  = labStd[0];        // labLstd
  feats[4]  = labStd[1];        // labAstd
  feats[5]  = labStd[2];        // labBstd
  feats[6]  = angGroundMean;    // angGroundMean
  feats[7]  = angGroundStd;     // angGroundStd
  feats[8]  = maxZ;             // height of top point
  feats[9]  = minZ;             // height of bottom point
  feats[10] = obbH;             // height of OBB
  feats[11] = obbXYmin;         // min dim of OBB in XY
  feats[12] = obbXYmax;         // max dim of OBB in XY
  feats[13] = obbXYratio;       // max / min dim in XY of OBB
  feats[14] = obbZXYratio;      // Z / XY of OBB
  feats[15] = obbXYarea;        // horizontal (XY) plane area of OBB
  feats[16] = obbZarea;         // vertical (ZX+ZY) plane area of OBB

  seg.rawFeatureCache = feats;

  return feats;
}

}  // namespace core
}  // namespace sg
