#include "features.h"

#include <mLibCore.h>
#include <segmentation/MeshSegment.h>
#include "./Skeleton.h"
#include "./Database.h"
#include "./Scene.h"
#include "./CentroidDatabase.h"
#include <algorithm>
#include <array>

namespace sg {
namespace core {

UINT SegmentFeatureGeneratorBasic::featureCount() const
{
    return 8;
}

std::vector<std::string> SegmentFeatureGeneratorBasic::fieldNames() const {
  std::vector<std::string> result;

  result.push_back("centroidZ");

  result.push_back("lengthX");
  result.push_back("lengthY");
  result.push_back("lengthZ");
  result.push_back("lengthDiagonal");

  result.push_back("aspectXY");
  result.push_back("aspectXZ");
  result.push_back("aspectYZ");

  return result;
}

std::vector<double> SegmentFeatureGeneratorBasic::generate(const MeshSegment& seg) const{
  std::vector<double> result;
  std::array<ml::vec3f, 3> normAxes;
  seg.normalizedAxes(&normAxes);
  const ml::vec3f& axesLengths = seg.axesLengths<ml::vec3f>();
  UINT Z = ml::math::maxIndex(fabs(ml::vec3f::dot(ml::vec3f::eZ, normAxes[0])),
                              fabs(ml::vec3f::dot(ml::vec3f::eZ, normAxes[1])),
                              fabs(ml::vec3f::dot(ml::vec3f::eZ, normAxes[2])));
  UINT X = (Z + 1) % 3;
  UINT Y = (Z + 2) % 3;

  if (axesLengths[X] > axesLengths[Y]) { std::swap(X, Y); }

  //
  // centroid height
  //
  result.push_back(seg.centroid<ml::vec3f>()[2]);

  //
  // absolute dimensions
  //
  result.push_back(axesLengths[X]);
  result.push_back(axesLengths[Y]);
  result.push_back(axesLengths[Z]);
  result.push_back(seg.diagonalLength());

  //
  // X-Y, X-Z and Y-Z aspect ratios
  //
  result.push_back(axesLengths[X] / std::max(0.01f, axesLengths[Y]));
  result.push_back(axesLengths[X] / std::max(0.01f, axesLengths[Z]));
  result.push_back(axesLengths[Y] / std::max(0.01f, axesLengths[Z]));

  return result;
}

std::vector<std::string> SegmentFeatureGeneratorSim::fieldNames() const {
  std::vector<std::string> result;

  result.push_back("centroidZ");

  result.push_back("lengthX");
  result.push_back("lengthY");
  result.push_back("lengthZ");

  result.push_back("aspectXY");
  result.push_back("aspectXZ");
  result.push_back("aspectYZ");

  return result;
}

std::vector<double> SegmentFeatureGeneratorSim::generate(const MeshSegment& seg) const{
  std::vector<double> result;
  std::array<ml::vec3f, 3> normAxes;
  seg.normalizedAxes(&normAxes);
  const ml::vec3f& axesLengths = seg.axesLengths<ml::vec3f>();
  UINT Z = ml::math::maxIndex(fabs(ml::vec3f::dot(ml::vec3f::eZ, normAxes[0])),
    fabs(ml::vec3f::dot(ml::vec3f::eZ, normAxes[1])),
    fabs(ml::vec3f::dot(ml::vec3f::eZ, normAxes[2])));
  UINT X = (Z + 1) % 3;
  UINT Y = (Z + 2) % 3;

  if (axesLengths[X] > axesLengths[Y]) { std::swap(X, Y); }

  //
  // centroid height
  //
  result.push_back(seg.centroid<ml::vec3f>()[2]);

  //
  // absolute dimensions
  //
  result.push_back(axesLengths[X]);
  result.push_back(axesLengths[Y]);
  result.push_back(axesLengths[Z]);

  //
  // X-Y, X-Z and Y-Z aspect ratios
  //
  result.push_back(axesLengths[X] / std::max(0.01f, axesLengths[Y]));
  result.push_back(axesLengths[X] / std::max(0.01f, axesLengths[Z]));
  result.push_back(axesLengths[Y] / std::max(0.01f, axesLengths[Z]));

  return result;
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

UINT SegmentSkelFeatureGeneratorBasic::featureCount() const
{
  return segmentFeatureJointCount * 2;
}

std::vector<std::string> SegmentSkelFeatureGeneratorBasic::fieldNames() const {
  std::vector<std::string> result;

  for (UINT jointIndex = 0; jointIndex < segmentFeatureJointCount; jointIndex++) {  
    result.push_back(Skeleton::jointName(segmentFeatureJoints[jointIndex]) + " zDisp");
    result.push_back(Skeleton::jointName(segmentFeatureJoints[jointIndex]) + " xyDist");
  }

  return result;
}

std::vector<double> SegmentSkelFeatureGeneratorBasic::generate(const SegmentSkeletonPair& pair) const {
  std::vector<double> result;

  //const Skeleton& skel = pair.second->makeSkeleton();
  for (UINT jointIndex = 0; jointIndex < segmentFeatureJointCount; jointIndex++) {  
    auto v = pair.second->transformedJoint(segmentFeatureJoints[jointIndex]);
    ml::vec3f centroid = pair.first->centroid<ml::vec3f>();

    //TODO: dist vs. closest point, or vs. centroid?
    result.push_back(v.z - centroid.z);

    auto closestPt = pair.first->closestPoint(v);
    float xyDist = ml::vec3f::dist( ml::vec3f(v.x, v.y, 0.0f), ml::vec3f(closestPt.x, closestPt.y, 0.0f));
    if(xyDist < 0.01f) xyDist = 0.0f;
    result.push_back(xyDist);
  }

  return result;
}

const bool useCentroidDistFeature = false;
const bool useSegmentFeatures = false;
const bool factorizeDistance = false;

std::vector<std::string> SceneSkelFeatureGeneratorBasic::fieldNames() const
{
    std::vector<std::string> result;

    SegmentFeatureGeneratorBasic segGenerator;
    SegmentSkelFeatureGeneratorBasic segPoseGenerator;

    for(const Centroid &c : _database->filteredCentroids(*_verb))
    {
        if(useCentroidDistFeature)
            result.push_back(c.id + "-dist");

        if(useSegmentFeatures)
            for(const std::string &s : segGenerator.fieldNames())
                result.push_back(c.id + "-" + s);

        //for(const std::string &s : segPoseGenerator.fieldNames())
        //    result.push_back(c.id + "-" + s);
        if(factorizeDistance)
        {
            result.push_back(c.id + " " + core::Skeleton::jointName(c.jointIndex) + " zDisp");
            result.push_back(c.id + " " + core::Skeleton::jointName(c.jointIndex) + " xyDist");
        }
        else
            result.push_back(c.id + " " + core::Skeleton::jointName(c.jointIndex) + " dist");
    }

    return result;
}

std::vector<double> SceneSkelFeatureGeneratorBasic::generate(const SceneSkelPair& pair) const
{
    using sg::segmentation::MeshSegment;

    SegmentFeatureGeneratorBasic segGenerator;
    SegmentSkelFeatureGeneratorBasic segPoseGenerator;

    std::vector<double> result;

    const Scene &scene = *pair.first;
    const TransformedSkeleton &skel = *pair.second;

    // TODO: this could be cached by the scene pose iterator
    // TODO: factor out radius into parameter
    auto segments = scene.getSegmentsInRadius(skel.transformedJoint(Skeleton::JointType_SpineBase), 1.0f);

    // maximum distance between a segment and any joint
    // TODO: parameterize
    const float sentinelDistance = 1.0f;
    const float activationThreshold = 0.4f;

    for(const Centroid &c : _database->filteredCentroids(*_verb))
    {
        //
        // find segment in segments that is closest to the signature for this centroid
        //
        const MeshSegment *closestSeg = NULL;
        double closestSegDist = -1.0;
        for(const MeshSegment *seg : segments)
        {
            double distSq = _database->distSq(c, *seg);
            if(closestSeg == NULL || distSq < closestSegDist)
            {
                closestSeg = seg;
                closestSegDist = distSq;
            }
        }

        bool acceptable = (closestSeg != NULL && closestSegDist < c.distanceSqThreshold);

        if(acceptable)
        {
            if(useCentroidDistFeature)
                result.push_back(closestSegDist);

            // note that these features are in un-scaled space. This is fine if we use classifiers
            // that are scale-agnostic.
            if(useSegmentFeatures)
            {
                // for multithreading, this cache must be populated beforehand!
                if (closestSeg->rawFeatureCache.size() == 0)
                    closestSeg->rawFeatureCache = segGenerator.generate(*closestSeg);

                for(double f : closestSeg->rawFeatureCache)
                    result.push_back(f);
            }

            auto v = skel.transformedJoint(c.jointIndex);
            ml::vec3f centroid = closestSeg->centroid<ml::vec3f>();
            auto closestPt = closestSeg->closestPoint(v);

            if(factorizeDistance)
            {
                result.push_back(v.z - centroid.z);

                float xyDist = ml::vec3f::dist( ml::vec3f(v.x, v.y, 0.0f), ml::vec3f(closestPt.x, closestPt.y, 0.0f));
                if(xyDist < 0.01f) xyDist = 0.0f;
                result.push_back(xyDist);
            }
            else
            {
                float dist = ml::vec3f::dist( v, closestPt );
                
                //if(dist < 0.01f) dist = 0.0f;
                
                if (dist < activationThreshold) dist = 0.0f;
                else dist = sentinelDistance;

                result.push_back(dist);
            }

            //for(double f : segPoseGenerator.generate(std::make_pair(closestSeg, &skel)))
            //    result.push_back( std::min(f, sentinelDistance) );
        }
        else
        {
            if(useCentroidDistFeature)
                result.push_back(c.distanceSqThreshold);

            if(useSegmentFeatures)
                for(UINT featureIndex = 0; featureIndex < segGenerator.featureCount(); featureIndex++)
                    result.push_back(0.0f);

            if(factorizeDistance)
            {
                result.push_back(sentinelDistance);
                result.push_back(sentinelDistance);
            }
            else
            {
                result.push_back(sentinelDistance);
            }
            

            //for(UINT featureIndex = 0; featureIndex < segPoseGenerator.featureCount(); featureIndex++)
            //    result.push_back(sentinelDistance);
        }
    }

    return result;
}

std::vector<std::string> InteractingSkelJointAggrFeatureGeneratorBasic::fieldNames() const
{
    std::vector<std::string> result;
    for (int iJointGroup = 0; iJointGroup < Skeleton::kNumJointGroups; iJointGroup++) {
      std::string& jointName = Skeleton::jointGroupName(iJointGroup);
      result.push_back(jointName + "_active");
      // TODO: Fill out features below
      if (m_useJointSegScores) {

      }
      if (m_useJointSegFeatures) {

      }
    }
    return result;
}

std::vector<double> InteractingSkelJointAggrFeatureGeneratorBasic::generate(const InteractingSkel& interSkel) const
{
    std::vector<double> result;
    const Skeleton::SegmentsByJointGroup& segs = Skeleton::groupSegmentsByJointGroup(*interSkel.second);
    
    for (int iJointGroup = 0; iJointGroup < Skeleton::kNumJointGroups; iJointGroup++) {
      const double active = (segs[iJointGroup].empty()) ? 0 : 1;  // Indicates presence of any segments
      result.push_back(active);
      // TODO: Fill out features below
      if (m_useJointSegScores) {

      }
      if (m_useJointSegFeatures) {

      }
    }
    return result;
}

}  // namespace core
}  // namespace sg
