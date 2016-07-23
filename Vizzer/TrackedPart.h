#pragma once

#include <mLibCore.h>
#include <opencv2/core/core.hpp>

// Represents a part being tracked by the ColorTracker
struct TrackedPart {
  cv::RotatedRect box;
  cv::Mat hueHist;
  ml::vec3f xyzMean;
  std::vector<ml::vec3f> xyzPoints;
  cv::Vec3b color;
  std::string id;

  TrackedPart() { }
  explicit TrackedPart(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) { MLIB_ERROR("Unable to open file"); }
    fs["id"] >> id;
    fs["hueHist"] >> hueHist;
    fs["color"] >> color;
    fs.release();
  }
  void saveOrDie(const std::string& filename) const {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) { MLIB_ERROR("Unable to open file"); }
    fs << "id" << id;
    fs << "hueHist" << hueHist;
    fs << "color" << color;
    fs.release();
  }
};

