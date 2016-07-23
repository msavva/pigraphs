#pragma once

#include <ext-boost/serialization.h>
#include <mLibCore.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "libsg.h"  // NOLINT

namespace sg {
namespace cvutil {

cv::RotatedRect rectToRotatedRect(const cv::Rect& r);

//! Return whether point p is in ROI (rotated rectangle with corners stored in roi)
bool isInROI(const cv::Point& p, const cv::RotatedRect& rrect);

//! Convert rrect selected region to cv::Mat logical mask on image
cv::Mat rotatedRectToMask(const cv::Mat& image, const cv::RotatedRect& rrect);

void drawLineHistogram(const cv::Mat& hist, const string& windowName, int width = 320,
                       int height = 240, const cv::Scalar& lineColor = cv::Scalar(0, 0, 255));

void drawBarHistogram(const cv::Mat& hist, const string& windowName,
                      int width = 320, int height = 240, bool hueColors = true);

void getHueHistogramDominantColor(const cv::Mat& hist, cv::Vec3b* rgb);

//! Reproject depth values in rrect region of depth using camera inverse intrinsics matrix intrinsicsInv
//! and extrinsics matrix E.  Outputs 3D points (x,y,z) as Vec3f's in xyz and returns mean non-zero point
ml::vec3f reprojectKinectDepthTo3D(const cv::Mat& depth, const cv::RotatedRect& rrect,
                                   const ml::mat4f& intrinsicsInv, const ml::mat4f& E, vec<ml::vec3f>* xyz);

//! Reproject depth values in depth using camera inverse intrinsics matrix intrinsicsInv
//! Returns cv::Mat with 3D points (x,y,z) as Vec3f's
cv::Mat reprojectKinectDepthTo3D(const cv::Mat& depth, const ml::mat4f& intrinsicsInv);

}  // namespace cvutil
}  // namespace sg

///** Serialization support for cv::Mat */
BOOST_SERIALIZATION_SPLIT_FREE(::cv::Mat)
namespace boost {
namespace serialization {

template <typename Archive>
void save(Archive& ar, const ::cv::Mat& m, const unsigned int version) {
  size_t elem_size = m.elemSize();
  int elem_type = m.type();

  ar & m.cols;
  ar & m.rows;
  ar & elem_size;
  ar & elem_type;

  const size_t data_size = m.cols * m.rows * elem_size;
  ar & boost::serialization::make_array(m.ptr(), data_size);
}

template <typename Archive>
void load(Archive& ar, ::cv::Mat& m, const unsigned int version) {
  int cols, rows, elem_type;
  size_t elem_size;

  ar & cols;
  ar & rows;
  ar & elem_size;
  ar & elem_type;

  m.create(rows, cols, elem_type);

  size_t data_size = m.cols * m.rows * elem_size;
  ar & boost::serialization::make_array(m.ptr(), data_size);
}

}  // namespace serialization
}  // namespace boost


