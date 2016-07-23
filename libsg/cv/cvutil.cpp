#include "common.h"  // NOLINT

#include "cv/cvutil.h"

#include <opencv2/imgproc/imgproc.hpp>

namespace sg {
namespace cvutil {

cv::RotatedRect rectToRotatedRect(const cv::Rect& r) {
  const float hw = r.width * 0.5f, hh = r.height * 0.5f;
  cv::Point2f c(r.x + hw, r.y + hh);
  return cv::RotatedRect(c, cv::Size2f(static_cast<float>(r.width), static_cast<float>(r.height)), 0.0f);
}

// Return whether point p is in ROI (rotated rectangle with corners stored in roi)
bool isInROI(const cv::Point& p, const cv::RotatedRect& rrect) {
  cv::Point2f roi[4];
  rrect.points(roi);
  auto product = [&p](cv::Point2f a, cv::Point2f b) {
    double k = (a.y - b.y) / (a.x - b.x);
    double j = a.y - k * a.x;
    return k * p.x - p.y + j;
  };
  double pro[4];
  for (int i = 0; i < 4; ++i) { pro[i] = product(roi[i], roi[(i + 1) % 4]); }
  if (pro[0] * pro[2] < 0 && pro[1] * pro[3] < 0) { return true; }
  return false;
}

cv::Mat rotatedRectToMask(const cv::Mat& image, const cv::RotatedRect& rrect) {
  cv::Mat mask(image.size(), CV_8U, cv::Scalar(0));
  cv::Point p;
  for (p.y = 0; p.y < image.rows; p.y++) {
    for (p.x = 0; p.x < image.cols; p.x++) {
      if (isInROI(p, rrect)) { mask.at<uchar>(p.y, p.x) = 255; }
    }
  }
  return mask;
}

void drawLineHistogram(const cv::Mat& hist, const string& windowName, int width,
                       int height, const cv::Scalar& lineColor) {
  const int numBins = hist.rows;
  const int binWidth = static_cast<int>(std::floor(static_cast<double>(width) / numBins));
  cv::Mat histImage(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
  for (int i = 1; i < numBins; i++) {
    const cv::Point p0(binWidth * (i - 1), height - cvRound(hist.at<float>(i - 1)));
    const cv::Point p1(binWidth * i, height - cvRound(hist.at<float>(i)));
    cv::line(histImage, p0, p1, lineColor, 2, 8, 0);
  }
  cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
  cv::imshow(windowName, histImage);
  cv::waitKey(1);
}

void drawBarHistogram(const cv::Mat& hist, const string& windowName,
                      int width, int height, bool hueColors) {
  cv::Mat histimg = cv::Mat::zeros(height, width, CV_8UC3);
  const int numBins = hist.rows;
  const int binW = histimg.cols / numBins;
  cv::Mat buf(1, numBins, CV_8UC3);
  for (int i = 0; i < numBins; i++) {
    const int ii = hueColors ? i : 0;  // Maps to hue range, or red as default
    buf.at<cv::Vec3b>(i)
      = cv::Vec3b(cv::saturate_cast<uchar>(ii * 180.f / numBins), 255, 255);
  }
  cv::cvtColor(buf, buf, cv::COLOR_HSV2BGR);
  for (int i = 0; i < numBins; i++) {
    int val = cv::saturate_cast<int>(hist.at<float>(i) * histimg.rows / 255);
    cv::rectangle(histimg, cv::Point(i * binW, histimg.rows),
                  cv::Point((i + 1)*binW, histimg.rows - val),
                  cv::Scalar(buf.at<cv::Vec3b>(i)), -1, 8);
  }
  cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
  cv::imshow(windowName, histimg);
  cv::waitKey(1);
}

void getHueHistogramDominantColor(const cv::Mat& hist, cv::Vec3b* rgb) {
  const int numBins = hist.rows;
  double minVal, maxVal;
  int minIdx[2], maxIdx[2];
  cv::minMaxIdx(hist, &minVal, &maxVal, minIdx, maxIdx);
  cv::Mat buf(1, 1, CV_8UC3);
  buf.at<cv::Vec3b>(0) = cv::Vec3b(cv::saturate_cast<uchar>(maxIdx[0] * 180.f / numBins), 255, 255);
  cv::cvtColor(buf, buf, cv::COLOR_HSV2BGR);
  *rgb = buf.at<cv::Vec3b>(0);
}

ml::vec3f reprojectKinectDepthTo3D(const cv::Mat& depth, const cv::RotatedRect& rrect,
                                   const ml::mat4f& intrinsicsInv, const ml::mat4f& E, vec<ml::vec3f>* xyz) {
  assert(depth.type() == CV_16UC1);  // Raw Kinect depth frame
  xyz->clear();
  ml::vec3f nonZeroMean(0, 0, 0);

  const cv::Rect bounds = rrect.boundingRect() &= cv::Rect(0, 0, depth.cols, depth.rows);
  cv::Mat roi(depth, bounds);
  const int maxX = bounds.x + bounds.width, maxY = bounds.y + bounds.height;
  for (int j = bounds.y; j < maxY; j++) {
    for (int i = bounds.x; i < maxX; i++) {
      if (!isInROI(cv::Point(i, j), rrect)) { continue; }

      const uint16_t dRaw = depth.at<uint16_t>(j, i);
      //const float d = getDepth(depth, i, j);
      //cv::Vec3f& v = xyz.at<cv::Vec3f>(j, i);
      if (dRaw > 0) {
        const float d = dRaw * 0.001f;  // Convert raw kinect to depth in m
        ml::vec4f p_camera = intrinsicsInv * ml::vec4f(d * i, d * j, 0.0f, d);
        ml::vec3f p(p_camera.x, p_camera.y, p_camera.w);
        const ml::vec3f P = E * p;
        xyz->push_back(P);
        nonZeroMean += P;
      } else {
        //v[0] = 0.f;  v[1] = 0.f;  v[2] = 0;
      }
    }
  }
  if (xyz->size() > 0) { nonZeroMean /= static_cast<float>(xyz->size()); }
  return nonZeroMean;
}

cv::Mat reprojectKinectDepthTo3D(const cv::Mat& depth, const ml::mat4f& intrinsicsInv) {
  assert(depth.type() == CV_16UC1);  // Raw Kinect depth frame
  cv::Mat xyz(depth.rows, depth.cols, CV_32FC3);  // xyz coords in camera space

  for (int j = 0; j < depth.rows; j++) {
    for (int i = 0; i < depth.cols; i++) {
      const uint16_t dRaw = depth.at<uint16_t>(j, i);
      cv::Vec3f& v = xyz.at<cv::Vec3f>(j, i);
      if (dRaw > 0) {
        const float d = dRaw * 0.001f;  // Convert raw kinect to depth in m
        ml::vec4f p_camera = intrinsicsInv * ml::vec4f(d * i, d * j, 0.0f, d);
        v[0] = p_camera.x;  v[1] = p_camera.y;  v[2] = p_camera.w;
      } else {
        v[0] = 0.f;  v[1] = 0.f;  v[2] = 0;
      }
    }
  }
  return xyz;
}

}  // namespace cvutil
}  // namespace sg
