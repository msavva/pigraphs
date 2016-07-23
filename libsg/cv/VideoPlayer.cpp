#include "common.h"  // NOLINT

#include "cv/VideoPlayer.h"

#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace sg {
namespace cvutil {

VideoPlayer::VideoPlayer()
  : m_pCap(nullptr) { }

VideoPlayer::VideoPlayer(const string& file)
  : m_pCap(new cv::VideoCapture(file)) { }

VideoPlayer::~VideoPlayer() {
  if (m_pCap != nullptr) { delete m_pCap; }
}

bool VideoPlayer::open(const string& file) {
  if (m_pCap == nullptr) {
    m_pCap = new cv::VideoCapture(file);
  }
  if (m_pCap->isOpened()) { m_pCap->release(); }
  return m_pCap->open(file);
}

bool VideoPlayer::isOpened() {
  return m_pCap->isOpened();
}

bool VideoPlayer::read(cv::Mat& frame) {
  return m_pCap->read(frame);
}

bool VideoPlayer::seekToFrame(size_t frameIdx) {
  return m_pCap->set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(frameIdx));
}

bool VideoPlayer::seekToTime(double timeInms) {
  return m_pCap->set(cv::CAP_PROP_POS_MSEC, timeInms);
}

bool VideoPlayer::seekToRatio(double ratio) {
  assert(ratio >= 0.0 && ratio <= 1.0);
  const size_t frameIdx = static_cast<size_t>(std::floor(ratio * numFrames()));
  return seekToFrame(frameIdx);
}

size_t VideoPlayer::numFrames() {
  return static_cast<size_t>(m_pCap->get(cv::CAP_PROP_FRAME_COUNT));
}

size_t VideoPlayer::getFramePosIndex() {
  return static_cast<size_t>(m_pCap->get(cv::CAP_PROP_POS_FRAMES));
}

double VideoPlayer::getFramePosTime() {
  return m_pCap->get(cv::CAP_PROP_POS_MSEC);
}

double VideoPlayer::getFramePosRatio() {
  return static_cast<double>(getFramePosIndex()) / numFrames();
}

}  // namespace cvutil
}  // namespace sg
