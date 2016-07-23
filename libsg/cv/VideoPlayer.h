#pragma once

#include "libsg.h"  // NOLINT

// Forward declarations
namespace cv {
  class Mat;
  class VideoCapture;
}

namespace sg {
namespace cvutil {

//! Wrapper for playing back video files using OpenCV VideoCapture interface
class VideoPlayer {
 public:
  VideoPlayer();
  ~VideoPlayer();
  //! Create VideoPlayer for given file
  explicit VideoPlayer(const string& file);
  //! Opens the given video file. Releases any existing open file and returns whether successfully opened file
  bool open(const string& file);
  //! Returns whether this VideoPlayer currently has an opened video file
  bool isOpened();
  //! Reads the frame at the current position into the given matrix and returns whether successful
  bool read(cv::Mat& frame);  // NOLINT
  //! Seeks to the given frame index and returns whether successful
  bool seekToFrame(size_t frameIdx);
  //! Seek to the given time in milliseconds from video start and return whether successful
  bool seekToTime(double timeInms);
  //! Seek to time point corresponding to given ratio in [0,1] of the total video running time
  bool seekToRatio(double ratio);
  //! Returns total number of frames in the opened video
  size_t numFrames();
  //! Returns the current frame position as frame index
  size_t getFramePosIndex();
  //! Returns the current frame position as time in milliseconds
  double getFramePosTime();
  //! Returns the current frame position as [0,1] ratio of total video running time
  double getFramePosRatio();

 private:
  cv::VideoCapture* m_pCap;
};

}  // namespace cvutil
}  // namespace sg


