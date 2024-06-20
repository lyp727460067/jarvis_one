#ifndef JARVIS_ESTIMATOR_PYRAMID_IMAGE_H
#define JARVIS_ESTIMATOR_PYRAMID_IMAGE_H

//
//
#include <execinfo.h>

#include <Eigen/Dense>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <queue>

#include "camera_models/camera_models/CataCamera.h"
#include "camera_models/camera_models/PinholeCamera.h"
#include "camera_models/camera_models/camera_factory.h"
#include "jarvis/estimator/parameters.h"
#include "jarvis/utility/tic_toc.h"
namespace jarvis {
namespace estimator {

//

struct PyramidImageOption {
  int layer = 5;
  int lk_win_size=21;
  Eigen::Vector2i image_size;
};

class PyramidImage {
 public:
  PyramidImage(const PyramidImageOption &option);
  void Build(const cv::Mat &image);
  const int Layer() { return option_.layer; }
  const std::vector<cv::Mat> &PrePyram() { return prev_img_pyramids_; }
  const std::vector<cv::Mat> &CurrPyram() { return curr_img_pyramids_; }

 private:
  const PyramidImageOption option_;
  // std::shared_ptr<uchar> 
  //     prev_pyramids_buffer_=nullptr; 
  // std::shared_ptr<uchar>
  //     curr_pyramids_buffer_ =nullptr; 
  std::vector<cv::Mat> prev_img_pyramids_;
  std::vector<cv::Mat> curr_img_pyramids_;
};
}  // namespace estimator
}  // namespace jarvis
#endif