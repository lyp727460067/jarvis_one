#ifndef JARVIS_SENSOR_IMAGE_DATA_H
#define JARVIS_SENSOR_IMAGE_DATA_H
#include "Eigen/Core"
#include <memory>
#include <opencv2/core.hpp>
#include "jarvis/common/time.h"
namespace jarvis {
namespace sensor {
struct ImageData {
  common::Time time;
  std::vector<cv::Mat> image;
  std::vector<std::vector<cv::Mat>> pyramid_derive;
  static std::string TypeName() { return "image"; }
  // std::vector<std::unique_ptr<cv::Mat>> images_;
};

}  // namespace sensor

}  // namespace jarvis

#endif