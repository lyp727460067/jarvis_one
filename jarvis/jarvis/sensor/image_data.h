#ifndef JARVIS_SENSOR_IMAGE_DATA_H
#define JARVIS_SENSOR_IMAGE_DATA_H
#include "Eigen/Core"
#include <memory>
#include <opencv2/core.hpp>

namespace jarvis {
namespace sensor {
struct ImageData {
  double time;
  std::vector<std::shared_ptr<cv::Mat>> image;
  // std::vector<std::unique_ptr<cv::Mat>> images_;
};

}  // namespace sensor

}  // namespace jarvis

#endif