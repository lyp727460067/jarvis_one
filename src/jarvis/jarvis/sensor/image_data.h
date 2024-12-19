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
  std::vector<std::vector<cv::Mat>> Pyramid() {
    std::vector<std::vector<cv::Mat>> result;
    for (size_t i = 0; i < pyramid_derive.size(); i++) {
      result.emplace_back();
      for (size_t j = 0; j < pyramid_derive[i].size(); j += 2)
        result.back().push_back(pyramid_derive[i][j]);
    }
    return result;
  }
  static std::string TypeName() { return "image"; }
};

}  // namespace sensor

}  // namespace jarvis

#endif