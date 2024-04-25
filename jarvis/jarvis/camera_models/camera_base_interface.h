

#ifndef __JARVIS_VIO__CAMERA_BASE_INTERFACE_
#define __JARVIS_VIO__CAMERA_BASE_INTERFACE_
#include <Eigen/Core>
#include <vector>

#include "glog/logging.h"
#include "opencv2/opencv.hpp"
namespace jarvis {
//
namespace camera_models {
class CameraBase {
 public:
  virtual Eigen::Vector2d UndistortPoint(const Eigen::Vector2d& uv) const {
    return {};
  };
  virtual Eigen::Vector2d DistortPoint(const Eigen::Vector2d& uv) const {
    // auto points = UndistortPointsNormal(
    //     std::vector<cv::KeyPoint>({cv::KeyPoint(uv.x(), uv.y(), 2)}));
    // LOG(INFO) << uv;
    // LOG(INFO)<<points[0];
    // return Project(points[0]);
    return {};
  };
  virtual std::vector<Eigen::Vector3d> UndistortPointsNormal(
      const std::vector<cv::KeyPoint>& key_points) const = 0;
  virtual Eigen::Vector2d Project(const Eigen::Vector3d& point) const = 0;
  virtual ~CameraBase() {}
};
//
}  // namespace camera_models
}  // namespace jarvis
#endif