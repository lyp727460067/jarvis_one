#ifndef __JARVIS_VIO_KEY_FRAME_DATA_H
#define __JARVIS_VIO_KEY_FRAME_DATA_H
#include <vector>
#include <optional>
#include "Eigen/Core"
#include "common/id.h"
#include "common/time.h"
#include "opencv2/core.hpp"
#include "transform/rigid_transform.h"
#include "transform/timestamped_transform.h"
//
#include "jarvis/estimator/imu_extrapolator.h"
namespace jarvis {
struct TrackingData {
  struct Data {
    common::Time time;
    estimator::ImuState imu_state;
    std::vector<cv::KeyPoint> key_points;
    std::vector<Eigen::Vector3d> tracking_map_points;
    std::shared_ptr<cv::Mat> image;
    transform::Rigid3d transform_imu_to_cam_;
  };
  std::shared_ptr<Data> data;
  int status = -1;
};
std::vector<Eigen::Vector3d> GetGlobleImuPose();
std::pair<double, jarvis::transform::Rigid3d> GetGlobleImuExtrapolatorPose();
//
}  // namespace jarvis

#endif
