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
namespace jarvis {
struct TrackingData {
  struct Data {
    common::Time time;
    // undistort不用传过来，有可能会增加点，导致存储消耗
    transform::Rigid3d pose;
    std::vector<Eigen::Vector3d> tracking_map_points;
    std::vector<cv::KeyPoint> key_points;
    std::shared_ptr<cv::Mat> image;
    std::vector<cv::KeyPoint> rkey_points;
    std::shared_ptr<cv::Mat> rimage;
    std::vector<uint64_t> outlier_pointclass_id;
    std::vector<cv::KeyPoint> extend_points;
    transform::Rigid3d transform_imu_to_cam_;
    std::optional<Eigen::Quaterniond> rwg;
  };
  std::shared_ptr<Data> data;
  int status = -1;
};

//
}  // namespace jarvis
#endif
