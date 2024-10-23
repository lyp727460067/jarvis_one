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
#include "jarvis/estimator/featureTracker/feature_tracker.h"
namespace jarvis {
namespace estimator {

struct ImuState {
  //
  Eigen::Vector3d p{0, 0, 0};
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  Eigen::Vector3d v{0, 0, 0};
  Eigen::Vector3d ba{0, 0, 0};
  Eigen::Vector3d bg{0, 0, 0};
  Eigen::Vector3d g{0, 0, 9.81};
  Eigen::Matrix<double, 15, 15> jacobian =
      Eigen::Matrix<double, 15, 15>::Identity();
  Eigen::Matrix<double, 15, 15> covariance =
      Eigen::Matrix<double, 15, 15>::Zero();
  transform::Rigid3d Pose() const { return transform::Rigid3d(p, q); }
  common::Time time;
  // ImuState()=default;
  ImuState SetBaBg(const Eigen::Vector3d &ba_, const Eigen::Vector3d &bg_) {
    ba = ba_;
    bg = bg_;
    return *this;
  }
};
inline std::ostream &operator<<(std::ostream &os, const ImuState state) {

    Eigen::Vector3d rpy = estimator::Utility::R2ypr(state.q.toRotationMatrix());
    os << state.Pose() << "{rpy:" << rpy.transpose() << "}" << ",ba" << "["
       << state.ba.transpose() << "]" << ",bg[" << state.bg.transpose()
       << "],v[" << state.v.transpose() << "]";
    return os;
}
}
// struct TrackingData {
//   struct Data {
//     common::Time time;
//     estimator::ImuState imu_state;
//     std::vector<cv::KeyPoint> key_points;
//     std::vector<Eigen::Vector3d> tracking_map_points;
//     std::shared_ptr<cv::Mat> image;
//     transform::Rigid3d transform_cam_to_imu;
//   };
//   std::shared_ptr<Data> data;
//   int status = -1;
// };
using  CameraId  =int;
using TrackingId = uint64_t;
struct FrameData {
  struct FeatureData {
    estimator::ImageFeatureTrackerData features;
    // std::map<TrackingId, double> depths;
    std::map<TrackingId,Eigen::Vector3d> map_points;
    std::map<TrackingId,cv::KeyPoint> key_points;//for display
  };  
  struct Data {
    common::Time time;
    uint64_t id;
    estimator::ImuState imu_state;
    std::map<CameraId, FeatureData> features_datas;
    
    std::vector<transform::Rigid3d> extric_camera_to_imu;
    transform::Rigid3d odo_to_imu_extric;
    double opt_dt;

    bool  is_key_frame=false;
  };
  std::shared_ptr<Data> data;
  int status = -1;
};
//

using TrackingData = FrameData;

// struct OptimizationStateData {
//   double **pose;
//   double **speed_bias;
//   double **feature;
//   double **ex_pose;
//   double **ex_pose_odom;
//   // double** retrive_pose;
//   double **td;
// };
extern bool restart ;
//
}  // namespace jarvis

#endif
