#include "pose_extrapolator_brige.h"
using namespace jarvis;
namespace jarvis_pic {

constexpr double KTimeImuVioOffsetTime = 1.5;
//

PoseExtrapolatorBrige::PoseExtrapolatorBrige(
    jarvis::common::Duration pose_queue_duration,
    double imu_gravity_time_constant, jarvis::common::Time start_time)
    : extrapolator_(
          new PoseExtrapolator(pose_queue_duration, imu_gravity_time_constant)),
      last_extrapolator_(
          new PoseExtrapolator(pose_queue_duration, imu_gravity_time_constant)),
      last_pose_time_(start_time) {}

void PoseExtrapolatorBrige::AddPose(common::Time time,
                                    const transform::Rigid3d& pose,
                                    bool is_v ) {
  //
  //
  if (is_v && pose_state_ == false) {
    auto time_pose = last_extrapolator_->ExtrapolatePose(time);
    vio_to_odom_transform_ = time_pose * pose.inverse();
    LOG(INFO) << "restart vio: " << vio_to_odom_transform_;
  }
  if (is_v) {
    extrapolator_->AddPose(time, vio_to_odom_transform_ * pose);
    last_extrapolator_->AddPose(time, vio_to_odom_transform_ * pose);
  } else {
    auto time_pose = last_extrapolator_->ExtrapolatePose(time);
    extrapolator_->AddPose(time, time_pose);
    last_extrapolator_->AddPose(time, time_pose);
  }
  if (is_v != pose_state_) {
    pose_state_ = is_v;
  }
 
  last_pose_time_ = time;
}
//
//
void PoseExtrapolatorBrige::AddImuData(const sensor::ImuData& imu_data) {
  extrapolator_->AddImuData(imu_data);
  catch_last_pose_ = extrapolator_->ExtrapolatePose(imu_data.time);
  last_extrapolator_->AddImuData(imu_data);
}
void PoseExtrapolatorBrige::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  extrapolator_->AddOdometryData(odometry_data);
  last_extrapolator_->AddOdometryData(odometry_data);
}
}  // namespace jarvis_pic