#include "pose_extrapolator_brige.h"
#include "jarvis/transform/transform.h"
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
    const transform::Rigid3d time_pose = last_extrapolator_->ExtrapolatePose(time);
    const Eigen::Vector3d ypr =
        transform::Rot2ypr(pose.rotation().toRotationMatrix()) * M_PI / 180.;
    const Eigen::Vector3d ypr1 = transform::Rot2ypr(time_pose.rotation().toRotationMatrix()) *
               M_PI / 180.;
    //
    const transform::Rigid3d time_pose1 =
        transform::Rigid3d(time_pose.translation(),
                           transform::RollPitchYaw(ypr[2], ypr[1], ypr1[0]));

    // double delta_yaw = ypr1[0] -ypr[0];
    // //
    // vio_to_odom_transform_ = transform::Rigid3d(time);
    vio_to_odom_transform_ = time_pose1 * pose.inverse();
    // vio_to_odom_transform_ = transform::Rigid3d(
    //     Eigen::Vector3d(vio_to_odom_transform_.translation().x(),
    //                     vio_to_odom_transform_.translation().y(), 0),
    //     vio_to_odom_transform_.rotation());
    const Eigen::Vector3d ypr2 = transform::Rot2ypr(
                    vio_to_odom_transform_.rotation().toRotationMatrix()) *
                M_PI / 180.;
    // 
    LOG(INFO)<<ypr2[0]<<" "<< ypr2[1]<<" "  <<ypr2[2]; 
    LOG(INFO) << "restart_vio: " << vio_to_odom_transform_;
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