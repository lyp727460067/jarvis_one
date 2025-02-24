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
      last_pose_time_(start_time) {}

void PoseExtrapolatorBrige::AddPose(common::Time time,
                                    const transform::Rigid3d& pose) {
  //
  if (common::ToSeconds(time - last_pose_time_) > KTimeImuVioOffsetTime) {
    vio_to_odom_transform_ = catch_last_pose_ * pose.inverse();
    LOG(INFO) << "restart vio: " << vio_to_odom_transform_;
  }
  last_pose_time_ = time;
  extrapolator_->AddPose(time, vio_to_odom_transform_ * pose);
}
//
//
void PoseExtrapolatorBrige::AddImuData(const sensor::ImuData& imu_data) {
  extrapolator_->AddImuData(imu_data);
  if (common::ToSeconds(imu_data.time - last_pose_time_) >
      KTimeImuVioOffsetTime) {
    LOG_EVERY_N(WARNING, 10) << "vio maby ivalid...";
    catch_last_pose_ = extrapolator_->ExtrapolatePose(imu_data.time);
    extrapolator_->AddPose(imu_data.time, catch_last_pose_);
  } else {
    catch_last_pose_ = extrapolator_->ExtrapolatePose(imu_data.time);
  }
}
void PoseExtrapolatorBrige::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  extrapolator_->AddOdometryData(odometry_data);
}
}  // namespace jarvis_pic