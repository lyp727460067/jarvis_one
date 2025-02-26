#ifndef JARVIS_POSE_EXTRAPOLATOR_H_
#define JARVIS_POSE_EXTRAPOLATOR_H_

#include <deque>
#include <memory>

#include "jarvis/common/time.h"
#include "jarvis/imu_tracker.h"
#include "jarvis/sensor/imu_data.h"
#include "jarvis/sensor/odometry_data.h"
#include "jarvis/transform/rigid_transform.h"

namespace jarvis {

struct ExtrapolationResult {
  std::vector<transform::Rigid3f> previous_poses;
  transform::Rigid3d current_pose;
  Eigen::Vector3d current_velocity;
  Eigen::Quaterniond gravity_from_tracking;
};


class PoseExtrapolator {
 public:
  friend void ResetImuTracker(PoseExtrapolator* extraplota,
                              const transform::Rigid3d& pose);

  explicit PoseExtrapolator(common::Duration pose_queue_duration,
                            double imu_gravity_time_constant);

  PoseExtrapolator(const PoseExtrapolator&) = delete;
  PoseExtrapolator& operator=(const PoseExtrapolator&) = delete;

  static std::unique_ptr<PoseExtrapolator> InitializeWithImu(
      common::Duration pose_queue_duration, double imu_gravity_time_constant,
      const sensor::ImuData& imu_data);

  common::Time GetLastPoseTime() const;
  common::Time GetLastExtrapolatedTime() const;
  void ClearPose() { timed_pose_queue_.clear(); }
  void AddPose(common::Time time, const transform::Rigid3d& pose);
  void AddImuData(const sensor::ImuData& imu_data);
  void AddOdometryData(const sensor::OdometryData& odometry_data);
  transform::Rigid3d ExtrapolatePose(common::Time time);

  ExtrapolationResult ExtrapolatePosesWithGravity(
      const std::vector<common::Time>& times);

  Eigen::Quaterniond EstimateGravityOrientation(common::Time time);

 private:
  void UpdateVelocitiesFromPoses();
  void TrimImuData();
  void TrimOdometryData();
  void AdvanceImuTracker(common::Time time, ImuTracker* imu_tracker) const;
  Eigen::Quaterniond ExtrapolateRotation(common::Time time,
                                         ImuTracker* imu_tracker) const;
  Eigen::Vector3d ExtrapolateTranslation(common::Time time);

  const common::Duration pose_queue_duration_;
  struct TimedPose {
    common::Time time;
    transform::Rigid3d pose;
  };
  std::deque<TimedPose> timed_pose_queue_;
  Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();

  const double gravity_time_constant_;
  std::deque<sensor::ImuData> imu_data_;
  std::unique_ptr<ImuTracker> imu_tracker_;
  std::unique_ptr<ImuTracker> odometry_imu_tracker_;
  std::unique_ptr<ImuTracker> extrapolation_imu_tracker_;
  TimedPose cached_extrapolated_pose_;

  std::deque<sensor::OdometryData> odometry_data_;
  Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();
};

}  // namespace jarvis

#endif