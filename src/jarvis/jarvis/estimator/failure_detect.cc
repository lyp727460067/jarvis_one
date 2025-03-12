#include "jarvis/estimator/failure_detect.h"
namespace jarvis {
namespace estimator {

bool FailureDetect::TimeLost(const common::Time& time) {
  if (!last_frame_data_.has_value()) {
    last_frame_data_ = time;
  }
  const double durition_time =
      common::ToSeconds(time - last_frame_data_.value());
  if (durition_time > options_.option_time_lost) {
    //
    LOG(ERROR) << "Time drition to big " << durition_time << ">"
               << options_.option_time_lost;
    //
    return true;
  }
  last_frame_data_ = time;
  return false;
}

bool FailureDetect::OdoZeroDetect(const SlideWindowResult& frame_data) {
  //
  if (options_.use_odom) return false;
  double odo_distance = 1;
  auto& distance = frame_data.latest_odo_distance;
  if (distance.has_value()) {
    odo_distance = distance.value();
  }

  // //
  //
  failuer_zero_odo_lost_.push_back((odo_distance < 0.01));
  if (int(failuer_zero_odo_lost_.size()) > options_.zero_odo_win_size) {
    failuer_zero_odo_lost_.erase(failuer_zero_odo_lost_.begin());
  }
  //
  if (std::count(failuer_zero_odo_lost_.begin(), failuer_zero_odo_lost_.end(),
                 true) != options_.zero_odo_win_size) {
    lost_last_poses_.clear();
    return false;
  }
  lost_last_poses_.push_back(
      transform::Rigid3d(frame_data.frame_data.data->imu_state.Pose()));
  //
  if (int(lost_last_poses_.size()) > options_.zero_odo_pose_size) {
    lost_last_poses_.erase(lost_last_poses_.begin());
  }
  //
  double z_distance = 0.0;
  double translation_distance = 0.0;
  double yaw_distance = 0.0;
  // if (options_.fail_detect_option.enable_odo_zero_lost_detect == 1) {
  for (size_t i = 1; i < lost_last_poses_.size(); i++) {
    const transform::Rigid3d delta_pose =
        lost_last_poses_[i - 1].inverse() * lost_last_poses_[i];
    translation_distance += delta_pose.translation().norm();
    z_distance += (delta_pose.translation().z());
    yaw_distance += (common::RadToDeg(transform::GetAngle(delta_pose)));
  }

  if (translation_distance >= options_.zero_translation_norm_max) {
    LOG(ERROR) << "Zero velocity translation detect: " << translation_distance
               << " > " << options_.zero_translation_norm_max;
    return true;
  }
  if (fabs(z_distance) >= options_.translation_z_max) {
    LOG(ERROR) << "Zero velocity z: " << z_distance << " > "
               << options_.translation_z_max;

    return true;
  }
  if (fabs(yaw_distance) >= options_.zero_ratation_max) {
    LOG(ERROR) << "Zero velocity yaw: " << yaw_distance << " > "
               << options_.zero_ratation_max;

    return true;
  }
  return false;
}
//
//
bool FailureDetect::Detect(const SlideWindowResult& frame_data) {
  double time_diff = 0.0;
  if (last_frame_data_.has_value()) {
    time_diff = common::ToSeconds(frame_data.frame_data.data->time -
                                  last_frame_data_.value());
    //
  }

  if (TimeLost(frame_data.frame_data.data->time)) {
    return true;
  }
  const FeatTrackInfo track_info = frame_data.feat_track_info;
  if (track_info.last_track_num < options_.track_feat_lost_min_num) {
    failuer_track_lost_.push_back(true);
    LOG(WARNING) << " little feature " << track_info.last_track_num;
  } else {
    failuer_track_lost_.push_back(false);
  }
  if (failuer_track_lost_.size() > size_t(options_.track_feat_lost_win_size)) {
    failuer_track_lost_.erase(failuer_track_lost_.begin());
  }
  if (std::count(failuer_track_lost_.begin(), failuer_track_lost_.end(),
                 true) >= options_.track_feat_lost_win_size) {
    LOG(ERROR) << " Feat lost! ";
    return true;
  }
  //
  const ImuState& state = frame_data.frame_data.data->imu_state;
  if (state.ba.norm() > options_.bas_norm_max) {
    LOG(ERROR) << " big IMU acc bias estimation " << state.ba.transpose();
    return true;
  }
  if (state.bg.norm() > options_.bgs_norm_max) {
    LOG(ERROR) << " big IMU gyr bias estimation " << state.bg.transpose();
    return true;
  }
  if (OdoZeroDetect(frame_data)) return true;
  if (!last_frame_poses_.has_value()) {
    last_frame_poses_ = frame_data.frame_data.data->imu_state.Pose();
    return false;
  }
  //
  const transform::Rigid3d curr_pose =
      frame_data.frame_data.data->imu_state.Pose();
  //

  const transform::Rigid3d delta_pose =
      last_frame_poses_.value().inverse() * curr_pose;

  const double delta_z_translation = delta_pose.translation().z();
  const double translation_threash_hold = options_.translation_norm_max;
  //
  double delta_angle = abs(common::RadToDeg(transform::GetAngle(delta_pose)));
  //
  // if (time_diff >= 0.001) {
  //   const double velocity_normal = delta_pose.translation().norm() / time_diff;
  //   LOG(INFO)<<velocity_normal ;
  //   if (velocity_normal > options_.max_velocity_normal) {
  //     LOG(ERROR) << " max_velocity_normal too big" << velocity_normal;
  //     return true;
  //   }
  // }
  last_frame_poses_ = curr_pose;
  if (delta_pose.translation().norm() > translation_threash_hold ||
      delta_angle > options_.ratation_max ||
      abs(delta_z_translation) > options_.translation_z_max) {
    LOG(ERROR) << " Delta pose too big " << delta_pose.translation().norm()
               << " " << delta_angle << " " << delta_z_translation;
    return true;
  }
  return false;
}
}  // namespace estimator
}  // namespace jarvis