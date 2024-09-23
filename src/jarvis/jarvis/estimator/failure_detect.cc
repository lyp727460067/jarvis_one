#include "jarvis/estimator/failure_detect.h"
namespace jarvis {
namespace estimator {

bool FailureDetect::TimeLost(const common::Time& time) {
  if (!last_frame_data_.has_value()) {
    last_frame_data_ = time;
  }
  const double durition_time =
      common::ToSeconds(time - last_frame_data_.value());
  if(durition_time>option_.option_time_lost){
    //
    LOG(ERROR) << "Time drition to big " << durition_time << ">"
               << option_.option_time_lost;
    //
    return true;
  }
  last_frame_data_ = time;
  return false;
}

bool FailureDetect::Detect(const FrameData& frame_data) {
  if (TimeLost(frame_data.data->time)) {
    return true;
  }
  return false;
  // if (restart) {
  //   restart = false;
  //   return true;
  // }
  // if (f_manager->last_track_num <
  //     options_.fail_detect_option.track_feat_lost_min_num) {
  //   failuer_track_lost_.push_back(true);
  //   LOG(WARNING) << " little feature " << f_manager->last_track_num;
  // } else {
  //   failuer_track_lost_.push_back(false);
  // }
  // if (failuer_track_lost_.size() >
  //     size_t(options_.fail_detect_option.track_feat_lost_win_size)) {
  //   failuer_track_lost_.erase(failuer_track_lost_.begin());
  // }
  // if (std::count(failuer_track_lost_.begin(), failuer_track_lost_.end(),
  //                true) >=
  //     options_.fail_detect_option.track_feat_lost_win_size) {
  //   LOG(ERROR) << " Feat lost! ";
  //   return true;
  // }
  // if (Bas[WINDOW_SIZE].norm() > options_.fail_detect_option.bas_norm_max) {
  //   LOG(ERROR) << " big IMU acc bias estimation " << Bas[WINDOW_SIZE].norm();
  //   return true;
  // }
  // if (Bgs[WINDOW_SIZE].norm() > options_.fail_detect_option.bgs_norm_max) {
  //   LOG(ERROR) << " big IMU gyr bias estimation " << Bgs[WINDOW_SIZE].norm();
  //   return true;
  // }

  // //

  // double odo_distance = 1;
  // if (options_.use_odom) {
  //   auto distance = odometry_factor_[WINDOW_SIZE]->GetObserveDistance();
  //   if (distance.has_value()) {
  //     odo_distance = distance.value();
  //   }
  // }

  // //
  // failuer_zero_lost_.push_back((odo_distance < 0.0001));
  // if (int(failuer_zero_lost_.size()) >
  //     options_.fail_detect_option.zero_odo_win_size) {
  //   failuer_zero_lost_.erase(failuer_zero_lost_.begin());
  // }

  // Eigen::Vector3d tmp_P = Ps[WINDOW_SIZE];
  // //
  // lost_last_poses_.push_back(
  //     transform::Rigid3d(tmp_P, Eigen::Quaterniond(Rs[WINDOW_SIZE])));
  // //
  // if (int(lost_last_poses_.size()) >
  //     options_.fail_detect_option.zero_odo_pose_size) {
  //   lost_last_poses_.erase(lost_last_poses_.begin());
  // }

  // //
  // double z_distance = 0.0;
  // double translation_distance = 0.0;
  // double yaw_distance = 0.0;
  // // if (options_.fail_detect_option.enable_odo_zero_lost_detect == 1) {
  // for (size_t i = 1; i < lost_last_poses_.size(); i++) {
  //   const transform::Rigid3d delta_pose =
  //       lost_last_poses_[i - 1].inverse() * lost_last_poses_[i];
  //   translation_distance += delta_pose.translation().norm();
  //   z_distance += (delta_pose.translation().z());
  //   yaw_distance += (common::RadToDeg(transform::GetAngle(delta_pose)));
  // }
  // // }
  // //
  // //
  // failuer_zero_feat_lost_.push_back(is_velocity_updates_[frame_count]);
  // // 

  // if (int(failuer_zero_feat_lost_.size()) >
  //     options_.fail_detect_option.zero_odo_win_size) {
  //   failuer_zero_feat_lost_.erase(failuer_zero_feat_lost_.begin());
  // }

  // bool is_zero_velocity = (std::count(failuer_zero_feat_lost_.begin(),
  //                                     failuer_zero_feat_lost_.end(), true) ==
  //                          options_.fail_detect_option.zero_odo_win_size);
  // //
  // //


  // // LOG_IF(WARNING, is_zero_velocity) << "feat detect zero velocity.. ";
  // //
  // if (options_.fail_detect_option.enable_odo_zero_lost_detect == 1) {
  //   is_zero_velocity |=
  //       (std::count(failuer_zero_lost_.begin(), failuer_zero_lost_.end(),
  //                   true) == options_.fail_detect_option.zero_odo_win_size);
  // }

  // if (!is_zero_velocity) {
  //   lost_last_poses_.clear();
  // }

  // //
  // //
  // if (is_zero_velocity) {
  //   if (translation_distance >=
  //       options_.fail_detect_option.zero_translation_norm_max) {
  //     LOG(ERROR) << "Zero velocity translation detect: " << translation_distance
  //                << " > "
  //                << options_.fail_detect_option.zero_translation_norm_max;
  //     return true;
  //   }
  //   if (fabs(z_distance) >= options_.fail_detect_option.translation_z_max) {
  //     LOG(ERROR) << "Zero velocity z: " << z_distance << " > "
  //                << options_.fail_detect_option.translation_z_max;

  //     return true;
  //   }
  //   if (fabs(yaw_distance) >= options_.fail_detect_option.zero_ratation_max) {
  //     LOG(ERROR) << "Zero velocity yaw: " << yaw_distance << " > "
  //                << options_.fail_detect_option.zero_ratation_max;

  //     return true;
  //   }
  // }

  // //
  // //
  // //
  // const double delta_translation = (tmp_P - last_P).norm();
  // const double delta_z_translation = abs(tmp_P.z() - last_P.z());
  // const double translation_threash_hold =
  //     options_.fail_detect_option.translation_norm_max;
  // //

  // if ((tmp_P - last_P).norm() > translation_threash_hold) {
  //   LOG(ERROR) << "Big translation !! " << (tmp_P - last_P).norm();
  //   return true;
  // }
  // //
  // const double translation_z_threash_hold =
  //     options_.fail_detect_option.translation_z_max;
  // //
  // if (abs(tmp_P.z() - last_P.z()) > translation_z_threash_hold) {
  //   LOG(ERROR) << " Big z translation" << tmp_P.z() - last_P.z();
  //   return true;
  // }

  // Eigen::Matrix3d tmp_R = Rs[WINDOW_SIZE];
  // Eigen::Matrix3d delta_R = tmp_R.transpose() * last_R;

  // const double rotaion_threash_hold = options_.fail_detect_option.ratation_max;
  // double delta_angle =
  //     common::RadToDeg(transform::GetYaw(Eigen::Quaterniond(delta_R)));
  // if (delta_angle > rotaion_threash_hold) {
  //   LOG(ERROR) << " Big delta_angle " << delta_angle;
  //   // return true;
  // }
  // return false;
}

}  // namespace estimator
}  // namespace jarvis