#include "jarvis/estimator/failure_detect.h"
namespace jarvis {
namespace estimator {
constexpr float kOdoLostDataTimeLenth = 2;
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
jarvis::transform::Rigid3d ToPoseInOdom(
  const jarvis::transform::Rigid3d& pose1,
  const jarvis::transform::Rigid3d& extric) {
const transform::Rigid3d transform_odom_to_imu = extric;
auto transform_cam_to_odom_map_ = transform::Rigid3d::Rotation(
    transform::RollPitchYaw(0, 0, transform::GetYaw(extric.rotation())));

const transform::Rigid3d pose =
    transform_cam_to_odom_map_ * pose1 * transform_odom_to_imu.inverse();
//

return jarvis::transform::Rigid3d(
    Eigen::Vector3d(pose.translation().x(), pose.translation().y(),
                    pose.translation().z()),
    pose.rotation());
}
//
template <typename T>
double ComputePosesS(std::deque<T>* datas, const jarvis::common::Time& time) {
  double delta_s = 0;
  if (datas->size() < 2) return 0;
  for (size_t i = 1; i < datas->size(); i++) {
    if (datas->at(i).time > time) break;
    delta_s += abs((datas->at(i - 1).pose.inverse() * datas->at(i).pose)
                       .translation()
                       .norm());
  }
  // LOG(INFO)<<delta_s ;
  return delta_s;
}
//
//
bool FailureDetect::OdoZeroDetect(const SlideWindowResult& frame_data) {
  const common::Time time = frame_data.frame_data.data->time;
  //
  DropData(time - common::FromSeconds(kOdoLostDataTimeLenth), &odometry_data_);
  DropData(time - common::FromSeconds(kOdoLostDataTimeLenth),
           &lost_last_poses_);
  //

  //
  lost_last_poses_.push_back(
      TimePose{frame_data.frame_data.data->time,
               ToPoseInOdom(frame_data.frame_data.data->imu_state.Pose(),
                            options_.transform_odom_to_imu)});
  //
  const double delta_odom_s = ComputePosesS(&odometry_data_, time);
  const double delta_pose_s = ComputePosesS(&lost_last_poses_, time);
  //
  const auto delta_s = delta_pose_s - delta_odom_s;
  if (delta_s > options_.odo_pose_delta_s) {
    LOG(ERROR) << " Delta pose too big " << delta_s << ",delta_pose_s"
               << delta_pose_s << ",delta_odom_s" << delta_odom_s;
    return true;
  }
  return false;
}
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
  //   const double velocity_normal = delta_pose.translation().norm() /
  //   time_diff; LOG(INFO)<<velocity_normal ; if (velocity_normal >
  //   options_.max_velocity_normal) {
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

void FailureDetect::AddOdometryData(const sensor::OdometryData& odom) {
  odometry_data_.push_back(odom);
  if (odometry_data_.size() > 2000) {
    LOG_EVERY_N(WARNING, 10) << "Odom data size too big.";
    odometry_data_.pop_front();
  }
}
//
template <typename T>
void FailureDetect::DropData(const common::Time& time, std::deque<T>* deque) {
  while (!deque->empty() && deque->front().time < time) {
    deque->pop_front();
  }
}

  //
}  // namespace estimator
}  // namespace jarvis