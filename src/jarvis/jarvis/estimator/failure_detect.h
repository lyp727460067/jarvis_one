#ifndef JARVIS_ESTIMATIOR_FAILURE_DETECT_H
#define JARVIS_ESTIMATIOR_FAILURE_DETECT_H
#include "jarvis/key_frame_data.h"
#include "jarvis/estimator/slide_window.h"
namespace jarvis {
namespace estimator {

struct FailureDetectOptoin {
  double option_time_lost = 3;
  int track_feat_lost_min_num = 2;
  int track_feat_lost_win_size = 10;
  double bas_norm_max = 0.5;
  double bgs_norm_max = 0.5;
  double translation_norm_max = 0.5;
  double translation_z_max = 0.1;
  double ratation_max = 20;
  double max_velocity_normal = 1.5;
  transform::Rigid3d transform_odom_to_imu ;
  bool use_odo_pose_compare =true;
  double min_odo_valid_distance =0.2;
  double odo_pose_delta_s =0.4;
  double odo_pose_compare_durition=2;
};

class FailureDetect {
 public:
  struct TimePose {
    jarvis::common::Time time;
    jarvis::transform::Rigid3d pose;
  };

  FailureDetect(const FailureDetectOptoin& option) : options_(option) {}
  bool Detect(const SlideWindowResult& frame_data);
  void AddOdometryData(const sensor::OdometryData& odometry_data);

 private:
  bool TimeLost(const common::Time& time);
  bool OdoZeroDetect(const SlideWindowResult& frame_data);
  template <typename T>
  void DropData(const common::Time& time, std::deque<T>* deque);
  // void TrimData(const common::Time& time);
  std::optional<common::Time> last_frame_data_;
  FailureDetectOptoin options_;
  std::vector<bool> failuer_track_lost_;
  std::vector<bool> failuer_zero_odo_lost_;
  std::optional<transform::Rigid3d> last_frame_poses_;
  std::deque<jarvis::sensor::OdometryData> odometry_data_;
  std::deque<TimePose> lost_last_poses_;
};

}  // namespace estimator
}  // namespace jarvis
#endif