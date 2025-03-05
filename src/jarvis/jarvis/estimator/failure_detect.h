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
  int enable_odo_zero_lost_detect = 0;
  double zero_translation_norm_max = 0.3;
  double zero_translation_z_max = 0.3;
  double zero_ratation_max = 0.1;
  int zero_odo_win_size = 10;
  int zero_odo_pose_size = 40;
  bool use_odom =false;
  double max_velocity_normal = 1.5;
};

class FailureDetect {
 public:
  FailureDetect(const FailureDetectOptoin& option) : options_(option) {}
  bool Detect(const SlideWindowResult& frame_data);

 private:
  bool TimeLost(const common::Time& time);
  bool OdoZeroDetect(const SlideWindowResult& frame_data);
  std::optional<common::Time> last_frame_data_;
  FailureDetectOptoin options_;
  std::vector<bool> failuer_track_lost_;
  std::vector<bool> failuer_zero_odo_lost_;
  std::optional<transform::Rigid3d> last_frame_poses_;
  std::vector<transform::Rigid3d> lost_last_poses_;
};

}  // namespace estimator
}  // namespace jarvis
#endif