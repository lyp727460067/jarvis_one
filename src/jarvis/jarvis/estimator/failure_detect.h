#ifndef JARVIS_ESTIMATIOR_FAILURE_DETECT_H
#define JARVIS_ESTIMATIOR_FAILURE_DETECT_H
#include "jarvis/key_frame_data.h"
namespace jarvis {
namespace estimator {

struct FailureDetectOptoin {
  double option_time_lost = 1;
  int track_feat_lost_min_num = 2;
  int track_feat_lost_win_size = 10;
  double bas_norm_max = 0.5;
  double bgs_norm_max = 0.5;
  double translation_norm_max = 0.2;
  double translation_z_max = 0.2;
  double ratation_max = 20;
  int enable_odo_zero_lost_detect = 0;
  double zero_translation_norm_max = 0.002;
  double zero_translation_z_max = 0.002;
  double zero_ratation_max = 0.001;
  int zero_odo_win_size = 10;
  int zero_odo_pose_size = 40;
};


class FailureDetect {
  public:
   FailureDetect(const FailureDetectOptoin& option) : option_(option) {}
   bool Detect(const FrameData& frame_data);

  private:
   bool TimeLost(const common::Time& time);
   std::optional<common::Time> last_frame_data_;
   FailureDetectOptoin  option_;
};

}  // namespace estimator
}  // namespace jarvis
#endif