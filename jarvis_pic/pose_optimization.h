#ifndef JARVIS_PIC_POSE_OPTIMIZATION_H
#define  JARVIS_PIC_POSE_OPTIMIZATION_H
#include "jarvis/transform/rigid_transform.h"
#include "jarvis/common/time.h"
#include "jarvis/sensor/fixed_frame_pose_data.h"
#include "jarvis/motion_filter.h"
#include "queue"
namespace jarvis_pic {
struct PoseOptimizationOption {
  int win_size = 20;
};
//
struct PoseData {
  jarvis::common::Time time;
  jarvis::transform::Rigid3d pose;
};
//
class PoseOptimization {
 public:
  PoseOptimization(const PoseOptimizationOption& option);
  //
  void AddPose(const PoseData& pose);
  //
  std::unique_ptr<jarvis::transform::Rigid3d> AddFixedFramePoseData(
      const jarvis::sensor::FixedFramePoseData& fix_data);
  std::unique_ptr<jarvis::transform::Rigid3d> GetDeltaPose();

 private:
  jarvis::transform::Rigid3d CeresUpdata(
      const jarvis::transform::Rigid3d pose_expect,
      const jarvis::sensor::FixedFramePoseData& fix_data);

  std::unique_ptr<jarvis::MotionFilter> pose_motion_filter_;
  std::unique_ptr<jarvis::MotionFilter> rtk_motion_filter_;
  std::deque<jarvis::sensor::FixedFramePoseData> rtk_pose_;
  std::deque<PoseData> rtk_pose_;
  jarvis::transform::Rigid3d pose_gps_to_local_;
};

}  // namespace jarvis_pic

#endif