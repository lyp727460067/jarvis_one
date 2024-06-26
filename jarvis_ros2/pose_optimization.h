#ifndef JARVIS_PIC_POSE_OPTIMIZATION_H
#define JARVIS_PIC_POSE_OPTIMIZATION_H
#include "jarvis/common/time.h"
#include "jarvis/motion_filter.h"
#include "jarvis/sensor/fixed_frame_pose_data.h"
#include "jarvis/transform/rigid_transform.h"
#include "jarvis/transform/transform_interpolation_buffer.h"
#include "queue"
namespace jarvis_pic {
struct PoseOptimizationOption {
  int win_size = 100;
  double fix_weitht_traslation =100;
  double fix_weitht_rotation =1;
};
//
struct PoseData {
  jarvis::common::Time time;
  jarvis::transform::Rigid3d pose;
};
//
class PoseOptimization {
 public:
  explicit PoseOptimization(const PoseOptimizationOption& option);
  //
  void AddPose(const PoseData& pose);
  //
  std::unique_ptr<jarvis::transform::Rigid3d> AddFixedFramePoseData(
      const jarvis::sensor::FixedFramePoseData& fix_data);

  std::unique_ptr<jarvis::transform::Rigid3d> AlignmentOptimization();
  int PoseSize() {
    return odom_pose_.size();
  }
 private:
  void Alignment();
  std::unique_ptr<jarvis::transform::Rigid3d> Optimization();
  PoseOptimizationOption options_;
  std::unique_ptr<jarvis::MotionFilter> pose_motion_filter_;
  std::unique_ptr<jarvis::MotionFilter> rtk_motion_filter_;
  std::deque<jarvis::sensor::FixedFramePoseData> rtk_pose_;
  std::deque<PoseData> odom_pose_;
  std::unique_ptr<jarvis::transform::TransformInterpolationBuffer>
      rtk_interpolateion_;
  // 
  jarvis::transform::Rigid3d pose_local_to_fix_;
  //
  // std::deque<PoseData> optimization_pose_;
};

}  // namespace jarvis_pic

#endif