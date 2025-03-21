#ifndef JARVIS_MAPPING_POSE_GRAPH_OP_H
#define JARVIS_MAPPING_POSE_GRAPH_OP_H
#include <deque>

//
#include "jarvis/common/id.h"
#include "jarvis/mapping/optimization_data_base.h"
#include "jarvis/transform/transform.h"
//
namespace jarvis {
namespace mapping {

struct PoseGraphOption {};
//

struct LocalMapPoseTime
{
  common::Time time;
  LocalMapId local_map_id;
  transform::Rigid3d local_pose;
};
class PoseGraphOptimize {
 public:
  void AddImuData(const sensor::ImuData& imu_data);
  //
  void AddLocalMapPose(const LocalMapId& local_map_id,
                       const LocalMapPoseTime& kf_pose);
  void AddKeyFrameDataPose(const KeyFrameId& id,
                           const KeyFramePoseTime& kf_pose);
  //
  //

  void Solve(const std::vector<PoseConstraint>& constraints);
  //
  void TrimLocalMapPose(LocalMapId& id);
  void TrimKeyFramePose(LocalMapId& id);

 private:
  std::vector<NodePose> extric_camera_to_imu_;
  std::map<LocalMapId, NodePose> ceres_local_map_poses_;
  std::map<KeyFrameId, NodePose> ceres_poses_;
};
//
}  // namespace mapping
}  // namespace jarvis

#endif