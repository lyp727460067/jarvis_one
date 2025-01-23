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



class PoseGraphOptimize :public BackOptimize {
 public:
  void AddImuData(const sensor::ImuData& imu_data) ;
  void AddLocalMapPose(const LocalMapTime& local_map_pose) ;
  void AddKfDataPose(const KfPoseTime& kf_pose) ;
  void Solve(const std::vector<PoseConstraint>& constraints) ;
  //

  // 如果在多次轨迹的情况，维持当前轨迹只有一个localmap跟踪就可以其他的可以删除掉
  void TrimLocalMapPose(LocalMapId& id);
  void TrimKfPose(LocalMapId& id);

 private:
  std::vector<NodePose> extric_camera_to_imu_;
  std::map<LocalMapId, NodePose> ceres_local_map_poses_;
  std::map<KeyFrameId, NodePose> ceres_poses_;
};
//
}  // namespace mapping
}  // namespace jarvis

#endif