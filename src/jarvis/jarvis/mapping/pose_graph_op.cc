#include "jarvis/mapping/pose_graph_op.h"

namespace jarvis {
namespace mapping {
void PoseGraphOptimize::AddImuData(const sensor::ImuData& imu_data) {}
void PoseGraphOptimize::AddLocalMapPose(const LocalMapTime& local_map_pose) {}
void PoseGraphOptimize::AddKfDataPose(const KfPoseTime& kf_pose) {}
void PoseGraphOptimize::Solve(const std::vector<PoseConstraint>& constraints) {}
//

// 如果在多次轨迹的情况，维持当前轨迹只有一个localmap跟踪就可以其他的可以删除掉
void PoseGraphOptimize::TrimLocalMapPose(LocalMapId& id) {}
void PoseGraphOptimize::TrimKfPose(LocalMapId& id) {}

}  // namespace mapping
}  // namespace jarvis
