#include "jarvis/mapping/local_map_optimization.h"
#include "jarvis/mapping/local_map.h"
namespace jarvis {
namespace mapping {

struct NodePose {
  Eigen::Vector3d t;
  Eigen::Quaterniond q;
};

void LocalMapOptimization::Optimize(
    std::map<LocalMapId, std::shared_ptr<LocalMap>>* local_maps) {
  std::map<KeyFrameId, NodePose> ceres_poses;
  std::map<LocalMapId, NodePose> ceres_local_map_poses;
  //
  std::map<MapPointId, Eigen::Vector3d> ceres_map_points;
  //
  //
  transform::Rigid3d fix_pose = local_maps->begin()->second->LocalPose();
  //
  for (auto& local_map : *local_maps) {
    transform::Rigid3d local_map_local_pose =
        fix_pose.inverse() * local_map.second->LocalPose();

    ceres_local_map_poses.emplace(local_map.first,
                                  NodePose{
                                      local_map_local_pose.translation(),
                                      local_map_local_pose.rotation(),
                                  });

    auto local_poses = local_map.second->AllKeyFrameRefPose();
    //
    for (const auto& pos : local_poses) {
      if (ceres_poses.count(pos.first)) continue;
      transform::Rigid3d pose_local_pose =
          fix_pose.inverse() * local_map.second->LocalPose() * pos.second;
      ceres_poses.emplace(pos.first, NodePose{
                                        pose_local_pose.translation(),
                                        pose_local_pose.rotation(),
                                    });
    }

    auto all_map_points = local_map.second->AllMapPoints();
    for (const auto& mp : all_map_points) {
      if (ceres_map_points.count(mp.id)) continue;
      //
      const Eigen::Vector3d pos = fix_pose.inverse() *
                                  local_map.second->LocalPose() *
                                  mp.data.data->pos;
      ceres_map_points.emplace(mp.id, pos);
    }
    //
  }



}
}  // namespace mapping
}  // namespace jarvis
