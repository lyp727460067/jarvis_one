#include "jarvis/mapping/local_map_optimization.h"

#include "jarvis/mapping/auto_factor/re_projection_err.h"
#include "jarvis/mapping/local_map.h"
namespace jarvis {
namespace mapping {

struct NodePose {
  Eigen::Vector3d t;
  Eigen::Quaterniond q;
  double ypr[3];
  double ypr_tmpe[3];
};

//
transform::Rigid3d ToTransform(const NodePose& node) {
  return transform::Rigid3d(node.t, node.q);
}
//
//
LocalMapOptimization::LocalMapOptimization(
    const LocalMapOptimizationOption& option)
    : options_(option), extric_camera_to_imu_(option.extric_camera_to_imu) {}
//
void LocalMapOptimization::Optimize(
    std::map<LocalMapId, LocalMap*>* local_maps) {
  //
  std::map<KeyFrameId, NodePose> ceres_poses;
  //
  std::map<LocalMapId, NodePose> ceres_local_map_poses;
  std::map<MapPointId, Eigen::Vector3d> ceres_map_points;
  //
  //
  transform::Rigid3d fix_pose = local_maps->begin()->second->LocalPose();
  //
  //
  ceres::Problem problem;
  ceres::LocalParameterization* quaternion_local =
      new ceres::EigenQuaternionParameterization;
  Eigen::Quaterniond ex_rotation[options_.track_sequence.size()];
  Eigen::Vector3d ex_traslation[options_.track_sequence.size()];
  //
  //
  LOG(INFO) << "Local op local mp size:" << local_maps->size();
  for (size_t i = 0; i < options_.track_sequence.size(); i++) {
    transform::Rigid3d extir_iverse =
        extric_camera_to_imu_[options_.track_sequence[i][0]];
    ex_rotation[i] = extir_iverse.rotation();
    ex_traslation[i] = extir_iverse.translation();
    problem.AddParameterBlock(ex_rotation[i].coeffs().data(), 4);
    problem.AddParameterBlock(ex_traslation[i].data(), 3);
    problem.SetParameterization(ex_rotation[i].coeffs().data(),
                                quaternion_local);
  }

  for (auto& local_map : *local_maps) {
    transform::Rigid3d local_map_local_pose =
        fix_pose.inverse() * local_map.second->LocalPose();

    ceres_local_map_poses.emplace(local_map.first,
                                  NodePose{
                                      local_map_local_pose.translation(),
                                      local_map_local_pose.rotation(),
                                  });
    LOG(INFO) << local_map_local_pose.translation().transpose();
    auto local_poses = local_map.second->AllKeyFrameRefPose();
    //
    for (const auto& pos : local_poses) {
      if (ceres_poses.count(pos.first)) continue;
      //
      transform::Rigid3d pose_local_pose = local_map_local_pose * pos.second;
      const Eigen::Vector3d ypr =
          transform::Rot2ypr(pose_local_pose.rotation().toRotationMatrix()) /
          180. * M_PI;
      //
      LOG(INFO) << pose_local_pose.translation();
      ceres_poses.emplace(pos.first, NodePose{
                                         pose_local_pose.translation(),
                                         pose_local_pose.rotation(),
                                         {ypr[0], ypr[1], ypr[2]},
                                     });
    }

    auto all_map_points = local_map.second->AllMapPoints();
    for (const auto& mp : all_map_points) {
      if (ceres_map_points.count(mp.id)) continue;
      //
      const Eigen::Vector3d pos = local_map_local_pose * mp.data.data->pos;
      ceres_map_points.emplace(mp.id, pos);
    }
    //
  }
  //
  for (auto& local_map : *local_maps) {
    const auto& all_kf_frames = local_map.second->AllKeyFrameDatas();
    auto& kf_rf_frames_poses = local_map.second->AllKeyFrameRefPose();
    //
    auto convisibility = local_map.second->GetCovisibility();
    for (auto const& kf_data : all_kf_frames) {
      //
      const auto& frame_map_features =
          convisibility->GetKeyFrameMapPointId(kf_data.id);
      const auto& frame_map_point = frame_map_features.first;
      const auto& frame_features = frame_map_features.second;

      for (size_t i = 0; i < frame_map_point.size(); i++) {
        //
        problem.AddResidualBlock(
            FourReProjectionBaErr::Creat(
                kf_data.data.data->features.at(frame_features[i]).f,
                ceres_poses.at(kf_data.id).ypr[2],
                ceres_poses.at(kf_data.id).ypr[1], options_.re_preject_weight),
            new ceres::HuberLoss(options_.huber_loss),
            ceres_poses.at(kf_data.id).t.data(),
            &ceres_poses.at(kf_data.id).ypr[0],
            ex_traslation[frame_features[i].sequence_id].data(),
            ex_rotation[frame_features[i].sequence_id].coeffs().data(),
            ceres_map_points.at(frame_map_point[i]).data(),
            ceres_local_map_poses.at(local_map.first).t.data());
      }
    }
  }
  //
  //
  problem.SetParameterBlockConstant(
      ceres_local_map_poses.begin()->second.t.data());

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = options_.max_num_iterations;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  LOG(INFO) << log_info::RED << summary.FullReport() << log_info::RESET;
  //

  // std::map<KeyFrameId, NodePose> ceres_poses;
  // std::map<LocalMapId, NodePose> ceres_local_map_poses;
  // std::map<MapPointId, Eigen::Vector3d> ceres_map_points;

  for (auto& local_map : *local_maps) {
    //
    transform::Rigid3d local_map_local_pose = transform::Rigid3d::Translation(
        ceres_local_map_poses.at(local_map.first).t);
    //
    auto local_map_data = local_map.second->MutableData();
    //
    //
    local_map_data->local_pose =
        fix_pose * transform::Rigid3d::Translation(
                       ceres_local_map_poses.at(local_map.first).t);

    auto& local_frame_poses = local_map_data->key_frames_ref_pose;
    for (auto& local_pose : local_frame_poses)
      for (auto& pos : local_frame_poses) {
        if (ceres_poses.count(pos.first)) {
          pos.second = local_map_local_pose.inverse() *
                       transform::Rigid3d(ceres_poses[pos.first].t,
                                          transform::RollPitchYaw(
                                              ceres_poses[pos.first].ypr[2],
                                              ceres_poses[pos.first].ypr[1],
                                              ceres_poses[pos.first].ypr[0])
                                              .normalized());
        }
      }
    //
    auto& all_local_map_points = local_map_data->map_points;
    //
    for (const auto& mp : all_local_map_points) {
      if (ceres_map_points.count(mp.id)) {
        // LOG(INFO)<<mp.data.data->pos;
        mp.data.data->pos =
            local_map_local_pose.inverse() * ceres_map_points.at(mp.id);
        // LOG(INFO)<<mp.data.data->pos;
      }
    }
  }
}
}  // namespace mapping
}  // namespace jarvis
