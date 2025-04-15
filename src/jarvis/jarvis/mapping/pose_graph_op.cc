#include "jarvis/mapping/pose_graph_op.h"

namespace jarvis {
namespace mapping {

void PoseGraphOptimize::AddImuData(const sensor::ImuData& imu_data) {
  imu_datas_.push(imu_data);
}
//
void PoseGraphOptimize::AddLocalMapPose(const LocalMapId& local_map_id,
                                        const LocalMapPoseTime& kf_pose) {
  //
  local_map_poses_.emplace(local_map_id, kf_pose);
  //
}
void PoseGraphOptimize::AddKeyFramePose(const KeyFrameId& id,
                                        const KeyFramePoseTime& kf_pose) {
  node_poses_.emplace(id, kf_pose);
}
//
//
//

void PoseGraphOptimize::DataToState(ceres::Problem* problem) {
  for (const auto& node_pose : node_poses_) {
    const Eigen::Vector3d rpy =
        common::ToRollPitchYaw(node_pose.second.pose.rotation());
    ceres_poses_.emplace(node_pose.first,
                         NodePose{node_pose.second.pose.translation(),
                                  node_pose.second.pose.rotation(),
                                  {rpy[0], rpy[1], rpy[2]},
                                  node_pose.second.pose});
  }
  for (const auto& l_pose : local_map_poses_) {
    const Eigen::Vector3d rpy =
        common::ToRollPitchYaw(node_pose.second.pose.rotation());
    //
    ceres_local_map_poses_.emplace(node_pose.first,
                                   NodePose{l_pose.second.pose.translation(),
                                            l_pose.second.pose.rotation(),
                                            {rpy[0], rpy[1], rpy[2]},
                                            l_pose.second.pose});
  }
  extric_camera_to_imu_.clear();
  for (size_t i = 0; i < options_.track_sequence.size(); i++) {
    extric_camera_to_imu_.push_back(
        NodePose{extirc[options_.track_sequence[i][0]].translation(),
                 extirc[options_.track_sequence[i][0]].rotation()});
  }

  for (auto& pose : ceres_poses_) {
    problem->AddParameterBlock(pose.second.t.data(), 3);
    problem->AddParameterBlock(pose.second.q.coeffs().data(), 4);
    if (froze_trajector_.count(pose.first.trajectory_id)) {
      problem->SetParameterBlockConstant(pose.second.t.data());
      problem->SetParameterBlockConstant(pose.second.q.coeffs().data());
    }
  }

  for (auto& l_pose : ceres_local_map_poses_) {
    problem->AddParameterBlock(l_pose.second.t.data(), 3);
    problem->AddParameterBlock(l_pose.second.q.coeffs().data(), 4);
    if (froze_trajector_.count(l_pose.first.trajectory_id)) {
      problem->SetParameterBlockConstant(l_pose.second.t.data());
      problem->SetParameterBlockConstant(l_pose.second.q.coeffs().data());
    }
  }
  //
  for (auto& ex_pose : extric_camera_to_imu_) {
    problem->AddParameterBlock(ex_pose.t.data(), 3);
    problem->AddParameterBlock(ex_pose.q.coeffs().data(), 4);
    if (options_.fix_extric) {
      problem->SetParameterBlockConstant(ex_pose.t.data());
      problem->SetParameterBlockConstant(ex_pose.q.coeffs().data());
    }
  }

  problem->SetParameterBlockConstant(ceres_poses_.begin()->second.t.data());
  problem->SetParameterBlockConstant(&ceres_poses_.begin()->second.ypr[0]);
  //
  problem->SetParameterBlockConstant(
      ceres_local_map_poses_.begin()->second.t.data());
  problem->SetParameterBlockConstant(
      &ceres_local_map_poses_.begin()->second.ypr[0]);
}
//
void PoseGraphOptimize::AddConstraintFactor(
    ceres::Problem* problem, const std::vector<PoseConstraint>& constraints) {
  for (const auto& constraint : constraints) {
    problem->AddResidualBlock(
        PoseGraphCostFunctor::Create(
            constraint.relative_pose,
            constraint.internal
                ? std::array<double, 2>{options_.constraint_t_weigth,
                                        options_.constraint_t_weigth}
                : std::array<double,
                             2>{options_.constraint_loop_closer_t_weigth,
                                options_.constraint_loop_closer_r_weigth}),
        constraint.internal ? new ceres::HuberLoss(options_.huber_scale)
                            : nullptr,
        ceres_local_map_poses_.at(constraint.node_i).t.data(),
        ceres_local_map_poses_.at(constraint.node_i).q.coeffs().data(),
        ceres_poses_.at(constraint.node_j).t.data(),
        ceres_poses_.at(constraint.node_j).q.coeffs().data());
  }
}

void PoseGraphOptimize::AddRelativeFactor(ceres::Problem* problem) {
  //
  KeyFrameId first_id = ceres_poses_.begin()->first;
  for (auto mit = std::next(ceres_poses_.begin()); mit != ceres_poses_.end();
       ++mit) {
    auto second_id = mit->first;
    if (first_id.trajectory_id == second_id.trajectory_id) {
      const auto relative_pose =
          ceres_poses_.at(first_id).local_pose.inverse() *
          ceres_poses_.at(second_id).local_pose;
      
      ceres::CostFunction* cost_function = PoseGraphCostFunctor::Create(
          relative_pose, std::array<double, 2>{options_.relative_t_weitht,
                                               options_.relative_r_weitht});
      problem->AddResidualBlock(cost_function, nullptr,
                                ceres_poses_.at(first_id).t.data(),
                                ceres_poses_.at(first_id).q.coeffs().data(),
                                ceres_poses_.at(second_id).t.data(),
                                ceres_poses_.at(second_id).q.coeffs().data());
      first_id = second_id;
    }
  }
}
//
void PoseGraphOptimize::Solve(const std::vector<PoseConstraint>& constraints) {
  ceres::Problem problem;
  //

  DataToState(&problem);
  AddImuFactor(&problem);
  AddRelativeFactor(&problem);
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = options_.max_num_iterations;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres::Solver::Summary summary;
  options.num_threads = options_.ceres_num_threads;
  ceres::Solve(options, &problem, &summary);
  LOG(INFO) << log_info::RED << summary.FullReport() << log_info::RESET;
}
//
void PoseGraphOptimize::TrimLocalMapPose(LocalMapId& id) {}
void PoseGraphOptimize::TrimKeyFramePose(LocalMapId& id) {}

}  // namespace mapping
}  // namespace jarvis
