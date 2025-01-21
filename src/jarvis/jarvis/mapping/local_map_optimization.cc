#include "jarvis/mapping/local_map_optimization.h"

#include "factor/projection_factor.h"
#include "jarvis/common/time.h"
#include "jarvis/estimator/factor/pose_local_parameterization.h"
#include "jarvis/mapping/auto_factor/pose_factor.h"
#include "jarvis/mapping/auto_factor/re_projection_err.h"
#include "jarvis/mapping/auto_factor/relative_pose_graph.h"
#include "jarvis/mapping/local_map.h"
namespace jarvis {
namespace mapping {

//
transform::Rigid3d ToTransform(const NodePose& node) {
  return transform::Rigid3d(node.t, node.q);
}
//
//
LocalMapOptimization::LocalMapOptimization(
    const LocalMapOptimizationOption& option)
    : options_(option) {
  for (size_t i = 0; i < options_.track_sequence.size(); i++) {
    extric_camera_to_imu_.push_back(
        NodePose{options_.extric_camera_to_imu[options_.track_sequence[i][0]]
                     .translation(),
                 options_.extric_camera_to_imu[options_.track_sequence[i][0]]
                     .rotation()});
  }
}
//
//
void LocalMapOptimization::Optimize(
    std::map<LocalMapId, std::shared_ptr<LocalMap>>* local_maps) {
  StrategyOptimize(local_maps);
  UpdateLocalMapData(local_maps);
  for (auto& local_map : *local_maps) {
    RemoveOutliersRejection(local_map.first, local_map.second);
  }
  extric_camera_to_imu_.clear();
  for (size_t i = 0; i < options_.track_sequence.size(); i++) {
    extric_camera_to_imu_.push_back(
        NodePose{options_.extric_camera_to_imu[options_.track_sequence[i][0]]
                     .translation(),
                 options_.extric_camera_to_imu[options_.track_sequence[i][0]]
                     .rotation()});
  }
  ceres_local_map_poses_.clear();
  ceres_map_points_.clear();
  ceres_poses_.clear();
}
//

int LocalMapOptimization::RemoveOutliersRejection(
    const LocalMapId& map_id, std::shared_ptr<LocalMap> local_map) {
  auto ReprojectionError = [](const Eigen::Vector3d world_point_i,
                              const transform::Rigid3d& pose_j,
                              const Eigen::Vector2d& uvj) {
    //
    const Eigen::Vector3d pts_cj = pose_j.inverse() * world_point_i;
    Eigen::Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj;
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);
  };
  int inliner = 0;

  auto local_map_data = local_map->MutableData();
  auto& all_local_map_points = local_map_data->map_points;
  auto& all_ref_pose = local_map_data->key_frames_ref_pose;
  //
  std::stringstream info;
  std::set<MapPointId> outliers_mp_id;
  for (const auto& mp : all_local_map_points) {
    if (ceres_map_points_.at(map_id).count(mp.id) == 0) continue;
    const auto all_con_kfs =
        local_map_data->covisibility.GetMapObservations(mp.id);
    double err_sum = 0;
    for (const auto& kf_id : all_con_kfs) {
      if (ceres_poses_.at(map_id).count(kf_id) == 0) continue;
      auto feat_id =
          local_map_data->covisibility.GetMapPointFeatureIndex(kf_id, mp.id);
      err_sum += ReprojectionError(
          mp.data.data->pos,
          all_ref_pose[kf_id] *
              ToTransform(extric_camera_to_imu_[feat_id.sequence_id]),
          local_map_data->key_frames_datas.at(kf_id)
              .data->features.at(feat_id)
              .f.head<2>());
    }
    err_sum = err_sum / all_con_kfs.size();
    if (err_sum >= options_.optimazation_outliers_rejection_th) {
      outliers_mp_id.insert(mp.id);
      // info << mp.id;
    } else {
      inliner++;
    }
  }
  LOG(INFO) << log_info::YELLOW
            << "Local_op outlier num: " << outliers_mp_id.size()
            << ",inliner:" << inliner << log_info::RESET;
  local_map->InsertOutOutliers(outliers_mp_id);
  return inliner;
}
//
void LocalMapOptimization::AddExtricToProblem(ceres::Problem* problem) {
  ceres::LocalParameterization* quaternion_local =
      new ceres::EigenQuaternionParameterization;
  for (size_t i = 0; i < options_.track_sequence.size(); i++) {
    problem->AddParameterBlock(extric_camera_to_imu_[i].q.coeffs().data(), 4);
    problem->AddParameterBlock(extric_camera_to_imu_[i].t.data(), 3);
    problem->SetParameterization(extric_camera_to_imu_[i].q.coeffs().data(),
                                 quaternion_local);

    if (options_.fix_extric) {
      problem->SetParameterBlockConstant(
          extric_camera_to_imu_[i].q.coeffs().data());
      problem->SetParameterBlockConstant(extric_camera_to_imu_[i].t.data());
    }
  }
}
//
void LocalMapOptimization::StrategyOptimize(
    std::map<LocalMapId, std::shared_ptr<LocalMap>>* local_maps) {
  //

  std::map<LocalMapId, NodePose>& ceres_local_map_poses =
      ceres_local_map_poses_;
  std::map<LocalMapId, std::map<MapPointId, Eigen::Vector3d>>&
      ceres_map_points = ceres_map_points_;
  std::map<LocalMapId, std::map<KeyFrameId, NodePose>>& ceres_poses =
      ceres_poses_;
  std::map<MapPointId, std::set<LocalMapId>> repeat_map_points;
  //

  ceres::Problem problem;
  ceres::LossFunction* loss_function =
      new ceres::HuberLoss(options_.huber_loss);
  ceres::LocalParameterization* quaternion_local =
      new ceres::EigenQuaternionParameterization;
  //
  transform::Rigid3d fix_local_map_pose =
      local_maps->begin()->second->LocalPose();
  LocalMapId fix_local_map_id = local_maps->begin()->first;
  for (auto& local_map : *local_maps) {
    //
    //
    ceres_local_map_poses[local_map.first].t =
        fix_local_map_pose.inverse() *
        local_map.second->LocalPose().translation();
    problem.AddParameterBlock(ceres_local_map_poses[local_map.first].t.data(),
                              3);
    problem.AddParameterBlock(&ceres_local_map_poses[local_map.first].ypr[0],
                              1);
    const auto& local_map_data = local_map.second->ConstData();
    for (const auto& ref_pose : local_map_data.key_frames_ref_pose) {
      transform::Rigid3d pose_local_pose = ref_pose.second;
      const Eigen::Vector3d ypr =
          transform::Rot2ypr(pose_local_pose.rotation().toRotationMatrix()) /
          180. * M_PI;
      //

      ceres_poses[local_map.first].emplace(
          ref_pose.first,
          NodePose{
              pose_local_pose.translation(),
              pose_local_pose.rotation(),
              {ypr[0], ypr[1], ypr[2]},
              local_map_data.key_frames_datas.at(ref_pose.first).data->pose});
    }
    auto all_map_points = local_map.second->AllMapPoints();
    ceres_map_points[local_map.first];
    for (const auto& mp : all_map_points) {
      repeat_map_points[mp.id].insert(local_map.first);
      if (ceres_map_points[local_map.first].count(mp.id)) continue;
      //
      const Eigen::Vector3d pos = mp.data.data->pos;
      ceres_map_points[local_map.first].emplace(mp.id, pos);
    }
  }
  for (auto it = repeat_map_points.begin(); it != repeat_map_points.end();) {
    if (it->second.size() < 2) {
      it = repeat_map_points.erase(it);
    } else {
      ++it;
    }
  }
  //

  LOG(INFO) << "Local op local mp size:" << local_maps->size();
  AddExtricToProblem(&problem);
  //
  for (const auto& local_map : ceres_map_points) {
    //
    const auto& all_kf_frames =
        local_maps->at(local_map.first)->AllKeyFrameDatas();
    auto& kf_rf_frames_poses =
        local_maps->at(local_map.first)->AllKeyFrameRefPose();
    for (const auto& mp_id : local_map.second) {
      const auto mp_obsers = local_maps->at(local_map.first)
                                 ->GetCovisibility()
                                 ->GetMapPointObserv(mp_id.first);

      for (const auto& ob_kf_f : mp_obsers) {
        const double weitht = options_.relative_weight;

        CHECK(ceres_poses.count(local_map.first));
        CHECK(ceres_map_points.count(local_map.first));
        CHECK(ceres_local_map_poses.count(local_map.first));
        problem.AddResidualBlock(
            FourReProjectionBaErr::Creat(
                all_kf_frames.at(ob_kf_f.first)
                    .data->features.at(ob_kf_f.second)
                    .f,
                ceres_poses[local_map.first].at(ob_kf_f.first).ypr[2],
                ceres_poses[local_map.first].at(ob_kf_f.first).ypr[1], weitht),
            new ceres::HuberLoss(options_.huber_loss),
            ceres_poses[local_map.first].at(ob_kf_f.first).t.data(),
            &ceres_poses[local_map.first].at(ob_kf_f.first).ypr[0],
            extric_camera_to_imu_[ob_kf_f.second.sequence_id].t.data(),
            extric_camera_to_imu_[ob_kf_f.second.sequence_id].q.coeffs().data(),
            ceres_map_points[local_map.first].at(mp_id.first).data());
      }
      if (repeat_map_points.count(mp_id.first) &&
          local_map.first != fix_local_map_id) {
        problem.AddResidualBlock(
            RepeatMapPointErr::Creat(options_.repeat_mp_weight), nullptr,
            ceres_map_points[fix_local_map_id].at(mp_id.first).data(),
            ceres_map_points[local_map.first].at(mp_id.first).data(),
            ceres_local_map_poses[local_map.first].t.data(),
            &ceres_local_map_poses[local_map.first].ypr[0]);
      }
    }
    //

    //
  }
  //
  //

  for (const auto& local_map : ceres_local_map_poses) {
    auto ceres_poses_it = ceres_poses[local_map.first].begin();

    for (auto it = ceres_poses_it; it != ceres_poses[local_map.first].end();
         ++it) {
      transform::Rigid3d delta_pose =
          ToTransform(ceres_local_map_poses[local_map.first]) *
          it->second.local_pose;

      
      // problem.AddResidualBlock(
      //     FourRePoseGraphErr::Creat(
      //         delta_pose.translation(), transform::GetYaw(delta_pose),
      //         ceres_local_map_poses[local_map.first].ypr[2],
      //         ceres_local_map_poses[local_map.first].ypr[1],
      //         options_.relative_weight),
      //     nullptr,
      //     ceres_local_map_poses[local_map.first].t.data(),
      //     &ceres_local_map_poses[local_map.first].ypr[0], it->second.t.data(),
      //     &it->second.ypr[0]);
    }
  }

  // for (const auto& mp_id : repeat_map_points) {
  //     //
  //     for (const auto& local_map_id : mp_id.second) {

  //       if (local_map_id != fix_local_map_id) {
  //         //  LOG(INFO) << mp_id.second.size() << local_map_id<<mp_id.first;
  //         // problem.AddResidualBlock(
  //         //     RepeatMapPointErr::Creat(options_.repeat_mp_weight),
  //         nullptr,
  //         //     ceres_map_points[fix_local_map_id].at(mp_id.first).data(),
  //         //     ceres_map_points[local_map_id].at(mp_id.first).data(),
  //         //     ceres_local_map_poses[local_map_id].t.data(),
  //         //     &ceres_local_map_poses[local_map_id].ypr[0]);

  //         // LOG(INFO) << repeat_map_points.size()<<" "
  //         //           << ceres_map_points[fix_local_map_id].size()<<" "
  //         //           << ceres_map_points[local_map_id].size();
  //       }
  //     }
  // }
  //
  //
  //
  //
  problem.SetParameterBlockConstant(
      ceres_local_map_poses.begin()->second.t.data());
  problem.SetParameterBlockConstant(
      &ceres_local_map_poses.begin()->second.ypr[0]);
  //
  problem.SetParameterBlockConstant(
      &ceres_poses.begin()->second.begin()->second.ypr[0]);
  //
  problem.SetParameterBlockConstant(
      ceres_poses.begin()->second.begin()->second.t.data());
  //

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = options_.max_num_iterations;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres::Solver::Summary summary;
  options.num_threads = options_.ceres_num_threads;
  ceres::Solve(options, &problem, &summary);
  LOG(INFO) << log_info::RED << summary.FullReport() << log_info::RESET;
  //

  //
}
//

void LocalMapOptimization::UpdateLocalMapData(
    std::map<LocalMapId, std::shared_ptr<LocalMap>>* local_maps) {
  std::map<LocalMapId, NodePose>& ceres_local_map_poses =
      ceres_local_map_poses_;
  std::map<LocalMapId, std::map<MapPointId, Eigen::Vector3d>>&
      ceres_map_points = ceres_map_points_;
  std::map<LocalMapId, std::map<KeyFrameId, NodePose>>& ceres_poses =
      ceres_poses_;
  transform::Rigid3d fix_local_map_pose =
      local_maps->begin()->second->LocalPose();
  for (auto& local_map : *local_maps) {
    //
    transform::Rigid3d local_map_local_pose = transform::Rigid3d::Translation(
        ceres_local_map_poses.at(local_map.first).t);
    //
    auto local_map_data = local_map.second->MutableData();
    //
    //
    local_map_data->local_pose =
        fix_local_map_pose *
        transform::Rigid3d(
            ceres_local_map_poses.at(local_map.first).t,
            transform::RollPitchYaw(
                ceres_local_map_poses.at(local_map.first).ypr[2],
                ceres_local_map_poses.at(local_map.first).ypr[1],
                ceres_local_map_poses.at(local_map.first).ypr[0]));

    auto& local_frame_poses = local_map_data->key_frames_ref_pose;
    auto& local_frame_poses_ceres = ceres_poses[local_map.first];
    auto& ceres_map_points_local_map = ceres_map_points[local_map.first];
    for (auto& local_pose : local_frame_poses)
      for (auto& pos : local_frame_poses) {
        //
        if (local_frame_poses_ceres.count(pos.first) == 0) continue;
        //
        pos.second = transform::Rigid3d(
            local_frame_poses_ceres[pos.first].t,
            transform::RollPitchYaw(local_frame_poses_ceres[pos.first].ypr[2],
                                    local_frame_poses_ceres[pos.first].ypr[1],
                                    local_frame_poses_ceres[pos.first].ypr[0])
                .normalized());
      }
    //
    auto& all_local_map_points = local_map_data->map_points;
    //
    for (const auto& mp : all_local_map_points) {
      if (ceres_map_points_local_map.count(mp.id)) {
        // LOG(INFO)<<mp.data.data->pos;
        mp.data.data->pos = ceres_map_points_local_map.at(mp.id);
        // LOG(INFO)<<mp.data.data->pos;
      }
    }
  }

  //
}

// /
std::vector<KeyFrameId>
EssentialGraphLocalMapOptimization::GetKeyLevelConnectedKeyFrames(
    const KeyFrameId& frame_id, const std::vector<int>& levels,
    const LocalMap& local_map) {
  const auto connect_frames_ids =
      local_map.GetCovisibility()->GetConnectedKeyFrames(frame_id);
  if (levels.size() == 1) {
    return connect_frames_ids;
  }

  std::vector<KeyFrameId> result;
  result.insert(result.begin(), connect_frames_ids.begin(),
                connect_frames_ids.end());
  for (auto connect_frames_id : connect_frames_ids) {
    auto scond_connet = GetKeyLevelConnectedKeyFrames(
        connect_frames_id, {levels.begin() + 1, levels.end()}, local_map);
    result.insert(result.begin(), scond_connet.begin(), scond_connet.end());
  }
  //
  return result;
}
//
//
void EssentialGraphLocalMapOptimization::StrategyOptimize(
    std::map<LocalMapId, std::shared_ptr<LocalMap>>* local_maps) {
  //
  CHECK_EQ(local_maps->size(), size_t(1))
      << "EssentialGraphLocalMapOptimization just need local mapsize=1";
  //

  auto& end_local_map_id = local_maps->rbegin()->first;
  auto& end_local_map = *local_maps->rbegin()->second;
  std::map<KeyFrameId, NodePose>& ceres_poses = ceres_poses_[end_local_map_id];
  NodePose& ceres_local_map_poses = ceres_local_map_poses_[end_local_map_id];
  //
  std::map<MapPointId, Eigen::Vector3d>& ceres_map_points =
      ceres_map_points_[end_local_map_id];
  //
  transform::Rigid3d fix_pose = local_maps->begin()->second->LocalPose();
  //
  ceres::Problem problem;
  ceres::LossFunction* loss_function =
      new ceres::HuberLoss(options_.huber_loss);
  ceres::LocalParameterization* quaternion_local =
      new ceres::EigenQuaternionParameterization;
  //
  AddExtricToProblem(&problem);

  //

  auto& end_local_map_ref_kfs = end_local_map.AllKeyFrameRefPose();
  //
  auto end_kf_id = end_local_map_ref_kfs.rbegin()->first;
  //
  //
  auto con_kfs = GetKeyLevelConnectedKeyFrames(
      end_kf_id, ess_options_.convisi_level_search_num, end_local_map);
  //

  std::set<KeyFrameId> conv_kfs_set;
  for (const auto& kf : con_kfs) {
    if (conv_kfs_set.count(kf)) {
    } else {
      conv_kfs_set.emplace(kf);
    }
  }

  if (conv_kfs_set.size() >= size_t(ess_options_.max_con_kf_num)) {
    conv_kfs_set.erase(
        std::prev(conv_kfs_set.begin(), ess_options_.max_con_kf_num),
        conv_kfs_set.end());
  }
  conv_kfs_set.insert(end_kf_id);
  LOG(INFO) << "Esetion local Map node size:" << conv_kfs_set.size();
  std::set<KeyFrameId> adjacent_kfs;
  const KeyFrameId near_id(
      end_kf_id.trajectory_id,
      end_kf_id.keyframe_index + -ess_options_.max_adjacent_kf_num);
  //
  KeyFrameId start_near_id(-1, 0);
  if (end_local_map_ref_kfs.count(near_id)) {
    start_near_id = std::min(*conv_kfs_set.begin(), near_id);
  } else {
    start_near_id = *conv_kfs_set.begin();
  }
  //
  for (size_t i = near_id.keyframe_index; i <= end_kf_id.keyframe_index; i++) {
    const KeyFrameId near_id_(end_kf_id.trajectory_id, i);
    if (end_local_map_ref_kfs.count(near_id_)) {
      // LOG(INFO)<<near_id_;
      // conv_kfs_set.emplace(near_id,0);
      // adjacent_kfs.insert(near_id);
    }
  }

  // for (int i = start_near_id.keyframe_index; i <= end_kf_id.keyframe_index;
  //      i++) {
  //   const KeyFrameId near_id(end_kf_id.trajectory_id, i);
  //   if (end_local_map_ref_kfs.count(near_id)) {
  //     adjacent_kfs.insert(near_id);
  //   }
  // }

  auto local_poses = end_local_map.AllKeyFrameRefPose();
  auto local_kf_datas = end_local_map.AllKeyFrameDatas();
  //
  for (const auto& pos : local_poses) {
    // if (ceres_poses.count(pos.first)) continue;
    if (conv_kfs_set.count(pos.first) == 0 &&
        adjacent_kfs.count(pos.first) == 0)
      continue;
    //
    transform::Rigid3d pose_local_pose = pos.second;
    const Eigen::Vector3d ypr =
        transform::Rot2ypr(pose_local_pose.rotation().toRotationMatrix()) /
        180. * M_PI;
    //

    if (ceres_poses.count(pos.first) == 0) {
      ceres_poses.emplace(pos.first,
                          NodePose{pose_local_pose.translation(),
                                   pose_local_pose.rotation(),
                                   {ypr[0], ypr[1], ypr[2]},
                                   local_kf_datas.at(pos.first).data->pose});
    }
    auto all_map_points = end_local_map.AllMapPoints();
    for (auto const& kf_data : local_kf_datas) {
      auto convisibility = end_local_map.GetCovisibility();
      const auto& frame_map_features =
          convisibility->GetKeyFrameMapPointId(kf_data.id);

      const auto& frame_map_point = frame_map_features.first;
      for (const auto& mp : frame_map_point) {
        if (ceres_map_points.count(mp)) continue;
        //
        const Eigen::Vector3d pos = all_map_points.at(mp).data->pos;
        ceres_map_points.emplace(mp, pos);
      }
    }
  }
  //
  const auto& all_kf_frames = end_local_map.AllKeyFrameDatas();
  auto& kf_rf_frames_poses = end_local_map.AllKeyFrameRefPose();
  // for (auto& local_map : *local_maps) {
  for (const auto& mp_id : ceres_map_points) {
    const auto mp_obsers =
        end_local_map.GetCovisibility()->GetMapPointObserv(mp_id.first);
    for (const auto& ob_kf_f : mp_obsers) {
      const double weitht = options_.re_preject_weight;
      if (ceres_poses.count(ob_kf_f.first) == 0) {
        transform::Rigid3d pose_local_pose =
            kf_rf_frames_poses.at(ob_kf_f.first);
        const Eigen::Vector3d ypr =
            transform::Rot2ypr(pose_local_pose.rotation().toRotationMatrix()) /
            180. * M_PI;
        ceres_poses.emplace(
            ob_kf_f.first,
            NodePose{pose_local_pose.translation(),
                     pose_local_pose.rotation(),
                     {ypr[0], ypr[1], ypr[2]},
                     all_kf_frames.at(ob_kf_f.first).data->pose});
        problem.AddParameterBlock(ceres_poses.at(ob_kf_f.first).t.data(), 3);
        problem.AddParameterBlock(&ceres_poses.at(ob_kf_f.first).ypr[0], 1);
        problem.SetParameterBlockConstant(
            ceres_poses.at(ob_kf_f.first).t.data());
        problem.SetParameterBlockConstant(
            &ceres_poses.at(ob_kf_f.first).ypr[0]);
      }
      problem.AddResidualBlock(
          FourReProjectionBaErr::Creat(all_kf_frames.at(ob_kf_f.first)
                                           .data->features.at(ob_kf_f.second)
                                           .f,
                                       ceres_poses.at(ob_kf_f.first).ypr[2],
                                       ceres_poses.at(ob_kf_f.first).ypr[1],
                                       weitht),
          new ceres::HuberLoss(options_.huber_loss),
          ceres_poses.at(ob_kf_f.first).t.data(),
          &ceres_poses.at(ob_kf_f.first).ypr[0],
          extric_camera_to_imu_[ob_kf_f.second.sequence_id].t.data(),
          extric_camera_to_imu_[ob_kf_f.second.sequence_id].q.coeffs().data(),
          ceres_map_points.at(mp_id.first).data());
    }
  }

  if (adjacent_kfs.size() >= 2) {
    auto adjacent_kfs_it = adjacent_kfs.begin();
    for (auto it = std::next(adjacent_kfs_it); it != adjacent_kfs.end(); ++it) {
      //
      //
      transform::Rigid3d delta_pose =
          ceres_poses.at(*adjacent_kfs_it).local_pose.inverse() *
          ceres_poses.at(*it).local_pose;
      //
      // problem.AddResidualBlock(
      //     FourRePoseGraphErr::Creat(delta_pose.translation(),
      //                               transform::GetYaw(delta_pose),
      //                               ceres_poses.at(*adjacent_kfs_it).ypr[2],
      //                               ceres_poses.at(*adjacent_kfs_it).ypr[1],
      //                               options_.relative_weight),
      //     nullptr, ceres_poses.at(*adjacent_kfs_it).t.data(),
      //     &ceres_poses.at(*adjacent_kfs_it).ypr[0],
      //     ceres_poses.at(*it).t.data(), &ceres_poses.at(*it).ypr[0]);
      // adjacent_kfs_it = it;
    }
    // for (const auto& id : adjacent_kfs) {
    //   problem.AddResidualBlock(
    //       TranslationCostFunctor::Create(
    //           ceres_poses.at(id).t,
    //           options_.relative_local_map_translation_weight),
    //       nullptr, ceres_poses.at(id).t.data());
    // }
  }
  //
  //
  //
  LOG(INFO) << "Local op size: " << ceres_poses.size();
  problem.SetParameterBlockConstant(ceres_poses.begin()->second.t.data());
  problem.SetParameterBlockConstant(&ceres_poses.begin()->second.ypr[0]);

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = options_.max_num_iterations;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres::Solver::Summary summary;
  options.num_threads = options_.ceres_num_threads;
  ceres::Solve(options, &problem, &summary);
  LOG(INFO) << log_info::RED << summary.FullReport() << log_info::RESET;
  //
}

void GraphLocalMapOptimization6TOF::StrategyOptimize(
    std::map<LocalMapId, std::shared_ptr<LocalMap>>* local_maps) {
//
#if 0
  std::map<KeyFrameId, NodePose>& ceres_poses = ceres_poses_;
  //
  std::map<LocalMapId, NodePose>& ceres_local_map_poses =
      ceres_local_map_poses_;
  std::map<MapPointId, Eigen::Vector3d>& ceres_map_points = ceres_map_points_;
  //
  //
  transform::Rigid3d fix_pose = local_maps->begin()->second->LocalPose();
  //
  //
  ceres::Problem problem;
  ceres::LossFunction* loss_function =
      new ceres::HuberLoss(options_.huber_loss);
  ceres::LocalParameterization* quaternion_local =
      new ceres::EigenQuaternionParameterization;
  //
  LOG(INFO) << "Local op local mp size:" << local_maps->size();
  for (size_t i = 0; i < options_.track_sequence.size(); i++) {
    problem.AddParameterBlock(extric_camera_to_imu_[i].q.coeffs().data(), 4);
    problem.AddParameterBlock(extric_camera_to_imu_[i].t.data(), 3);
    problem.SetParameterization(extric_camera_to_imu_[i].q.coeffs().data(),
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
    auto local_poses = local_map.second->AllKeyFrameRefPose();
    //
    for (const auto& pos : local_poses) {
      if (ceres_poses.count(pos.first)) continue;
      //
      transform::Rigid3d pose_local_pose = local_map_local_pose * pos.second;
      ceres_poses.emplace(pos.first, NodePose{pose_local_pose.translation(),
                                              pose_local_pose.rotation()});
      //   const Eigen::Vector3d ypr =
      //       transform::Rot2ypr(pose_local_pose.rotation().toRotationMatrix())
      //       / 180. * M_PI;
      //   //
      //   ceres_poses.emplace(pos.first, NodePose{
      //                                      pose_local_pose.translation(),
      //                                      pose_local_pose.rotation(),
      //                                      {ypr[0], ypr[1], ypr[2]},
      //                                  });
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
            ReProjectionBaErr::Create(
                kf_data.data.data->features.at(frame_features[i]).f,
                options_.re_preject_weight),
            loss_function, ceres_poses.at(kf_data.id).t.data(),
            ceres_poses.at(kf_data.id).q.coeffs().data(),
            //
            extric_camera_to_imu_[frame_features[i].sequence_id].t.data(),
            extric_camera_to_imu_[frame_features[i].sequence_id]
                .q.coeffs()
                .data(),
            //
            ceres_map_points.at(frame_map_point[i]).data(),
            ceres_local_map_poses.at(local_map.first).t.data());

        if (options_.only_pose_graph) {
          problem.SetParameterBlockConstant(
              ceres_map_points.at(frame_map_point[i]).data());
        }
      }
    }
  }
  //
  problem.SetParameterBlockConstant(
      ceres_local_map_poses.begin()->second.t.data());

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = options_.max_num_iterations;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  LOG(INFO) << log_info::RED << summary.FullReport() << log_info::RESET;
  //
#endif
}

}  // namespace mapping
}  // namespace jarvis
