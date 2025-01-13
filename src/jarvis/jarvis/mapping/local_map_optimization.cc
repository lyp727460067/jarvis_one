#include "jarvis/mapping/local_map_optimization.h"
#include "jarvis/estimator/factor/pose_local_parameterization.h"
#include "factor/projection_factor.h"
#include "jarvis/common/time.h"

#include "jarvis/mapping/auto_factor/re_projection_err.h"
#include "jarvis/mapping/auto_factor/relative_pose_graph.h"
#include "jarvis/mapping/local_map.h"
#include "jarvis/mapping/auto_factor/pose_factor.h"
namespace jarvis {
namespace mapping {

struct NodePose {
  Eigen::Vector3d t;
  Eigen::Quaterniond q;
  double ypr[3];
  transform::Rigid3d local_pose;
};

//
transform::Rigid3d ToTransform(const NodePose& node) {
  return transform::Rigid3d(node.t, node.q);
}
//
//
LocalMapOptimization::LocalMapOptimization(const LocalMapOptimizationOption &option)
    : options_(option), extric_camera_to_imu_(option.extric_camera_to_imu) {
    para_Ex = new double *[options_.extric_camera_to_imu.size()];
    for (size_t i = 0; i < options_.extric_camera_to_imu.size(); i++) {
        para_Ex[i] = new double[jarvis::estimator::SIZE_POSE];
        memset(para_Ex[i], 0, sizeof(double) * jarvis::estimator::SIZE_POSE);
        para_Ex[i][0] = options_.extric_camera_to_imu[i].translation().x();
        para_Ex[i][1] = options_.extric_camera_to_imu[i].translation().y();
        para_Ex[i][2] = options_.extric_camera_to_imu[i].translation().z();
        para_Ex[i][3] = options_.extric_camera_to_imu[i].rotation().w();
        para_Ex[i][4] = options_.extric_camera_to_imu[i].rotation().x();
        para_Ex[i][5] = options_.extric_camera_to_imu[i].rotation().y();
        para_Ex[i][6] = options_.extric_camera_to_imu[i].rotation().z();
    }
}


LocalMapOptimization::~LocalMapOptimization(){
    for(size_t i = 0; i < options_.extric_camera_to_imu.size(); i++)
        delete[] para_Ex[i];
    delete[] para_Ex;
}


void LocalMapOptimization::Optimize(LocalMapOptimizationData *data) {
        std::cout << "start optimize" << std::endl;
        ceres::Problem problem;
        ceres::LossFunction *loss_function = new ceres::HuberLoss(options_.huber_loss);
        ceres::ParameterBlockOrdering *ordering = new ceres::ParameterBlockOrdering();

        int num_OptKF = data->frame_datas.size();
        int num_MapPoints = data->con_map_points.size();
        std::cout << "key frame count: " << num_OptKF << std::endl;
        std::cout << "map point count: " << num_MapPoints << std::endl;

        // 为优化变量分配空间
        para_Pose = new double *[num_OptKF];
        for (int i = 0; i < num_OptKF; i++) {
            para_Pose[i] = new double[jarvis::estimator::SIZE_POSE];
            memset(para_Pose[i], 0, sizeof(double) * jarvis::estimator::SIZE_POSE);
        }

        para_MapPoint = new double *[num_MapPoints];
        for (int i = 0; i < num_MapPoints; i++) {
            para_MapPoint[i] = new double[jarvis::estimator::SIZE_MAPPOINT];
            memset(para_MapPoint[i], 0, sizeof(double) * jarvis::estimator::SIZE_MAPPOINT);
        }

        para_SpeedBias = new double *[num_OptKF];
        for (int i = 0; i < num_OptKF; i++) {
            para_SpeedBias[i] = new double[jarvis::estimator::SIZE_SPEEDBIAS];
            memset(para_SpeedBias[i], 0, sizeof(double) * jarvis::estimator::SIZE_SPEEDBIAS);
        }

        // 设置初值并记录id
        std::map<KeyFrameId, int> KeyFrameIds;
        int j = 0;
        for (auto &frame_data : data->frame_datas) {
            const auto t = frame_data.second.pose.translation();
            const auto q = frame_data.second.pose.rotation();
            para_Pose[j][0] = t.x();
            para_Pose[j][1] = t.y();
            para_Pose[j][2] = t.z();
            para_Pose[j][3] = q.w();
            para_Pose[j][4] = q.x();
            para_Pose[j][5] = q.y();
            para_Pose[j][6] = q.z();
            KeyFrameIds[frame_data.first] = j;
            j++;
        }

        std::map<MapPointId, int> MapPointIds;
        j = 0;
        for (auto &pt : data->con_map_points) {
            const Eigen::Vector3d p = pt.second.pos;
            para_MapPoint[j][0] = p.x();
            para_MapPoint[j][1] = p.y();
            para_MapPoint[j][2] = p.z();
            MapPointIds[pt.first] = j;
            j++;
        }

        // 将关键帧pose作为优化变量加入问题
        for (int i = 0; i < num_OptKF; i++) {
            ceres::LocalParameterization *local_parameterization =
                new jarvis::estimator::PoseLocalParameterization();
            problem.AddParameterBlock(para_Pose[i], jarvis::estimator::SIZE_POSE, local_parameterization);
            ordering->AddElementToGroup(para_Pose[i], 1);
        }
        // 将第一帧位姿固定
        problem.SetParameterBlockConstant(para_Pose[0]);

        // 将地图点坐标作为优化变量加入问题
        for (int i = 0; i < num_MapPoints; i++) {
            problem.AddParameterBlock(para_MapPoint[i], jarvis::estimator::SIZE_MAPPOINT);
            ordering->AddElementToGroup(para_MapPoint[i], 1);

            if (options_.only_pose_graph) {
                problem.SetParameterBlockConstant(para_MapPoint[i]);
            }
        }

        // 将外参作为优化变量加入问题
        for (size_t i = 0; i < options_.extric_camera_to_imu.size(); i++) {
            ceres::LocalParameterization *local_parameterization =
                new jarvis::estimator::PoseLocalParameterization();
            problem.AddParameterBlock(para_Ex[i], jarvis::estimator::SIZE_POSE, local_parameterization);
            ordering->AddElementToGroup(para_Ex[i], 1);

            if (!options_.optimize_extric) {
                problem.SetParameterBlockConstant(para_Ex[i]);
            }
        }

        // 将IMU作为优化变量加入问题
        if (options_.optimize_imu) {
            for (int i = 0; i < num_OptKF - 1; i++) {
                problem.AddParameterBlock(para_SpeedBias[i], jarvis::estimator::SIZE_SPEEDBIAS);
                ordering->AddElementToGroup(para_SpeedBias[i], 1);
            }
        }

        // 增加约束
        int constraint_count = 0;
        for (auto &map_point : data->con_map_points) {
            for (auto &point_data : map_point.second.con_frame_datas) {
                ceres::CostFunction *cost_function;
                cost_function = ProjectionFactor::Create(
                    data->feature_datas[point_data.second]->f.x(),
                    data->feature_datas[point_data.second]->f.y());

                
                problem.AddResidualBlock(cost_function, loss_function,
                                         para_Pose[KeyFrameIds[point_data.first]],
                                         para_MapPoint[MapPointIds[map_point.first]],
                                         para_Ex[point_data.second.sequence_id]);
                // std::cout << "Pose: " << para_Pose[KeyFrameIds[point_data.first]][0] << ", " 
                //           << para_Pose[KeyFrameIds[point_data.first]][1] << ", " 
                //           << para_Pose[KeyFrameIds[point_data.first]][2] << ", " 
                //           << para_Pose[KeyFrameIds[point_data.first]][3] << ", " 
                //           << para_Pose[KeyFrameIds[point_data.first]][4] << ", " 
                //           << para_Pose[KeyFrameIds[point_data.first]][5] << ", " 
                //           << para_Pose[KeyFrameIds[point_data.first]][6] << std::endl;
                // std::cout << "map point: " << para_MapPoint[MapPointIds[map_point.first]][0] << ", "
                //           << para_MapPoint[MapPointIds[map_point.first]][1] << ", "
                //           << para_MapPoint[MapPointIds[map_point.first]][2] << std::endl;
                constraint_count++;
            }
        }
        std::cout << "constraints count: " << constraint_count << std::endl;

        // 注意map是有序的,按frame的id排序,而IMU值为两相邻frame构建约束,故顺序构建即可
        // imu_datas的大小比frame_datas少1
        if (options_.optimize_imu) {
            int i = 0;
            j = 1;
            CHECK_EQ(data->frame_datas.size() - 1, data->imu_datas.size());
            for (auto &imu_data : data->imu_datas) {
                jarvis::estimator::IMUFactor *imu_factor =
                    new jarvis::estimator::IMUFactor(imu_data.second);
                problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i],
                                          para_Pose[j], para_SpeedBias[j]);
                i++;
                j++;
            }
        }



        // 问题求解
        ceres::Solver::Options options;
        options.linear_solver_ordering.reset(ordering);
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.num_threads = 1;
        options.trust_region_strategy_type = ceres::DOGLEG;
        options.sparse_linear_algebra_library_type = ceres::NO_SPARSE;
        options.use_explicit_schur_complement = true; // 是否使用显式计算的Schur补矩阵，默认为false
        options.use_nonmonotonic_steps = true;        // 是否允许目标函数值没有严格下降
        options.max_num_iterations = 5;
        ceres::Solver::Summary summary;
        jarvis::estimator::TicToc tt;
        ceres::Solve(options, &problem, &summary);
        std::cout << "solve time: " << tt.toc() << std::endl;

        // 获取结果
        j = 0;
        for (auto &frame_data : data->frame_datas) {
            Eigen::Vector3d t(para_Pose[j][0], para_Pose[j][1], para_Pose[j][2]);
            Eigen::Quaterniond q(para_Pose[j][3], para_Pose[j][4], para_Pose[j][5], para_Pose[j][6]);
            frame_data.second.pose = transform::Rigid3d(t, q);
            j++;
        }

        // 释放內存
        for (int i = 0; i < num_OptKF; i++)
            delete[] para_Pose[i];

        for (int i = 0; i < num_MapPoints; i++)
            delete[] para_MapPoint[i];

        delete[] para_Pose;
        delete[] para_MapPoint;
}


//
//
void LocalMapOptimization::Optimize(
    std::map<LocalMapId, std::shared_ptr<LocalMap>>* local_maps) {
  StrategyOptimize(local_maps);
}
//
void LocalMapOptimization::StrategyOptimize(
    std::map<LocalMapId, std::shared_ptr<LocalMap>>* local_maps) {
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
    if (options_.fix_extric) {
      problem.SetParameterBlockConstant(ex_rotation[i].coeffs().data());
      problem.SetParameterBlockConstant(ex_traslation[i].data());
    }
  }

  for (auto& local_map : *local_maps) {
    transform::Rigid3d local_map_local_pose =
        fix_pose.inverse() * local_map.second->LocalPose();
    LOG(INFO)<<local_map_local_pose ;
    ceres_local_map_poses.emplace(local_map.first,
                                  NodePose{
                                      local_map_local_pose.translation(),
                                      local_map_local_pose.rotation(),
                                  });
    LOG(INFO) << local_map_local_pose.translation().transpose();
    auto local_poses = local_map.second->AllKeyFrameRefPose();
    auto local_kf_datas = local_map.second->AllKeyFrameDatas();
    //
    for (const auto& pos : local_poses) {
      if (ceres_poses.count(pos.first)) continue;
      //
      transform::Rigid3d pose_local_pose = local_map_local_pose * pos.second;
      const Eigen::Vector3d ypr =
          transform::Rot2ypr(pose_local_pose.rotation().toRotationMatrix()) /
          180. * M_PI;
      //
      ceres_poses.emplace(pos.first,
                          NodePose{pose_local_pose.translation(),
                                   pose_local_pose.rotation(),
                                   {ypr[0], ypr[1], ypr[2]},
                                   local_kf_datas.at(pos.first).data->pose});


                                   
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

  auto ceres_poses_it = ceres_poses.begin();
  for (auto it = std::next(ceres_poses_it); it != ceres_poses.end(); ++it) {
    //
    transform::Rigid3d delta_pose =
        ceres_poses_it->second.local_pose.inverse() * it->second.local_pose;
    //
    problem.AddResidualBlock(
        FourRePoseGraphErr::Creat(
            delta_pose.translation(), transform::GetYaw(delta_pose),
            ceres_poses_it->second.ypr[2], ceres_poses_it->second.ypr[1],
            options_.relative_weight),
        nullptr, ceres_poses_it->second.t.data(),
        &ceres_poses_it->second.ypr[0], it->second.t.data(),
        &it->second.ypr[0]);
    ceres_poses_it = it;
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
  options.num_threads = options_.ceres_num_threads;
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
    // local_map_data->local_pose =
    //     fix_pose * transform::Rigid3d::Translation(
    //                    ceres_local_map_poses.at(local_map.first).t);

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
//
// /
std::vector<std::pair<KeyFrameId, int>>
EssentialGraphLocalMapOptimization::GetKeyLevelConnectedKeyFrames(
    const KeyFrameId& frame_id, const std::vector<int>& levels,
    const LocalMap& local_map) {
  const auto connect_frames_ids =
      local_map.GetCovisibility()->GetOrderConnectedKeyFrames(frame_id,
                                                              *levels.begin());
  if (levels.size() == 1) {
    return connect_frames_ids;
  }

  std::vector<std::pair<KeyFrameId, int>> result;
  for (auto connect_frames_id : connect_frames_ids) {
    auto scond_connet = GetKeyLevelConnectedKeyFrames(
        connect_frames_id.first, {levels.begin() + 1, levels.end()}, local_map);
    result.insert(result.begin(), scond_connet.begin(), scond_connet.end());
  }
  //
  return result;
}
//
//
void EssentialGraphLocalMapOptimization::StrategyOptimize(
    std::map<LocalMapId, std::shared_ptr<LocalMap>>* local_maps) {
  std::map<KeyFrameId, NodePose> ceres_poses;
  //
  std::map<LocalMapId, NodePose> ceres_local_map_poses;
  std::map<MapPointId, Eigen::Vector3d> ceres_map_points;
  transform::Rigid3d fix_pose = local_maps->begin()->second->LocalPose();
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
    if (options_.fix_extric) {
      problem.SetParameterBlockConstant(ex_rotation[i].coeffs().data());
      problem.SetParameterBlockConstant(ex_traslation[i].data());
    }
  }
  //

  auto& end_local_map = *local_maps->rbegin()->second;
  auto& end_local_map_ref_kfs = end_local_map.AllKeyFrameRefPose();
  //
  auto end_kf_id = end_local_map_ref_kfs.rbegin()->first;
  //
  //
  auto con_kfs = GetKeyLevelConnectedKeyFrames(
      end_kf_id, ess_options_.convisi_level_search_num, end_local_map);
  //

  std::map<KeyFrameId, int> conv_kfs_set;
  for (const auto& kf : con_kfs) {
    if (conv_kfs_set.count(kf.first)) {
      conv_kfs_set.at(kf.first) += kf.second;
    } else {
      conv_kfs_set.emplace(kf);
    }
  }


  // if (conv_kfs_set.size() >= ess_options_.max_con_kf_num) {
  //   conv_kfs_set.erase(
  //       conv_kfs_set.begin(),
  //       std::prev(conv_kfs_set.end(), ess_options_.max_con_kf_num));
  // }
  LOG(INFO) << "Esetion local Map node size:" << conv_kfs_set.size();
  std::set<KeyFrameId> adjacent_kfs;
  const KeyFrameId near_id(
      end_kf_id.trajectory_id,
      end_kf_id.keyframe_index + -ess_options_.max_adjacent_kf_num);
  //
  KeyFrameId start_near_id(-1, 0);
  if (end_local_map_ref_kfs.count(near_id)) {
    start_near_id = std::min(conv_kfs_set.begin()->first, near_id);
  } else {
    start_near_id = conv_kfs_set.begin()->first;
  }
  //
  for (int i = start_near_id.keyframe_index; i <= end_kf_id.keyframe_index;
       i++) {
    const KeyFrameId near_id(end_kf_id.trajectory_id, i);
    if (end_local_map_ref_kfs.count(near_id)) {
      adjacent_kfs.insert(near_id);
    }
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
    auto local_kf_datas = local_map.second->AllKeyFrameDatas();
    //
    for (const auto& pos : local_poses) {
      // if (ceres_poses.count(pos.first)) continue;
      if (conv_kfs_set.count(pos.first) == 0 &&
          adjacent_kfs.count(pos.first) == 0)
        continue;
      //
      transform::Rigid3d pose_local_pose = local_map_local_pose * pos.second;
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
      auto all_map_points = local_map.second->AllMapPoints();
      for (auto const& kf_data : local_kf_datas) {
        auto convisibility = end_local_map.GetCovisibility();
        const auto& frame_map_features =
            convisibility->GetKeyFrameMapPointId(kf_data.id);

        const auto& frame_map_point = frame_map_features.first;
        for (const auto& mp : frame_map_point) {
          if (ceres_map_points.count(mp)) continue;
          //
          const Eigen::Vector3d pos =
              local_map_local_pose * all_map_points.at(mp).data->pos;
          ceres_map_points.emplace(mp, pos);
        }
      }
    }
  }
  //

  for (auto& local_map : *local_maps) {
    const auto& all_kf_frames = local_map.second->AllKeyFrameDatas();
    auto& kf_rf_frames_poses = local_map.second->AllKeyFrameRefPose();
    //
    auto convisibility = local_map.second->GetCovisibility();
    for (auto const& kf_data : all_kf_frames) {
      if (ceres_poses.count(kf_data.id) == 0) continue;
      const auto& frame_map_features =
          convisibility->GetKeyFrameMapPointId(kf_data.id);
      const auto& frame_map_point = frame_map_features.first;
      const auto& frame_features = frame_map_features.second;

      for (size_t i = 0; i < frame_map_point.size(); i++) {
        //
        // const double weitht =
        //     conv_kfs_set.count(kf_data.id)
        //         ? conv_kfs_set.at(kf_data.id) * options_.re_preject_weight
        //         : options_.re_preject_weight;
        const double weitht = options_.re_preject_weight;
        problem.AddResidualBlock(
            FourReProjectionBaErr::Creat(
                kf_data.data.data->features.at(frame_features[i]).f,
                ceres_poses.at(kf_data.id).ypr[2],
                ceres_poses.at(kf_data.id).ypr[1], weitht),
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
  if (adjacent_kfs.size() >= 2) {
    auto adjacent_kfs_it = adjacent_kfs.begin();
    for (auto it = std::next(adjacent_kfs_it); it != adjacent_kfs.end(); ++it) {
      //
      //
      transform::Rigid3d delta_pose =
          ceres_poses.at(*adjacent_kfs_it).local_pose.inverse() *
          ceres_poses.at(*it).local_pose;
      //
      problem.AddResidualBlock(
          FourRePoseGraphErr::Creat(delta_pose.translation(),
                                    transform::GetYaw(delta_pose),
                                    ceres_poses.at(*adjacent_kfs_it).ypr[2],
                                    ceres_poses.at(*adjacent_kfs_it).ypr[1],
                                    options_.relative_weight),
          nullptr, ceres_poses.at(*adjacent_kfs_it).t.data(),
          &ceres_poses.at(*adjacent_kfs_it).ypr[0],
          ceres_poses.at(*it).t.data(), &ceres_poses.at(*it).ypr[0]);
      adjacent_kfs_it = it;
    }
    for (const auto& id : adjacent_kfs) {
      problem.AddResidualBlock(
          TranslationCostFunctor::Create(
              ceres_poses.at(id).t,
              options_.relative_local_map_translation_weight),
          nullptr, ceres_poses.at(id).t.data());
    }
  }
  //
  //
  //
  LOG(INFO) << "Local op size: " << ceres_poses.size();
  problem.SetParameterBlockConstant(
      ceres_local_map_poses.begin()->second.t.data());
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
        //
        if (ceres_poses.count(pos.first) == 0) continue;
        //
        pos.second = local_map_local_pose.inverse() *
                     transform::Rigid3d(
                         ceres_poses[pos.first].t,
                         transform::RollPitchYaw(ceres_poses[pos.first].ypr[2],
                                                 ceres_poses[pos.first].ypr[1],
                                                 ceres_poses[pos.first].ypr[0])
                             .normalized());
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


void LocalMapOptimization::Optimize2(
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
  ceres::LossFunction *loss_function = new ceres::HuberLoss(options_.huber_loss);
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
      ceres_poses.emplace(pos.first, NodePose{
                                         pose_local_pose.translation(),
                                         pose_local_pose.rotation()});
      //   const Eigen::Vector3d ypr =
      //       transform::Rot2ypr(pose_local_pose.rotation().toRotationMatrix()) /
      //       180. * M_PI;
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
            loss_function,
            ceres_poses.at(kf_data.id).t.data(),
            ceres_poses.at(kf_data.id).q.coeffs().data(),
            ex_traslation[frame_features[i].sequence_id].data(),
            ex_rotation[frame_features[i].sequence_id].coeffs().data(),
            ceres_map_points.at(frame_map_point[i]).data(),
            ceres_local_map_poses.at(local_map.first).t.data());
        
        if (options_.only_pose_graph) {
            problem.SetParameterBlockConstant(ceres_map_points.at(frame_map_point[i]).data());
        }
      }
    }
  }
  //
  // 固定參数不参与优化
  if(!options_.optimize_extric){
    for (size_t i = 0; i < options_.track_sequence.size(); i++) {
        problem.SetParameterBlockConstant(ex_rotation[i].coeffs().data());
        problem.SetParameterBlockConstant(ex_traslation[i].data());
    }
  }

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
                                              ceres_poses[pos.first].q.normalized());
              //   pos.second = local_map_local_pose.inverse() *
              //                transform::Rigid3d(ceres_poses[pos.first].t,
              //                                   transform::RollPitchYaw(
              //                                       ceres_poses[pos.first].ypr[2],
              //                                       ceres_poses[pos.first].ypr[1],
              //                                       ceres_poses[pos.first].ypr[0])
              //                                       .normalized());
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
