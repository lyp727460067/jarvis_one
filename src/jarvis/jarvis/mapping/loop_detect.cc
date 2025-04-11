#include "jarvis/mapping/loop_detect.h"

namespace jarvis {
namespace mapping {
//
void LoopDetect::Detect(
    const std::pair<LocalMapId, std::shared_ptr<LocalMap>>& local_map,
    const std::map<KeyFrameId, KeyFrameData>& kf_datas, double min_score) {
  //
  CHECK_GT(kf_datas.size(), 3);

  auto first_kf_data = *kf_datas.begin()->second;
  //
  loop_result_catchs_.emplace_back();
  loop_result_catchs_.back().target_local_id = local_map.first;
  loop_result_catchs_.bac auto detect_node_task =
      std::make_unique<common::Task>();
  std::map<KeyFrameId, KeyFrameData> kf_datas_temp = kf_datas;
  k().target_kf_id = *kf_datas.begin()->first;
  //

  LoopResultData* this_kf_result_catch_ptr = &loop_result_catchs_.back();
  //
  if (!data_base_insert_task_hanlde.count(local_map.first)) {
    auto data_base_task = std::make_unique<common::Task>();
    data_base_insert_task_hanlde[local_map.first] = data_base_task->SetWorkItem(
        [first_kf_data, local_map, this, this_kf_result_catch_ptr]() {
          for (const auto& data :
               local_map.second->ConstData().key_frames_datas) {
            key_frame_data_base_[local_map.first]->AddData(data.id,
                                                           data.data.data);
          }
        });
    thread_pool_->Schedule(std::move(data_base_task));
  }

  auto detect_node_task = std::make_unique<common::Task>();
  auto detect_node_task_handle = detect_node_task->SetWorkItem([]() {
    //
    std::unique_ptr<ConstraintConsistentFilter> consistent_filter =
        std::make_unique<ConstraintConsistentFilter>(
            options_.constraint_consistent_filter_num);
    //
    for (auto kf_data : kf_datas) {
      DetectForOne(*local_map.second, kf_data.first, kf_data.second,
                   key_frame_data_base_[local_map.first].get(),
                   this_kf_result_catch_ptr, consistent_filter.get(),
                   min_score);
    };
  });
  //
  detect_node_task->AddDependency(
      data_base_insert_task_hanlde[local_map.first]);
  thread_pool_->Schedule(std::move(detect_node_task));
  //
  finish_task_->AddDependency(detect_node_task_handle);
  //
  finish_task_->SetWorkItem([this_kf_result_catch_ptr]() {
    CalculatedSingleResultFinish(this_kf_result_catch_ptr);
  });
  //
  thread_pool_->Schedule(std::move(finish_task_));
  finish_task_ = std::make_unique<common::Task>();
}
//

void LoopDetect::CalculatedSingleResultFinish(LoopResultData* data) {}
//
void LoopDetect::ContinueAndDistanceCheck(std::shared_ptr<LocalMap> local_map,
                                          LoopResultData* data) {}
void LoopDetect::WhenDone(
    std::function<void(std::vector<std::shared_ptr<LoopDetctResult>>)>&&
        result) {}

void LoopDetect::DetectForOne(
    std::shared_ptr<LocalMap> local_map, const KeyFrameData& kf_data,
    KeyFrameId kf_id;
    const KeyFrameDataBase* data_base, const double min_score,
    std::unique_ptr<ConstraintConsistentFilter>* consistent_filte,
    LoopResultData* result) {
  //
  const auto candidate_kfs = data_base->FindSimilarCandidate(
      kf_data.data, NotNeedToDetectKf(local_map), min_score);
  //

  auto const best_candidata_kfs =
      FilterBestDbowResultWithCovisibility(candidate_kfs);
  if (best_candidata_kfs.empty()) {
    (*consistent_filter) = std::make_unique<ConstraintConsistentFilter>(
        options_.constraint_consistent_filter_num);
    return;
  }
  //
  auto filter_candidate_ids =
      FilterCandidata(candidate_kfs, &consistent_filter);
  if (filter_candidate_ids.empty()) {
    return;
  }

  for (const auto& candidata_kf : filter_candidate_ids) {
    auto constraint = ComputeConstraint(candidata_kf, id);
    if (constraint != nullptr) {
      consistent_filter = std::make_unique<ConstraintConsistentFilter>(
          options_.constraint_consistent_filter_num);
      //
    }
  }
}
//

std::unique_ptr<LoopDetctResult> LoopDetect::ComputeConstraint(
    std::shared_ptr<LocalMap> local_map, KeyFrameId kf_id;
    const KeyFrameData& kf_data, LoopResultData* result) {
  //
  const auto& all_key_frame_data = const_map_manager_->KeyAllFrameDatas();
  auto const& candidata_data = all_key_frame_data.at(candidate_id);
  auto const& target_data = all_key_frame_data.at(target_id);
  auto const& target_key_points = target_data.constant_data->key_points;
  auto const& target_descriptor = target_data.constant_data->descriptors;
  auto const& target_dbow_vec = target_data.constant_data->dbow_data;
  //
  const auto candidata_map_points_with_ids =
      const_map_manager_->GetKeyFrameMapPointsData(candidate_id);
  CHECK(camera_base_);
  const std::vector<Eigen::Vector3d> normal_points =
      camera_base_->UndistortPointsNormal(target_key_points);

#ifdef __ENABLE_MAP_BUILDER_TIME_COST_PRINT__
  auto start = std::chrono::high_resolution_clock::now();
#endif
  auto pnp_pose = ComputePnpPose(candidate_id, candidata_map_points_with_ids,
                                 target_data, normal_points);
  //

  if (pnp_pose.second.empty()) {
    // LOG(INFO) << "pnp empty";
    return nullptr;
  }
  auto pose = pnp_pose.first;
  //
  const transform::Rigid3d candidate_correct_pose =
      target_data.pose * pnp_pose.first *
      all_key_frame_data.at(candidate_id).pose;
  //
  //

  auto const target_key_frame_map_point_ids =
      const_map_manager_->GetKeyFrameMapPointsData(target_id);
  auto target_mp_project_candidata_check_pair_ids =
      CheckCandidate(candidata_data, target_key_frame_map_point_ids,
                     candidate_correct_pose, {});

  //
  auto candidate_mp_project_target_check_pair_ids = CheckCandidate(
      target_data, candidata_map_points_with_ids, pose.inverse(), {});
  //

  if (options_.print_constraint_info) {
    LOG(INFO) << "target_mp_project_candidata_check_pair_ids "
              << target_mp_project_candidata_check_pair_ids.size();
    LOG(INFO) << "candidate_mp_project_target_check_pair_ids "
              << candidate_mp_project_target_check_pair_ids.size();
    //
  }
  if (target_mp_project_candidata_check_pair_ids.size() <
          options_.candidata_reproject_min_num ||
      candidate_mp_project_target_check_pair_ids.size() <
          options_.candidata_reproject_min_num) {
    if (target_mp_project_candidata_check_pair_ids.size() <
            options_.candidata_reproject_second_min_num ||
        candidate_mp_project_target_check_pair_ids.size() <
            options_.candidata_reproject_second_min_num) {
    }
    if (!((candidate_mp_project_target_check_pair_ids.size() >
               target_mp_project_candidata_check_pair_ids.size() * 2 &&
           target_mp_project_candidata_check_pair_ids.size() >
               options_.candidata_reproject_second_min_num) ||
          (target_mp_project_candidata_check_pair_ids.size() >
               candidate_mp_project_target_check_pair_ids.size() * 3 &&
           candidate_mp_project_target_check_pair_ids.size() >
               options_.candidata_reproject_second_min_num

           ))) {
      return nullptr;
    }
  }

  //
  std::vector<Eigen::Vector2d> normal_2d_temp;
  std::vector<Eigen::Vector3d> map_points_temp;
  std::vector<std::pair<int, int>> temp_pair_id;

  for (int i = 0; i < candidate_mp_project_target_check_pair_ids.size(); i++) {
    // 地图点删除，要主要判断
    if (candidata_map_points_with_ids.first.count(
            candidate_mp_project_target_check_pair_ids[i].second) == 0) {
      continue;
    }
    std::pair<int, int> pair_points(
        candidate_mp_project_target_check_pair_ids[i].second,
        candidate_mp_project_target_check_pair_ids[i].first);
    if (std::find_if(pnp_pose.second.begin(), pnp_pose.second.end(),
                     [&](const std::pair<int, int>& v) {
                       return (v.first == pair_points.first) ||
                              (v.second == pair_points.second);
                     }) == pnp_pose.second.end()) {
      temp_pair_id.emplace_back(pair_points);
    }
  }
  // test_data::WriteImageWithKeyPoint(
  //     *candidata_data.constant_data->image,
  //     *target_data.constant_data->image,
  //     candidata_data.constant_data->key_points, target_key_points,
  //     kTestResultDir +
  //         std::to_string(common::ToUniversal(target_data.constant_data->time))
  //         +
  //         ".png",
  //     " ", temp_pair_id,
  //     candidata_data.constant_data->tracking_index_size,
  //     target_data.constant_data->tracking_index_size);

  // temp_pair_id.insert(
  //     temp_pair_id.end(),
  //     pnp_pose.second.begin(), pnp_pose.second.end());

  for (int i = 0; i < temp_pair_id.size(); i++) {
    // if (!candidata_map_points_with_ids.second
    //          .at(candidata_map_points_with_ids.first.at(temp_pair_id[i].first))
    //          .data->new_construct)
    // continue;

    // // 先暂时用position 代替
    map_points_temp.push_back(
        candidata_map_points_with_ids.second
            .at(candidata_map_points_with_ids.first.at(temp_pair_id[i].first))
            .global_pose);
    normal_2d_temp.push_back(
        normal_points.at(temp_pair_id[i].second).head<2>());
  }

  // auto pnp_pose1 = ComputePoseWithPnp(map_points_temp, normal_2d_temp,
  //                                    transform::Rigid3d::Identity());

  // pose = pnp_pose1.first;
  //
  //
  // auto matched_id =
  //     SearchByProjection(candidate_id, target_id, pose.inverse(), {});
  // const auto& all_map_points = const_map_manager_->GetAllMapPoints();
  // for (auto& index : matched_id) {
  //   // // 先暂时用position 代替
  //   // if (!all_map_points.at(index.second).data->new_construct) continue;

  //   map_points_temp.push_back(all_map_points.at(index.second).global_pose);
  //   normal_2d_temp.push_back(normal_points.at(index.first).head<2>());
  // }
#ifdef __ENABLE_MAP_BUILDER_TIME_COST_PRINT__
  start = std::chrono::high_resolution_clock::now();
#endif
  pose = OptimizationPose(map_points_temp, normal_2d_temp, pnp_pose.first,
                          std::array<double, 2>{10, 10});
  //

#ifdef __ENABLE_MAP_BUILDER_TIME_COST_PRINT__
  LOG(INFO) << "OptimizationPose time: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::high_resolution_clock::now() - start)
                   .count();
#endif
  const auto delta_pose = pose * candidata_data.pose;
  //
  const double delta_yaw =
      NormalizeAngle(transform::GetYaw(candidata_data.pose.rotation()) -
                     transform::GetYaw(pose.inverse().rotation()));
  //
  //
  if (abs(delta_yaw) > common::DegToRad(options_.constraint_max_yaw) ||
      delta_pose.translation().norm() > options_.constraint_max_distance) {
    LOG(WARNING) << "Detel yaw : " << delta_yaw
                 << "Delta Pose : " << delta_pose.translation().norm()
                 << "Is More Than option..";
    return nullptr;
  }

  if (options_.print_constraint_info) {
    // LOG(INFO) << "SearchByProjection size : " << matched_id.size();
    std::stringstream info;
    info << "\ntarget " << target_id << "candidate: " << candidate_id
         << " pnp pose " << pnp_pose.first << "optimization " << pose;
    info << "\nlocal pose ... "
         << " target: " << target_data.constant_data->pose
         << " candidate: " << candidata_data.constant_data->pose;
    info << "delta pose :" << delta_pose << " yaw :" << delta_yaw;
    LOG(INFO) << info.str();
  }

  // {
  // Eigen::Matrix3d calibration_temp;
  // calibration_temp << 0.99999212, -0.00372248, 0.00138305, -0.00374066,
  //     -0.99990349, 0.01337998, 0.00133311, -0.01338505, -0.99990953;
  // const auto calibaraion_ratation = Eigen::Quaterniond(calibration_temp);

  // const auto euler_candidate = transform::Rot2ypr(
  //     (candidata_data.pose.rotation() * calibaraion_ratation.conjugate())
  //         .toRotationMatrix());
  // const auto euler_target = transform::Rot2ypr(
  //     (pose.inverse().rotation() * calibaraion_ratation.conjugate())
  //         .toRotationMatrix());
  // //

  // LOG(INFO) << "imu rot2ypr :"
  //           << NormalizeAngleDeg(euler_candidate.x() - euler_target.x());

  // LOG(INFO) << "transform get angle:"
  //           << common::RadToDeg(NormalizeAngle(
  //                  transform::GetYaw(candidata_data.pose.rotation() *
  //                                    calibaraion_ratation.conjugate()) -
  //                  transform::GetYaw(pose.inverse().rotation() *
  //                                    calibaraion_ratation.conjugate())));
  // //
  // }

  return std::make_unique<Constraint>(Constraint{
      target_id, candidate_id,
      Constraint::Pose{delta_pose,
                       options_.same_mapping_loop_translation_weight,
                       options_.same_mapping_loop_rotation_weight, delta_yaw},
      Constraint::Tag::rot_pos,
      std::make_shared<Constraint::MatchId>(Constraint::MatchId{
          std::move(temp_pair_id),
          std::move(target_mp_project_candidata_check_pair_ids),
          /*std::move(matched_id)*/})});
  //
}
//
std::set<KeyFrameId> LoopDetect::NotNeedToDetectKf(
    const std::shared_ptr<LocalMap>& local_map) {
  // 取最末端不要找回环了
  return {}
}

//
std::vector<std::pair<KeyFrameId, double>>
LoopDetect::FilterBestDbowResultWithCovisibility(
    std::shared_ptr<LocalMap> local_map,
    const std::unordered_map<KeyFrameId, double>& similar_with_score_ids) {
  //
  // for (const auto id : similar_with_score_ids) {
  //   LOG(INFO) << id.first << " " << id.second;
  // }
  if (similar_with_score_ids.empty()) return {};
  //
  float best_acc_score = 0;
  std::vector<std::pair<KeyFrameId, double>> acc_connected_frame_with_score_ids;
  for (const auto& score_id : similar_with_score_ids) {
    float best_score = score_id.second;
    double acc_score = best_score;
    KeyFrameId best_similar_frame_id(score_id.first);
    const auto covisibility_ids =
        const_map_manager_->GetCovisibility()->GetOrderConnectedKeyFrames(
            score_id.first, 10);
    //
    for (const auto& id_score : covisibility_ids) {
      auto const& id = id_score.first;
      if (similar_with_score_ids.count(id) == 0) continue;
      acc_score += similar_with_score_ids.at(id);
      if (similar_with_score_ids.at(id) > best_score) {
        best_score = similar_with_score_ids.at(id);
        best_similar_frame_id = id;
      }
    }
    //
    acc_connected_frame_with_score_ids.emplace_back(best_similar_frame_id,
                                                    acc_score);
    if (acc_score > best_acc_score) {
      best_acc_score = acc_score;
    }
  }
  CHECK(!acc_connected_frame_with_score_ids.empty());
  // Return all those keyframes with a score higher than 0.75*bestScore
  float min_score_to_retain =
      options_.min_filter_dbow_covisi_score * best_acc_score;
  std::set<KeyFrameId> result_filter;
  std::vector<std::pair<KeyFrameId, double>> result;
  for (const auto id_acc_score : acc_connected_frame_with_score_ids) {
    if (id_acc_score.second > min_score_to_retain) {
      if (result_filter.count(id_acc_score.first)) continue;
      result_filter.insert(id_acc_score.first);
      result.emplace_back(id_acc_score.first, id_acc_score.second);
    }
  }
  //
  std::sort(result.begin(), result.end(),
            [](const std::pair<KeyFrameId, double>& lhs,
               const std::pair<KeyFrameId, double>& rhs) {
              return lhs.second > rhs.second;
            });
  return std::move(result);
}
std::set<KeyFrameId> LoopDetect::FilterCandidata(
    const std::unordered_map<KeyFrameId, double>& candidate_kfs,
    std::unique_ptr<ConstraintConsistentFilter>* consistent_filter) {
  //

  for (auto const& id : best_candidata_kfs) {
    (*consistent_filter)
        ->Update(id.first,
                 const_map_manager_->GetCovisibility()->GetConnectedKeyFrames(
                     id.first));
  }
  auto const filter_candidate_ids = (*consistent_filter)->Result();
  if (filter_candidate_ids.empty()) {
    // LOG(WARNING) << "Consistent_filter not continue.";
    return {};
  }
  // std::set<KeyFrameId> filter_candidate_ids;
  // for (const auto kf : best_candidata_kfs) {
  //   filter_candidate_ids.insert(kf.first);
  // }
  return filter_candidate_ids;
}
}  // namespace mapping
}  // namespace jarvis
