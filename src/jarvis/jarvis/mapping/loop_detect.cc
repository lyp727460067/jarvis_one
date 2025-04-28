#include "jarvis/mapping/loop_detect.h"

#include "jarvis/mapping/auto_factor/pose_factor.h"
#include "jarvis/mapping/auto_factor/re_projection_err.h"
#include "jarvis/mapping/match/des_matcher.h"
#include "jarvis/mapping/match/pic_writer.h"
//
#include "jarvis/alg/pnp_wrapper.h"
namespace jarvis {
namespace mapping {
//
#define Tag log_info::YELLOW
//
LoopDetect::LoopDetect(const LoopDetectOption& option,
                       common::ThreadPool* thread_pool,
                       const std::map<int, camera_models::CameraPtr>& cameras)
    : options_(option), thread_pool_(thread_pool), cameras_(cameras) {
  options_.project_option.PorjectPoint =
      [this](const transform::Rigid3d& cam_pose, const Eigen::Vector3d& point,
             int s, Eigen::Vector2d* p) {
        const Eigen::Vector3d p_point = cam_pose.inverse() * point;
        if (point.z() < 0.1) return false;
        Eigen::Vector2d b;
        cameras_.at(s)->spaceToPlane(p_point, b);
        *p = b;
        return true;
      };
  finish_task_ = std::make_unique<common::Task>();
  when_done_task_ = std::make_unique<common::Task>();
  LOG(INFO)<<options_.image_boxs.size();
  LOG(INFO)<<options_.track_sequence.size();
  CHECK(!options_.image_boxs.empty());
  CHECK(!options_.track_sequence.empty());
  //
}
void LoopDetect::Detect(
    const std::pair<LocalMapId, std::shared_ptr<LocalMap>>& local_map,
    const std::map<KeyFrameId, KeyFrameData>& kf_datas,
    const double min_score) {
  //

  std::lock_guard<std::mutex> lock(mutex_);
  CHECK(local_map.second->AllKeyFrameDatas().size() != 0);
  CHECK_GT(kf_datas.size(), 3);
  const KeyFrameData first_kf_data = kf_datas.begin()->second;
  //
  // loop_result_catchs_.push_back(nullptr);
  // std::unique_ptr<LoopDetctResult>* this_kf_result_catch_ptr =
  //     &loop_result_catchs_.back();

  // LOG(INFO)<<loop_result_catchs_.size();
  std::map<KeyFrameId, KeyFrameData> kf_datas_temp = kf_datas;
  //
  if (!data_base_insert_task_hanlde.count(local_map.first)) {
    CHECK(key_frame_data_base_
              .emplace(local_map.first,
                       new KeyFrameDataBase(options_.key_frame_data_option))
              .second);
    auto data_base_task = std::make_unique<common::Task>();
    data_base_task->SetWorkItem([first_kf_data, local_map, this]() {
      for (const auto& data : local_map.second->ConstData().key_frames_datas) {
        key_frame_data_base_[local_map.first]->AddData(data.id, data.data.data);
      }
    });
    data_base_insert_task_hanlde[local_map.first] =
        thread_pool_->Schedule(std::move(data_base_task));
  }
  auto detect_node_task = std::make_unique<common::Task>();
  detect_node_task->SetWorkItem([=]() {
    //
    // LOG(INFO)<<this_kf_result_catch_ptr ;
    // CHECK((*this_kf_result_catch_ptr) == nullptr)<<this_kf_result_catch_ptr;
    std::unique_ptr<ConstraintConsistentFilter> consistent_filter =
        std::make_unique<ConstraintConsistentFilter>(
            options_.constraint_consistent_filter_num);
    //
    for (auto kf_data : kf_datas) {
      // CHECK((*this_kf_result_catch_ptr) == nullptr)<<this_kf_result_catch_ptr;
      auto loop_result =
          DetectForOne(local_map.second, kf_data.first, kf_data.second,
                       key_frame_data_base_[local_map.first].get(),
                       &consistent_filter, min_score);
      if (loop_result) {
        std::lock_guard<std::mutex> lock(mutex_);
        loop_result->local_map_id =   local_map.first;
        loop_result_catchs_.push_back(std::move(loop_result));
        // CHECK((*this_kf_result_catch_ptr) == nullptr)<<this_kf_result_catch_ptr;
        // (*this_kf_result_catch_ptr) = std::move(loop_result);
        return ;
      }
    };
  });
  detect_node_task->AddDependency(
      data_base_insert_task_hanlde[local_map.first]);
  auto detect_node_task_handle =
      thread_pool_->Schedule(std::move(detect_node_task));
  //
  finish_task_->AddDependency(detect_node_task_handle);
  //
}
//

//
std::unique_ptr<LoopDetctResult> LoopDetect::DetectForOne(
    std::shared_ptr<LocalMap> local_map, KeyFrameId kf_id,
    const KeyFrameData& kf_data, const KeyFrameDataBase* data_base,
    std::unique_ptr<ConstraintConsistentFilter>* consistent_filter,
    const double min_score) {
  //
  const auto candidate_kfs = data_base->FindSimilarCandidate(
      kf_data.data, NotNeedToDetectKf(local_map), min_score);
  // //
  //
  auto const best_candidata_kfs =
      FilterBestDbowResultWithCovisibility(local_map, candidate_kfs);
  if (best_candidata_kfs.empty()) {
    (*consistent_filter) = std::make_unique<ConstraintConsistentFilter>(
        options_.constraint_consistent_filter_num);
    return nullptr;
  }
  //
  for (auto const& id : best_candidata_kfs) {
    (*consistent_filter)
        ->Update(id.first,
                 local_map->ConstData().covisibility.GetConnectedKeyFrames(
                     id.first));
  }
  auto const filter_candidate_ids = (*consistent_filter)->Result();
  if (filter_candidate_ids.empty()) {
    return nullptr;
  }

  for (const auto& candidata_kf : filter_candidate_ids) {
    // LOG(INFO)<<candidata_kf <<" "<<kf_id;
    auto constraint =
        ComputeConstraint(local_map, candidata_kf, kf_id, kf_data);
    if (constraint != nullptr) {
      *consistent_filter = std::make_unique<ConstraintConsistentFilter>(
          options_.constraint_consistent_filter_num);
      //
      return constraint;
    }
  }
  return nullptr;
}
//

void LoopDetect::WriteCheckMatchResult(
    const KeyFrameData& first_data, const KeyFrameData& second_data,
    const std::vector<std::pair<FeatureId, FeatureId>>& match_ids) {
  if (options_.test_match_pic_write_path.empty()) return;
  match::WriteImageWithKeyPoint(options_.test_match_pic_write_path,
                                *first_data.data, *second_data.data, match_ids);
}
//

bool LoopDetect::IsMapPointsValid(
    const std::map<FeatureId, Eigen::Vector3d>& map_points) {
  return true;
}

std::pair<transform::Rigid3d, std::vector<std::pair<FeatureId, FeatureId>>>
LoopDetect::ComputePnpPose(std::shared_ptr<LocalMap> local_map,
                           const KeyFrameId& canditate_id,
                           const KeyFrameId& target_id,
                           const KeyFrameData& target_data) {
  //
  CHECK(canditate_id!=target_id);
  CHECK(local_map->ConstData().key_frames_datas.Contains(canditate_id));
  auto const& candidata_data =
      local_map->ConstData().key_frames_datas.at(canditate_id);
  auto const& target_descriptor = target_data.data->descriptors;
  auto const& target_key_points = target_data.data->features;
  auto const& target_dbow_vec = target_data.data->dbow_data;
  //
  auto paired_id = match::DbowFindMathed(
      target_descriptor, candidata_data.data->descriptors,
      target_data.data->dbow_data, candidata_data.data->dbow_data,
      options_.dbow_match_describe_distance_threashold,
      candidata_data.data->map_point_ids);

  std::map<int, int> s_num;
  for (int i = 0; i < paired_id.size(); i++) {
    s_num[paired_id[i].first.sequence_id]++;
  }
  //
  for (int i = 0; i < options_.track_sequence.size(); i++) {
    if (s_num.count(i)) {
      if (s_num[i] > options_.dbow_search_match_num / 3) continue;
    }
    LOG(WARNING) << " DbowFindMathed. "  << i <<" size: "<< s_num[i]
                 << " LE " << options_.dbow_search_match_num / 3;
    return {};
  }
  //
  if (paired_id.size() < options_.dbow_search_match_num) {
    LOG(WARNING) << " DbowFindMathed.   size: " << paired_id.size() << " LE "
                 << options_.dbow_search_match_num;
    return {};
  }
  WriteCheckMatchResult(target_data, candidata_data, paired_id);
  const auto& all_map_points = local_map->ConstData().map_points;
  std::map<FeatureId, Eigen::Vector3d> map_points_temp;
  std::map<FeatureId, FeatureData> features_temp;

  CHECK(candidata_data.data);
  CHECK(!candidata_data.data->map_point_ids.empty()) << canditate_id;
  for (int i = 0; i < paired_id.size(); i++) {
    //
    //
    auto target_index = paired_id[i].first;
    auto candidat_index = paired_id[i].second;
    if (options_.pnp_solve_typ != int(alg::SolveType::use_muty_cam) &&
        target_index.sequence_id != 0) {
      continue;
    }
    if (candidata_data.data->map_point_ids.count(candidat_index) == 0) continue;

    const auto map_point_id =
        candidata_data.data->map_point_ids.at(candidat_index);
    //
    if (all_map_points.Contains(map_point_id) == 0) continue;
    //
    map_points_temp.emplace(target_index,
                            all_map_points.at(map_point_id).data->pos);
    features_temp.emplace(target_index, target_key_points.at(target_index));
  }
  // LOG(INFO)<<map_points_temp.size();
  if (map_points_temp.size() < options_.min_pnp_need_features_num) {
    LOG(WARNING) << " map_points less for pnp.   size: "
                 << map_points_temp.size() << " LE "
                 << options_.min_pnp_need_features_num;
    return {};
  };
  if (!IsMapPointsValid(map_points_temp)) {
    LOG(ERROR) << "Mappoints ivalid";
    return {};
  }

  auto pnp_pose = CalculatePoseUsingPnP(
      alg::SolveType(options_.pnp_solve_typ), options_.pnp_solver_option,
      map_points_temp, features_temp, transform::Rigid3d::Identity());
  //
  std::vector<std::pair<FeatureId, FeatureId>> inlier_pairs;
  for (int i = 0; i < paired_id.size(); i++) {
    if (pnp_pose.second.count(paired_id[i].first) &&
        candidata_data.data->map_point_ids.count(paired_id[i].second)) {
      inlier_pairs.emplace_back(paired_id[i].second, paired_id[i].first);
    }
  }

  if (inlier_pairs.size() < options_.min_pnp_inliers_num) {
    LOG(WARNING) << "pnp inli size: " << inlier_pairs.size() << " LE "
                 << options_.min_pnp_inliers_num;
    return {};
  }

  LOG(INFO) << Tag << "Pnp inli: " << inlier_pairs.size()
            << " pose:" << pnp_pose.first << log_info::RESET;
  //
  for (int i = 0; i < paired_id.size(); i++) {
    if (paired_id[i].first.sequence_id != 0 &&
        options_.pnp_solve_typ != int(alg::SolveType::use_muty_cam)) {
      inlier_pairs.emplace_back(paired_id[i].second, paired_id[i].first);
    }
  }

  return {pnp_pose.first, std::move(inlier_pairs)};
}
//

std::vector<std::pair<FeatureId, MapPointId>>
LoopDetect::SearchForAdditionalMapPoints(
    std::shared_ptr<LocalMap> map, const KeyFrameId& candidate_id,
    const transform::Rigid3d& pose, const KeyFrameData& target_kf_data,
    const std::set<MapPointId>& already_matched_mp_ids,
      const std::set<FeatureId>& already_matched_feats) {
  auto connect_frames_ids =
      map->ConstData().covisibility.GetKeyLevelConnectedKeyFrames(
          candidate_id, options_.convisi_level_search_num);
  //
  connect_frames_ids.push_back(candidate_id);
  //

  std::map<int, std::unique_ptr<match::AreaSearch>> area_searchs =
      match::AreaSearch::CreateAreaSearchFromeKeyFrameData(
          options_.image_boxs, options_.area_search_grid_lenth,
          *target_kf_data.data);

  match::ProjectionOption project_option = options_.additional_project_option;
  project_option.PorjectPoint = [this, pose, &target_kf_data](
                                    const transform::Rigid3d& cam_pose,
                                    const Eigen::Vector3d& point, int s,
                                    Eigen::Vector2d* p) {
    const transform::Rigid3d c_pose = target_kf_data.data->CameraPose(pose, s);
    const Eigen::Vector3d p_point = c_pose.inverse() * point;
    if (point.z() < 0.1) return false;
    Eigen::Vector2d b;
    cameras_.at(s)->spaceToPlane(p_point, b);
    *p = b;
    return true;
  };

  std::set<MapPointId> connect_mp_points;
  for (const auto& kf_id : connect_frames_ids) {
    auto map_points = map->GetKeyFrameMapPoints(kf_id);
    for (const auto& mp_id : map_points) {
      connect_mp_points.insert(mp_id.first);
    }
  }

  std::set<FeatureId> match_features_id;
  std::vector<std::pair<FeatureId, MapPointId>> result;
  for (const auto& mp_id : connect_mp_points) {
    if(already_matched_mp_ids.count(mp_id))continue;
    if (map->ConstData().map_points.Contains(mp_id)) {
      auto index = SearchMatchesByProjection(
          project_option, *target_kf_data.data, area_searchs,
          map->ConstData().map_points.at(mp_id));
      if (match_features_id.count(index) || already_matched_feats.count(index))
        continue;
      if (index != FeatureId{-1, 0}) {
        match_features_id.insert(index);
        result.emplace_back(index, mp_id);
      }
    }
  }
  return result;
}

std::vector<std::pair<FeatureId, MapPointId>>
LoopDetect::CheckValidityByProjections(
    const KeyFrameMapPointsDataWithFeatIds& target_map_points,
    const transform::Rigid3d& correct_candidate_pose,
    const KeyFrameData& candidate_kf_data,
    const std::set<MapPointId>& already_matched,
    const match::ProjectionOption &project_option 
  ) {
  auto const key_frame_map_point_ids = target_map_points.first;
  std::vector<std::pair<FeatureId, MapPointId>>
      paired_id_target_map_to_candidate;
  //
  //


  std::map<int, std::unique_ptr<match::AreaSearch>> area_searchs =
      match::AreaSearch::CreateAreaSearchFromeKeyFrameData(
          options_.image_boxs, options_.area_search_grid_lenth,
          *candidate_kf_data.data);

  //
    std::set<FeatureId> match_features_id;
  for (const auto map_point_id : key_frame_map_point_ids) {
    // 只有合并了点后有可能出现这种情况
    if (already_matched.count(map_point_id.second)) {
      continue;
    }
    auto index = SearchMatchesByProjection(
      project_option, *candidate_kf_data.data, area_searchs,
        target_map_points.second.at(map_point_id.second));
    //
    if (match_features_id.count(index)) continue;
    if (index == FeatureId{-1, 0}) continue;
    paired_id_target_map_to_candidate.emplace_back(index, map_point_id.second);
    match_features_id.insert(index);
  }
  return paired_id_target_map_to_candidate;
}

//
std::unique_ptr<LoopDetctResult> LoopDetect::ComputeConstraint(
    std::shared_ptr<LocalMap> local_map, const KeyFrameId& candidate_id,
    const KeyFrameId& target_id, const KeyFrameData& target_kf_data) {
  auto pnp_pose =
      ComputePnpPose(local_map, candidate_id, target_id, target_kf_data);
  //
  if (pnp_pose.second.empty()) {
    return nullptr;
  }
  const transform::Rigid3d imu_pose =
      target_kf_data.data->ImuPose(pnp_pose.first, 0);
  const transform::Rigid3d pnp_imu_pose = imu_pose;
  KeyFrameMapPointsDataWithFeatIds canditate_map_points ;
  //
  auto frame_feat_map_points_ids =
      local_map->ConstData().covisibility.GetKeyFrameMapPointId(candidate_id);
  //
  for (int i = 0; i < frame_feat_map_points_ids.first.size(); i++) {
    canditate_map_points.first.emplace(frame_feat_map_points_ids.second[i],
                                       frame_feat_map_points_ids.first[i]);
    canditate_map_points.second.Insert(frame_feat_map_points_ids.first[i],
                                       local_map->ConstData().map_points.at(
                                           frame_feat_map_points_ids.first[i]));
  }
  //
  std::set<MapPointId> already_matched_ids;
  //
  match::ProjectionOption project_option = options_.project_option;
  project_option.PorjectPoint = [this, imu_pose, &target_kf_data](
                                    const transform::Rigid3d& cam_pose,
                                    const Eigen::Vector3d& point, int s,
                                    Eigen::Vector2d* p) {
    const transform::Rigid3d c_pose =
        target_kf_data.data->CameraPose(imu_pose, s);
    const Eigen::Vector3d p_point = c_pose.inverse() * point;
    if (point.z() < 0.1) return false;
    if (p) {
      Eigen::Vector2d b;
      cameras_.at(s)->spaceToPlane(p_point, b);
      *p = b;
    }

    return true;
  };
  //

  //
  auto candidate_projection_to_target_kf_id = CheckValidityByProjections(
      canditate_map_points, imu_pose, target_kf_data, {}, project_option);

  //
  auto canditate_pose =
      local_map->ConstData().key_frames_ref_pose.at(candidate_id);

  const auto target_in_local_map_pose =
      local_map->LocalPose().inverse() * target_kf_data.data->pose;

  const transform::Rigid3d candidate_correct_pose =
      target_in_local_map_pose * pnp_pose.first.inverse() * canditate_pose;

  std::map<FeatureId, FeatureId> target_candidate_match_feat_ids;
  std::map<MapPointId, FeatureId> target_feat_ids_map_points_ids;
  //
  KeyFrameMapPointsDataWithFeatIds target_map_points;
  //

  //
  // project_option.PorjectPoint = [this, &pnp_pose, &target_kf_data](
  //                                   const transform::Rigid3d& cam_pose,
  //                                   const Eigen::Vector3d& point, int s,
  //                                   Eigen::Vector2d* p) {
  //   target_kf_data.data->CameraPose(pnp_pose.first, s);
  //   const Eigen::Vector3d p_point = cam_pose.inverse() * point;
  //   if (point.z() < 0.1) return false;
  //   Eigen::Vector2d b;
  //   cameras_.at(s)->spaceToPlane(p_point, b);
  //   *p = b;
  //   return true;
  // };

  const KeyFrameData& candidate_kf_data =
      local_map->ConstData().key_frames_datas.at(candidate_id);
  // auto target_projection_to_candidate_kf_id = CheckValidityByProjections(
  //     target_map_points, pnp_pose.first, candidate_kf_data, {}, project_option);
  // //

  //
  // for (const auto& pair_id : target_projection_to_candidate_kf_id) {
  //   if (target_candidate_match_feat_ids.at(target_feat_ids_map_points_ids.at(
  //           pair_id.second)) == pair_id.first) {
  //   }
  // }

  if (candidate_projection_to_target_kf_id.size() <
          options_.candidata_reproject_min_num/* ||
      target_projection_to_candidate_kf_id.size() <
          options_.candidata_reproject_min_num*/) {
    LOG(WARNING) << "CheckValidityByProjections  "
                 << candidate_projection_to_target_kf_id.size() << " "
                 << options_.candidata_reproject_min_num;

    //  << " target_projection_to_candidate_kf_id size: "
    //  << target_projection_to_candidate_kf_id.size();

    return nullptr;
  }
  //
  std::set<MapPointId> already_matched_mp_ids;
  std::set<FeatureId> already_matched_feats;
  for (const auto& match_id : pnp_pose.second) {
    already_matched_mp_ids.insert(
        candidate_kf_data.data->map_point_ids.at(match_id.first));
    already_matched_feats.insert(match_id.second);
  }
  //
  auto candidate_additional_map_points_ids = SearchForAdditionalMapPoints(
      local_map, candidate_id, imu_pose, target_kf_data, already_matched_mp_ids,
      already_matched_feats);
  candidate_additional_map_points_ids.clear();
  LOG(INFO) << Tag << "Imu pose " << imu_pose
            << "SearchForAdditionalMapPoints at " << candidate_id
            << "size :" << candidate_additional_map_points_ids.size()
            << log_info::RESET;
  for (const auto& match_id : pnp_pose.second) {
    candidate_additional_map_points_ids.emplace_back(
        match_id.second,
        candidate_kf_data.data->map_point_ids.at(match_id.first));
  }


  std::map<MapPointId, Eigen::Vector3d> candidate_additional_map_points_datas;
  //

  //
  for (const auto& map_point_data : local_map->AllMapPoints()) {
    candidate_additional_map_points_datas.emplace(
        map_point_data.id, map_point_data.data.data->pos);
  }
  if (candidate_additional_map_points_ids.size() < 10) return nullptr;
  
  //
  transform::Rigid3d init_pose =  imu_pose;
  std::stringstream inter_info;
  inter_info << "opitmize max inter: " << options_.max_num_iterations;
  int inliner = 100;
  inliner = RemoveOutliersRejection(target_kf_data.data->extric_camera_to_imu,
                                    candidate_additional_map_points_datas,
                                    target_kf_data.data->features, init_pose,
                                   0.2,
                                    candidate_additional_map_points_ids);
  for (int i = 0; i < options_.max_num_iterations; i++) {
    //
    if (inliner < options_.pnp_optimize_min_iniler) {
      LOG(ERROR) << "PNP optimize inline " << inliner << "<"
                 << options_.pnp_optimize_min_iniler << init_pose;
      return nullptr;
    }
    init_pose =
        Optimize(init_pose, target_kf_data.data->extric_camera_to_imu,
                 candidate_additional_map_points_datas,
                 candidate_additional_map_points_ids, target_kf_data,
                 std::array<double, 2>{options_.op_weight, options_.op_weight});
    inliner = RemoveOutliersRejection(target_kf_data.data->extric_camera_to_imu,
                                      candidate_additional_map_points_datas,
                                      target_kf_data.data->features, init_pose,
                                      options_.outlier_min_err / 460,
                                      candidate_additional_map_points_ids);
    inter_info << ".inter:" << i << " pose:" << init_pose
               << ",inliner num:" << inliner;
    //
  }
  LOG(INFO) << Tag << inter_info.str() << log_info::RESET;
  //
  if (candidate_additional_map_points_ids.size() <
      options_.pnp_optimize_min_iniler) {
        LOG(ERROR) << "PNP optimize inline "
        << candidate_additional_map_points_ids.size() << "<"
        << options_.pnp_optimize_min_iniler;
    return nullptr;
  }
  const auto& target_pose = target_kf_data.data->pose;
  const auto& candidata_pose = candidate_kf_data.data->pose;
  const auto delta_pose = init_pose ;

  const double delta_yaw = transform::GetYaw(init_pose.inverse().rotation());
  //
  if (delta_pose.translation().norm() > options_.constraint_max_distance) {
    LOG(WARNING) << "Detel yaw : " << common::RadToDeg(delta_yaw)
                 << "Delta Pose : " << delta_pose.translation().norm()
                 << " Is greater option.." << options_.constraint_max_distance
                 << " " << options_.constraint_max_yaw;
    return nullptr;
  }
  std::stringstream info;
  info << " target " << target_id << "candidate: " << candidate_id
       << " pnp pose " << pnp_imu_pose << "optimization " << init_pose;
  info << " local_map_pose  " << local_map->LocalPose()
       << " target: " << target_pose << " candidate: " << candidata_pose;
  info << "delta pose :" << delta_pose << " yaw :" << delta_yaw;
  LOG(INFO) << Tag << info.str() << log_info::RESET;

  return std::make_unique<LoopDetctResult>(
      LoopDetctResult{target_id, {-1, 0}, init_pose, candidate_id});
  //
}
//
std::set<KeyFrameId> LoopDetect::NotNeedToDetectKf(
    const std::shared_ptr<LocalMap>& local_map) {
  // 取最末端不要找回环了
  return {};
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
        local_map->ConstData().covisibility.GetOrderConnectedKeyFrames(
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
//

//
void LoopDetect::CalculatedSingleResultFinish() {
  // 多个回环轨迹的话 校验pose
}
//
void LoopDetect::ContinueAndDistanceCheck(
    std::shared_ptr<LocalMap> local_map,
    std::unique_ptr<LoopDetctResult>* data) {}
//

void LoopDetect::NotifyFinish() {
  std::lock_guard<std::mutex> lock(mutex_);
  finish_task_->SetWorkItem([this]() {
    // 可以检查一下连续性
    CalculatedSingleResultFinish();
  });
  auto finish_node_task_handle =
      thread_pool_->Schedule(std::move(finish_task_));
  finish_task_ = std::make_unique<common::Task>();
  when_done_task_->AddDependency(finish_node_task_handle);
}

void LoopDetect::WhenDone(
    std::function<void(std::vector<std::unique_ptr<LoopDetctResult>>)>
        call_back) {
  std::lock_guard<std::mutex> lock(mutex_);
  when_done_task_->SetWorkItem([this, call_back] {
    std::vector<std::unique_ptr<LoopDetctResult>> result;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto& r : loop_result_catchs_) {
        if (r == nullptr) continue;
        result.push_back(std::move(r));
      }
      loop_result_catchs_.clear();
    }
    if (call_back) {
      call_back(std::move(result));
    }
  });
  thread_pool_->Schedule(std::move(when_done_task_));
  when_done_task_ = std::make_unique<common::Task>();
}
//
//


int LoopDetect::RemoveOutliersRejection(
    const std::vector<transform::Rigid3d>& extric_camera_to_imu,
    const std::map<MapPointId, Eigen::Vector3d> map_points,
    const MapById<FeatureId, FeatureData> target_features,
    const transform::Rigid3d& pose, const float outlier,
    std::vector<std::pair<FeatureId, MapPointId>>& matched_ids

) {
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
  for (auto it = matched_ids.begin(); it != matched_ids.end();) {
    auto& constraist_matchs = *it;
    //
    transform::Rigid3d exti = extric_camera_to_imu
        [options_.track_sequence[constraist_matchs.first.sequence_id][0]];

    float err = ReprojectionError(
        map_points.at(constraist_matchs.second), pose * exti,
        target_features.at(constraist_matchs.first).f.head<2>());
    if (err > outlier) {
      it = matched_ids.erase(it);
      continue;
    }
    inliner++;
    ++it;
  }
  return inliner;
}

transform::Rigid3d LoopDetect::Optimize(
    const transform::Rigid3d& init_pose,
    const std::vector<transform::Rigid3d>& extric_camera_to_imu,
    const std::map<MapPointId, Eigen::Vector3d>& map_points,
    const std::vector<std::pair<FeatureId, MapPointId>>& matched_ids,
    const KeyFrameData& candidate_kf_data,
    const std::array<double, 2>& weight) {
  CHECK_GE(matched_ids.size(),8);
  ceres::Problem problem;
  //
  ceres::LocalParameterization* quaternion_local =
      new ceres::EigenQuaternionParameterization;

  Eigen::Quaterniond rotation = init_pose.inverse().rotation();
  Eigen::Vector3d traslation = init_pose.inverse().translation();
  //
  problem.AddParameterBlock(traslation.data(), 3);
  problem.AddParameterBlock(rotation.coeffs().data(), 4);
  problem.SetParameterization(rotation.coeffs().data(), quaternion_local);
  //
  Eigen::Quaterniond ex_rotation[options_.track_sequence.size()];
  Eigen::Vector3d ex_traslation[options_.track_sequence.size()];
  for (size_t i = 0; i < options_.track_sequence.size(); i++) {
    // LOG(INFO)<<extric_camera_to_imu[options_.track_sequence[i][0]];
    transform::Rigid3d extir_iverse =
        extric_camera_to_imu[options_.track_sequence[i][0]].inverse();
    ex_rotation[i] = extir_iverse.rotation();
    ex_traslation[i] = extir_iverse.translation();
    problem.AddParameterBlock(ex_rotation[i].coeffs().data(), 4);
    problem.AddParameterBlock(ex_traslation[i].data(), 3);
    problem.SetParameterBlockConstant(ex_rotation[i].coeffs().data());
    problem.SetParameterBlockConstant(ex_traslation[i].data());

    //
    problem.SetParameterization(ex_rotation[i].coeffs().data(),
                                quaternion_local);
  }

  for (const auto& match_id : matched_ids) {
    const Eigen::Vector3d& mp_pos = map_points.at(match_id.second);
    auto const& nomal_point =
        candidate_kf_data.data->features.at(match_id.first);
    problem.AddResidualBlock(
        ReProjectionErr::Creat(nomal_point.f.head<2>(), mp_pos, weight[0]),
      new ceres::HuberLoss(5.0) , traslation.data(), rotation.coeffs().data(),
        ex_traslation[match_id.first.sequence_id].data(),
        ex_rotation[match_id.first.sequence_id].coeffs().data());
    //
  }

  // problem.AddResidualBlock(
  //     TranslationCostFunctor::Create(init_pose.inverse().translation(),
  //                                    options_.op_init_t_weight),
  //     nullptr, traslation.data());

  // problem.AddResidualBlock(
  //     YawRotationDeltaCostFunctor::Create(common::DegToRad(ypr[0]),
  //                                         options_.op_init_r_weight),
  //     nullptr, &yaw);

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 3;  // options_.max_num_iterations;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // LOG(INFO) << Tag<< summary.BriefReport() << log_info::RESET;
  // LOG(INFO) << summary.FullReport();
    
  return transform::Rigid3d(traslation, rotation).inverse();
}

}  // namespace mapping
}  // namespace jarvis
