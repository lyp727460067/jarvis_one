#include "jarvis/mapping/map_manager.h"

#include <mutex>
// #include <opencv2/core/eigen.hpp>
#include <set>
#include <vector>

#include "Eigen/Core"
#include "glog/logging.h"
#include "jarvis/common/math.h"
#include "jarvis/common/task.h"
#include "jarvis/mapping/covisibility.h"
#include "jarvis/mapping/key_frame_database.h"
#include "jarvis/mapping/map_manager.h"
#include "jarvis/mapping/mapping_data.h"
#include "jarvis/mapping/match/des_matcher.h"
#include "jarvis/mapping/match/pic_writer.h"
namespace jarvis {
namespace mapping {
#define Tag log_info::YELLOW
MapManager::MapManager(const MapManagerOption &option,
                       MapPointConstruct *map_point_construct,
                       std::map<int, camera_models::CameraPtr> camera,
                       common::ThreadPool *thread_pool,
                       LocalMapUpdateCallBack call_back)
    : options_(option),
      map_point_construct_(map_point_construct),
      thread_pool_(thread_pool),
      localmap_update_callback_(std::move(call_back)) {
  //
  enable_loop_closure_ = thread_pool_ ? true : false;
  if(!enable_loop_closure_ )true;
  if (option.local_map_op_use_6dof) {
    local_optimization_ = std::make_unique<GraphLocalMapOptimization6TOF>(
        option.local_map_optimization_option);
  } else {
    local_optimization_ = std::make_unique<LocalMapOptimization>(
        option.local_map_optimization_option);
  }
  work_item_queue_ =
      std::make_unique<WorkItemQueue>("map_manager", thread_pool, [this]() {
        loop_detect_->WhenDone(
            [this](std::vector<std::unique_ptr<LoopDetctResult>> &&result) {
              Optimization(std::move(result));
            });
      });
  pose_graph_optimizer_ =
      std::make_unique<PoseGraphOptimize>(options_.pose_graph_option);
  loop_detect_ = std::make_unique<LoopDetect>(option.loop_detect_option,
                                              thread_pool, camera);
  //
  loop_detect_kf_sampler_ = std::make_unique<common::FixedRatioSampler>(
      options_.constraint_compute_sampler);
  //
}

void MapManager::ComputeConstaints(const KeyFrameId &id,
                                   const double min_score) {
  //
  for (const auto &local_map_id : local_maps_) {
    ComputeLoopConstaint(local_map_id.id, id, min_score);
  }
  //
  if (new_local_map_id_.has_value()) {
    new_local_map_id_.reset();
    std::vector<KeyFrameId> candidate_kfs;
    auto local_map_id = new_local_map_id_.value();
    new_local_map_id_.reset();
    auto new_local_map = local_maps_.at(local_map_id).local_map;
    for (const auto &id : extend_key_frames_ids_) {
      ComputeLoopConstaint(local_map_id, id, min_score);
    }
  }
}
//
void MapManager::ComputeLoopConstaint(const LocalMapId &local_map_id,
                                      const KeyFrameId &key_frame_id,
                                      const double min_score) {
  auto last_connection_time =
      last_trajectory_connect_time_.at(key_frame_id.trajectory_id)
          .at(local_map_id.trajectory_id);
  //

  const auto &kf_data = key_frames_datas_.at(key_frame_id);
  auto kf_time = key_frames_datas_.at(key_frame_id).data->time;
  if (key_frame_id.trajectory_id == local_map_id.trajectory_id /*||
      kf_time < last_connection_time +
                    common::FromSeconds(
                        options_.global_constraint_search_after_n_seconds)*/) {
    const transform::Rigid3d delta_pose =
        local_maps_.at(local_map_id).local_map->LocalPose().inverse() *
        kf_data.data->pose;
    if (delta_pose.translation().norm() >
        options_.same_trajectory_max_loop_detect_distance) {
      return;
    }

  } else if (!loop_detect_kf_sampler_->Pulse()) {
    return;
  }
  std::map<KeyFrameId, KeyFrameData> continuous_ids;
  for (int i = -options_.continuous_candidate_loop_frame / 2;
       i < options_.continuous_candidate_loop_frame / 2; i++) {
    //
    const KeyFrameId id(key_frame_id.trajectory_id,
                        key_frame_id.keyframe_index + i);
    if (extend_key_frames_ids_.count(id)) {
      continuous_ids.emplace(id, key_frames_datas_.at(id));
    }
  }
  //
  LOG(INFO) << Tag << "LoopDetect  -> local_map" << local_map_id
            << ",keyframeid " << key_frame_id << log_info::RESET;

  loop_detect_->Detect(
      std::pair<LocalMapId, std::shared_ptr<LocalMap>>(
          local_map_id, local_maps_.at(local_map_id).local_map),
      continuous_ids, min_score);
}
//
void MapManager::ExtendedKeyFrameData(const LocalMap &local_map,
                                      const KeyFrameId &id,
                                      KeyFrameData::Data* data) {
   if(!enable_loop_closure_)return ;                               
  work_item_queue_->AddWorkItem([=]() {
    LOG(INFO)<<"!";
    map_point_construct_->ExtractExtendData(local_map, data);

    LOG(INFO)<<"!";
    extend_key_frames_ids_.insert(id);
    //

    LOG(INFO)<<"!";
    double covi_min_score = ComputeCovisibleMinScore(local_map, id);
    ComputeConstaints(id, covi_min_score);
    //

    LOG(INFO)<<"!";
    ++num_kf_num_since_last_loop_closure_;
    if (options_.pose_graph_optimize_min_kf_min_num > 0 &&
        num_kf_num_since_last_loop_closure_ >
            num_kf_num_since_last_loop_closure_) {
      num_kf_num_since_last_loop_closure_ = 0;
      return WorkItem::Result::kInterruptForImmediateRun;
    }
    return WorkItem::Result::Normal;
  });
}
//
//
double MapManager::ComputeCovisibleMinScore(const LocalMap &local_map,
                                            const KeyFrameId &id) {
  //
  if()
  const std::vector<KeyFrameId> connected_key_frame_ids =
      local_map.ConstData().covisibility.GetConnectedKeyFrames(id);
  //
  const auto &key_frames_datas = local_map.ConstData().key_frames_datas;
  float min_score = 1;
  auto const &curr_frame_bow_vev = key_frames_datas.at(id).data->dbow_data;
  for (const auto &connected_id : connected_key_frame_ids) {
    CHECK(key_frames_datas.Contains(connected_id)) << connected_id;
    auto const &bow_vec_connected =
        key_frames_datas.at(connected_id).data->dbow_data;
    float score = bow_vec_connected.Score(curr_frame_bow_vev);
    if (score < min_score) min_score = score;
  }

  return min_score;
}
//
//
//
void MapManager::UpdataLocalMapConstraint(const LocalMapId &id,
                                          std::shared_ptr<LocalMap> local_map) {
  for (const auto &re_local_kf_pose :
       local_map->ConstData().key_frames_ref_pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    pose_constraints_.push_back(PoseConstraint{
        id, re_local_kf_pose.first, re_local_kf_pose.second,
        transform::GetYaw(re_local_kf_pose.second.rotation()), true});
  }
}
//
void MapManager::ReconstructLocalMapOptimization(
    std::map<LocalMapId, std::shared_ptr<LocalMap>> &local_maps) {
  local_optimization_->Optimize(&local_maps);

}

//
std::shared_ptr<LocalMap> MapManager::ReconstructLocalMap(
    std::shared_ptr<LocalMap> local_map) {
  std::set<KeyFrameId> new_update_ids;
  //
  std::shared_ptr<LocalMap> new_local_map_ptr =
      std::make_shared<LocalMap>(*local_map);
  LocalMap &new_local_map = *new_local_map_ptr;
  //
  for (const auto &data : local_map->AllKeyFrameDatas()) {
    if (previous_local_map_trimed_key_frames_id_.count(data.id)) continue;
    map_point_construct_->ConstructExtend(new_local_map,
                                          &key_frames_datas_.at(data.id));
    new_local_map.AddKeyFrameData(data.id, data.data);

  }
  //
  return new_local_map_ptr;
}

void MapManager::UpdataActiveTrackLocalMap(
    std::shared_ptr<LocalMap> local_map) {
  if (localmap_update_callback_) {
    localmap_update_callback_(local_map);
  }
}

//
void  MapManager::AddLocalMap(int trajectory,
                                   std::shared_ptr<LocalMap> local_map) {
 if(enable_loop_closure_)return ;  
  work_item_queue_->AddWorkItem([&]() {
    auto new_local_map = ReconstructLocalMap(local_map);
    const auto local_map_id =
        local_maps_.Append(trajectory, LocalMapData{new_local_map});
    //
    std::map<LocalMapId, std::shared_ptr<LocalMap>> op_local_maps;
    op_local_maps.emplace(local_map_id, new_local_map);
    if (options_.enable_local_map_full_op) {
      ReconstructLocalMapOptimization(op_local_maps);
    }

    //
    //
    UpdataActiveTrackLocalMap(local_map);
    //
    new_local_map->UpdadataExtendFinishData(true);
    UpdateKeyframeDataUsingPrunedLocalMap(local_map);
    PruneRedundantLocalMap(local_map_id);
    //
    //
    pose_graph_optimizer_->AddLocalMapPose(
        local_map_id,
        LocalMapPoseTime{
            local_map->ConstData().key_frames_datas.begin()->data.data->time,
            local_map->LocalPose()});
   //在删减之后在更新约束
    UpdataLocalMapConstraint(local_map_id,local_map);
    //
    new_local_map_id_ = local_map_id;
    return WorkItem::Result::Normal;
  });
  //
}
//
KeyFrameId MapManager::AddKeyFrameData(int trajectory,
                                       const KeyFrameData &data) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto kf_id = key_frames_datas_.Append(trajectory, data);
  return kf_id;
}
//
void MapManager::PruneRedundantLocalMap(const LocalMapId &new_local_map_id) {
  // 不能删除当前最新的图
}

//
void MapManager::UpdateLocalOpLocalMap(
    std::map<LocalMapId, std::shared_ptr<LocalMap>> *op_local_maps) {}

//
void MapManager::UpdateKeyframeDataUsingPrunedLocalMap(
    std::shared_ptr<LocalMap> new_local_map) {
  std::set<KeyFrameId> previous_local_map_trimed_key_frames_id_temp;
  const auto &full_local_map_ids =
      new_local_map->ConstData().removed_keyframes_ids_before_trim;
  const auto &local_map_kf_ids = new_local_map->ConstData().key_frames_ref_pose;
  for (const auto &id : full_local_map_ids) {
    if (!local_map_kf_ids.count(id)) {
      previous_local_map_trimed_key_frames_id_temp.insert(id);
    }
  }
  previous_local_map_trimed_key_frames_id_ =
      std::move(previous_local_map_trimed_key_frames_id_temp);
  for (const auto &id : previous_local_map_trimed_key_frames_id_) {
    key_frames_datas_.Trim(id);
  }
};
//
//
void MapManager::TrimOptimizedLocalMap() {
  std::set<KeyFrameId> finish_key_frame_ids;
  std::set<KeyFrameId> unfinished_key_frame_ids;
  std::set<LocalMapId> finish_local_map_ids;
  // for (const auto &local_map : local_maps_) {
  //   auto all_local_map_key_frame_ids =
  //       local_map.data.local_map->GetTrimBeforKeyFrameId();
  //   if (local_map.data.local_map->IsOptimization()) {
  //     finish_key_frame_ids.merge(all_local_map_key_frame_ids);
  //     finish_local_map_ids.insert(local_map.id);
  //   } else {
  //     unfinished_key_frame_ids.merge(all_local_map_key_frame_ids);
  //   }
  // }
  // std::vector<KeyFrameId> trim_ids;
  // std::set_difference(finish_key_frame_ids.begin(), finish_key_frame_ids.end(),
  //                     unfinished_key_frame_ids.begin(),
  //                     unfinished_key_frame_ids.end(),
  //                     std::back_inserter(trim_ids));
  // for (const auto &id : trim_ids) {
  //   TrimKeyFrameData(id);
  // }
  // for (auto const local_map_id : finish_local_map_ids) {
  //   std::lock_guard<std::mutex> lock(mutex_);
  //   local_maps_.Trim(local_map_id);
  // }
}

void MapManager::TrimKeyFrameData(const KeyFrameId &id) {
  key_frames_datas_.Trim(id);
}
//
//
void MapManager::Optimization(
    std::vector<std::unique_ptr<LoopDetctResult>> &&loop_constraint) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    LOG(INFO) << Tag
              << "Start opimize ,constraint size :" << loop_constraint.size()
              << log_info::RESET;

    for (auto &&l_constraint : loop_constraint) {
      pose_constraints_.push_back(PoseConstraint{
          l_constraint->local_map_id,
          l_constraint->kf_id,
          l_constraint->relative_pose,
          l_constraint->relative_yaw,
      });
    }
  }
  //
  pose_graph_optimizer_->Solve(pose_constraints_);
  auto global_local_map_pose = pose_graph_optimizer_->GetPoseGraphLocalMapPose();
  auto global_kf_pose = pose_graph_optimizer_->GetPoseGraphNodePose();
  for (const auto &g_pose : global_local_map_pose) {
    local_maps_.at(g_pose.first).globla_pose =
        transform::Rigid3d(g_pose.second.t, g_pose.second.q);
  }
  //
  KeyFrameId last_key_frame_id(0, 0);
  for (const auto &g_pose : global_kf_pose) {
    key_frames_datas_.at(g_pose.first).global_pose =
        transform::Rigid3d(g_pose.second.t, g_pose.second.q);
    last_key_frame_id = g_pose.first;
  }
  local_to_global_transform_ =
      key_frames_datas_.at(last_key_frame_id).global_pose.inverse() *
      key_frames_datas_.at(last_key_frame_id).data->pose;
  //
  //跟新没有优化的pose
}


std::map<KeyFrameId, transform::TimestampedTransform>
MapManager::GetAllKeyFramePose() {
  std::vector<std::shared_ptr<LocalMap>> finish_local_maps;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &local_map_data : local_maps_) {
      if (local_map_data.data.local_map->IsOptimization()) {
        finish_local_maps.push_back(local_map_data.data.local_map);
      }
    }
  }
  std::map<KeyFrameId, transform::TimestampedTransform> result;
  for (const auto &local_map : finish_local_maps) {
    const auto all_ref_pose = local_map->AllKeyFrameRefPose();
    for (const auto &ref_pose : all_ref_pose) {
      if (result.count(ref_pose.first)) continue;
      result.emplace(ref_pose.first,
                     transform::TimestampedTransform{
                         key_frames_datas_.at(ref_pose.first).data->time,
                         local_map->LocalPose() * ref_pose.second});
    }
  }

  // for (const auto &key_frame_data : key_frames_datas_) {
  //   result.emplace(
  //       key_frame_data.id,
  //       transform::TimestampedTransform{key_frame_data.data.data->time,
  //                                       key_frame_data.data.data->global_pos});
  // }
  return result;
}

std::vector<Eigen::Vector3d> MapManager::GetAllMapPoints() {
  std::vector<std::shared_ptr<LocalMap>> finish_local_maps;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &local_map_data : local_maps_) {
      if (local_map_data.data.local_map->IsOptimization()) {
        finish_local_maps.push_back(local_map_data.data.local_map);
      }
    }
  }
  std::vector<Eigen::Vector3d> result;
  std::set<MapPointId> existing_map_points;
  for (const auto &local_map : finish_local_maps) {
    const auto all_local_map_points = local_map->AllMapPoints();
    for (const auto &map_point : all_local_map_points) {
      if (existing_map_points.count(map_point.id)) continue;
      result.push_back(local_map->LocalPose() * map_point.data.data->pos);
      existing_map_points.insert(map_point.id);
    }
  }
  return result;
}

MapManager::~MapManager() {}

}  // namespace mapping
}  // namespace jarvis
