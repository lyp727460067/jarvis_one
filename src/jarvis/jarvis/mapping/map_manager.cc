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

MapManager::MapManager(const MapManagerOption &option,
                       MapPointConstruct *map_point_construct,
                       common::ThreadPool *thread_pool,
                       LocalMapUpdateCallBack call_back)
    : options_(option),
      map_point_construct_(map_point_construct),
      thread_pool_(thread_pool),
      localmap_update_callback_(std::move(call_back)) {
  if (option.use_6_tof_op) {
    local_optimization_ = std::make_unique<GraphLocalMapOptimization6TOF>(
        option.local_map_optimization_option);
  } else {
    local_optimization_ = std::make_unique<LocalMapOptimization>(
        option.local_map_optimization_option);
  }
  work_item_queue_ =
      std::make_unique<WorkItemQueue>("map_manager", thread_pool, [this]() {
        loop_detect_->WhenDone(
            [this](std::vector<std::shared_ptr<LoopDetctResult>> &&result) {
              RunOptimization(result);
            });
      });
}

void MapManager::ComputeConstaints(const KeyFrameId &id) {
  //
  for (const auto &local_map_id : local_maps_) {
    ComputeLoopConstaint(local_map_id, id);
  }
  //
  if (new_local_map_id_.has_value()) {
    new_local_map_id_.reset();
    std::vector<KeyFrameId> candidate_kfs;
    auto local_map_id = new_local_map_id_.value();
    new_local_map_id_.reset();
    auto new_local_map = local_maps_.at(local_map_id).local_map;
    for (const auto &id : extend_key_frames_ids_) {
      ComputeLoopConstaint(local_map_id, id);
    }
  }
}
//
void MapManager::ComputeLoopConstaint(const LocalMapId &local_map_id,
                                      const KeyFrameId &key_frame_id) {
  auto last_connection_time =
      last_trajectory_connect_time_[key_frame_id.trajectory_id]
                                   [local_map_id.trajectory_id];
  //

  const auto &kf_data = key_frames_datas_.at(key_frame_id);
  auto kf_time = key_frames_datas_.at(key_frame_id).data->time;
  if (key_frame_id.trajectory_id == local_map_id.trajectory_id ||
      kf_time < last_connection_time +
                    common::FromSeconds(
                        options_.global_constraint_search_after_n_seconds)) {
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
    const KeyFrameId id(key_frame_id.trajectory_id_, key_frame_id.index + i);
    if (extend_key_frames_ids_.count(id)) {
      continuous_ids.emplace(id, key_frames_datas_.at(id));
    }
  }
  //
  loop_detect_->Detect(std::pair<LocalMapId, LocalMap>(
                           local_map_id, local_maps_.at(local_map_id)),
                       continuous_ids);
}
//
void MapManager::ExtendedKeyFrameData(const LocalMap &local_map,
                                      const KeyFrameId &id,
                                      KeyFrameData *data) {
  if (!options_.enable_loop_closure) return;
  work_item_queue_->AddWorkItem([=]() {
    map_point_construct_->ExtractExtendData(local_map, data);
    extend_key_frames_ids_.insert(id);
    ComputeConstaints(id);
    loop_detect_->NotifyNodeAdditionFinished();
    //
    ++num_kf_num_since_last_loop_closure_;
    if (options_.optimize_min_kf_min_num > 0 &&
        num_kf_num_since_last_loop_closure_ >
            num_kf_num_since_last_loop_closure_) {
      num_kf_num_since_last_loop_closure_ = 0;
      return WorkItem::Result::kInterruptForImmediateRun;
    }
    return WorkItem::Result::Normal;
  });
}
//
void MapManager::RunOptimization(
    std::vector<std::shared_ptr<LoopDetctResult>> &) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    op_constraints_.insert(op_constraints_.end(), result.begin(), result.end());
  }
}
//

void MapManager::ReconstructLocalMapOptimization(
    const std::map<LocalMapId, std::shared_ptr<LocalMap>> &local_maps) {
  local_opimization_->Optimize(&local_maps);
  //
  if (localmap_update_callback_) {
    localmap_update_callback_(&local_maps);
    UpdateLocalOpLocalMap(&op_local_maps);
  }
  //
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
    if (last_new_update_key_frame_ids_.count(data.id)) {
      new_local_map.AddKeyFrameData(data.id, data.data);
    } else {
      map_point_construct_->ConstructExtend(new_local_map,
                                            &key_frames_datas_.at(data.id));
      new_local_map.AddKeyFrameData(data.id, data.data);
    }
    new_update_ids.insert(data.id);
  }

  //
  last_new_update_key_frame_ids_ = std::move(new_update_ids);
  return new_local_map_ptr;
}

//
LocalMapId MapManager::AddLocalMap(int trajectory,
                                   std::shared_ptr<LocalMap> local_map) {
  if (!options_.enable_loop_closure) return;
  work_item_queue_->AddWorkItem([&]() {
    auto new_local_map = ReconstructLocalMap(local_map);
    const auto local_map_id =
        local_maps_.Append(trajectory, LocalMapData{new_local_map});
    //
    std::map<LocalMapId, std::shared_ptr<LocalMap>> op_local_maps;
    op_local_maps.emplace(local_map_id, new_local_map);
    ReconstructLocalMapOptimization(op_local_maps);
    if (options_.need_update_track_local_map) {
      UpdataActiveTrackLocalMap(local_map);
    }
    new_local_map->UpdadataExtendFinishData(true);
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

//
void MapManager::ComputeLoopConstaints(
    const LocalMapId &local_map_id,
    const std::vector<KeyFrameId> &key_frame_id) {}

void MapManager::UpdateLocalOpLocalMap(
    std::map<LocalMapId, std::shared_ptr<LocalMap>> *op_local_maps) {
  // auto const &all_ref_data =
  //     op_local_maps->cbegin()->second->AllKeyFrameRefPose();
  // const auto end_kf_data_id = all_ref_data.crbegin()->first;
  // transform::Rigid3d global_kf_pose =
  //     local_maps_.at(op_local_maps->cbegin()->first).global_pose *
  //     all_ref_data.at(end_kf_data_id);
  // local_to_global_transform_ =
  //     global_kf_pose.inverse() * op_local_maps->cbegin()
  //                                   ->second->AllKeyFrameDatas()
  //                                   .at(end_kf_data_id)
  //                                   .data->pose;
}

void MapManager::TrimOptimizedLocalMap() {
  std::set<KeyFrameId> finish_key_frame_ids;
  std::set<KeyFrameId> unfinished_key_frame_ids;
  std::set<LocalMapId> finish_local_map_ids;
  for (const auto &local_map : local_maps_) {
    auto all_local_map_key_frame_ids =
        local_map.data.local_map->GetTrimBeforKeyFrameId();
    if (local_map.data.local_map->IsOptimization()) {
      finish_key_frame_ids.merge(all_local_map_key_frame_ids);
      finish_local_map_ids.insert(local_map.id);
    } else {
      unfinished_key_frame_ids.merge(all_local_map_key_frame_ids);
    }
  }
  std::vector<KeyFrameId> trim_ids;
  std::set_difference(finish_key_frame_ids.begin(), finish_key_frame_ids.end(),
                      unfinished_key_frame_ids.begin(),
                      unfinished_key_frame_ids.end(),
                      std::back_inserter(trim_ids));
  for (const auto &id : trim_ids) {
    TrimKeyFrameData(id);
  }
  for (auto const local_map_id : finish_local_map_ids) {
    std::lock_guard<std::mutex> lock(mutex_);
    local_maps_.Trim(local_map_id);
  }
}

//
//
void MapManager::Optimization() {
  //
  std::vector<PoseConstraint>

      pose_graph_optimize_->Solve();
}

//
bool MapManager::IsRunOptimization() {}

void MapManager::UpdateNewFinishLocalMapLoop(
    const LocalMapId &local_map_id, std::shared_ptr<LocalMap> &,
    const std::vector<KeyFrameId> &candidate_kf) {
  // std::map<LocalMapId, std::shared_ptr<LocalMap>> local_maps;
  // for (const auto &kf_id : candidate_kf) {
  //   std::lock_guard<std::mutex> lock(mutex_);
  //   loop_closure_->Detect(
  //       local_maps, kf_id, key_frames_datas_.at(kf_id),
  //       [this](std::vector<std::unique_ptr<LoopDetectResult>> result) {
  //         LOG(INFO) << "Loop detect.";
  //         UpdateLoopConstraint(std::move(result));
  //       });
  // }
}

void MapManager::UpdateLoopConstraint(
    std::vector<std::unique_ptr<LoopDetectResult>> result) {
  std::lock_guard<std::mutex> lock(mutex_);
  // loop_constraints_.insert(loop_constraints_.end(), result.begin(),
  //                          result.end());
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
