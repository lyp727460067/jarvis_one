#include "jarvis/mapping/map_manger.h"

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
#include "jarvis/mapping/map_manger.h"
#include "jarvis/mapping/mapping_data.h"
#include "jarvis/mapping/match/des_matcher.h"
#include "jarvis/mapping/match/pic_writer.h"
// /
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
    local_opimization_ = std::make_unique<GraphLocalMapOptimization6TOF>(
        option.local_map_optimization_option);
  } else {
    local_opimization_ = std::make_unique<LocalMapOptimization>(
        option.local_map_optimization_option);
  }
}
//
//
LocalMapId MapManager::AddLocalMap(int trajector,
                                   std::shared_ptr<LocalMap> local_map) {
  //
  if (!options_.enable_loop_closure) return;
  const auto local_map_id =
      local_maps_.Append(trajector, LocalMapData{local_map});
      ///
      std::vector<KeyFrameId> candidata_kfs;
  {
    for (const auto &kf : key_frames_datas_) {
      //
      const double kf_relative_local_map_distance =
          (local_map->LocalPose().inverse() * kf.data.data->pose)
              .translation()
              .norm();
      if (loop_detect_sampler_->Pulse() &&
          kf_relative_local_map_distance <
              options_.same_trajector_max_loop_detct_distance) {
        candidata_kfs.push_back(kf.id);
      }
    }
  }
  //
  for (const auto &kf : candidata_kfs) {
    std::map<KeyFrameId, KeyFrameData> continuous_ids;
    for (int i = -options_.continuous_candidate_loop_frame / 2;
         i < options_.continuous_candidate_loop_frame / 2; i++) {
      const KeyFrameId id(kf.trajectory_id_, kf.index + i);
      if (key_frames_datas_.Contains(id)) {
        continuous_ids.emplace(id, key_frames_datas_.at(id))
      }
      loop_detect_->Detect({local_map_id, local_map}, continuous_ids);
      //
    }
  }
  //

  last_new_update_key_frame_ids_ = std::move(new_update_ids);
  UpdateNewFinishLocalMapLoop(local_map_id,
                              local_maps_.at(local_map_id).local_map,
                              std::move(candidata_kfs));
}
//
//

//
void MapManager::UpdateLocalOpLocalMap(
    std::map<LocalMapId, std::shared_ptr<LocalMap>> *op_local_maps) {
  auto const &all_ref_data =
      op_local_maps->cbegin()->second->AllKeyFrameRefPose();
  const auto end_kf_data_id = all_ref_data.crbegin()->first;
  //
  //
  transform::Rigid3d gloable_kf_pose =
      local_maps_.at(op_local_maps->cbegin()->first).globla_pose *
      all_ref_data.at(end_kf_data_id);
  //
  //
  local_to_globla_transform_ =
      gloable_kf_pose.inverse() * op_local_maps->cbegin()
                                      ->second->AllKeyFrameDatas()
                                      .at(end_kf_data_id)
                                      .data->pose;
}

void MapManager::TrimOptimizedLocalMap() {
  std::set<KeyFrameId> finsh_key_frame_ids;
  std::set<KeyFrameId> unfinsh_key_frame_ids;
  std::set<LocalMapId> finish_local_map_ids;
  for (const auto &local_map : local_maps_) {
    auto all_local_map_key_frame_ids =
        local_map.data.local_map->GetTrimBeforKeyFrameId();
    if (local_map.data.local_map->IsOptimization()) {
      finsh_key_frame_ids.merge(all_local_map_key_frame_ids);
      finish_local_map_ids.insert(local_map.id);
    } else {
      unfinsh_key_frame_ids.merge(all_local_map_key_frame_ids);
    }
  }
  std::vector<KeyFrameId> trim_id;
  std::set_difference(finsh_key_frame_ids.begin(), finsh_key_frame_ids.end(),
                      unfinsh_key_frame_ids.begin(),
                      unfinsh_key_frame_ids.end(),
                      std::back_insert_iterator(trim_id));
  for (const auto &id : trim_id) {
    TrimKeyFrameData(id);
  }
  for (auto const local_map_id : finish_local_map_ids) {
    std::lock_guard<std::mutex> lock(mutex_);
    local_maps_.Trim(local_map_id);
  }
}
//

KeyFrameId MapManager::AddKeyFrameData(int trajector,
                                       const KeyFrameData &data) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto kf_id = key_frames_datas_.Append(trajector, data);
  if (!options_.enable_loop_closure) return kf_id;
  std::map<LocalMapId, std::shared_ptr<LocalMap>> local_maps;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (loop_detect_kf_sampler_->Pulse()) {
      for (auto it = local_maps_.BeginOfTrajectory(trajector);
           it != local_maps_.EndOfTrajectory(trajector); ++it) {
        const transform::Rigid3d delta_pose =
            local_maps_.at(it->id).local_map->LocalPose().inverse() *
            data.data->pose;
        if (delta_pose.translation().norm() <
            options_.max_loop_detct_distance) {
          local_maps.emplace(it->id, it->data.local_map);
        }
      }
    }
  }
  loop_closure_->Detect(
      local_maps, std::map<KeyFrameId, KeyFrameData>{},
      [this](std::vector<std::unique_ptr<LoopDetctResult>> result) {
        UpdateLoopConstraint(std::move(result));
        LOG(INFO) << "Loop detet.";
      });
  //
  if (IsRunOptimization()) {
    auto full_op_process_tast = std::make_unique<common::Task>();
    full_op_process_tast->SetWorkItem([this]() {
      Optimization();
      UpdateOpimizeData();
    });
    //
  }

  return kf_id;
}
//
bool MapManager::IsRunOptimization() {
  CHECK(false);
  return false;
}
//
void MapManager::UpdateNewFinishLocalMapLoop(
    const LocalMapId &local_map_id, std::shared_ptr<LocalMap> &,
    const std::vector<KeyFrameId> &candidata_kf) {
  //
  // std::map<LocalMapId, std::shared_ptr<LocalMap>> local_maps;
  // for (const auto &kf_id : candidata_kf) {
  //   std::lock_guard<std::mutex> lock(mutex_);
  //   loop_closure_->Detect(
  //       local_maps, kf_id, key_frames_datas_.at(kf_id),
  //       [this](std::vector<std::unique_ptr<LoopDetctResult>> result) {
  //         LOG(INFO) << "Loop detet.";
  //         UpdateLoopConstraint(std::move(result));
  //       });
  // }
}
//

void MapManager::UpdateLoopConstraint(
    std::vector<std::unique_ptr<LoopDetctResult>> result) {
  std::lock_guard<std::mutex> lock(mutex_);
  // loop_constraints_.insert(loop_constraints_.end(), result.begin(),
  //                          result.end());
}
    //
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
  //
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

  //
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
    //
    std::vector<Eigen::Vector3d> result;
    std::set<MapPointId> exisit_map_points;
    for (const auto &local_map : finish_local_maps) {
      const auto all_local_map_points = local_map->AllMapPoints();
      for (const auto &map_point : all_local_map_points) {
        if (exisit_map_points.count(map_point.id)) continue;
        result.push_back(local_map->LocalPose() * map_point.data.data->pos);
        exisit_map_points.insert(map_point.id);
      }
    }
    return result;
  }
  //

  MapManager::~MapManager() {
    //
  }

}  // namespace mapping
}  // namespace jarvis