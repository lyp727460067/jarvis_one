#include "jarvis/mapping/map_builder.h"

#include <unistd.h>

#include <map>
#include <memory>
#include <set>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "jarvis/common/id.h"
namespace jarvis {
namespace mapping {
//

//
MappingBuilder::MappingBuilder(const MapBuilderOption &option,
                               dbow::Vocabulary *voc)
    : options_(option) {
  //
  LOG(INFO) << "local track " << options_.enable_local_track;
  //

  //
  if (option.enable_loop_closure) {
    map_point_construct_ = std::make_unique<MapPointConstruct>(
        option.map_point_construct_option, options_.cameras, voc);
    thread_pool_ = std::make_unique<common::ThreadPool>(options_.thread_num);
    map_manager_ = std::make_unique<MapManager>(
        options_.map_manager_option, map_point_construct_.get(),
        options_.cameras, thread_pool_.get(),
        [this](const std::shared_ptr<LocalMap> local_map) {
          if (!options_.updated_active_track_localmap_data_from_mapmanger)
            return;
          std::map<LocalMapId, std::shared_ptr<LocalMap>> op_local_maps;
          op_local_maps.emplace(LocalMapId(0, 0), local_map);
          UpdataActiveWithOpLocal(&op_local_maps);
        });
    LOG(INFO) << "Enable mapp manger..";
  } else {
    map_point_construct_ = std::make_unique<MapPointConstruct>(
        option.map_point_construct_option, options_.cameras, nullptr);
    map_manager_ = std::make_unique<MapManager>(options_.map_manager_option,
                                                map_point_construct_.get(),
                                                options_.cameras, nullptr);
  }
  //
  //
  //
  if (options_.enable_local_track) {
    local_map_track_ =
        std::make_unique<LocalMapTrack>(option.local_map_track_option);
  }
  //
  key_frame_filter_ =
      std::make_unique<KeyFrameFilter>(option.key_frame_filter_option);
  //
  if (options_.enable_track_map_opti) {
    when_done_task_ = std::make_unique<common::Task>();
    track_local_map_op_sampler_ = std::make_unique<common::FixedRatioSampler>(
        option.track_map_opti_sampler);
    LocalMapOptimizationOption op_option(option.track_local_map_opt_option);
    //
    op_option.track_sequence =
        options_.map_manager_option.local_map_optimization_option
            .track_sequence;
    //
    op_option.extric_camera_to_imu =
        options_.map_manager_option.local_map_optimization_option
            .extric_camera_to_imu;
    //
    LocalMapOptimizationOption fi_op_option(option.finish_track_local_map_opt_option);

    fi_op_option.track_sequence =
        options_.map_manager_option.local_map_optimization_option
            .track_sequence;
    //
    fi_op_option.extric_camera_to_imu =
        options_.map_manager_option.local_map_optimization_option
            .extric_camera_to_imu;

    track_local_map_opimization_ =
        std::make_unique<EssentialGraphLocalMapOptimization>(op_option);

    finish_track_local_map_opimization_ =
        std::make_unique<LocalMapOptimization>(fi_op_option);    
  }
  //
  //
  LocalMapOption local_map_option = options_.local_map_option;
  local_map_option.cameras = options_.cameras;
  local_map_option.image_boxs = options_.image_boxs;
  active_local_maps_ = std::make_unique<ActiveLocalMap>(local_map_option);
  //

  work_item_queue_ =
      std::make_unique<WorkItemQueue>("map_builder", thread_pool_.get());

  LOG(INFO) << "mapping construct done.";
}
//
std::shared_ptr<LocalMapMatchResult> MappingBuilder::TrackLocalMap(
    const TrackingData &frame_data) {
  if (local_map_track_ == nullptr) return nullptr;
  if (local_map_front_) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto result_match = local_map_track_->Track(
        local_map_front_,
        map_point_construct_->TrackDataToKeyFrameData(frame_data));
    if (result_match == nullptr) return nullptr;
    local_map_match_result_catch_[frame_data.data->time] = result_match;

    if (local_map_match_result_catch_.size() > 10) {
      local_map_match_result_catch_.erase(
          local_map_match_result_catch_.begin());
    }
    return result_match;
  }
  return nullptr;
}
void MappingBuilder::TrimKeyFrameData(
    const std::shared_ptr<LocalMap> &front_local_map) {
  //
  if (front_local_map == nullptr || options_.enable_loop_closure) return;
  //

  if (options_.enable_local_opimization) {
    map_manager_->TrimOptimizedLocalMap();
    return;
  }
  std::set<KeyFrameId> last_key_frame_ids;
  for (const auto &key_frame_data : front_local_map->AllKeyFrameDatas()) {
    last_key_frame_ids.insert(key_frame_data.id);
  }

  std::set<KeyFrameId> new_key_frame_ids;
  for (const auto &key_frame_data :
       active_local_maps_->GetLocalMap().front()->AllKeyFrameDatas()) {
    new_key_frame_ids.insert(key_frame_data.id);
  }
  std::vector<KeyFrameId> trim_id;
  std::set_difference(last_key_frame_ids.begin(), last_key_frame_ids.end(),
                      new_key_frame_ids.begin(), new_key_frame_ids.end(),
                      std::back_insert_iterator(trim_id));
  for (const auto &id : trim_id) {
    map_manager_->TrimKeyFrameData(id);
  }
}
//

//

void MappingBuilder::UpdataActiveWithOpLocal(
    std::map<LocalMapId, std::shared_ptr<LocalMap>> *op_local_maps) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!local_map_front_) return;
  for (auto &local_map : *op_local_maps) {
    local_map_front_->UpdateExistData(*local_map.second);
  }
}

//

//
void MappingBuilder::TrackLocalMapOptimize(
    LocalMapOptimization *optimizer,
    std::map<LocalMapId, std::shared_ptr<LocalMap>> *local_map) {
  //
  optimizer->Optimize(local_map);
  UpdataActiveWithOpLocal(local_map);
}
//

void MappingBuilder::LocalTrackOptimize(
    std::shared_ptr<LocalMap> last_finish_local_map) {
  //
  std::map<LocalMapId, std::shared_ptr<LocalMap>> op_local_maps;
  op_local_maps[{0, op_local_maps.size()}] =
      std::make_shared<LocalMap>(*last_finish_local_map);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    *op_local_maps[{0, op_local_maps.size() - 1}] = *last_finish_local_map;
  }
  UpdataActiveWithOpLocal(&op_local_maps);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    op_local_maps[{0, op_local_maps.size()}] =
        std::make_shared<LocalMap>(*local_map_front_);
    *op_local_maps[{0, op_local_maps.size() - 1}] = *local_map_front_;
  }
  //
  if (options_.enable_track_map_opti && track_local_map_op_sampler_->Pulse()) {
    work_item_queue_->AddWorkItem([this, op_local_maps]() {
      //
      std::map<LocalMapId, std::shared_ptr<LocalMap>> op_local_maps_temp =
          op_local_maps;
      if (op_local_maps_temp.size() == 2) {
        if (op_local_maps_temp.size() == 2) {
          op_local_maps_temp.erase(op_local_maps_temp.begin());

          // TrackLocalMapOptimize(finish_track_local_map_opimization_.get(),
          //                       &op_local_maps_temp);
        }

        return WorkItem::Result::Normal;
      }
      if (op_local_maps_temp.rbegin()->second->Size() >
          options_.local_map_option.max_kf_num) {
        if (op_local_maps_temp.size() == 2) {
          op_local_maps_temp.erase(op_local_maps_temp.begin());
        }
        TrackLocalMapOptimize(track_local_map_opimization_.get(),
                              &op_local_maps_temp);
      }

      return WorkItem::Result::Normal;
    });
  }
}
//
void MappingBuilder::AddTrackingData(const int t, const TrackingData &data) {
  if (!key_frame_filter_->IsKeyFrame(data)) {
    return;
  }
  //
  if (options_.enable_local_track || options_.enable_local_opimization) {
    std::shared_ptr<LocalMapMatchResult> local_map_track_result = nullptr;
    if (local_map_match_result_catch_.count(data.data->time)) {
      local_map_track_result =
          local_map_match_result_catch_.at(data.data->time);
      CHECK(local_map_track_result);
      auto current_time_it =
          local_map_match_result_catch_.lower_bound(data.data->time);
      //
      local_map_match_result_catch_.erase(local_map_match_result_catch_.begin(),
                                          current_time_it);
    }
    auto key_frame_data = map_point_construct_->TrackDataToKeyFrameData(
        data, local_map_track_result);
    //

    // /
    //
    auto local_to_globla_transfom = map_manager_->GetLocalToGlobalTransform();
    key_frame_data.global_pose =
        local_to_globla_transfom * key_frame_data.data->pose;
    //
    auto key_frame_id = map_manager_->AddKeyFrameData(t, key_frame_data);
    active_local_maps_->AddKeyFrameData(key_frame_id, key_frame_data);
    if (local_map_front_ == nullptr) {
      local_map_front_ = active_local_maps_->GetLocalMap().front();
    }

    if (active_local_maps_->GetLocalMap().front() != local_map_front_) {
      if (options_.enable_loop_closure) {
        map_manager_->AddLocalMap(t, local_map_front_);
      }
      std::shared_ptr<LocalMap> last_local_map_front = nullptr;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        last_local_map_front = local_map_front_;
        local_map_front_ = active_local_maps_->GetLocalMap().front();
      }

      if (options_.enable_track_map_opti) {
        work_item_queue_->AddWorkItem([this, last_local_map_front]() {
          LocalTrackOptimize(last_local_map_front);
          return WorkItem::Result::Normal;
        });
      }
      work_item_queue_->AddWorkItem([this, last_local_map_front]() {
        TrimKeyFrameData(last_local_map_front);
        return WorkItem::Result::Normal;
      });
    }
    //
    std::shared_ptr<KeyFrameData::Data> data = key_frame_data.data;
    map_manager_->ExtendedKeyFrameData(*local_map_front_, key_frame_id,
                                       data.get());
  }
}
//
std::map<KeyFrameId, transform::TimestampedTransform>
MappingBuilder::GetAllKeyFramePose() {
  return map_manager_->GetAllKeyFramePose();
}
//
std::vector<Eigen::Vector3d> MappingBuilder::GetAllMapPoints() {
  return map_manager_->GetAllMapPoints();
}

void MappingBuilder::AddImuData(const sensor::ImuData &imu_data) {
  // AddWorkItem([=]() { return WorkItem::Result::Normal; });
}
void MappingBuilder::AddFixData(const sensor::FixedFramePoseData &fix_data) {
  // AddWorkItem([=]() { return WorkItem::Result::Normal; });
}

void MappingBuilder::AddOdometryData(const sensor::OdometryData &odo_data) {}
//

transform::Rigid3d MappingBuilder::Relocaiton(const TrackingData &frame_data) {
  CHECK(false);
  return {};
}

//
MappingBuilder::~MappingBuilder() {

}

}  // namespace mapping
}  // namespace jarvis
