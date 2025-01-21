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
  if (option.enable_local_opimization || option.enable_loop_closure ||
      option.enable_track_map_opti) {
    map_point_construct_ = std::make_unique<MapPointConstruct>(
        option.map_point_construct_option, options_.cameras, voc);

    thread_pool_ = std::make_unique<common::ThreadPool>(options_.thread_num);
    map_manager_ = std::make_unique<MapManager>(
        options_.map_manager_option, map_point_construct_.get(),
        thread_pool_.get(),
        [this](std::map<LocalMapId, std::shared_ptr<LocalMap>> *op_local_maps) {
          UpdataActiveWithOpLocal(op_local_maps);
        });
    LOG(INFO) << "Enable mapp manger..";
  } else {
    map_point_construct_ = std::make_unique<MapPointConstruct>(
        option.map_point_construct_option, options_.cameras, nullptr);
    map_manager_ = std::make_unique<MapManager>(
        options_.map_manager_option, map_point_construct_.get(), nullptr);
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
    auto local_to_globla_transfom = map_manager_->GetLocalToGlobla();
    key_frame_data.data->global_pos =
        local_to_globla_transfom * key_frame_data.data->pose;
    LOG(INFO) << local_to_globla_transfom;
    //
    auto key_frame_id = map_manager_->AddKeyFrameData(t, key_frame_data);
    active_local_maps_->AddKeyFrameData(key_frame_id, key_frame_data);
    if (local_map_front_ == nullptr) {
      local_map_front_ = active_local_maps_->GetLocalMap().front();
    }

    std::map<LocalMapId, std::shared_ptr<LocalMap>> op_local_maps;

    
    if (active_local_maps_->GetLocalMap().front() != local_map_front_) {
      if (options_.enable_local_opimization || options_.enable_loop_closure) {
        map_manager_->AddLocalMap(t, local_map_front_);
      }
      std::shared_ptr<LocalMap> last_local_map_front=nullptr;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        last_local_map_front = local_map_front_;
        local_map_front_ = active_local_maps_->GetLocalMap().front();
      }
      
      if (options_.enable_track_map_opti) {
        op_local_maps[{0, op_local_maps.size()}] =
            std::make_shared<LocalMap>(*last_local_map_front);
        {
          std::lock_guard<std::mutex> lock(mutex_);
          *op_local_maps[{0, op_local_maps.size() - 1}] = *last_local_map_front;
        }

        if (options_.enable_track_map_opti) {
          UpdataActiveWithOpLocal(&op_local_maps);
        }
      }

      // AddWorkItem([this, last_local_map_front]() {
      TrimKeyFrameData(last_local_map_front);
      // return WorkItem::Result::Normal;
      // });
    }
    //
    op_local_maps[{0, op_local_maps.size()}] =
        std::make_shared<LocalMap>(*local_map_front_);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      *op_local_maps[{0, op_local_maps.size() - 1}] = *local_map_front_;
    }
    //
    if (options_.enable_track_map_opti) {
      AddWorkItem([this, op_local_maps]() {
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

        if (track_local_map_op_sampler_->Pulse() &&
            op_local_maps_temp.rbegin()->second->Size() >
                options_.local_map_option.max_kf_num) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
          }
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
}
//
void MappingBuilder::AddWorkItem(
    const std::function<WorkItem::Result()> &work_item) {
  std::lock_guard<std::mutex> lock(work_queue_mutex_);
  if (work_queue_ == nullptr) {
    work_queue_ = std::make_unique<WorkQueue>();
    auto task = std::make_unique<common::Task>();
    task->SetWorkItem([this]() { DrainWorkQueue(); });
    thread_pool_->Schedule(std::move(task));
  }

  const auto now = std::chrono::steady_clock::now();
  work_queue_->push_back({now, work_item});
}

//
void MappingBuilder::DrainWorkQueue() {
  WorkItem::Result process_work_queue = WorkItem::Result::Normal;
  //   LOG(INFO)<<"DrainWorkQueue";
  size_t work_queue_size;
  while (process_work_queue == WorkItem::Result::Normal) {
    std::function<WorkItem::Result()> work_item;
    {
      std::lock_guard<std::mutex> locker(work_queue_mutex_);
      if (work_queue_->empty()) {
        work_queue_.reset();
        return;
      }
      work_item = work_queue_->front().task;
      work_queue_->pop_front();
      work_queue_size = work_queue_->size();
    }
    process_work_queue = work_item();
  }
  if (process_work_queue == WorkItem::Result::kRunLocalOptimization) {
  }
  DrainWorkQueue();
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
  if (work_queue_ == nullptr) return;

  size_t work_queue_size = 0;
  {
    std::function<WorkItem::Result()> work_item;
    work_queue_size = work_queue_->size();
  }

  while (work_queue_size) {
    {
      std::lock_guard<std::mutex> locker(work_queue_mutex_);
      work_queue_size = work_queue_->size();
      usleep(1000);
    }
    LOG(INFO) << "wait work_queue_size " << work_queue_size;
  }
}

}  // namespace mapping
}  // namespace jarvis
