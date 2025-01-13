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
MappingBuilder::MappingBuilder(const MapBuilderOption &option,dbow::Vocabulary *voc)
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
        [this](std::map<LocalMapId, LocalMap *> *op_local_maps) {
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
        option.track_map_opti_culling_sampler);
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
    track_local_map_opimization_ =
        std::make_unique<LocalMapOptimization>(op_option);
  }
  //
  //
  LocalMapOption local_map_option = options_.local_map_option;
  local_map_option.cameras =  options_.cameras;
  local_map_option.image_boxs=  options_.image_boxs;
  active_local_maps_ = std::make_unique<ActiveLocalMap>(
      local_map_option);
  //
  LOG(INFO)<<"mapping construct done.";
}
//
std::unique_ptr<LocalMapMatchResult> MappingBuilder::TrackLocalMap(
    const TrackingData &frame_data) {

  if (local_map_track_ == nullptr) return nullptr;
  if (local_map_front_) {
    std::lock_guard<std::mutex> lock(mutex_);
    return local_map_track_->Track(
        local_map_front_,
        map_point_construct_->TrackDataToKeyFrameData(frame_data));
  }
  return nullptr;
}
void MappingBuilder::TrimKeyFrameData() {
  //
  if (local_map_front_ == nullptr || options_.enable_loop_closure) return;
  //
  
  if (options_.enable_local_opimization) {
    map_manager_->TrimOptimizedLocalMap();
    return;
  }
  std::set<KeyFrameId> last_key_frame_ids;
  for (const auto &key_frame_data : local_map_front_->AllKeyFrameDatas()) {
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
    std::map<LocalMapId, LocalMap *> *op_local_maps) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!local_map_front_) return;
  for (auto &local_map : *op_local_maps) {
    local_map_front_->UpdateExistData(*local_map.second);
  }
}

//

void MappingBuilder::TrackLocalMapOptimize() {
  if (!track_local_map_op_sampler_->Pulse() || local_map_front_ == nullptr)
    return;
  if (local_map_front_->Size() < 10) return;
  std::map<LocalMapId, LocalMap *> op_local_maps;
  // {
    // std::lock_guard<std::mutex> lock(mutex_);
    LocalMap tem_local_map(*local_map_front_);
    tem_local_map = *local_map_front_;
    op_local_maps[{0, 0}] = &tem_local_map;
  // }
  track_local_map_opimization_->Optimize(&op_local_maps);
  UpdataActiveWithOpLocal(&op_local_maps);
  //
  // auto local_map_op_task = std::make_unique<common::Task>();
  // local_map_op_task->SetWorkItem(
  //     [&]() { track_local_map_opimization_->Optimize(&op_local_maps); });
  // auto local_map_op_task_handle =
  //     thread_pool_->Schedule(std::move(local_map_op_task));
  // when_done_task_->AddDependency(local_map_op_task_handle);
  // when_done_task_->SetWorkItem(
  //     [&] { UpdataActiveWithOpLocal(&op_local_maps); });
  //
}

//
void MappingBuilder::AddTrackingData(const int t, const TrackingData &data) {
  if (!key_frame_filter_->IsKeyFrame(data)) {
    return;
  }
  //
  if (options_.enable_local_track || options_.enable_local_opimization) {
    auto key_frame_data = map_point_construct_->TrackDataToKeyFrameData(data);
    auto local_to_globla_transfom = map_manager_->GetLocalToGlobla();
    key_frame_data.data->global_pos =
        local_to_globla_transfom * key_frame_data.data->pose;
    //
    auto key_frame_id = map_manager_->AddKeyFrameData(t, key_frame_data);
    active_local_maps_->AddKeyFrameData(key_frame_id, key_frame_data);
    if (local_map_front_ == nullptr) {
      local_map_front_ = active_local_maps_->GetLocalMap().front();
    }
    if (active_local_maps_->GetLocalMap().front() != local_map_front_) {
      if (options_.enable_local_opimization || options_.enable_loop_closure) {
        map_manager_->AddLocalMap(t, local_map_front_);
      }
      TrimKeyFrameData();
      std::lock_guard<std::mutex> lock(mutex_);
      local_map_front_ = active_local_maps_->GetLocalMap().front();
    }
    if (options_.enable_track_map_opti) {
      TrackLocalMapOptimize();
    }
  }
}
//
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
