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
MappingBuilder::MappingBuilder(const MapBuilderOption &option)
    : options_(option) {
  //
  LOG(INFO) << "local track " << options_.enable_local_track;

  if (option.enable_local_opimization || option.enable_loop_closure) {
    map_point_construct_ = std::make_unique<MapPointConstruct>(
        option.map_point_construct_option,
        std::make_unique<dbow::Vocabulary>(
            dbow::GetVocabulary(0, option.vocabulary_filebrif)));

    map_manager_ = std::make_unique<MapManager>(
        option.map_manager_option, map_point_construct_.get(), true);
    LOG(INFO) << "Enable mapp manger..";
  } else {
    map_point_construct_ = std::make_unique<MapPointConstruct>(
        option.map_point_construct_option, nullptr);
    map_manager_ = std::make_unique<MapManager>(
        option.map_manager_option, map_point_construct_.get(), false);
  }
  //
  //
  //

  local_map_track_ =
      std::make_unique<LocalMapTrack>(option.local_map_track_option);
  //
  key_frame_filter_ =
      std::make_unique<KeyFrameFilter>(option.key_frame_filter_option);
}
//
std::unique_ptr<transform::Rigid3d> MappingBuilder::TrackLocalMap(
    const TrackingData &frame_data) {
  if (local_map_front_) {
    return local_map_track_->Track(
        local_map_front_,
        map_point_construct_->TrackDataToKeyFrameData(frame_data));
  }
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
void MappingBuilder::AddLocalMap() {}
//
//
// 必须放在线程的队列里面按顺序执行
void MappingBuilder::UpdataFinishLocalMapData(
    std::shared_ptr<LocalMap> local_map) {
  // for (const auto &data : local_map_finish_temp_->AllKeyFrameDatas()) {
  //   //
  //   map_point_construct_->ConstructExtend(
  //       local_map_finish_temp_, &map_manager_->GetKeyFrameId(data.id));
  //   //
  // }
}
//
//
//
void MappingBuilder::AddTrackingData(const int t, const TrackingData &data) {
  if (!key_frame_filter_->IsKeyFrame(data)) {
    return;
  }
  //
  if (options_.enable_local_track || options_.enable_local_opimization) {
    auto key_frame_data = map_point_construct_->TrackDataToKeyFrameData(data);
    auto key_frame_id = map_manager_->AddKeyFrameData(t, key_frame_data);
    active_local_maps_->AddKeyFrameData(key_frame_id, key_frame_data);

    if (active_local_maps_->GetLocalMap().front() != local_map_front_) {
      if (options_.enable_local_opimization || options_.enable_loop_closure) {
        map_manager_->AddLocalMap(t, local_map_front_);
      }
      TrimKeyFrameData();
      local_map_front_ = active_local_maps_->GetLocalMap().front();
    }
  }
}
//
//
std::map<KeyFrameId, transform::TimestampedTransform>
MappingBuilder::GetAllKeyFramePose() {
  // 可能存在线程安全!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  std::map<KeyFrameId, transform::TimestampedTransform> result;
  const auto &key_frame_datas = map_manager_->AllKeyFrameDatas();
  for (auto const &key_frame : key_frame_datas) {
    result.emplace(key_frame.id,
                   transform::TimestampedTransform{key_frame.data.data->time,
                                                   key_frame.data.data->pose});
  }
  return result;
}
//
std::vector<Eigen::Vector3d> MappingBuilder::GetAllMapPoints() {
  // std::lock_guard<std::mutex> lock(mutex_);
  // 可能存在线程安全
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
