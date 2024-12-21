#include "jarvis/mapping/map_builder.h"

#include "jarvis/common/id.h"

#include <map>
#include <memory>
#include <set>
#include <unistd.h>

#include "Eigen/Core"
#include "Eigen/Geometry"
namespace jarvis {
namespace mapping {
//

//
MappingBuilder::MappingBuilder(const MapBuilderOption &option)
    : options_(option) {
  //
  LOG(INFO)<<"local track "<<options_.enable_local_track;
  if (!option.enable_local_opimization) {
    map_manager_ = std::make_unique<MapManager>(option.map_manager_option,
                                                option.cameras, nullptr);

    LOG(INFO) << "Local op false..";
  } else {
    map_manager_ = std::make_unique<MapManager>(
        option.map_manager_option, option.cameras,
        std::make_unique<dbow::Vocabulary>(
            dbow::GetVocabulary(0, option.vocabulary_filebrif)));
  }
  //
  //
  //
  data_fuse_ = std::make_unique<MappingDataFuse>(this);

  data_culling_ = std::make_unique<DataCulling>(option.data_culling_option,
                                                data_fuse_.get());
  //
  local_map_optimization_ = std::make_unique<LocalMapOptimization>(
      option.local_map_optimization_option);
  local_map_track_ =
      std::make_unique<LocalMapTrack>(option.local_map_track_option);
  //
  key_frame_filter_ =
      std::make_unique<KeyFrameFilter>(option.key_frame_filter_option);
  //
  local_mapping_optimization_sampler_ =
      std::make_unique<common::FixedRatioSampler>(
          option.local_optimization_ration);
  culling_sampler_ = std::make_unique<common::FixedRatioSampler>(
      option.culling_sampler_ration);

  work_queue_ = std::make_unique<WorkQueue>();
    if (option.enable_local_opimization) {
  thread_ = std::thread([this]() {
    while (!kill_thread_) {
      DrainWorkQueue();
      usleep(1000);
    }
  });
    }
}
//
//
void MappingBuilder::KeyFrameDataFuse(const KeyFrameId &frame_id) {
  // //
  // auto conn_frames =
  //     map_manager_->Covisibility()->GetConnectedKeyFrames(frame_id);
  // //
 
  std::set<KeyFrameId> culling_key_frame_ids;
  const auto &all_key_frame = map_manager_->KeyAllFrameDatas();

  auto const time_it = all_key_frame.lower_bound(
      frame_id.trajectory_id, all_key_frame.at(frame_id).data->time);
  if(time_it ==all_key_frame.end())return ;
  for (auto it = std::prev(time_it,options_.culling_win_size); it != time_it;
       ++it) {
    if (it == all_key_frame.end()) break;
    if (culling_sampler_->Pulse()) {
      culling_key_frame_ids.insert(it->id);
      break;
    }
  }

  for (const auto c_id : culling_key_frame_ids) {
    LOG(INFO)<<c_id<<frame_id;
    if (!map_manager_->KeyAllFrameDatas().Contains(c_id)) continue;
    data_culling_->CullingMapSimilarMap(c_id);
    data_culling_->KeyFrameCulling(c_id);
  }
}
//

LocalMapOptimizationData MappingBuilder::ParseLocalMapData(
    const KeyFrameId &frame_id) {
  CHECK(false);
  return {};
}
//
void MappingBuilder::LocalPorcess(const KeyFrameId &frame_id) {
  // 找到符合对应的数据

  if (local_mapping_optimization_sampler_->Pulse()) {
    LocalMapOptimizationData local_data;
    local_map_optimization_->Optimize(&local_data);
  }
};
//
void MappingBuilder::AddTrackingData(const int t, const TrackingData &data) {
   
  if (!key_frame_filter_->IsKeyFrame(data)) {
    return;
  }
  //
  if (options_.enable_local_track) {
    std::map<int,
             std::map<uint64_t, std::tuple<Eigen::Vector3d, mapping::Descriptor,
                                           FeatureId>>>
        front_map_points_data;
    auto key_frame_data =
        map_manager_->ExtractKeyFrameData(data, &front_map_points_data);
    local_map_track_->AddTracingData(key_frame_data, front_map_points_data);
    //
  }

  if(!options_.enable_local_opimization)return ;
  AddWorkItem([=]() {
    //前端的匹配的地图单独维护，AddTrackingData和后端地图保持一直，要不另外的线程有问题
    auto key_frame_id = map_manager_->AddTrackingData(t, data);
    //这里估计比较耗费时间
    LOG(INFO)<<"start ExtendKeyFrameData";
    map_manager_->ExtendKeyFrameData(key_frame_id);
    
    KeyFrameDataFuse(key_frame_id);
    if (local_mapping_process_num_ > options_.constant_local_process_num) {
      local_mapping_process_num_ = 0;
      LocalPorcess(key_frame_id);
    }
    return WorkItem::Result::Normal;
  });
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
  std::lock_guard<std::mutex> lock(mutex_);
  // 可能存在线程安全
  std::vector<Eigen::Vector3d> result;
  auto &all_map_points = map_manager_->GetAllMapPoints();
  for (const auto &mp : all_map_points) {
    // if (mp.data.data->Extend()) {
      result.push_back(mp.data.data->Pos());
    // }
  }
  return result;
}

void MappingBuilder::AddImuData(const sensor::ImuData &imu_data) {
  // AddWorkItem([=]() { return WorkItem::Result::Normal; });
}
void MappingBuilder::AddFixData(const sensor::FixedFramePoseData &fix_data) {
  // AddWorkItem([=]() { return WorkItem::Result::Normal; });
}

void MappingBuilder::AddOdometryData(const sensor::OdometryData &odo_data) {}
//
std::unique_ptr<transform::Rigid3d> MappingBuilder::TrackLocalMap(
    const TrackingData &frame_data) {
  return local_map_track_->Track(
      map_manager_->ExtractKeyFrameData(frame_data, nullptr));
}
transform::Rigid3d MappingBuilder::Relocaiton(const TrackingData &frame_data) {
  CHECK(false);
  return {};
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
        return;
      }
      work_item = work_queue_->front().task;
      work_queue_->pop_front();
      work_queue_size = work_queue_->size();
    }
    process_work_queue = work_item();
  }
  if (process_work_queue == WorkItem::Result::kRunLocalOptimization) {
    LocalOptimization();
  }
  DrainWorkQueue();
}

void MappingBuilder::AddWorkItem(
    const std::function<WorkItem::Result()> &work_item) {
  std::lock_guard<std::mutex> lock(work_queue_mutex_);
  const auto now = std::chrono::steady_clock::now();
  work_queue_->push_back({now, work_item});
}

//
//
MappingBuilder::MappingDataFuse::MappingDataFuse(MappingBuilder *map_builder)
    : map_builder_(map_builder) {}
//
//
void MappingBuilder::MappingDataFuse::FuseMapPoint(
    const KeyFrameId &key_frame_id,
    const std::map<MapPointId, std::map<KeyFrameId, FeatureId>> &matches) {
  std::lock_guard<std::mutex> lock(map_builder_->mutex_);
  map_builder_->map_manager_->FuseMapPoint(key_frame_id, matches);
}
//
//
void MappingBuilder::MappingDataFuse::CullKeyFrame(
    const std::set<KeyFrameId> &target) {
  std::lock_guard<std::mutex> lock(map_builder_->mutex_);
  for (const auto id : target) {
    // 如果是在回环里面的帧的话就不要删除了//记得后续功能增加的时候要添加
    map_builder_->map_manager_->TrimKeyFrame(id);
  }
}
//
//
const MapById<MapPointId, mapping::MapPointData>
MappingBuilder::MappingDataFuse::GetMapPoints(const KeyFrameId &id) {
  return map_builder_->map_manager_->GetKeyFrameMapPointsData(id).second;
}
//
//
//
const std::set<KeyFrameId> MappingBuilder::MappingDataFuse::GetMapObservations(
    const MapPointId &map_point_id) {
  //
  // const common::Time time =
  //     map_builder_->map_manager_->KeyAllFrameDatas().at(frame_id).data->time;

  return map_builder_->map_manager_->Covisibility()->GetMapObservations(
      map_point_id);
}
//
//
Eigen::Vector2d MappingBuilder::MappingDataFuse::PorjectPoint(
    const Eigen::Vector3d &point, int s) {
  Eigen::Vector2d b;
  map_builder_->options_.cameras.at(s)->spaceToPlane(point, b);
  return b;
}
//
const MapById<KeyFrameId, KeyFrameData> &
MappingBuilder::MappingDataFuse::GetAllKeyFramesData() {
  return map_builder_->map_manager_->KeyAllFrameDatas();
}

//
std::vector<std::pair<KeyFrameId, int>>
MappingBuilder::MappingDataFuse::GetKeyLevelConnectedKeyFrames(
    const KeyFrameId &frame_id, const std::vector<int> &levels) {
  //
  // std::vector<std::pair<KeyFrameId, int>> result;
  const auto connect_frames_ids =
      map_builder_->map_manager_->Covisibility()->GetOrderConnectedKeyFrames(
          frame_id, *levels.begin());
  // const common::Time time =
  //     map_builder_->map_manager_->KeyAllFrameDatas().at(frame_id).data->time;

  // for (const auto &con_frame_id : connect_frames_ids) {
  //   const common::Time conne_time =
  //       map_builder_->map_manager_->KeyAllFrameDatas()
  //           .at(con_frame_id.first)
  //           .data->time;
  //   if (conne_time > time) continue;
  //   result.push_back(con_frame_id);
  // }

  if (levels.size() == 1) {
    return connect_frames_ids;
  }

  std::vector<std::pair<KeyFrameId, int>> result;
  for (auto connect_frames_id : connect_frames_ids) {
    auto scond_connet = GetKeyLevelConnectedKeyFrames(
        connect_frames_id.first, {levels.begin() + 1, levels.end()});
    result.insert(result.begin(), scond_connet.begin(), scond_connet.end());
  }

  return result;
}

MappingBuilder::~MappingBuilder(){
    kill_thread_ = true;
    if (thread_.joinable()) {
      thread_.join();
    }
  }
  
}  // namespace mapping
}  // namespace jarvis
