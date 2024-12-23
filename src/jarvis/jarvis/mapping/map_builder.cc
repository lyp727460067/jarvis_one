#include "jarvis/mapping/map_builder.h"

#include "jarvis/common/id.h"

#include <map>
#include <memory>
#include <set>

#include "Eigen/Core"
#include "Eigen/Geometry"
namespace jarvis {
namespace mapping {
//
//
void MappingBuilder::KeyFrameDataFuse(const KeyFrameId &frame_id) {
  //
  auto conn_frames =
      map_manager_->GetCovisibility()->GetConnectedKeyFrames(frame_id);
  //
  std::stringstream info;
  for (auto const &frame : conn_frames) {
    info << frame << " ";
  }
  std::set<KeyFrameId> culling_key_frame_ids;
  const auto &all_key_frame = map_manager_->KeyAllFrameDatas();
  const int kCullingKeyFrameLenth = 30;
  if (all_key_frame.SizeOfTrajectoryOrZero(frame_id.trajectory_id) <
      kCullingKeyFrameLenth)
    return;

  for (auto it =
           std::prev(all_key_frame.EndOfTrajectory(frame_id.trajectory_id),
                     kCullingKeyFrameLenth);
       it != std::prev(all_key_frame.EndOfTrajectory(frame_id.trajectory_id));
       ++it) {
    if (culling_sampler_->Pulse()) {
      culling_key_frame_ids.insert(it->id);
      break;
    }
  }

  for (const auto c_id : culling_key_frame_ids) {
    if (!map_manager_->KeyAllFrameDatas().Contains(c_id)) continue;
    data_culling_->CullingMapSimilarMap(c_id);
    data_culling_->KeyFrameCulling(c_id);
  }
}
//

LocalMapOptimizationData MappingBuilder::ParseLocalMapData(
    const KeyFrameId &frame_id) {}
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
  std::lock_guard<std::mutex> lock(mutex_);
  auto key_frame_id = map_manager_->AddTrackingData(t, data);
  //
  AddWorkItem([=]() {
    //这里估计比较耗费时间
    map_manager_->GenerateForExtendKeyPoint(key_frame_id);
    
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

void MappingBuilder::AddImuData(sensor::ImuData &imu_data) {
  AddWorkItem([=]() {
    return WorkItem::Result::Normal;
  });
}
void MappingBuilder::AddFixData(const sensor::FixedFramePoseData &fix_data) {
  AddWorkItem([=]() {
        return WorkItem::Result::Normal;
  });
}

//
transform::Rigid3d MappingBuilder::TrackLocalMap(
    const TrackingData &frame_data) {}
transform::Rigid3d MappingBuilder::Relocaiton(const TrackingData &frame_data) {}

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
  if (work_queue_ == nullptr) {
    work_queue_ = std::make_unique<WorkQueue>();
    if (thread_.joinable()) {
      thread_.join();
    }
    thread_ = std::thread([this]() { DrainWorkQueue(); });
  }
  const auto now = std::chrono::steady_clock::now();
  work_queue_->push_back({now, work_item});
}

MappingBuilder::MappingDataFuse::MappingDataFuse(MappingBuilder *map_builder)
    : map_builder_(map_builder) {}
//
//
void MappingBuilder::MappingDataFuse::FuseMapPoint(
    const KeyFrameId &key_frame_id,
    const std::map<MapPointId, std::map<KeyFrameId, FeatureId>> &matches) {
  // for (auto const &matche : matches) {
  //   LOG(INFO) << "fuse map points " << matche.first << "in " << key_frame_id;
  // }

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
const std::set<KeyFrameId> &MappingBuilder::MappingDataFuse::GetMapObservations(
    const MapPointId &map_point_id) {
  return {};
}
//
//
Eigen::Vector2d MappingBuilder::MappingDataFuse::PorjectPoint(
    const Eigen::Vector3d &point,int s) {
      return {};
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
  const auto connect_frames_ids =
      map_builder_->map_manager_->GetCovisibility()->GetOrderConnectedKeyFrames(
          frame_id, *levels.begin());

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
}  // namespace mapping
}  // namespace jarvis
