#include "jarvis/mapping/map_manger.h"

#include <mutex>
// #include <opencv2/core/eigen.hpp>
#include <set>
#include <vector>

#include "Eigen/Core"
#include "glog/logging.h"
#include "jarvis/common/math.h"
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
                       bool enable_local_opimization)
    : options_(option), map_point_construct_(map_point_construct) {
 if (enable_local_opimization) {
  work_queue_ = std::make_unique<WorkQueue>();
    thread_ = std::thread([this]() {
      while (!kill_thread_) {
        DrainWorkQueue();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
  }
}
//
//
LocalMapId MapManager::AddLocalMap(int trajector,
                                   std::shared_ptr<LocalMap> local_map) {
  //
  auto local_map_id = local_maps_.Append(trajector, LocalMapData{local_map});
  //
  AddWorkItem([this,local_map_id]() {
    std::set<KeyFrameId> new_update_ids;
    std::shared_ptr<LocalMap> local_map = nullptr;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      local_map = local_maps_.at(local_map_id).local_map;
    }
    LocalMap new_local_map(*local_map);

    for (const auto &data : local_map->AllKeyFrameDatas()) {
      //
      if (last_new_update_key_frame_ids_.count(data.id)) {
        new_local_map.AddKeyFrameData(data.id, data.data);
      } else {
        map_point_construct_->ConstructExtend(new_local_map,
                                              &key_frames_datas_.at(data.id));
        new_local_map.AddKeyFrameData(data.id, data.data);
      }
      new_update_ids.insert(data.id);
    }

    new_local_map.UpdadataExtendFinishData();
    *local_map = std::move(new_local_map);
    last_new_update_key_frame_ids_ = std::move(new_update_ids);
    return WorkItem::Result::Normal;
  });
  // needed_opt_local_maps_.insert(local_map_id);如果loopdetect开启的话就放在线程池慢慢优化
  return local_map_id;
}
//

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



//
void MapManager::DrainWorkQueue() {
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
  }
  DrainWorkQueue();
}

KeyFrameId MapManager::AddKeyFrameData(int trajector,
                                       const KeyFrameData &data) {
  std::lock_guard<std::mutex> lock(mutex_);
  return key_frames_datas_.Append(trajector, data);
}
//
std::map<KeyFrameId, transform::TimestampedTransform>
MapManager::GetAllKeyFramePose() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::map<KeyFrameId, transform::TimestampedTransform> result;
  for (const auto &key_frame_data : key_frames_datas_) {
    result.emplace(
        key_frame_data.id,
        transform::TimestampedTransform{key_frame_data.data.data->time,
                                        key_frame_data.data.data->global_pos});
  }
  LOG(INFO)<< result.size();
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
void MapManager::AddWorkItem(
    const std::function<WorkItem::Result()> &work_item) {
  std::lock_guard<std::mutex> lock(work_queue_mutex_);
  const auto now = std::chrono::steady_clock::now();
  work_queue_->push_back({now, work_item});
}
MapManager::~MapManager(){
  kill_thread_=true;
  if (thread_.joinable()) {
    thread_.join();
  }
  if(work_queue_==nullptr)return;

  size_t work_queue_size = 0;
  {
    std::function<WorkItem::Result()> work_item;
    work_queue_size = work_queue_->size();
  }

  while (work_queue_size) {
    std::function<WorkItem::Result()> work_item;
    {
      std::lock_guard<std::mutex> locker(work_queue_mutex_);
      work_item = work_queue_->front().task;
      work_queue_->pop_front();
      work_queue_size = work_queue_->size();
    }
    LOG(INFO) << "wait work_queue_size " << work_queue_size;
    work_item();
  }

  //
}

}  // namespace mapping
}  // namespace jarvis