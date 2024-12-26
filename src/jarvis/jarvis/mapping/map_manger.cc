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
        usleep(1000);
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
    auto local_map = local_maps_.at(local_map_id).local_map;
    LOG(INFO)<<local_map_id ;
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

void MapManager::AddWorkItem(
    const std::function<WorkItem::Result()> &work_item) {
  std::lock_guard<std::mutex> lock(work_queue_mutex_);
  const auto now = std::chrono::steady_clock::now();
  work_queue_->push_back({now, work_item});
}

}  // namespace mapping
}  // namespace jarvis