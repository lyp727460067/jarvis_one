#ifndef __JARVIS_MAPPING_MAPMANAGER_H__
#define __JARVIS_MAPPING_MAPMANAGER_H__


#include <map>
#include <memory>
#include <set>

#include "Eigen/Core"
#include "Eigen/Geometry"
//
#include "jarvis/camera_models/camera_models/camera.h"
//
#include "jarvis/common/id.h"
#include "jarvis/key_frame_data.h"
#include "jarvis/mapping/covisibility.h"
#include "jarvis/mapping/key_frame_database.h"
#include "jarvis/mapping/key_point_exract.h"
#include "jarvis/mapping/local_map.h"
#include "jarvis/mapping/map_point_construct.h"
#include "jarvis/mapping/mapping_data.h"
#include "jarvis/mapping/match/des_matcher.h"
#include "jarvis/transform/transform.h"
namespace jarvis {
namespace mapping {
//

struct MapManagerOption {
};
//

struct LocalMapData {
  std::shared_ptr<LocalMap> local_map;
  transform::Rigid3d globla_pose;
};
struct WorkItem {
  enum class Result { Normal, kRunLocalOptimization };
  std::chrono::steady_clock::time_point time;
  std::function<Result()> task;
};
class MapManager {
 public:
  MapManager(const MapManagerOption &option, MapPointConstruct *,
             bool enable_local_opimization);
  //
  //
  ~MapManager();
  KeyFrameId AddKeyFrameData(int trajector, const KeyFrameData &data) ;
  LocalMapId AddLocalMap(int trajector, std::shared_ptr<LocalMap> data);
  void TrimKeyFrameData(const KeyFrameId &id) {
    std::lock_guard<std::mutex> lock(mutex_);
    key_frames_datas_.Trim(id);
  }
  //
  KeyFrameData &GetKeyFrameId(const KeyFrameId &id) {
    return key_frames_datas_.at(id);
  }
  void TrimOptimizedLocalMap();
  
  std::vector<Eigen::Vector3d> GetAllMapPoints() ;
  //
  std::map<KeyFrameId, transform::TimestampedTransform> GetAllKeyFramePose();

 private:
  void DrainWorkQueue();
  void AddWorkItem(const std::function<WorkItem::Result()> &work_item);

  std::thread thread_;
  std::mutex work_queue_mutex_;
  using WorkQueue = std::deque<WorkItem>;
  std::unique_ptr<WorkQueue> work_queue_;
  std::mutex mutex_;
  std::set<KeyFrameId> last_new_update_key_frame_ids_;  
  MapManagerOption options_;
  MapById<KeyFrameId,  KeyFrameData> key_frames_datas_;
  MapById<LocalMapId, LocalMapData> local_maps_;
  MapPointConstruct *map_point_construct_;
  bool kill_thread_  =false;

  //
};
}  // namespace mapping
}  // namespace jarvis
#endif
