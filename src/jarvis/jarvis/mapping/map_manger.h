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
#include "jarvis/common/thread_pool.h"
namespace jarvis {
namespace mapping {
//

struct MapManagerOption {
    LocalMapOptimizationOption local_map_optimization_option;
};
//

struct LocalMapData {
  std::shared_ptr<LocalMap> local_map;
  transform::Rigid3d globla_pose;
};


using LocalMapUpdateCallBack = std::function<void(
    std::map<LocalMapId, std::shared_ptr<LocalMap>> *op_local_maps)>;

class MapManager {
 public:
  MapManager(const MapManagerOption &option, MapPointConstruct *,
             common::ThreadPool *thread_pool,
             LocalMapUpdateCallBack call_back = nullptr);
  //
  transform::Rigid3d GetLocalToGlobla() { return local_to_globla_transform_; }
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
  void UpdateLocalOpLocalMap(
      std::map<LocalMapId, std::shared_ptr<LocalMap>> *op_local_maps);

 private:
  std::mutex mutex_;
  std::unique_ptr<LocalMapOptimization> local_opimization_;

  std::set<KeyFrameId> last_new_update_key_frame_ids_;  
  MapManagerOption options_;
  MapById<KeyFrameId,  KeyFrameData> key_frames_datas_;
  MapById<LocalMapId, LocalMapData> local_maps_;
  MapPointConstruct *map_point_construct_;
  LocalMapUpdateCallBack localmap_update_callback_;
  transform::Rigid3d local_to_globla_transform_;
  common::ThreadPool* thread_pool_;
  //
};
}  // namespace mapping
}  // namespace jarvis
#endif
