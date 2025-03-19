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
#include "jarvis/mapping/key_frame_database.h"
#include "jarvis/mapping/local_map.h"
#include "jarvis/mapping/map_point_construct.h"
#include "jarvis/mapping/mapping_data.h"
#include "jarvis/transform/transform.h"
#include "jarvis/common/thread_pool.h"
#include "jarvis/mapping/loop_detect.h"
#include "jarvis/mapping/local_map_optimization.h"
namespace jarvis {
namespace mapping {
//

struct MapManagerOption {
  bool use_6_tof_op = false;
  double loop_detect__sampler = 0.05;
  double same_trajector_max_loop_detct_distance =15;
  int continuous_candidate_loop_frame =6;
  double max_loop_detct_time = 5;
  bool enable_loop_closure  =false;
  LoopDetectOption loop_detect_option;
  LocalMapOptimizationOption local_map_optimization_option;
};
//



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
  std::unique_ptr<common::FixedRatioSampler> loop_detect_sampler_;
  std::unique_ptr<common::FixedRatioSampler> loop_detect_kf_sampler_;
  //
  bool IsRunOptimization();
  bool Optimization(){
    CHECK(false);
    return true;
  }
  void UpdateOpimizeData(){}
  void UpdateNewFinishLocalMapLoop(const LocalMapId &local_map_id,
                                   std::shared_ptr<LocalMap> &,
                                   const std::vector<KeyFrameId> &candidata_kf);
  //
  //
  //
  void UpdateLoopConstraint(
      std::vector<std::unique_ptr<LoopDetctResult>> result);
  //
  std::mutex mutex_;
  std::unique_ptr<LocalMapOptimization> local_opimization_;

  std::unique_ptr<LoopDetect> loop_detect_;
  std::set<KeyFrameId> last_new_update_key_frame_ids_;  
  MapManagerOption options_;
  MapById<KeyFrameId,  KeyFrameData> key_frames_datas_;
  MapById<LocalMapId, LocalMapData> local_maps_;
  MapPointConstruct *map_point_construct_;
  common::ThreadPool* thread_pool_;
  LocalMapUpdateCallBack localmap_update_callback_;
  transform::Rigid3d local_to_globla_transform_;
  std::vector<std::unique_ptr<LoopDetctResult>> loop_constraints_;
  //
};
}  // namespace mapping
}  // namespace jarvis
#endif
