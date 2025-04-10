#ifndef JARVIS_MAPPING_MAP_MANAGER_H_
#define JARVIS_MAPPING_MAP_MANAGER_H_

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <vector>

#include "Eigen/Core"
#include "jarvis/common/task.h"
#include "jarvis/mapping/covisibility.h"
#include "jarvis/mapping/key_frame_database.h"
#include "jarvis/mapping/local_map_optimization.h"
#include "jarvis/mapping/map_point_construct.h"
#include "jarvis/mapping/mapping_data.h"
#include "jarvis/mapping/match/des_matcher.h"
#include "jarvis/mapping/match/pic_writer.h"
#include <optional>
#include "jarvis/common/thread_pool.h"
#include "jarvis/mapping/loop_detect.h"
#include "jarvis/mapping/pose_graph_op.h"
//
#include "jarvis/transform/rigid_transform.h"
#include "jarvis/mapping/work_item_queue.h"
//
//
namespace jarvis {
namespace mapping {

class MapManager {
 public:
  struct MapManagerOption {
    bool enable_loop_closure = true;
    bool use_6_tof_op = false;
    double same_trajectory_max_loop_detect_distance = 50.0;
    double max_loop_detct_distance = 50.0;
    int continuous_candidate_loop_frame = 10;
    LocalMapOptimizationOption local_map_optimization_option;
    bool need_update_track_local_map = false;
    double global_constraint_search_after_n_seconds = 10;
    int 
  };

  using LocalMapUpdateCallBack = std::function<void(const LocalMapId&)>;

  MapManager(const MapManagerOption& option,
             MapPointConstruct* map_point_construct,
             common::ThreadPool* thread_pool,
             LocalMapUpdateCallBack call_back);

  LocalMapId AddLocalMap(int trajector, std::shared_ptr<LocalMap> local_map);
  void UpdateLocalOpLocalMap(
      std::map<LocalMapId, std::shared_ptr<LocalMap>>* op_local_maps);
  void TrimOptimizedLocalMap();
  KeyFrameId AddKeyFrameData(int trajector, const KeyFrameData& data);
  bool IsRunOptimization();
  void UpdateNewFinishLocalMapLoop(
      const LocalMapId& local_map_id, std::shared_ptr<LocalMap>&,
      const std::vector<KeyFrameId>& candidata_kf);
  void UpdateLoopConstraint(
      std::vector<std::unique_ptr<LoopDetctResult>> result);
  std::map<KeyFrameId, transform::TimestampedTransform> GetAllKeyFramePose();
  std::vector<Eigen::Vector3d> GetAllMapPoints();

  ~MapManager();
   
 private:
  void Optimization();
  void UpdateOptimizeData();
  void TrimKeyFrameData(const KeyFrameId& id);
  std::shared_ptr<LocalMap> ReconstructLocalMap(
      std::shared_ptr<LocalMap> local_map);
   void UpdataActiveTrackLocalMap(std::shared_ptr<LocalMap>local_map);
   //

   void ComputeLoopConstaint(const LocalMapId& local_map_id,
                             const KeyFrameId& key_frame_id);
   //
   void ComputeConstaints(const KeyFrameId& id);
   void ExtendedKeyFrameData(const LocalMap& local_map, const KeyFrameId& id,
                             KeyFrameData* data);
   //
   //
   void ReconstructLocalMapOptimization(
       const std::map<LocalMapId, std::shared_ptr<LocalMap>>&);
   //
   void RunPoseGraphOptimization(
       std::vector<std::shared_ptr<LoopDetctResult>>&);
   std::optional<LocalMapId> new_local_map_id_;
   MapManagerOption options_;
   MapPointConstruct* map_point_construct_;
   common::ThreadPool* thread_pool_;
   std::unique_ptr<PoseGraphOptimize> pose_graph_optimize_;
   LocalMapUpdateCallBack localmap_update_callback_;
   std::unique_ptr<LocalMapOptimization> local_optimization_;
   std::unique_ptr<GraphLocalMapOptimization6TOF> local_optimization_6_tof_;
   std::unique_ptr<LoopDetect> loop_detect_;
   std::unique_ptr<common::FixedRatioSampler> loop_detect_sampler_;
   std::unique_ptr<common::FixedRatioSampler> loop_detect_kf_sampler_;
   MapById<LocalMapId, LocalMapData> local_maps_;
   MapById<KeyFrameId, KeyFrameData> key_frames_datas_;
   std::vector<KeyFrameId> last_new_update_key_frame_ids_;
   //
   std::vector<std::shared_ptr<LoopDetctResult>> op_constraints_;
   //
   std::set<KeyFrameId> extend_key_frames_ids_;
   std::unique_ptr<WorkItemQueue> work_item_queue_;
   std::mutex mutex_;
   std::unique_ptr<common::Task> when_op_done_task_;
   transform::Rigid3d local_to_global_transform_;
   //
   std::map<uint64_t, std::map<uint64_t, common::Time>>
       last_trajectory_connect_time_;
    //
};

}  // namespace mapping
}  // namespace jarvis

#endif  // JARVIS_MAPPING_MAP_MANAGER_H_