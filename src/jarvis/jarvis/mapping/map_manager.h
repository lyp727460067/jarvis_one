#ifndef JARVIS_MAPPING_MAP_MANAGER_H_
#define JARVIS_MAPPING_MAP_MANAGER_H_

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <vector>

#include "Eigen/Core"
#include "jarvis/common/fixed_ratio_sampler.h"
#include "jarvis/common/task.h"
#include "jarvis/common/thread_pool.h"
#include "jarvis/mapping/covisibility.h"
#include "jarvis/mapping/key_frame_database.h"
#include "jarvis/mapping/local_map_optimization.h"
#include "jarvis/mapping/loop_detect.h"
#include "jarvis/mapping/map_point_construct.h"
#include "jarvis/mapping/mapping_data.h"
#include "jarvis/mapping/match/des_matcher.h"
#include "jarvis/mapping/match/pic_writer.h"
#include "jarvis/mapping/pose_graph_op.h"
//
#include "jarvis/mapping/work_item_queue.h"
#include "jarvis/transform/rigid_transform.h"
//
//
#include "jarvis/mapping/work_item_queue.h"
namespace jarvis {
namespace mapping {

struct MapManagerOption {
  PoseGraphOptimizeOption pose_graph_option;
  LoopDetectOption loop_detect_option;
  LocalMapOptimizationOption local_map_optimization_option;
  //
  bool local_map_op_use_6dof =false;
  bool pose_graph_op_use_6dof =false;
  double same_trajectory_max_loop_detect_distance = 50.0;
  double max_loop_detct_distance = 50.0;
  int continuous_candidate_loop_frame = 10;
  int pose_graph_optimize_min_kf_min_num = 100;
  double constraint_compute_sampler = 0.1;
  double global_constraint_search_after_n_seconds = 10;
};
//
class MapManager {
 public:
 
  using LocalMapUpdateCallBack =
      std::function<void(const std::shared_ptr<LocalMap>)>;

  MapManager(const MapManagerOption& option,
             MapPointConstruct* map_point_construct,
             std::map<int, camera_models::CameraPtr> camera,
             common::ThreadPool* thread_pool = nullptr,
             LocalMapUpdateCallBack call_back = nullptr);

  void AddLocalMap(int trajector, std::shared_ptr<LocalMap> local_map);
  void UpdateLocalOpLocalMap(
      std::map<LocalMapId, std::shared_ptr<LocalMap>>* op_local_maps);
  void TrimOptimizedLocalMap();
  KeyFrameId AddKeyFrameData(int trajector, const KeyFrameData& data);
  void UpdateNewFinishLocalMapLoop(const LocalMapId& local_map_id,
                                   std::shared_ptr<LocalMap>&,
                                   const std::vector<KeyFrameId>& candidata_kf);

  std::map<KeyFrameId, transform::TimestampedTransform> GetAllKeyFramePose();
  std::vector<Eigen::Vector3d> GetAllMapPoints();

  void TrimKeyFrameData(const KeyFrameId& id);
  ~MapManager();
  transform::Rigid3d GetLocalToGlobalTransform(){
    return local_to_global_transform_ ;
  }
  void ExtendedKeyFrameData(const LocalMap& local_map, const KeyFrameId& id,
                            KeyFrameData::Data* data);

 private:
  void Optimization(std::vector<std::unique_ptr<LoopDetctResult>>&&);
  void UpdateOptimizeData();
  void UpdataLocalMapConstraint(const LocalMapId& id,
                                std::shared_ptr<LocalMap> local_map);
  std::shared_ptr<LocalMap> ReconstructLocalMap(
      std::shared_ptr<LocalMap> local_map);
  void UpdataActiveTrackLocalMap(std::shared_ptr<LocalMap> local_map);
  //

  void ComputeLoopConstaint(const LocalMapId& local_map_id,
                            const KeyFrameId& key_frame_id,
                            const double min_score);
  //
  void ComputeConstaints(const KeyFrameId& id, const double min_score);
 
  //
  //
  double ComputeCovisibleMinScore(const LocalMap& local_map,
                                  const KeyFrameId& id);
  void ReconstructLocalMapOptimization(
      std::map<LocalMapId, std::shared_ptr<LocalMap>>&);
  //
  void UpdateKeyframeDataUsingPrunedLocalMap(
      std::shared_ptr<LocalMap> new_local_map);
  //
  // 局部地图看看有没有重复度高的
  void PruneRedundantLocalMap(const LocalMapId& new_local_map_id);
  //
  void RunPoseGraphOptimization(std::vector<std::shared_ptr<LoopDetctResult>>&);
  std::optional<LocalMapId> new_local_map_id_;
  MapManagerOption options_;
  MapPointConstruct* map_point_construct_;
  common::ThreadPool* thread_pool_;
  LocalMapUpdateCallBack localmap_update_callback_;
  std::unique_ptr<LocalMapOptimization> local_optimization_;
  std::unique_ptr<PoseGraphOptimize> pose_graph_optimizer_;
  std::unique_ptr<LoopDetect> loop_detect_;
  //
  std::unique_ptr<common::FixedRatioSampler> loop_detect_kf_sampler_;
  //
  MapById<LocalMapId, LocalMapData> local_maps_;
  MapById<KeyFrameId, KeyFrameData> key_frames_datas_;
  std::vector<KeyFrameId> last_new_update_key_frame_ids_;
  //
  int num_kf_num_since_last_loop_closure_ = 0;
  std::vector<std::shared_ptr<LoopDetctResult>> op_constraints_;
  //
  std::vector<PoseConstraint> pose_constraints_;
  std::set<KeyFrameId> extend_key_frames_ids_;
  std::unique_ptr<WorkItemQueue> work_item_queue_;
  std::mutex mutex_;
  std::unique_ptr<common::Task> when_op_done_task_;
  transform::Rigid3d local_to_global_transform_;
  //
  std::set<KeyFrameId> previous_local_map_trimed_key_frames_id_;
  std::map<int, std::map<int, common::Time>> last_trajectory_connect_time_;
  //
  bool enable_loop_closure_ = true;
};

}  // namespace mapping
}  // namespace jarvis

#endif  // JARVIS_MAPPING_MAP_MANAGER_H_