#ifndef __JARVIS_MAPPING_BUILDER_H__
#define __JARVIS_MAPPING_BUILDER_H__

#include <map>
#include <memory>
#include <set>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "jarvis/common/fixed_ratio_sampler.h"
#include "jarvis/key_frame_data.h"
#include "jarvis/mapping/key_frame_filter.h"
#include "jarvis/mapping/local_map_track.h"
#include "jarvis/mapping/map_manger.h"
#include "jarvis/sensor/fixed_frame_pose_data.h"
//
#include "jarvis/mapping/data_culling.h"
#include "jarvis/mapping/local_map_optimization.h"
#include "jarvis/sensor/odometry_data.h"
#include "jarvis/mapping/loop_detect.h"
namespace jarvis {
namespace mapping {
//
struct MapBuilderOption {
  bool enable = true;
  MapManagerOption map_manager_option;
  DataCullingOption data_culling_option;
  LocalMapTrackOption local_map_track_option;
  KeyFrameFilterOption key_frame_filter_option;
  LocalMapOptimizationOption local_map_optimization_option;
  LoopDetectOption loop_detect_option;
  
  //
  //
  //
  std::string vocabulary_filebrif = "/home/lyp/project/vslam/jarvis/jarvis.dbow";
  int constant_local_process_num = 20;
  float culling_sampler_ration = 0.1;
  float local_optimization_ration = 0.1;
  std::vector<std::vector<int>> track_sequence;
  std::vector<transform::Rigid3d> extric_camera_to_imu;
  std::map<int, camera_models::CameraPtr> cameras;
  int culling_win_size  =20;
};

//

struct WorkItem {
  enum class Result { Normal, kRunLocalOptimization };
  std::chrono::steady_clock::time_point time;
  std::function<Result()> task;
};

class MappingBuilder {
 public:
  MappingBuilder(const MapBuilderOption& option);
  void AddTrackingData(const int t, const TrackingData& track_data);
  //
  void AddFixData(const sensor::FixedFramePoseData& fix_data);
  void AddImuData(const sensor::ImuData& imu_data);
  void AddOdometryData(const sensor::OdometryData& odo_data);
  //
  std::unique_ptr<transform::Rigid3d> TrackLocalMap(const TrackingData& frame_data);
  transform::Rigid3d Relocaiton(const TrackingData& frame_data);
  //
  std::vector<Eigen::Vector3d> GetAllMapPoints();
  std::map<KeyFrameId, transform::TimestampedTransform> GetKeyFrameGlobalPose(){
    CHECK(false);
    return {};
  }
  transform::Rigid3d GetLocalToGlobalTransform(){
    CHECK(false);
    return {};
  }
  std::map<KeyFrameId, transform::TimestampedTransform> GetAllKeyFramePose();

 private:
  //
  LocalMapOptimizationData ParseLocalMapData(const KeyFrameId& frame_id);
  void LocalPorcess(const KeyFrameId& frame_id);
  void LocalOptimization() {}
  void DrainWorkQueue();
  void KeyFrameDataFuse(const KeyFrameId& frame_id);
  void AddWorkItem(const std::function<WorkItem::Result()>& work_item);
  std::unique_ptr<common::FixedRatioSampler> culling_sampler_;
  //
  std::unique_ptr<common::FixedRatioSampler>
      local_mapping_optimization_sampler_;
  std::unique_ptr<LocalMapOptimization> local_map_optimization_;
  std::unique_ptr<MapManager> map_manager_;
  std::unique_ptr<LocalMapTrack> local_map_track_;
  std::unique_ptr<DataCulling> data_culling_;
  std::unique_ptr<DataFuse> data_fuse_;
  std::unique_ptr<KeyFrameFilter> key_frame_filter_;
  std::thread thread_;
  std::mutex work_queue_mutex_;
  using WorkQueue = std::deque<WorkItem>;
  std::unique_ptr<WorkQueue> work_queue_;
  std::mutex mutex_;
  int local_mapping_process_num_ = 0;
  MapBuilderOption options_;
  bool kill_thread_=false;
  transform::Rigid3d local_to_globla_;
  class MappingDataFuse : public DataFuse {
   public:
    MappingDataFuse(MappingBuilder* map_builder);

    void FuseMapPoint(
        const KeyFrameId& key_frame_id,
        const std::map<MapPointId, std::map<KeyFrameId, FeatureId>>& matches)
        override;

    void CullKeyFrame(const std::set<KeyFrameId>& target) override;
    const MapById<MapPointId, mapping::MapPointData> GetMapPoints(
        const KeyFrameId& id) override;
    //
    const std::set<KeyFrameId> GetMapObservations(
        const MapPointId& map_point_id) override;
    Eigen::Vector2d PorjectPoint(const Eigen::Vector3d& point, int s) override;
    //
    const MapById<KeyFrameId, KeyFrameData>& GetAllKeyFramesData() override;
    std::vector<std::pair<KeyFrameId, int>> GetKeyLevelConnectedKeyFrames(
        const KeyFrameId& frame_id, const std::vector<int>& levels) override;
    MappingBuilder* map_builder_;
  };
  friend MappingDataFuse;
};
}  // namespace mapping
}  // namespace jarvis
#endif
