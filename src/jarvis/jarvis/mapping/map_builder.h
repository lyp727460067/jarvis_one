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
#include "jarvis/mapping/map_point_construct.h"
namespace jarvis {
namespace mapping {
//
struct MapBuilderOption {
  bool enable_local_track =false;
  bool enable_local_opimization =false;
  bool enable_loop_closure =false;
  MapManagerOption map_manager_option;
  LocalMapOption local_map_option;
  LocalMapTrackOption local_map_track_option;
  KeyFrameFilterOption key_frame_filter_option;

  LoopDetectOption loop_detect_option;
  MapPointConstructOption map_point_construct_option; 
  //
  //
  //
  std::string vocabulary_filebrif = "/home/lyp/project/vslam/jarvis/jarvis.dbow";
  std::vector<std::vector<int>> track_sequence;
  std::vector<transform::Rigid3d> extric_camera_to_imu;
  std::map<int, camera_models::CameraPtr> cameras;
  std::vector<Eigen::AlignedBox2i> image_boxs;
  
};

class MappingBuilder {
 public:
  MappingBuilder(const MapBuilderOption& option,dbow::Vocabulary *voc);
  ~MappingBuilder();
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
  //
  std::shared_ptr<LocalMap> GetLocalMap() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return local_map_front_;
  }

 private:
  //
  void TrimKeyFrameData();
  LocalMapOptimizationData ParseLocalMapData(const KeyFrameId& frame_id);
  void AddWorkItem(const std::function<WorkItem::Result()>& work_item);
  //
  std::unique_ptr<MapPointConstruct> map_point_construct_;
  std::unique_ptr<MapManager> map_manager_;
  std::unique_ptr<LocalMapTrack> local_map_track_;
  //
  std::shared_ptr<LocalMap> local_map_front_;
  std::unique_ptr<ActiveLocalMap> active_local_maps_;
  //
  std::unique_ptr<KeyFrameFilter> key_frame_filter_;
  mutable std::mutex mutex_;
  int local_mapping_process_num_ = 0;
  MapBuilderOption options_;
  bool kill_thread_=false;
  transform::Rigid3d local_to_globla_;
 
};
}  // namespace mapping
}  // namespace jarvis
#endif
