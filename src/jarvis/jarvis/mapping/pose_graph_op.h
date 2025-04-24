#ifndef JARVIS_MAPPING_POSE_GRAPH_OP_H
#define JARVIS_MAPPING_POSE_GRAPH_OP_H
#include <deque>

#include "jarvis/sensor/fixed_frame_pose_data.h"
#include "jarvis/sensor/odometry_data.h"
//
#include "jarvis/common/id.h"
#include "jarvis/mapping/optimization_data_base.h"
#include "jarvis/transform/transform.h"
//
namespace jarvis {
namespace mapping {

struct PoseGraphOptimizeOption {
  std::vector<std::vector<int>> track_sequence;
  std::vector<transform::Rigid3d> extric_camera_to_imu;
  transform::Rigid3d extric_odo_to_imu;
  transform::Rigid3d extric_rtk_to_imu;
  bool fix_extric = true;
  int max_num_iterations = 20;
  int ceres_num_threads = 4;
  double relative_t_weitht = 1e3;
  double relative_r_weitht = 1e3;
  double huber_scale = 2.0;
  double constraint_loop_closer_t_weigth = 1e3;
  double constraint_loop_closer_r_weigth = 1e3;
  double constraint_t_weigth = 1e3;
  double constraint_r_weigth = 1e3;
};
//

struct LocalMapPoseTime {
  common::Time time;
  transform::Rigid3d pose;
};

class PoseGraphOptimize {
 public:
  PoseGraphOptimize(const PoseGraphOptimizeOption option) : options_(option) {
    CHECK(options_.ceres_num_threads != 0);
    CHECK(options_.max_num_iterations != 0);

  }
  //
  void AddFixData(const sensor::FixedFramePoseData& fix_data);
  void AddImuData(const sensor::ImuData& imu_data);
  void AddOdometryData(const sensor::OdometryData& odo_data);
  void AddLocalMapPose(const LocalMapId& local_map_id,
                       const LocalMapPoseTime& kf_pose);
  void AddKeyFramePose(const KeyFrameId& id, const KeyFramePoseTime& kf_pose);
  //
  //
  std::vector<NodePose>& GetExtricCameratoImu() {
    return extric_camera_to_imu_;
  }

  const std::map<KeyFrameId, NodePose>& GetPoseGraphNodePose() {
    return ceres_poses_;
  }
  const std::map<LocalMapId, NodePose>& GetPoseGraphLocalMapPose() {
    return ceres_local_map_poses_;
  }
  //
  void Solve(const std::vector<PoseConstraint>& constraints);
  //
  void TrimLocalMapPose(LocalMapId& id);
  void TrimKeyFramePose(KeyFrameId& id);
  
 private:
  virtual void DataToState(ceres::Problem* problem);
  //
  virtual void AddRelativeFactor(ceres::Problem* problem);
  virtual void AddImuFactor(ceres::Problem* problem);
  virtual void AddConstraintFactor(
      ceres::Problem* problem, const std::vector<PoseConstraint>& constraints);
  //
  PoseGraphOptimizeOption options_;
  //
  std::set<int> froze_trajector_;
  // op data
  std::vector<NodePose> extric_camera_to_imu_;
  NodePose extric_odo_to_imu_;
  NodePose extric_fix_frame_to_imu_;
  std::map<LocalMapId, NodePose> ceres_local_map_poses_;
  std::map<KeyFrameId, NodePose> ceres_poses_;

  //data
  std::map< LocalMapId, LocalMapPoseTime> local_map_poses_;
  std::map< KeyFrameId, KeyFramePoseTime> node_poses_;
  std::queue<sensor::ImuData> imu_datas_;
  std::deque<sensor::OdometryData> odometry_data_;
  std::deque<sensor::ImuData> imu_data_;
  std::mutex mutex_;
};

class FourPoseGraphOptimize : public PoseGraphOptimize {};
//
}  // namespace mapping
}  // namespace jarvis

#endif