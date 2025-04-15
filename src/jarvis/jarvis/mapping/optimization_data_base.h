#ifndef JARVIS_MAPPING_OPTIMIZATION_DATA_BASE_H
#define JARVIS_MAPPING_OPTIMIZATION_DATA_BASE_H
#include <deque>
#include <jarvis/mapping/local_map.h>
#include "jarvis/common/id.h"
#include "jarvis/transform/transform.h"
//
namespace jarvis {
namespace mapping {

//
struct LocalMapTime {
  LocalMapId id;
  common::Time time;
  std::shared_ptr<LocalMap> local_map;
};
//
//
struct KeyFramePoseTime {
  KeyFrameId id;
  common::Time time;
  transform::Rigid3d pose;
};
//
//
struct PoseConstraint {
  LocalMapId node_i;
  KeyFrameId node_j;
  transform::Rigid3d relative_pose;
  double yaw =0.0;
  bool internal =false;
};
struct NodePose {
  Eigen::Vector3d t{0, 0, 0};
  Eigen::Quaterniond q{1, 0, 0, 0};
  double ypr[3] = {0, 0, 0};
  transform::Rigid3d local_pose;
};
//
class BackOptimize {
 public:
  // virtual void AddImuData(const sensor::ImuData& imu_data) = 0;
  // virtual void AddLocalMapPose(const LocalMapTime& local_map_pose) = 0;
  // virtual void AddKfDataPose(const KfPoseTime& kf_pose) = 0;
  // virtual void Solve(const std::vector<PoseConstraint>& constraints) = 0;
  // virtual void UpdateData() = 0;
};

//
}  // namespace mapping
}  // namespace jarvis

#endif