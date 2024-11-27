#ifndef __JARVIS_MAPPING_MAP_OPTIMIZATION__H__
#define __JARVIS_MAPPING_MAP_OPTIMIZATION__H__

#include <map>
#include <memory>
#include <set>
#include "jarvis/common/id.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "jarvis/common/time.h"
#include "jarvis/sensor/fixed_frame_pose_data.h"
#include "jarvis/sensor/imu_data.h"
#include "jarvis/transform/transform.h"
#include "opencv2/opencv.hpp"
//

#include "jarvis/mapping/mapping_data.h"
namespace jarvis {
namespace mapping {
//
struct LocalMapOptimizationOption {
  std::vector<transform::Rigid3d> extric_camera_to_imu;
  bool optimize_intric = false;
  bool use_rtk = false;
  bool only_pose_graph = false;
};

//
struct LocalMapOptimizationData {
  struct MapPointData {
    Eigen::Vector3d pos;
    std::map<KeyFrameId, FeatureId> con_frame_datas;
    std::map<FeatureId, FeatureData> feature_datas;
  };
  std::map<MapPointId, MapPointData> con_map_points;
  struct FrameData {
    common::Time time;
    transform::Rigid3d pose;
  };
  std::map<KeyFrameId, FrameData> frame_datas;

};

class LocalMapOptimization {
 public:
  LocalMapOptimization(const LocalMapOptimizationOption &option);
  void AddFixData(const sensor::FixedFramePoseData& fix_data);
  void AddImuData(sensor::ImuData& imu_data);
  //
  void Optimize(LocalMapOptimizationData* data);

};

}  // namespace mapping
}  // namespace jarvis
#endif
