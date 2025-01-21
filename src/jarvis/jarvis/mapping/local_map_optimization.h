#ifndef __JARVIS_MAPPING_LOCAL_MAP_OPTIMIZATION__H__
#define __JARVIS_MAPPING_LOCAL_MAP_OPTIMIZATION__H__

#include <map>
#include <memory>
#include <set>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "jarvis/common/id.h"
#include "jarvis/common/time.h"
#include "jarvis/sensor/fixed_frame_pose_data.h"
#include "jarvis/sensor/imu_data.h"
#include "jarvis/transform/transform.h"
#include "opencv2/opencv.hpp"
//
#include "jarvis/estimator/factor/integration_base.h"
#include "jarvis/estimator/factor/imu_factor.h"
//
#include "jarvis/mapping/mapping_data.h"
namespace jarvis {
namespace mapping {
//
class LocalMap;
struct LocalMapOptimizationOption {
  std::vector<transform::Rigid3d> extric_camera_to_imu;
  std::vector<std::vector<int>> track_sequence;
  bool essential_graph =  false;
  int max_num_iterations =10;
  double repeat_mp_weight  =300;
  double re_preject_weight  =300;
  double relative_weight = 3000;
  double relative_local_map_translation_weight = 3000;
  double huber_loss =0.1;
  bool optimize_intric = false;
  bool optimize_extric = false;
  bool optimize_imu = false;
  bool use_rtk = false;
  bool only_pose_graph = false;
  bool fix_extric = true;
  int ceres_num_threads =6;
  double optimazation_outliers_rejection_th = 5.0 / 377;
  struct EssentialGraphOption {
    int max_con_kf_num = 10;
    int max_adjacent_kf_num = 10;
    std::vector<int> convisi_level_search_num{5 ,3};
  } sssential_graph_option;
};
//
struct NodePose {
  Eigen::Vector3d t{0, 0, 0};
  Eigen::Quaterniond q{1, 0, 0, 0};
  double ypr[3] = {0, 0, 0};
  transform::Rigid3d local_pose;
};
//
class LocalMapOptimization {
 public:
  LocalMapOptimization(const LocalMapOptimizationOption& option);
  virtual ~LocalMapOptimization() {}
  // for rtk
  void AddFixData(const sensor::FixedFramePoseData& fix_data) {}
  // 原始IMU数据，需要积分
  void AddImuData(sensor::ImuData& imu_data) {}
  void Optimize(std::map<LocalMapId, std::shared_ptr<LocalMap>>* local_maps);
 protected:
  virtual void StrategyOptimize(
      std::map<LocalMapId, std::shared_ptr<LocalMap>>* local_maps);
  int RemoveOutliersRejection(const LocalMapId&map_id,std::shared_ptr<LocalMap> local_maps);
  void UpdateLocalMapData(std::map<LocalMapId, std::shared_ptr<LocalMap>>*);
  std::queue<sensor::ImuData> imu_datas_;
  std::queue<sensor::FixedFramePoseData> fix_datas_;
  LocalMapOptimizationOption  options_;
  std::vector<NodePose> extric_camera_to_imu_;
  std::map<LocalMapId, NodePose> ceres_local_map_poses_;
  std::map<LocalMapId, std::map<MapPointId, Eigen::Vector3d>> ceres_map_points_;
  std::map<LocalMapId, std::map<KeyFrameId, NodePose>> ceres_poses_;
  void AddExtricToProblem(ceres::Problem*problem);
};

class EssentialGraphLocalMapOptimization : public LocalMapOptimization {
 public:
  EssentialGraphLocalMapOptimization(const LocalMapOptimizationOption& option)
      : LocalMapOptimization(option),
        ess_options_(option.sssential_graph_option) {}
  void StrategyOptimize(
      std::map<LocalMapId, std::shared_ptr<LocalMap>>* local_maps);

 private:
  std::vector<KeyFrameId> GetKeyLevelConnectedKeyFrames(
      const KeyFrameId& frame_id, const std::vector<int>& levels,
      const LocalMap& local_map);
  LocalMapOptimizationOption::EssentialGraphOption ess_options_;
};

class GraphLocalMapOptimization6TOF : public LocalMapOptimization {
 public:
  GraphLocalMapOptimization6TOF(const LocalMapOptimizationOption& option)
      : LocalMapOptimization(option),
        ess_options_(option.sssential_graph_option) {}
  void StrategyOptimize(
      std::map<LocalMapId, std::shared_ptr<LocalMap>>* local_maps);

 private:
  std::vector<std::pair<KeyFrameId, int>> GetKeyLevelConnectedKeyFrames(
      const KeyFrameId& frame_id, const std::vector<int>& levels,
      const LocalMap& local_map);
  LocalMapOptimizationOption::EssentialGraphOption ess_options_;
};

}  // namespace mapping
}  // namespace jarvis
#endif
