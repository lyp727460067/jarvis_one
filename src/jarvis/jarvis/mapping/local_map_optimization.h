#ifndef __JARVIS_MAPPING_MAP_OPTIMIZATION__H__
#define __JARVIS_MAPPING_MAP_OPTIMIZATION__H__

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
  struct EssentialGraphOption {
    int max_con_kf_num = 10;
    int max_adjacent_kf_num = 10;
    std::vector<int> convisi_level_search_num{5 ,3};
  } sssential_graph_option;
};

//
struct LocalMapOptimizationData {
  struct MapPointData {
    Eigen::Vector3d pos;
    std::map<KeyFrameId, FeatureId> con_frame_datas;  // 地图点对应的观察帧及特征
  };
  std::map<MapPointId, MapPointData> con_map_points;
  struct FrameData {
    // common::Time time;
    std::string time;
    transform::Rigid3d pose;
  };
  std::map<KeyFrameId, FrameData> frame_datas;
  std::map<FeatureId, FeatureData*> feature_datas;
  // 数量为frame_datas的数量-1,表示KeyFrameId与frame_datas中其前一帧keyframe间的预积分
  std::map<KeyFrameId, jarvis::estimator::IntegrationBase*> imu_datas;
  //
};

class LocalMapOptimization {
 public:
  LocalMapOptimization(const LocalMapOptimizationOption& option);
  ~LocalMapOptimization();
  // for rtk
  void AddFixData(const sensor::FixedFramePoseData& fix_data) {}
  // 原始IMU数据，需要积分
  void AddImuData(sensor::ImuData& imu_data) {}
  //
    // input: 需要优化的帧数据
  void Optimize(LocalMapOptimizationData* data);
  void Optimize(std::map<LocalMapId, std::shared_ptr<LocalMap>>* local_maps);

 protected:
  virtual void StrategyOptimize(
      std::map<LocalMapId, std::shared_ptr<LocalMap>>* local_maps);

  std::queue<sensor::ImuData> imu_datas_;
  std::queue<sensor::FixedFramePoseData> fix_datas_;
  LocalMapOptimizationOption  options_;
  std::vector<transform::Rigid3d> extric_camera_to_imu_;

private:
  double **para_Pose; // x,y,z,qw,qx,qy,qz
  double **para_MapPoint;
  double **para_Ex; // x,y,z,qw,qx,qy,qz
  double **para_SpeedBias; // vx vy vz bax bay baz bgx bgy bgz
};

class EssentialGraphLocalMapOptimization : public LocalMapOptimization {
 public:
  EssentialGraphLocalMapOptimization(const LocalMapOptimizationOption& option)
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
