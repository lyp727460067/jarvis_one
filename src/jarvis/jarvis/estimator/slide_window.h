#ifndef _JARVIS_ESTIMATOR_SLIDE_WINDOW_H
#define _JARVIS_ESTIMATOR_SLIDE_WINDOW_H
#include <queue>

#include "jarvis/common/time.h"
#include "jarvis/estimator/data_base.h"
#include "jarvis/estimator/featureTracker/feature_tracker.h"
#include "jarvis/estimator/feature_manager.h"
#include "jarvis/estimator/initialization_interface.h"
#include "jarvis/estimator/optimization.h"
#include "jarvis/estimator/updater_zero_velocity.h"
#include "jarvis/sensor/imu_data.h"
#include "jarvis/key_frame_data.h"
#include "jarvis/estimator/marginalization.h"
#include "jarvis/transform/rigid_transform.h"
namespace jarvis {
namespace estimator {
struct SlideWindowOption {
  ImuOption imu_option;
  OdomFactorOption odom_factor_option;
  bool use_stereo  =true;
  FeatureManagerOption feature_manager_option;
  UpdataZeroVelocityOption updata_zerovelocity_option;
  OptimizationOption opti_option;
  std::vector<transform::Rigid3d> extric_camera_to_imu;
  bool enable_zero_velocity = 0;
  int win_size=6;
  int op_prior_match_min_num =10;
  // double optimazation_outliers_rejection_th = 0.3;
  double rejection_points_depth_max_th = 30;
  std::vector<std::vector<int>> track_sequence;
  double camera_imu_time_offset=0;
  common::ThreadPool* thread_pool=nullptr; 
//
};
//
//
struct SlideWindowResult {
  FrameData frame_data;
  double final_cost;
  FeatTrackInfo feat_track_info;
  std::optional<double> latest_odo_distance = 0;
  TrackingData slide_out_data;
};


//
using PriorFactorFunction = std::function<std::shared_ptr<LocalMapMatchResult>(
    const TrackingData& track_data)>;
class SlideWindow {
 public:
  SlideWindow(const SlideWindowOption& option, DataBase* data_base,
              const std::unique_ptr<InitializationResult>& init_data,
              PriorFactorFunction prior_factor = nullptr);
  //
  std::unique_ptr<SlideWindowResult> AddFeatureData(const FrameData&);
  //
  std::map<CameraId, std::set<TrackFeatureId>>& RejectionOutliers() {
    return rejection_outliers_;
  }
  std::map<int, Eigen::Vector3d> PredictNextFrame(
      const transform::Rigid3d& predit_imu_pose, int s);
  // /
  UpdataZeroVelocity* GetUpdataZeroVelocity() {
    return update_zero_velocity_.get();
  }

 private:
  void SlideData(bool);
  TrackingData GetratePriorData(bool generate_point=false,int k=0);
  SlideWindowOption options_;
  //
  std::vector<ImuState> imu_states_;
  std::map<common::Time, sensor::ImageData> images_;
  //
  std::vector<std::shared_ptr<IntegrationBase>> integration_base_;
  std::vector<bool> zero_velocity_factor_state_;
  std::vector<std::shared_ptr<OdomFactor>>odoms_factor_;
  //

  std::unique_ptr<FeatureManagers> feature_managers_;
  //
  std::map<CameraId, std::shared_ptr<FeatureManager>> init_feature_managers_;
  std::map<common::Time, std::map<CameraId, FrameData::FeatureData>>
      init_feature_datas_;
  //
  std::unique_ptr<Optimization> optimization_;
  std::unique_ptr<UpdataZeroVelocity> update_zero_velocity_=nullptr;
  std::unique_ptr<Marginalization> marginalizer_;

  void SlideNew(); 
  DataBase* data_base_;
  int init_steady_num_=0;
  //
  void StateToFrameData();
  void FrameDataToState();
  OptimizationStateData* opt_data_=nullptr;
  std::vector<transform::Rigid3d> extric_camera_to_imu_;
  transform::Rigid3d odo_to_imu_extric_;
  double camera_imu_time_offset_ = 0;
  std::map<CameraId, std::set<TrackFeatureId>> rejection_outliers_;
  //
  PriorFactorFunction prior_factor_;
  //
  bool first_init_= true; 
  int init_slide_new_num  =0;
  // std::vector<FrameData> frames_datas_;
  common::Time last_feature_time_;
  uint64_t global_id_ = 0;
  int has_prio_pose =0;
};
}  // namespace estimator
}  // namespace jarvis
#endif
