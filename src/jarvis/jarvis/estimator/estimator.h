#ifndef JARVIS_ESTIMATOR_ESTIMATOR_H
#define JARVIS_ESTIMATOR_ESTIMATOR_H
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "ceres/ceres.h"
#include "data_base.h"
#include "jarvis/common/fixed_ratio_sampler.h"
#include "jarvis/common/time.h"
#include "jarvis/estimator/featureTracker/feature_tracker.h"
#include "jarvis/estimator/feature_manager.h"
#include "jarvis/estimator/initialization_interface.h"
#include "jarvis/estimator/pose_predict.h"
#include "jarvis/estimator/slide_window.h"
#include "jarvis/key_frame_data.h"
#include "jarvis/option_parse.h"
#include "jarvis/sensor/image_data.h"
#include "jarvis/sensor/imu_data.h"
#include "jarvis/sensor/odometry_data.h"
#include "jarvis/transform/rigid_transform.h"
#include "jarvis/utility/tic_toc.h"
#include "jarvis/utility/utility.h"
// #include "jarvis/tracking/tracking_interface.h"
#include "parameters.h"
namespace jarvis {
namespace estimator {
//

struct FailureDetectOptoin {
  int track_feat_lost_min_num = 2;
  int track_feat_lost_win_size = 10;
  double bas_norm_max = 0.5;
  double bgs_norm_max = 0.5;
  double translation_norm_max = 0.2;
  double translation_z_max = 0.2;
  double ratation_max = 20;
  int enable_odo_zero_lost_detect = 0;
  double zero_translation_norm_max = 0.002;
  double zero_translation_z_max = 0.002;
  double zero_ratation_max = 0.001;
  int zero_odo_win_size = 10;
  int zero_odo_pose_size = 40;
};

struct EstimatorOption {
  // SlideWindowOption slide_windows_option;
  FeatureManagerOption feature_manager_option;
  FeatureTrackerOption feature_track_option;
  CalibrateOption calibrate_option;
  ImuOption imu_option;
  FailureDetectOptoin fail_detect_option;
  UpdataZeroVelocityOption updata_zerovelocity_option;
  OdomFactorOption odom_factor_option;
  bool enable_zero_velocity = 0;
  ;
  int use_imu = 1;
  int use_odom = 1;
  double data_base_lenth = 1;
  int use_cam_num = 1;
  int estimate_td = 1;
  int estimate_extrinsic = 1;
  double init_td = 0;
  double optimazation_outliers_rejection_th = 3;
  double rejection_points_depth_max_th = 30;
  double use_stereo_sample_ration = 0.05;
  double init_rotation_th = 10;
  double init_bas_normal_max = 0.3;
  int convin_used_num = 4;
};

class Estimator {
 public:
  enum TrackState { LOST = 0, INIT = 1, TRACKING = 2 };
  Estimator(const EstimatorOption &options);
  //   Estimator(const std::string &config_file);
  std::unique_ptr<TrackingData> AddImageData(const sensor::ImageData &images);
  //
  void AddImuData(const sensor::ImuData &imu_data);
  ~Estimator();
  void AddOdometryData(const sensor::OdometryData &odometry_data);
  // std::thread trackThread;
  // std::thread processThread;
  private:
  std::unique_ptr<FeatureTracker> feature_tracker_ = nullptr;

  std::shared_ptr<FeatureManager> f_manager = nullptr;
  
  const EstimatorOption options_;

  std::unique_ptr<common::FixedRatioSampler> stereo_sample_;
  std::unique_ptr<DataBase> data_base_ = nullptr;
  jarvis::transform::Rigid3d transform_imu_to_robot_;
  std::unique_ptr<PosePredit> pose_predit_;

  common::Time last_time_;
  std::unique_ptr<SlideWindow> slide_wondows_;
  ImuState imu_state_;
  std::unique_ptr<InitializationInterface> initializer_;
  double estimator_td_ = 0;
  uint64_t frame_id_ = 0;
};
std::unique_ptr<Estimator> TrackerFactory(const std::string &config_file);

}  // namespace estimator
}  // namespace jarvis

#endif