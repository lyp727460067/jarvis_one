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
#include "jarvis/estimator/initial/initialization_stero_imu.h"
// #include "jarvis/tracking/tracking_interface.h"
#include "parameters.h"
#include "jarvis/estimator/failure_detect.h"
#include "jarvis/common/thread_pool.h"
namespace jarvis {
namespace estimator {
//

struct EstimatorOption {
  SlideWindowOption slide_windows_option;
  std::vector<FeatureTrackerOption> feature_track_options;
  ImuOption imu_option;
  FailureDetectOptoin fail_detect_option;
  //
  SteroImuInitializationOption stero_imu_init_option;
  std::vector<std::vector<int>> track_sequence;
  int use_stero = 0;
  //
  double data_base_lenth = 1;

  double use_stereo_sample_ration = 0.3;
  //
  int win_size=6;
  int thread_num = 10;
  common::ThreadPool* thread_pool=nullptr;
};
struct EstimatorResult {
  TrackingData front_data;
  TrackingData slide_out_data;
};
class Estimator {
 public:
  enum TrackState { LOST = 0, INIT = 1, TRACKING = 2 };
  Estimator(const EstimatorOption &options);
  //   Estimator(const std::string &config_file);
  std::unique_ptr<EstimatorResult> AddImageData(const sensor::ImageData &images);
  //
  void SetPriorFactorFunction(PriorFactorFunction prior_factor) {
    prior_factor_ = std::move(prior_factor);
  }
  void AddImuData(const sensor::ImuData &imu_data);
  ~Estimator();
  void AddOdometryData(const sensor::OdometryData &odometry_data);
  //
  void AddSwBigenInitalPose(transform::Rigid3d &pose);
  //
  // std::thread trackThread;
  // std::thread processThread;
  private:
  //
  void  PredictPtsInNextFrame(const FrameData&frame_data,
                     const transform::Rigid3d&predit_pose);
  // 
  std::map<int, std::unique_ptr<FeatureTracker>> feature_trackers_;
  std::map<int, std::unique_ptr<InitializationInterface>> initials_;
  EstimatorOption options_;
  std::unique_ptr<common::FixedRatioSampler> stereo_sample_;
  std::unique_ptr<DataBase> data_base_ = nullptr;
  jarvis::transform::Rigid3d transform_imu_to_robot_;
  std::unique_ptr<PosePredit> pose_predit_;
  std::unique_ptr<FailureDetect> failure_detect_;
  common::Time last_time_;
  common::ThreadPool* thread_pool_;
  std::unique_ptr<SlideWindow> slide_wondows_;
  ImuState imu_state_;
  double estimator_td_ = 0;
  uint64_t frame_id_ = 0;
  int testnum_  =0;
  PriorFactorFunction prior_factor_;
  std::unique_ptr<common::Task> when_done_task_ ;
  
};


}  // namespace estimator
}  // namespace jarvis

#endif