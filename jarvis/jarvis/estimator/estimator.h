/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef JARVIS_ESTIMATOR_ESTIMATOR_H
#define JARVIS_ESTIMATOR_ESTIMATOR_H
#include <ceres/ceres.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mutex>
#include <opencv2/core/eigen.hpp>
#include <queue>
#include <thread>
#include <unordered_map>

#include "jarvis/estimator/feature_manager.h"
#include "jarvis/common/time.h"
#include "jarvis/estimator/factor/imu_factor.h"
#include "jarvis/estimator/factor/marginalization_factor.h"
#include "jarvis/estimator/factor/pose_local_parameterization.h"
#include "jarvis/estimator/factor/projectionOneFrameTwoCamFactor.h"
#include "jarvis/estimator/factor/projectionTwoFrameOneCamFactor.h"
#include "jarvis/estimator/factor/projectionTwoFrameTwoCamFactor.h"
#include "jarvis/estimator/featureTracker/feature_tracker.h"
#include "jarvis/estimator/initial/initial_alignment.h"
#include "jarvis/estimator/initial/initial_ex_rotation.h"
#include "jarvis/estimator/initial/initial_sfm.h"
#include "jarvis/estimator/initial/solve_5pts.h"
#include "jarvis/key_frame_data.h"
#include "jarvis/option_parse.h"
#include "jarvis/transform/rigid_transform.h"
#include "jarvis/utility/tic_toc.h"
#include "jarvis/utility/utility.h"
#include "jarvis/sensor/image_data.h"
#include "jarvis/sensor/imu_data.h"
#include "jarvis/estimator/updater_zero_velocity.h"
#include "jarvis/common/fixed_ratio_sampler.h"
// #include "jarvis/tracking/tracking_interface.h"
#include "parameters.h"
namespace jarvis {
namespace estimator {
//
class ImuExtrapolator;

struct FailureDetectOptoin
{
  int track_feat_lost_min_num =2;
  int track_feat_lost_win_size =  10;
  double bas_norm_max =0.5;
  double bgs_norm_max =0.5;
  double translation_norm_max =0.2;
  double translation_z_max =0.2;
  double ratation_max =20;

};

struct EstimatorOption {
  // SlideWindowOption slide_windows_option;
  FeatureManagerOption feature_manager_option;
  FeatureTrackerOption feature_track_option;
  CalibrateOption calibrate_option;
  ImuOption imu_option;
  FailureDetectOptoin fail_detect_option;
  UpdataZeroVelocityOption updata_zerovelocity_option;
  bool enable_zero_velocity =0;;
  int  use_imu = 1;
  int use_cam_num = 1;
  int estimate_td = 1;
  int estimate_extrinsic =1;
  double init_td = 0;
  double optimazation_outliers_rejection_th=3;
  double use_stereo_sample_ration=0.05; 
};

class Estimator {
 public:
  enum TrackState {LOST = 0, INIT = 1,TRACKING = 2 };
  Estimator(const EstimatorOption &options);
//   Estimator(const std::string &config_file);
  std::unique_ptr<TrackingData> AddImageData(const sensor::ImageData &images);
  //
  void AddImuData(const sensor::ImuData &imu_data);
  ~Estimator();
  bool IsStereo();
  void setParameter();

  // interface
  void initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r);
  void inputIMU(double t, const Eigen::Vector3d &linearAcceleration,
                const Eigen::Vector3d &angularVelocity);
  void inputFeature(double t, const ImageFeatureTrackerData &featureFrame);
  void inputImage(double t, const cv::Mat &_img,
                  const cv::Mat &_img1 = cv::Mat());
  void processIMU(double t, double dt,
                  const Eigen::Vector3d &linear_acceleration,
                  const Eigen::Vector3d &angular_velocity);
  int processImage(const ImageFeatureTrackerData &image, const double header);
  int processMeasurements();

  // internal
  void clearState();
  bool initialStructure();
  bool visualInitialAlign();
  bool relativePose(Eigen::Matrix3d &relative_R, Eigen::Vector3d &relative_T,
                    int &l);
  void slideWindow();
  void slideWindowNew();
  void slideWindowOld();
  void optimization();
  void vector2double();
  void double2vector();
  bool failureDetection();

  bool getIMUInterval(
      double t0, double t1,
      std::vector<std::pair<double, Eigen::Vector3d>> &accVector,
      std::vector<std::pair<double, Eigen::Vector3d>> &gyrVector);
  bool GetImuInterval(
      double t0, double t1,
      std::vector<std::pair<double, Eigen::Vector3d>> &accVector,
      std::vector<std::pair<double, Eigen::Vector3d>> &gyrVector);

  void getPoseInWorldFrame(Eigen::Matrix4d &T);
  void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);
  void predictPtsInNextFrame();
  void outliersRejection(std::set<int> &removeIndex);
  double reprojectionError(Eigen::Matrix3d &Ri, Eigen::Vector3d &Pi,
                           Eigen::Matrix3d &rici, Eigen::Vector3d &tici,
                           Eigen::Matrix3d &Rj, Eigen::Vector3d &Pj,
                           Eigen::Matrix3d &ricj, Eigen::Vector3d &ticj,
                           double depth, Eigen::Vector3d &uvi,
                           Eigen::Vector3d &uvj);
  void updateLatestStates();
  void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration,
                      Eigen::Vector3d angular_velocity);
  bool IMUAvailable(double t);
  void initFirstIMUPose(
      std::vector<std::pair<double, Eigen::Vector3d>> &accVector);

  enum SolverFlag { INITIAL = 0, NON_LINEAR };

  enum MarginalizationFlag { MARGIN_OLD = 0, MARGIN_SECOND_NEW = 1 };
  std::mutex mProcess;
  std::mutex mBuf;
  std::mutex mPropagate;
  std::queue<std::pair<double, Eigen::Vector3d>> accBuf;
  std::queue<std::pair<double, Eigen::Vector3d>> gyrBuf;
  std::queue<std::pair<double, ImageFeatureTrackerData>> featureBuf;
  double prevTime = 0, curTime = 0;
  double prev_time_ = 0;
  bool openExEstimation = false;

  // std::thread trackThread;
  // std::thread processThread;

  std::unique_ptr<FeatureTracker> feature_tracker_ = nullptr;

  SolverFlag solver_flag;
  MarginalizationFlag marginalization_flag;
  Eigen::Vector3d g;

  Eigen::Matrix3d ric[2];
  Eigen::Vector3d tic[2];

  Eigen::Vector3d Ps[(WINDOW_SIZE + 1)];
  Eigen::Vector3d Vs[(WINDOW_SIZE + 1)];
  Eigen::Matrix3d Rs[(WINDOW_SIZE + 1)];
  Eigen::Vector3d Bas[(WINDOW_SIZE + 1)];
  Eigen::Vector3d Bgs[(WINDOW_SIZE + 1)];
  std::pair<double, ImageFeatureTrackerData> images_[(WINDOW_SIZE + 1)];
  double td = 0.0;

  Eigen::Matrix3d back_R0, last_R, last_R0;
  Eigen::Vector3d back_P0, last_P, last_P0;
  double Headers[(WINDOW_SIZE + 1)];

  IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
  bool is_velocity_updates_[(WINDOW_SIZE + 1)];
  Eigen::Vector3d acc_0, gyr_0;

  std::vector<double> dt_buf[(WINDOW_SIZE + 1)];
  std::vector<Eigen::Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
  std::vector<Eigen::Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

  int frame_count = 0;
  int sum_of_outlier = 0, sum_of_back = 0, sum_of_front = 0, sum_of_invalid = 0;
  int inputImageCnt = 0;

  std::unique_ptr<FeatureManager> f_manager = nullptr;
  MotionEstimator m_estimator;
  InitialEXRotation initial_ex_rotation;

  bool first_imu = false;
  bool is_valid = false, is_key = false;
  bool failure_occur = false;

  std::vector<Eigen::Vector3d> point_cloud;
  std::vector<Eigen::Vector3d> margin_cloud;
  std::vector<Eigen::Vector3d> key_poses;
  double initial_timestamp = 0.0;

  double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
  double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
  double para_Feature[NUM_OF_F][SIZE_FEATURE];
  double para_Ex_Pose[2][SIZE_POSE];
  double para_Retrive_Pose[SIZE_POSE];
  double para_Td[1][1];
  double para_Tr[1][1];
  double angle_ = 0.0;
  int loop_window_index = 0;
  std::unique_ptr<ImuExtrapolator> imu_extrapolator_;
  MarginalizationInfo *last_marginalization_info = nullptr;
  std::vector<double *> last_marginalization_parameter_blocks;

  std::map<double, ImageFrame> all_image_frame;
  IntegrationBase *tmp_pre_integration = nullptr;

  Eigen::Vector3d initP;
  Eigen::Matrix3d initR;

  double latest_time = 0.0;
  Eigen::Vector3d latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0,
      latest_gyr_0;
  Eigen::Quaterniond latest_Q;

  bool initFirstPoseFlag = false;
  bool initThreadFlag = false;
  const EstimatorOption options_;
  int estimate_extrinsic_ = 2;
  Alignment alignment_;
  std::vector<bool> failuer_track_lost_;
  std::unique_ptr<UpdataZeroVelocity> update_zero_velocity_;
  std::unique_ptr<common::FixedRatioSampler> stereo_sample_;
};
std::unique_ptr<Estimator> TrackerFactory(const std::string &config_file);

}  // namespace estimator
}  // namespace jarvis

#endif