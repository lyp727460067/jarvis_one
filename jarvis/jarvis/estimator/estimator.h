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

#include "feature_manager.h"
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
#include "jarvis/transform/rigid_transform.h"
#include "jarvis/utility/tic_toc.h"
#include "jarvis/utility/utility.h"
#include "sensor/image_data.h"
#include "sensor/imu_data.h"
// #include "jarvis/tracking/tracking_interface.h"
#include "parameters.h"
namespace jarvis {
namespace estimator {
//

struct EstimatorResult {
  std::unique_ptr<TrackingData> tracking_result;
  struct FeatureResult {
    common::Time time;
    std::shared_ptr<cv::Mat> images;
    std::vector<cv::KeyPoint> key_points;
    transform::Rigid3d pose;
  } feature_result;
};

class Estimator {
 public:
  Estimator(const std::string &config_file);
  std::unique_ptr<EstimatorResult> AddImageData(
      const sensor::ImageData &images);
  //
  void AddImuData(const sensor::ImuData &imu_data);
  ~Estimator();

  void setParameter();

  // interface
  void initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r);
  void inputIMU(double t, const Vector3d &linearAcceleration,
                const Vector3d &angularVelocity);
  void inputFeature(double t, const ImageFeatureTrackerResult &featureFrame);
  void inputImage(double t, const cv::Mat &_img,
                  const cv::Mat &_img1 = cv::Mat());
  void processIMU(double t, double dt, const Vector3d &linear_acceleration,
                  const Vector3d &angular_velocity);
  void processImage(const ImageFeatureTrackerResult &image,
                    const double header);
  void processMeasurements();
  void changeSensorType(int use_imu, int use_stereo);

  // internal
  void clearState();
  bool initialStructure();
  bool visualInitialAlign();
  bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
  void slideWindow();
  void slideWindowNew();
  void slideWindowOld();
  void optimization();
  void vector2double();
  void double2vector();
  bool failureDetection();
  bool getIMUInterval(double t0, double t1,
                      vector<pair<double, Eigen::Vector3d>> &accVector,
                      vector<pair<double, Eigen::Vector3d>> &gyrVector);
  void getPoseInWorldFrame(Eigen::Matrix4d &T);
  void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);
  void predictPtsInNextFrame();
  void outliersRejection(set<int> &removeIndex);
  double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici,
                           Vector3d &tici, Matrix3d &Rj, Vector3d &Pj,
                           Matrix3d &ricj, Vector3d &ticj, double depth,
                           Vector3d &uvi, Vector3d &uvj);
  void updateLatestStates();
  void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration,
                      Eigen::Vector3d angular_velocity);
  bool IMUAvailable(double t);
  void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);

  enum SolverFlag { INITIAL = 0, NON_LINEAR };

  enum MarginalizationFlag { MARGIN_OLD = 0, MARGIN_SECOND_NEW = 1 };

  std::mutex mProcess;
  std::mutex mBuf;
  std::mutex mPropagate;
  queue<pair<double, Eigen::Vector3d>> accBuf;
  queue<pair<double, Eigen::Vector3d>> gyrBuf;
  queue<pair<double, ImageFeatureTrackerResult>> featureBuf;
  double prevTime = 0, curTime = 0;
  bool openExEstimation = false;

  std::thread trackThread;
  std::thread processThread;

  FeatureTracker featureTracker;

  SolverFlag solver_flag;
  MarginalizationFlag marginalization_flag;
  Vector3d g;

  Matrix3d ric[2];
  Vector3d tic[2];

  Vector3d Ps[(WINDOW_SIZE + 1)];
  Vector3d Vs[(WINDOW_SIZE + 1)];
  Matrix3d Rs[(WINDOW_SIZE + 1)];
  Vector3d Bas[(WINDOW_SIZE + 1)];
  Vector3d Bgs[(WINDOW_SIZE + 1)];
  std::pair<double, ImageFeatureTrackerResult> images_[(WINDOW_SIZE + 1)];
  double td = 0.0;

  Matrix3d back_R0, last_R, last_R0;
  Vector3d back_P0, last_P, last_P0;
  double Headers[(WINDOW_SIZE + 1)];

  IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
  Vector3d acc_0, gyr_0;

  vector<double> dt_buf[(WINDOW_SIZE + 1)];
  vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
  vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

  int frame_count = 0;
  int sum_of_outlier = 0, sum_of_back = 0, sum_of_front = 0, sum_of_invalid = 0;
  int inputImageCnt = 0;

  std::unique_ptr<FeatureManager> f_manager;
  MotionEstimator m_estimator;
  InitialEXRotation initial_ex_rotation;

  bool first_imu = false;
  bool is_valid = false, is_key = false;
  bool failure_occur = false;

  vector<Vector3d> point_cloud;
  vector<Vector3d> margin_cloud;
  vector<Vector3d> key_poses;
  double initial_timestamp = 0.0;

  double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
  double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
  double para_Feature[NUM_OF_F][SIZE_FEATURE];
  double para_Ex_Pose[2][SIZE_POSE];
  double para_Retrive_Pose[SIZE_POSE];
  double para_Td[1][1];
  double para_Tr[1][1];

  int loop_window_index = 0;

  MarginalizationInfo *last_marginalization_info = nullptr;
  vector<double *> last_marginalization_parameter_blocks;

  map<double, ImageFrame> all_image_frame;
  IntegrationBase *tmp_pre_integration = nullptr;

  Eigen::Vector3d initP;
  Eigen::Matrix3d initR;

  double latest_time = 0.0;
  Eigen::Vector3d latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0,
      latest_gyr_0;
  Eigen::Quaterniond latest_Q;

  bool initFirstPoseFlag = false;
  bool initThreadFlag = false;
};

std::unique_ptr<Estimator> TrackerFactory(const std::string &config_file);

}  // namespace estimator
}  // namespace jarvis
#endif