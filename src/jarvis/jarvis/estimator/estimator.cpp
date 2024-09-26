/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "jarvis/estimator/estimator.h"
#include "jarvis/common/time.h"
#include "ceres/tiny_solver.h"
#include "ceres/tiny_solver_autodiff_function.h"
#include "glog/logging.h"
#include "imu_extrapolator.h"
#include "jarvis/common/time.h"
#include "jarvis/estimator/parameters.h"
#include "jarvis/option_parse.h"
namespace jarvis {
bool restart = false;
std::vector<Eigen::Vector3d> kGlobleImuPose;
std::pair<double, transform::Rigid3d> kGlobleImuExtrapolatorPose;

std::vector<Eigen::Vector3d> GetGlobleImuPose() { return kGlobleImuPose; }
std::pair<double, transform::Rigid3d> GetGlobleImuExtrapolatorPose() {
  return kGlobleImuExtrapolatorPose;
}

namespace estimator {
namespace {
}
// ofstream cam_time_babg("/tmp/cam_time_babg.txt");
//
// Estimator(const EstimatorOption &options);

Estimator::Estimator(const EstimatorOption &options)
    : options_(options),
      alignment_(AlignmentOption{
          options_.calibrate_option.extric_camera_to_imu[0],
          Eigen::Vector3d{0, 0, options_.imu_option.gravity_normal}}) {
  estimate_extrinsic_ = options_.estimate_extrinsic;
  LOG(INFO) << options_.calibrate_option.extric_camera_to_imu[0];
  //

//
  f_manager = std::make_unique<FeatureManager>(options_.feature_manager_option);
  //

  data_base_ = std::make_unique<DataBase>(options_.data_base_lenth);
  stereo_sample_ = std::make_unique<common::FixedRatioSampler>(
      options_.use_stereo_sample_ration);
  if (options_.enable_zero_velocity) {
    update_zero_velocity_ = std::make_unique<UpdataZeroVelocity>(
        options_.updata_zerovelocity_option);
  }
  transform_imu_to_robot_ = transform::Rigid3d::Identity();
  // options_.calibrate_option.extric_camera_to_imu[0] *
  // options_.calibrate_option.extric_camera_to_robot.inverse();

  // imu_extrapolator_ = std::make_unique<ImuExtrapolator>();
  LOG(INFO) << "init begins";
  initThreadFlag = false;

  for (int i = 0; i < WINDOW_SIZE + 1; i++) {
    pre_integrations[i] = nullptr;
    Headers[i] = 0.0;
    odometry_factor_[i] = nullptr;
    images_[i] = std::make_pair<double, ImageFeatureTrackerData>(0, {});
  }

  clearState();
  // readParameters(config_file);
  setParameter();
  feature_tracker_ =
      std::make_unique<FeatureTracker>(options_.feature_track_option);
  //
}

Estimator::~Estimator() {
  for (int i = 0; i < WINDOW_SIZE + 1; i++) {
    //
    if (pre_integrations[i] != nullptr) {
      delete pre_integrations[i];
    }
    if (odometry_factor_[i] != nullptr) {
      delete odometry_factor_[i];
    }
  }
  if (tmp_pre_integration != nullptr) {
    delete tmp_pre_integration;
  }
  if (last_marginalization_info != nullptr) {
    delete last_marginalization_info;
  }
  //
  for (auto frame_it = all_image_frame.begin();
       frame_it != all_image_frame.end(); ++frame_it) {
    if (frame_it->second.pre_integration != nullptr) {
      delete frame_it->second.pre_integration;
    }
  }
  //
}
//
cv::KeyPoint EigenToCv(const Eigen::Vector2d &p) {
  return cv::KeyPoint(p.x(), p.y(), 2);
}

namespace {
std::map<int, int> track_num;
std::unique_ptr<TrackingData> ExtractKeyFrameMapPoints(
    const Estimator &estimator, const ImageFeatureTrackerData &feature_result) {
  TrackingData result{};
  result.data = std::make_shared<TrackingData::Data>();
  for (const auto &p : feature_result.data->features) {
    result.data->key_points.push_back(
        EigenToCv(p.second.camera_features[0].uv));
    result.data->key_points.back().class_id = p.first;
    result.data->key_points.back().octave =
        feature_result.data->tracker_features_num[p.first];
  }

  result.data->image =
      std::make_shared<cv::Mat>(feature_result.data->images[0].clone());
  return std::make_unique<TrackingData>(result);
}
}  // namespace
//
std::unique_ptr<TrackingData> Estimator::AddImageData(
    const sensor::ImageData &images) {
  TicToc add_image_data_cost;
  track_num.clear();
  //
  //

  inputImageCnt++;
  ImageFeatureTrackerData featureFrame;
  TicToc featureTrackerTime;

  //
  std::vector<std::pair<double, Eigen::Vector3d>> accVector, gyrVector;
  // double angle = 0.0;
  double d_time = common::ToSeconds(images.time - common::FromUniversal(0));
  //
  if (GetImuInterval(prev_time_, d_time, accVector, gyrVector)) {
    Eigen::Quaterniond delta_q = Eigen::Quaterniond::Identity();
    if (!accVector.empty()) {
      for (size_t i = 1; i < accVector.size(); i++) {
        auto dt = gyrVector[i].first - gyrVector[i - 1].first;
        delta_q *= Utility::deltaQ(gyrVector[i].second * dt);
      }
      auto angle = common::RadToDeg(
          transform::GetYaw(transform::Rigid3d::Rotation(delta_q)));
      // LOG(INFO) << angle;
      angle_ = angle_ * 0.2 + 0.8 * angle;
      // LOG(INFO) << angle_ << " " << angle;
    }

    //
  }

  prev_time_ = d_time;
  TicToc trackTime;
  bool use_stere = false;
  if (is_velocity_updates_[WINDOW_SIZE-1]) {
    use_stere = true;
  } else {
    if (stereo_sample_->Pulse()) {
      use_stere = true;
    }
  }
  if (solver_flag == INITIAL || use_stere) {
    featureFrame = feature_tracker_->trackImage(
        d_time, *images.image[0], *images.image[1], &track_num, angle_);

  } else {
    featureFrame = feature_tracker_->trackImage(d_time, *images.image[0],
                                                cv::Mat(), &track_num, angle_);
  }
  // LOG(INFO) << " trackImage : " <<trackTime.toc();

  if (update_zero_velocity_) {
    update_zero_velocity_->AddImageKeyPoints(images.time, featureFrame);
  }

  //
  featureBuf.push(std::make_pair(d_time, featureFrame));
  TicToc processTime;
  auto state = processMeasurements();
  LOG_EVERY_N(INFO, 100) << "One frame cost: " << add_image_data_cost.toc();
  // cost_time_hisgram_.push_back(add_image_data_cost.toc());
  // if (cost_time_hisgram_.size() >= 100) {
  //   cost_time_hisgram_.erase(cost_time_hisgram_.begin());
  // }
  // LOG_EVERY_N(INFO, 100) << "One frame costcost:"
  //                        << " " << cost_time_hisgram_.back() << "\n"
  //                        << common::DrawVbars(cost_time_hisgram_);
  //
  auto tracking_data = ExtractKeyFrameMapPoints(*this, featureFrame);
  tracking_data->data->time = images.time;
  LOG_EVERY_N(INFO,10)<<"opti ex0:"<<tic[0].transpose();

  tracking_data->data->transform_cam_to_imu =
      transform::Rigid3d(tic[0], Eigen::Quaterniond(ric[0]));
  if (solver_flag == INITIAL) {
    if (TrackState(state) == TrackState::LOST) {
      tracking_data->status = TrackState::LOST;      
    } else {
      tracking_data->status = TrackState::INIT;
    }
    // LOG(INFO) << Eigen::Quaterniond(Rs[frame_count]);
    // ImuState imu_state_data = ImuState{
    //     transform::Rigid3d({0, 0, 0}, Eigen::Quaterniond(Rs[frame_count]))};
    // //
    // tracking_data->data->imu_state = ImuState{imu_state_data};
  } else {
    // auto odom_temp =   data_base_->InterpolateOdometry(images.time);

    if (TrackState(state) == TrackState::LOST) {
      tracking_data->status = TrackState(state);
    } else {
      if (stable_init_cout < 10) {
        tracking_data->status = TrackState::INIT;
        stable_init_cout++;
      } else {
        tracking_data->status = TrackState(state);
      }
    }
  }

  if (abs(options_.calibrate_option.extric_camera_to_imu[0]
              .translation()
              .norm() -
          tic[0].norm()) > 0.2) {
    LOG(ERROR) << "opti ex error,lost." << tic[0].transpose();
    tracking_data->status = TrackState::LOST;
  }

  auto imu_state_data = ImuState{
      transform::Rigid3d(Ps[frame_count], Eigen::Quaterniond(Rs[frame_count])),
      Vs[frame_count], Bas[frame_count], Bgs[frame_count], g};
  tracking_data->data->imu_state = ImuState{imu_state_data};

  data_base_->TrimData(images.time);
  return tracking_data;
}
//
void Estimator::AddImuData(const sensor::ImuData &imu_data) {
  double d_time = common::ToSeconds(imu_data.time - common::FromUniversal(0));
  if (update_zero_velocity_) {
    update_zero_velocity_->AddImu(imu_data);
  }
  data_base_->AddImu(imu_data);
}

bool Estimator::IsStereo() {
  if (options_.use_cam_num == 2) return true;
  return false;
}
//
void Estimator::clearState() {
  mProcess.lock();
  // while (!accBuf.empty()) accBuf.pop();
  // while (!gyrBuf.empty()) gyrBuf.pop();
  // while (!featureBuf.empty()) featureBuf.pop();

  prevTime = -1;
  curTime = 0;
  openExEstimation = 0;
  initP = Eigen::Vector3d(0, 0, 0);
  initR = Eigen::Matrix3d::Identity();
  inputImageCnt = 0;
  initFirstPoseFlag = false;

  for (int i = 0; i < WINDOW_SIZE + 1; i++) {
    Rs[i].setIdentity();
    Ps[i].setZero();
    Vs[i].setZero();
    Bas[i].setZero();
    Bgs[i].setZero();
    dt_buf[i].clear();
    for (int j = 0; j < SIZE_POSE; j++) {
      para_Pose[i][j] = 0.0;
    }
    for (int j = 0; j < SIZE_SPEEDBIAS; j++) {
      para_SpeedBias[i][j] = 0.0;
    }

    linear_acceleration_buf[i].clear();
    angular_velocity_buf[i].clear();

    if (pre_integrations[i] != nullptr) {
      delete pre_integrations[i];
    }
    pre_integrations[i] = nullptr;
  }
  for (int i = 0; i < NUM_OF_F; i++) {
    for (int j = 0; j < SIZE_FEATURE; j++) {
      para_Feature[i][j] = 0.0;
    }
  }
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < SIZE_POSE; j++) {
      para_Ex_Pose[i][j] = 0.0;
    }
  }
  for (int i = 0; i < SIZE_POSE; i++) {
    para_Retrive_Pose[i] = 0.0;
  }
  para_Td[0][0] = 0.0;
  para_Tr[0][0] = 0.0;
  for (int i = 0; i < options_.use_cam_num; i++) {
    tic[i] = Eigen::Vector3d::Zero();
    ric[i] = Eigen::Matrix3d::Identity();
  }

  first_imu = false, sum_of_back = 0;
  sum_of_front = 0;
  frame_count = 0;
  solver_flag = INITIAL;
  initial_timestamp = 0;
  all_image_frame.clear();

  if (tmp_pre_integration != nullptr) delete tmp_pre_integration;
  if (last_marginalization_info != nullptr) delete last_marginalization_info;

  tmp_pre_integration = nullptr;
  last_marginalization_info = nullptr;
  last_marginalization_parameter_blocks.clear();
  f_manager->clearState();
  failuer_track_lost_.clear();
  failure_occur = 0;

  mProcess.unlock();
}

void Estimator::setParameter() {
  mProcess.lock();
  for (int i = 0; i < options_.use_cam_num; i++) {
    tic[i] = options_.calibrate_option.extric_camera_to_imu[i].translation();
    ric[i] = options_.calibrate_option.extric_camera_to_imu[i]
                 .rotation()
                 .toRotationMatrix();
    LOG(INFO) << " \nexitrinsic cam " << i << "\n"
              << ric[i] << "\n"
              << tic[i].transpose();
  }
  ProjectionTwoFrameTwoCamFactor::sqrt_info =
      FOCAL_LENGTH / 1.5 * Eigen::Matrix2d::Identity();
  ProjectionOneFrameTwoCamFactor::sqrt_info =
      FOCAL_LENGTH / 1.5 * Eigen::Matrix2d::Identity();
  // td = options_.init_td;
  g = Eigen::Vector3d{0, 0, options_.imu_option.gravity_normal};
  LOG(INFO) << "set g " << g.transpose();
  mProcess.unlock();
}

void Estimator::inputImage(double t, const cv::Mat &_img,
                           const cv::Mat &_img1) {
  // printf("process time: %f\n", processTime.toc());
}
void Estimator::AddOdometryData(const sensor::OdometryData &odometry_data) {
  data_base_->AddOdometry(
      sensor::OdometryData{odometry_data.time, odometry_data.pose});
}

void Estimator::inputFeature(double t,
                             const ImageFeatureTrackerData &featureFrame) {
  mBuf.lock();
  featureBuf.push(std::make_pair(t, featureFrame));
  mBuf.unlock();
  processMeasurements();
}
//
bool Estimator::GetImuInterval(
    double t0, double t1,
    std::vector<std::pair<double, Eigen::Vector3d>> &accVector,
    std::vector<std::pair<double, Eigen::Vector3d>> &gyrVector) {
  return false;
  // auto acc_buf_temp = accBuf;
  // auto gyr_buf_temp = gyrBuf;

  // if (acc_buf_temp.empty()) {
  //   printf("not receive imu\n");
  //   return false;
  // }
  // // printf("get imu from %f %f\n", t0, t1);

  // // accBuf.back().first);
  // // if (t1 <= acc_buf_temp.back().first) {
  // while (!acc_buf_temp.empty() && acc_buf_temp.front().first <= t0) {
  //   acc_buf_temp.pop();
  //   gyr_buf_temp.pop();
  // }
  // while (!acc_buf_temp.empty() && acc_buf_temp.front().first < t1) {
  //   accVector.push_back(acc_buf_temp.front());
  //   acc_buf_temp.pop();
  //   gyrVector.push_back(gyr_buf_temp.front());
  //   gyr_buf_temp.pop();
  // }
  // // accVector.push_back(acc_buf_temp.front());
  // // gyrVector.push_back(gyr_buf_temp.front());
  // // } else {
  // //   LOG(WARNING)<<"wait for imu";
  // //   return false;
  // // }
}
//
bool Estimator::getIMUInterval(
    double t0, double t1,
    std::vector<std::pair<double, Eigen::Vector3d>> &accVector,
    std::vector<std::pair<double, Eigen::Vector3d>> &gyrVector) {
  const common::Time start_time = common::Time(common::FromSeconds(t0));
  const common::Time end_time = common::Time(common::FromSeconds(t1));
  //
  if (!data_base_->HasImuData(end_time)) {
    LOG(WARNING) << "imu base not have " << end_time << " data";
    return false;
  }
  std::vector<sensor::ImuData> result =
      data_base_->GetImuIntervalData(start_time, end_time);
  if (result.empty()) {
    LOG(WARNING) << "GetImuIntervalData empty " << start_time << " "
                 << end_time;
    return false;
  }
  for (const auto &r : result) {
    accVector.push_back({common::ToSeconds(r.time - common::FromUniversal(0)),
                         r.linear_acceleration});
    gyrVector.push_back({common::ToSeconds(r.time - common::FromUniversal(0)),
                         r.angular_velocity});
  }
  return true;
}

bool Estimator::IMUAvailable(double t) { return true; }

int Estimator::processMeasurements() {
  // printf("process measurments\n");
  std::pair<double, ImageFeatureTrackerData> feature;
  std::vector<std::pair<double, Eigen::Vector3d>> accVector, gyrVector;
  if (!featureBuf.empty()) {
    feature = featureBuf.front();
    curTime = feature.first + td;
    if (odometry_factor_[frame_count] == nullptr) {
      odometry_factor_[frame_count] =
          new OdomFactor(options_.odom_factor_option, data_base_.get());
    }
    const common::Time start_time = common::Time(common::FromSeconds(prevTime));
    const common::Time end_time = common::Time(common::FromSeconds(curTime));
    odometry_factor_[frame_count]->ComputeObserve(start_time, end_time);

    // while (1) {
    //   if ((!options_.use_imu || IMUAvailable(feature.first + td)))
    //     break;
    //   else {
    //     LOG(WARNING)<<"wait for imu ... \n";
    //     if(!initFirstPoseFlag){
    //        featureBuf.pop();
    //        return TrackState::INIT;
    //     }
    //     break;
    //   }
    // }

    if (options_.use_imu) {
      if (!getIMUInterval(prevTime, curTime, accVector, gyrVector)) {
        LOG(ERROR) << "Imu data invalid!!!";
        if (initFirstPoseFlag) {
          LOG(ERROR) << "return Lost  q!!!"
                     << "curr: " << common::Time(common::FromSeconds(curTime))
                     << "last : "
                     << common::Time(common::FromSeconds(prevTime))<<"imu size: "<<accVector.size();

          prevTime = curTime;
          return TrackState::LOST;
        }
      }
    }

    if (initFirstPoseFlag) {
      double delta_time = curTime - prevTime;
      if (abs(delta_time) > 0.3) {
        LOG(ERROR) << "Image data lost!!  " << delta_time;
        prevTime = curTime;
        return TrackState::LOST;
      }
    }

    featureBuf.pop();
    if (options_.use_imu && !accVector.empty()) {
      LOG(INFO) << "image interval ["
                << common::Time(common::FromSeconds(prevTime)) << ","
                << common::Time(common::FromSeconds(curTime))
                << ",peri: " << curTime - prevTime
                << "],imu num: " << accVector.size();

      if (!initFirstPoseFlag) initFirstIMUPose(accVector);
      for (size_t i = 0; i < accVector.size(); i++) {
        double dt;
        if (i == 0)
          dt = accVector[i].first - prevTime;
        else if (i == accVector.size() - 1)
          dt = curTime - accVector[i - 1].first;
        else
          dt = accVector[i].first - accVector[i - 1].first;
        // LOG(INFO)<< accVector[i].second.transpose();
        processIMU(accVector[i].first, dt, accVector[i].second,
                   gyrVector[i].second);
      }
      // LOG(INFO)<<Ps[frame_count]-Ps[frame_count-1]<<" "<<Ps[frame_count];
    }
    prevTime = curTime;
    if (!initFirstPoseFlag) return TrackState::INIT;
    if (processImage(feature.second, feature.first) != TrackState::TRACKING) {
      return TrackState::LOST;
    }
  }
  return TrackState::TRACKING;
}

void Estimator::initFirstIMUPose(
    std::vector<std::pair<double, Eigen::Vector3d>> &accVector) {
  if (accVector.empty()) return;
  LOG(INFO) << "Init first impu pose.";
  initFirstPoseFlag = true;
  // return;
  Eigen::Vector3d averAcc(0, 0, 0);
  int n = (int)accVector.size();
  for (size_t i = 0; i < accVector.size(); i++) {
    averAcc = averAcc + accVector[i].second;
  }
  averAcc = averAcc / n;
  LOG(INFO) << " averge acc : " << averAcc.transpose();
  Eigen::Matrix3d R0 = Utility::g2R(averAcc);
  double yaw = Utility::R2ypr(R0).x();
  R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
  Rs[0] = R0;
  LOG(INFO) << "Init R0: \n" << Rs[0];
}

void Estimator::initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r) {
  Ps[0] = p;
  Rs[0] = r;
  initP = p;
  initR = r;
}

void Estimator::processIMU(double t, double dt,
                           const Eigen::Vector3d &linear_acceleration,
                           const Eigen::Vector3d &angular_velocity) {
  if (!first_imu) {
    first_imu = true;
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
  }

  if (!pre_integrations[frame_count]) {
    pre_integrations[frame_count] = new IntegrationBase{
        options_.imu_option, acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
  }
  if (frame_count != 0) {
    pre_integrations[frame_count]->push_back(dt, linear_acceleration,
                                             angular_velocity);
    // if(solver_flag != NON_LINEAR)
    // LOG(INFO)<< linear_acceleration.transpose();
    // LOG(INFO)<< angular_velocity.transpose();
    tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

    dt_buf[frame_count].push_back(dt);
    linear_acceleration_buf[frame_count].push_back(linear_acceleration);
    angular_velocity_buf[frame_count].push_back(angular_velocity);

    int j = frame_count;
    Eigen::Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
    // LOG(INFO)<<un_gyr * dt;
    // LOG(INFO)<<Utility::deltaQ(un_gyr * dt);
    Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
    Eigen::Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
    Vs[j] += dt * un_acc;
  }
  acc_0 = linear_acceleration;
  gyr_0 = angular_velocity;
}
std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>
ToStruct(const ImageFeatureTrackerData &image) {
  std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>
      result{};
  for (const auto &feature : image.data->features) {
    for (const auto &image_feature : feature.second.camera_features) {
      Eigen::Matrix<double, 7, 1> f;
      f << image_feature.normal_points.x(), image_feature.normal_points.y(),
          image_feature.normal_points.z(), image_feature.uv.x(),
          image_feature.uv.y(), image_feature.uv_velocity.x(),
          image_feature.uv_velocity.y();
      result[feature.first].emplace_back(image_feature.id, f);
    }
  }
  return result;
}
void Estimator::InitFailureRestart() {
  marginalization_flag = MARGIN_OLD;
  for (int i = 0; i <= WINDOW_SIZE; i++) {
    Bgs[i] = Eigen::Vector3d::Zero();
    Bas[i] = Eigen::Vector3d::Zero();
  }

  for (int i = 0; i <= WINDOW_SIZE; i++) {
    CHECK(pre_integrations[i]);
    pre_integrations[i]->repropagate(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  }
  std::map<double, ImageFrame>::iterator frame_it;
  for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end();
       ++frame_it) {
    if(frame_it->second.pre_integration!=nullptr){
      frame_it->second.pre_integration->repropagate(Eigen::Vector3d::Zero(),
                                                  Eigen::Vector3d::Zero());
    }

  }
  td=0.0;
  slideWindow();
}
int Estimator::processImage(const ImageFeatureTrackerData &image,
                            const double header) {
  // /
  int feat_cout = f_manager->getFeatureCount();
  if (feat_cout < 4) {
    LOG(ERROR) << "It's spinning too fast";
    continue_track_feat_lost_.push_back(true);
  } else {
    continue_track_feat_lost_.push_back(false);
  }
  // LOG(INFO) << "delta_time" <<  header - last_time;
  const bool feature_margin_flag = f_manager->getFeatureCount()<15;
  if (f_manager->addFeatureCheckParallax(frame_count, image, td)) {
    marginalization_flag = MARGIN_OLD;
  } else {
    marginalization_flag = MARGIN_SECOND_NEW;
  }
  if (solver_flag != INITIAL) {
    if (update_zero_velocity_ &&
        update_zero_velocity_->AtState({})->IsZeroVelocity()) {
      is_velocity_updates_[frame_count] = true;
    } else {
      is_velocity_updates_[frame_count] = false;
    }

    // if (is_velocity_updates_[frame_count]) {
    //   marginalization_flag = MARGIN_SECOND_NEW;
    // }

  }
  // if(feature_margin_flag){
  //   marginalization_flag = MARGIN_OLD;
  // }
  std::stringstream info;
  info << "New image " << (marginalization_flag ? "Non-keyframe" : "Keyframe")
       << "(" << common::Time(common::FromSeconds(header)) << ")"
       << " coming, Adding feature points " << image.data->features.size()
       << "," << "number of feature: " << f_manager->getFeatureCount();
  VLOG(kGlogLevel) << info.str();
  
  images_[frame_count] = {image.data->time, image};
  Headers[frame_count] = header;
  ImageFrame imageframe(ToStruct(image), header);
  imageframe.pre_integration = tmp_pre_integration;
  all_image_frame.insert(std::make_pair(header, imageframe));
  tmp_pre_integration = new IntegrationBase{options_.imu_option, acc_0, gyr_0,
                                            Bas[frame_count], Bgs[frame_count]};

  if (estimate_extrinsic_ == 2) {
    LOG(INFO) << "calibrating extrinsic param, rotation movement is needed";
    if (frame_count != 0) {
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres =
          f_manager->getCorresponding(frame_count - 1, frame_count);
      Eigen::Matrix3d calib_ric;
      if (initial_ex_rotation.CalibrationExRotation(
              corres, pre_integrations[frame_count]->delta_q, calib_ric)) {
        LOG(WARNING) << "initial extrinsic rotation calib success";
        LOG(WARNING) << "initial extrinsic rotation: " << calib_ric;
        ric[0] = calib_ric;
        // RIC[0] = calib_ric;
        estimate_extrinsic_ = 1;
      }
    }
  }
  if (solver_flag == INITIAL) {
    // monocular + IMU initilization
    if (options_.use_cam_num == 1 && options_.use_imu) {
      if (frame_count == WINDOW_SIZE) {
        bool result = false;

        if (estimate_extrinsic_ != 2 && (header - initial_timestamp) > 0.1) {
          result = initialStructure();
          initial_timestamp = header;
        }
        if (result) {
          optimization();

          updateLatestStates();
          solver_flag = NON_LINEAR;
          slideWindow();
          LOG(INFO) << "Initialization finish!";
        } else
          slideWindow();
      }
    }

    // stereo + IMU initilization
    if (options_.use_cam_num == 2 && options_.use_imu) {
      LOG(INFO) << "Init with Stereo. frame count: " << frame_count
                <<" time: "<<common::Time(common::FromSeconds(header));
      bool pnp_state =
          f_manager->initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
      f_manager->triangulate(frame_count, Ps, Rs, tic, ric);
      init_pnp_states_.push_back(pnp_state);
      // LOG(INFO) << std::count(init_pnp_states_.begin(), init_pnp_states_.end(),
      //                         true);
      if(f_manager)
      if (frame_count == WINDOW_SIZE) {
        if (/*InitialImuIsValida(1)&&*/
            (std::count(init_pnp_states_.begin(), init_pnp_states_.end(),
                        true) == WINDOW_SIZE + 1)) {
          std::map<double, ImageFrame>::iterator frame_it;
          int i = 0;
          for (frame_it = all_image_frame.begin();
               frame_it != all_image_frame.end(); ++frame_it) {
            frame_it->second.R = Rs[i];
            frame_it->second.T = Ps[i];
            i++;
          }
          alignment_.solveGyroscopeBias(all_image_frame, Bgs);

          if (Bgs[WINDOW_SIZE].norm() <
                  options_.fail_detect_option.bgs_norm_max) {
            // if (!failureDetection()) {
            for (int i = 0; i <= WINDOW_SIZE; i++) {
              pre_integrations[i]->repropagate(Eigen::Vector3d::Zero(), Bgs[i]);
            }
            
            optimization_max_num_iterations_=10;
            optimizaion_cam_weight_ = FOCAL_LENGTH*2;
            //
            // std::set<int> removeIndex;
            // outliersRejection(removeIndex, 2);
            // f_manager->removeOutlier(removeIndex);
            // feature_tracker_->removeOutliers(removeIndex);
            //
            optimization();
            optimization_max_num_iterations_=1;
            optimizaion_cam_weight_ = FOCAL_LENGTH / 1.5;
            updateLatestStates();
            for (int i = 0; i <= WINDOW_SIZE; i++) {
              LOG(INFO) << "init  " << i << " bas :" << Bas[i].transpose()
                        << " bgs :" << Bgs[i].transpose() << " \nps "
                        << Ps[i].transpose() << " rs "
                        << Eigen::Quaterniond(Rs[i]);
            }
            if (Bas[WINDOW_SIZE].norm() > options_.init_bas_normal_max ||
                Bgs[WINDOW_SIZE].norm() >
                    options_.fail_detect_option.bgs_norm_max) {
              LOG(ERROR) << "init optimization bias err";
              return TrackState::LOST;
            }
            solver_flag = NON_LINEAR;
            slideWindow();
          
            LOG(INFO) << "Initialization finish!";
          } else {
            LOG(ERROR)<<"bias arr....,reinit...";
            // InitFailureRestart();
            return TrackState::LOST;
          }
        } else {
           return TrackState::LOST;
          // InitFailureRestart();
        }
        init_pnp_states_.erase(init_pnp_states_.begin());
      }
    }

    // stereo only initilization
    if (options_.use_cam_num == 2 && !options_.use_imu) {
      f_manager->initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
      f_manager->triangulate(frame_count, Ps, Rs, tic, ric);
      optimization();

      if (frame_count == WINDOW_SIZE) {
        optimization();
        updateLatestStates();
        solver_flag = NON_LINEAR;
        slideWindow();
        LOG(INFO) << "Initialization finish!";
      }
    }

    if (frame_count < WINDOW_SIZE) {
      frame_count++;
      int prev_frame = frame_count - 1;
      Ps[frame_count] = Ps[prev_frame];
      Vs[frame_count] = Vs[prev_frame];
      Rs[frame_count] = Rs[prev_frame];
      Bas[frame_count] = Bas[prev_frame];
      Bgs[frame_count] = Bgs[prev_frame];
    }

  } else {
    TicToc t_solve;
    // if (!options_.use_imu) {
    // if(!imageframe.pre_integration->IsValid()){
    //   LOG(WARNING)<<"IMU Avalibal.use pnp init..";
    //   f_manager->initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
    //   // sleep(4);
    // }
    f_manager->triangulate(frame_count, Ps, Rs, tic, ric);
    optimization();
    std::set<int> removeIndex;
    if (!is_velocity_updates_[frame_count]) {
      outliersRejection(removeIndex, options_.convin_used_num);
    }
    if (removeIndex.size() > size_t(f_manager->getFeatureCount() * 0.8) ||
        final_cost_ > 1e5) {
      LOG(ERROR) << "reproject erro fete num to big." << removeIndex.size()
                 << " " << f_manager->getFeatureCount() << " " << final_cost_;
      continue_track_feat_lost1_.push_back(true);
    }
    LOG(INFO) << "reproject erro fete num to big." << removeIndex.size() << " "
              << f_manager->getFeatureCount() << " " << final_cost_;
    f_manager->removeOutlier(removeIndex);
    feature_tracker_->removeOutliers(removeIndex);
    //
    predictPtsInNextFrame();
    VLOG(kGlogLevel) << "solver costs: " << t_solve.toc() << " ms"
                     << ",remove outlier: " << removeIndex.size();
    if (failureDetection()) {
      // LOG(ERROR) << "failure detection!";
      // failure_occur = 1;
      // clearState();
      // setParameter();
      // CHECK(false);
      LOG(ERROR) << "system reboot!";
      return TrackState::LOST;
    }
    // static int  frame = 0;
    // if (marginalization_flag == MARGIN_OLD) {
    //   LOG(INFO) << frame << "normal  bas :" << Bas[0].transpose()
    //             << " bgs :" << Bgs[0].transpose() << "ps " <<
    //             Ps[0].transpose()
    //             << " rs " << Eigen::Quaterniond(Rs[0]);
    //   frame ++;
    // }

    slideWindow();
    f_manager->removeFailures();
    // prepare output of VINS

    // CHECK(frame<20);
    key_poses.clear();
    for (int i = 0; i <= WINDOW_SIZE; i++) key_poses.push_back(Ps[i]);
  }

  last_R = Rs[WINDOW_SIZE];
  last_P = Ps[WINDOW_SIZE];
  last_R0 = Rs[0];
  last_P0 = Ps[0];
  updateLatestStates();

  return TrackState::TRACKING;
}
bool Estimator::InitialImuIsValida(int type) {
  std::map<double, ImageFrame>::iterator frame_it;
  Eigen::Vector3d sum_g = Eigen::Vector3d::Zero();
  for (frame_it = all_image_frame.begin(), ++frame_it;
       frame_it != all_image_frame.end(); ++frame_it) {
    double dt = frame_it->second.pre_integration->sum_dt;
    Eigen::Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
    sum_g += tmp_g;
  }
  Eigen::Vector3d aver_g;
  aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
  double var = 0;

  double max_var = 0;
  Eigen::Quaterniond rataion = Eigen::Quaterniond::Identity();
  double delta_yaw = 0;
  for (frame_it = all_image_frame.begin(), ++frame_it;
       frame_it != all_image_frame.end(); ++frame_it) {
    double dt = frame_it->second.pre_integration->sum_dt;
    Eigen::Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
    //
    delta_yaw = std::fmax(
        delta_yaw,
        abs(common::RadToDeg(transform::GetAngle(transform::Rigid3d::Rotation(
            frame_it->second.pre_integration->delta_q)))));
    //
    var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
    max_var = std::fmax(max_var,(tmp_g - aver_g).norm());
    // cout << "frame g " << tmp_g.transpose() << endl;
  }
  var = sqrt(var / ((int)all_image_frame.size() - 1));
  VLOG(kGlogLevel) << "IMU variation " << var;
  LOG(INFO) << "IMU variation " << var<<" max : "<<max_var;
  // delta_yaw = delta_yaw / ((int)all_image_frame.size() - 1);
  if (type != 0) {
    LOG(ERROR) << "IMU ration" << delta_yaw;
    if (delta_yaw > options_.init_rotation_th /*|| (var < 0.25 && var > 0.01)*/) {
      LOG(ERROR) << "IMU ratation not <1! " << delta_yaw;
      return false;
    }
    return true;
  }
  if (var < 0.25) {
    LOG(INFO) << "IMU excitation not enouth!";
    return false;
  }
  return true;
}
bool Estimator::initialStructure() {
  InitialImuIsValida() ;
  TicToc t_sfm;
  // check imu observibility
  LOG(INFO) << frame_count;
  // global sfm
  Eigen::Quaterniond Q[frame_count + 1];
  Eigen::Vector3d T[frame_count + 1];
  std::map<int, Eigen::Vector3d> sfm_tracked_points;
  std::vector<SFMFeature> sfm_f;
  for (auto &it_per_id : f_manager->feature) {
    int imu_j = it_per_id.start_frame - 1;
    SFMFeature tmp_feature;
    tmp_feature.state = false;
    tmp_feature.id = it_per_id.feature_id;
    for (auto &it_per_frame : it_per_id.feature_per_frame) {
      imu_j++;
      Eigen::Vector3d pts_j = it_per_frame.point;
      tmp_feature.observation.push_back(
          std::make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
    }
    sfm_f.push_back(tmp_feature);
  }
  Eigen::Matrix3d relative_R;
  Eigen::Vector3d relative_T;
  int l;
  if (!relativePose(relative_R, relative_T, l)) {
    LOG(INFO) << "Not enough features or parallax; Move device around";
    return false;
  }
  GlobalSFM sfm;
  if (!sfm.construct(frame_count + 1, Q, T, l, relative_R, relative_T, sfm_f,
                     sfm_tracked_points)) {
    VLOG(kGlogLevel) << "global SFM failed!";
    marginalization_flag = MARGIN_OLD;
    return false;
  }

  // solve pnp for all frame
  std::map<double, ImageFrame>::iterator frame_it;
  std::map<int, Eigen::Vector3d>::iterator it;
  frame_it = all_image_frame.begin();
  for (int i = 0; frame_it != all_image_frame.end(); ++frame_it) {
    // provide initial guess
    cv::Mat r, rvec, t, D, tmp_r;
    if ((frame_it->first) == Headers[i]) {
      frame_it->second.is_key_frame = true;
      frame_it->second.R = Q[i].toRotationMatrix() *
                           options_.calibrate_option.extric_camera_to_imu[0]
                               .rotation()
                               .conjugate();
      frame_it->second.T = T[i];
      i++;
      continue;
    }
    if ((frame_it->first) > Headers[i]) {
      i++;
    }
    Eigen::Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
    Eigen::Vector3d P_inital = -R_inital * T[i];
    cv::eigen2cv(R_inital, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_inital, t);

    frame_it->second.is_key_frame = false;
    std::vector<cv::Point3f> pts_3_vector;
    std::vector<cv::Point2f> pts_2_vector;
    for (auto &id_pts : frame_it->second.points) {
      int feature_id = id_pts.first;
      for (auto &i_p : id_pts.second) {
        it = sfm_tracked_points.find(feature_id);
        if (it != sfm_tracked_points.end()) {
          Eigen::Vector3d world_pts = it->second;
          cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
          pts_3_vector.push_back(pts_3);
          Eigen::Vector2d img_pts = i_p.second.head<2>();
          cv::Point2f pts_2(img_pts(0), img_pts(1));
          pts_2_vector.push_back(pts_2);
        }
      }
    }
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    if (pts_3_vector.size() < 6) {
      LOG(WARNING) << "pts_3_vector size " << pts_3_vector.size();
      VLOG(kGlogLevel) << "Not enough points for solve pnp !";
      return false;
    }
    if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1)) {
      VLOG(kGlogLevel) << "solve pnp fail!";
      return false;
    }
    cv::Rodrigues(rvec, r);
    Eigen::MatrixXd R_pnp, tmp_R_pnp;
    cv::cv2eigen(r, tmp_R_pnp);
    R_pnp = tmp_R_pnp.transpose();
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);
    T_pnp = R_pnp * (-T_pnp);
    frame_it->second.R =
        R_pnp * options_.calibrate_option.extric_camera_to_imu[0]
                    .rotation()
                    .conjugate();
    frame_it->second.T = T_pnp;
  }
  if (visualInitialAlign()) {
    return true;

  }

  else {
    LOG(INFO) << "misalign visual structure with IMU";
    return false;
  }
}

bool Estimator::visualInitialAlign() {
  TicToc t_g;
  Eigen::VectorXd x;
  // solve scale

  bool result = alignment_.VisualIMUAlignment(all_image_frame, Bgs, g, x);
  if (!result) {
    VLOG(kGlogLevel) << "solve g failed!";
    return false;
  }

  // change state
  for (int i = 0; i <= frame_count; i++) {
    Eigen::Matrix3d Ri = all_image_frame[Headers[i]].R;
    Eigen::Vector3d Pi = all_image_frame[Headers[i]].T;
    Ps[i] = Pi;
    Rs[i] = Ri;
    all_image_frame[Headers[i]].is_key_frame = true;
  }

  double s = (x.tail<1>())(0);
  for (int i = 0; i <= WINDOW_SIZE; i++) {
    pre_integrations[i]->repropagate(Eigen::Vector3d::Zero(), Bgs[i]);
  }
  for (int i = frame_count; i >= 0; i--) {
    const auto &tic =
        options_.calibrate_option.extric_camera_to_imu[0].translation();
    Ps[i] = s * Ps[i] - Rs[i] * tic - (s * Ps[0] - Rs[0] * tic);
  }

  int kv = -1;
  std::map<double, ImageFrame>::iterator frame_i;
  for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end();
       ++frame_i) {
    if (frame_i->second.is_key_frame) {
      kv++;
      Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
    }
  }

  Eigen::Matrix3d R0 = Utility::g2R(g);
  double yaw = Utility::R2ypr(R0 * Rs[0]).x();
  R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
  g = R0 * g;
  // Eigen::Matrix3d rot_diff = R0 * Rs[0].transpose();
  Eigen::Matrix3d rot_diff = R0;
  for (int i = 0; i <= frame_count; i++) {
    Ps[i] = rot_diff * Ps[i];
    Rs[i] = rot_diff * Rs[i];
    Vs[i] = rot_diff * Vs[i];
  }
  VLOG(kGlogLevel) << "g0     " << g.transpose();
  VLOG(kGlogLevel) << "my R0  " << Utility::R2ypr(Rs[0]).transpose();

  f_manager->clearDepth();
  f_manager->triangulate(frame_count, Ps, Rs, tic, ric);

  return true;
}

bool Estimator::relativePose(Eigen::Matrix3d &relative_R,
                             Eigen::Vector3d &relative_T, int &l) {
  // find previous frame which contians enough correspondance and parallex with
  // newest frame
  for (int i = 0; i < WINDOW_SIZE; i++) {
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres;
    corres = f_manager->getCorresponding(i, WINDOW_SIZE);
    LOG(INFO) << corres.size();
    if (corres.size() > 20) {
      double sum_parallax = 0;
      double average_parallax;
      for (int j = 0; j < int(corres.size()); j++) {
        Eigen::Vector2d pts_0(corres[j].first(0), corres[j].first(1));
        Eigen::Vector2d pts_1(corres[j].second(0), corres[j].second(1));
        double parallax = (pts_0 - pts_1).norm();
        sum_parallax = sum_parallax + parallax;
      }
      average_parallax = 1.0 * sum_parallax / int(corres.size());
      LOG(INFO) << average_parallax * 460;
      if (average_parallax * 460 > 10 &&
          m_estimator.solveRelativeRT(corres, relative_R, relative_T)) {
        l = i;
        VLOG(kGlogLevel)
            << "average_parallax " << average_parallax * 460 << " choose l" << l
            << " and newest frame to triangulate the whole structure ";
        return true;
      }
    }
  }
  return false;
}

void Estimator::vector2double() {
  for (int i = 0; i <= WINDOW_SIZE; i++) {
    para_Pose[i][0] = Ps[i].x();
    para_Pose[i][1] = Ps[i].y();
    para_Pose[i][2] = Ps[i].z();
    Eigen::Quaterniond q{Rs[i]};
    para_Pose[i][3] = q.x();
    para_Pose[i][4] = q.y();
    para_Pose[i][5] = q.z();
    para_Pose[i][6] = q.w();

    if (options_.use_imu) {
      para_SpeedBias[i][0] = Vs[i].x();
      para_SpeedBias[i][1] = Vs[i].y();
      para_SpeedBias[i][2] = Vs[i].z();
      // LOG(INFO)<<Vs[i].transpose();
      para_SpeedBias[i][3] = Bas[i].x();
      para_SpeedBias[i][4] = Bas[i].y();
      para_SpeedBias[i][5] = Bas[i].z();

      para_SpeedBias[i][6] = Bgs[i].x();
      para_SpeedBias[i][7] = Bgs[i].y();
      para_SpeedBias[i][8] = Bgs[i].z();
    }
  }

  // LOG(INFO) << Vs[WINDOW_SIZE].transpose();
  // LOG(INFO) << Bgs[WINDOW_SIZE].transpose();
  if (options_.use_odom) {
    para_Ex_Pose_Odom[0][0] = transform_imu_to_robot_.translation().x();
    para_Ex_Pose_Odom[0][1] = transform_imu_to_robot_.translation().y();
    para_Ex_Pose_Odom[0][2] = transform_imu_to_robot_.translation().z();
    para_Ex_Pose_Odom[0][3] = transform_imu_to_robot_.rotation().x();
    para_Ex_Pose_Odom[0][4] = transform_imu_to_robot_.rotation().y();
    para_Ex_Pose_Odom[0][5] = transform_imu_to_robot_.rotation().z();
    para_Ex_Pose_Odom[0][6] = transform_imu_to_robot_.rotation().w();
  }
  for (int i = 0; i < options_.use_cam_num; i++) {
    para_Ex_Pose[i][0] = tic[i].x();
    para_Ex_Pose[i][1] = tic[i].y();
    para_Ex_Pose[i][2] = tic[i].z();
    Eigen::Quaterniond q{ric[i]};
    para_Ex_Pose[i][3] = q.x();
    para_Ex_Pose[i][4] = q.y();
    para_Ex_Pose[i][5] = q.z();
    para_Ex_Pose[i][6] = q.w();
    // if (IsStereo()) break;
  }

  Eigen::VectorXd dep = f_manager->getDepthVector();
  for (int i = 0; i < f_manager->getFeatureCount(); i++)
    para_Feature[i][0] = dep(i);
  LOG_EVERY_N(INFO, 100) << "Td : " << std::to_string(td);
  para_Td[0][0] = td;
}

void Estimator::double2vector() {
  Eigen::Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
  Eigen::Vector3d origin_P0 = Ps[0];

  if (failure_occur) {
    origin_R0 = Utility::R2ypr(last_R0);
    origin_P0 = last_P0;
    failure_occur = 0;
  }
  std::stringstream info;
  if (options_.use_imu) {
    Eigen::Vector3d origin_R00 =
        Utility::R2ypr(Eigen::Quaterniond(para_Pose[0][6], para_Pose[0][3],
                                          para_Pose[0][4], para_Pose[0][5])
                           .toRotationMatrix());
    double y_diff = origin_R0.x() - origin_R00.x();

    // TODO
    Eigen::Matrix3d rot_diff = Utility::ypr2R(Eigen::Vector3d(y_diff, 0, 0));

    if (abs(abs(origin_R0.y()) - 90) < 1.0 ||
        abs(abs(origin_R00.y()) - 90) < 1.0) {
      VLOG(kGlogLevel) << "euler singular point!";
      rot_diff = Rs[0] * Eigen::Quaterniond(para_Pose[0][6], para_Pose[0][3],
                                            para_Pose[0][4], para_Pose[0][5])
                             .toRotationMatrix()
                             .transpose();
    }

    for (int i = 0; i <= WINDOW_SIZE; i++) {
      Rs[i] = rot_diff * Eigen::Quaterniond(para_Pose[i][6], para_Pose[i][3],
                                            para_Pose[i][4], para_Pose[i][5])
                             .normalized()
                             .toRotationMatrix();

      Ps[i] = rot_diff * Eigen::Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                         para_Pose[i][1] - para_Pose[0][1],
                                         para_Pose[i][2] - para_Pose[0][2]) +
              origin_P0;

      Vs[i] =
          rot_diff * Eigen::Vector3d(para_SpeedBias[i][0], para_SpeedBias[i][1],
                                     para_SpeedBias[i][2]);
      Bas[i] = Eigen::Vector3d(para_SpeedBias[i][3], para_SpeedBias[i][4],
                               para_SpeedBias[i][5]);

      Bgs[i] = Eigen::Vector3d(para_SpeedBias[i][6], para_SpeedBias[i][7],
                               para_SpeedBias[i][8]);
    }
    LOG_EVERY_N(INFO, 60) << "bas: " << Bas[WINDOW_SIZE].transpose()
                          << " bgs: " << Bgs[WINDOW_SIZE].transpose();
  } else {
    for (int i = 0; i <= WINDOW_SIZE; i++) {
      Rs[i] = Eigen::Quaterniond(para_Pose[i][6], para_Pose[i][3],
                                 para_Pose[i][4], para_Pose[i][5])
                  .normalized()
                  .toRotationMatrix();

      Ps[i] =
          Eigen::Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
    }
  }

  // cam_time_babg<<info.str()<<std::endl;
  if (options_.use_imu) {
    for (int i = 0; i < options_.use_cam_num; i++) {
      tic[i] = Eigen::Vector3d(para_Ex_Pose[i][0], para_Ex_Pose[i][1],
                               para_Ex_Pose[i][2]);
      ric[i] = Eigen::Quaterniond(para_Ex_Pose[i][6], para_Ex_Pose[i][3],
                                  para_Ex_Pose[i][4], para_Ex_Pose[i][5])
                   .normalized()
                   .toRotationMatrix();
    }
  }

  if (options_.use_odom) {
    transform_imu_to_robot_ = transform::Rigid3d(
        Eigen::Vector3d(para_Ex_Pose_Odom[0][0], para_Ex_Pose_Odom[0][1],
                        para_Ex_Pose_Odom[0][2]),
        Eigen::Quaterniond(para_Ex_Pose_Odom[0][6], para_Ex_Pose_Odom[0][3],
                           para_Ex_Pose_Odom[0][4], para_Ex_Pose_Odom[0][5]));
  }

  Eigen::VectorXd dep = f_manager->getDepthVector();
  for (int i = 0; i < f_manager->getFeatureCount(); i++)
    dep(i) = para_Feature[i][0];
  f_manager->setDepth(dep);

  if (options_.use_imu) {
    td = para_Td[0][0];
  }
}

bool Estimator::failureDetection() {
  if (restart) {
    restart = false;
    return true;
  }
  if (f_manager->last_track_num <
      options_.fail_detect_option.track_feat_lost_min_num) {
    failuer_track_lost_.push_back(true);
    LOG(WARNING) << " little feature " << f_manager->last_track_num;
  } else {
    failuer_track_lost_.push_back(false);
  }
  if (failuer_track_lost_.size() >
      size_t(options_.fail_detect_option.track_feat_lost_win_size)) {
    failuer_track_lost_.erase(failuer_track_lost_.begin());
  }
  if (std::count(failuer_track_lost_.begin(), failuer_track_lost_.end(),
                 true) >=
      options_.fail_detect_option.track_feat_lost_win_size) {
    LOG(ERROR) << " Feat lost! ";
    return true;
  }
  if (Bas[WINDOW_SIZE].norm() > options_.fail_detect_option.bas_norm_max) {
    LOG(ERROR) << " big IMU acc bias estimation " << Bas[WINDOW_SIZE].norm();
    return true;
  }
  if (Bgs[WINDOW_SIZE].norm() > options_.fail_detect_option.bgs_norm_max) {
    LOG(ERROR) << " big IMU gyr bias estimation " << Bgs[WINDOW_SIZE].norm();
    return true;
  }

  //

  double odo_distance = 1;
  if (options_.use_odom) {
    auto distance = odometry_factor_[WINDOW_SIZE]->GetObserveDistance();
    if (distance.has_value()) {
      odo_distance = distance.value();
    }
  }

  //
  failuer_zero_lost_.push_back((odo_distance < 0.0001));
  if (int(failuer_zero_lost_.size()) >
      options_.fail_detect_option.zero_odo_win_size) {
    failuer_zero_lost_.erase(failuer_zero_lost_.begin());
  }

  Eigen::Vector3d tmp_P = Ps[WINDOW_SIZE];
  //
  lost_last_poses_.push_back(
      transform::Rigid3d(tmp_P, Eigen::Quaterniond(Rs[WINDOW_SIZE])));
  //
  if (int(lost_last_poses_.size()) >
      options_.fail_detect_option.zero_odo_pose_size) {
    lost_last_poses_.erase(lost_last_poses_.begin());
  }

  //
  double z_distance = 0.0;
  double translation_distance = 0.0;
  double yaw_distance = 0.0;
  // if (options_.fail_detect_option.enable_odo_zero_lost_detect == 1) {
  for (size_t i = 1; i < lost_last_poses_.size(); i++) {
    const transform::Rigid3d delta_pose =
        lost_last_poses_[i - 1].inverse() * lost_last_poses_[i];
    translation_distance += delta_pose.translation().norm();
    z_distance += (delta_pose.translation().z());
    yaw_distance += (common::RadToDeg(transform::GetAngle(delta_pose)));
  }
  // }
  //
  //
  // if (final_cost_ > 1e5) {
  //   LOG(ERROR) << "final cost too big" << final_cost_;
  //   return true;
  // }

  while (continue_track_feat_lost_.size() > 40) {
    continue_track_feat_lost_.erase(continue_track_feat_lost_.begin());
  }
  while (continue_track_feat_lost1_.size() > 20) {
    continue_track_feat_lost1_.erase(continue_track_feat_lost1_.begin());
  }
  if (std::count(continue_track_feat_lost_.begin(),
                 continue_track_feat_lost_.end(), true) > 20 ||
      std::count(continue_track_feat_lost1_.begin(),
                 continue_track_feat_lost1_.end(), true) > 10) {
    LOG(ERROR) << "Continue track lost ...";
    return true;
  }

  failuer_zero_feat_lost_.push_back(is_velocity_updates_[frame_count]);
  // 

  if (int(failuer_zero_feat_lost_.size()) >
      options_.fail_detect_option.zero_odo_win_size) {
    failuer_zero_feat_lost_.erase(failuer_zero_feat_lost_.begin());
  }
  bool is_zero_velocity =false;
  // bool is_zero_velocity = (std::count(failuer_zero_feat_lost_.begin(),
  //                                     failuer_zero_feat_lost_.end(), true) ==
  //                          options_.fail_detect_option.zero_odo_win_size);
  // //
  //


  // LOG_IF(WARNING, is_zero_velocity) << "feat detect zero velocity.. ";
  //
  if (options_.fail_detect_option.enable_odo_zero_lost_detect == 1) {
    is_zero_velocity |=
        (std::count(failuer_zero_lost_.begin(), failuer_zero_lost_.end(),
                    true) == options_.fail_detect_option.zero_odo_win_size);
  }

  if (!is_zero_velocity) {
    lost_last_poses_.clear();
  }

  //
  //
  if (is_zero_velocity) {
    if (translation_distance >=
        options_.fail_detect_option.zero_translation_norm_max) {
      LOG(ERROR) << "Zero velocity translation detect: " << translation_distance
                 << " > "
                 << options_.fail_detect_option.zero_translation_norm_max;
      return true;
    }
    if (fabs(z_distance) >= options_.fail_detect_option.translation_z_max) {
      LOG(ERROR) << "Zero velocity z: " << z_distance << " > "
                 << options_.fail_detect_option.translation_z_max;

      return true;
    }
    if (fabs(yaw_distance) >= options_.fail_detect_option.zero_ratation_max) {
      LOG(ERROR) << "Zero velocity yaw: " << yaw_distance << " > "
                 << options_.fail_detect_option.zero_ratation_max;

      return true;
    }
  }

  //
  //
  //
  const double delta_translation = (tmp_P - last_P).norm();
  const double delta_z_translation = abs(tmp_P.z() - last_P.z());
  const double translation_threash_hold =
      options_.fail_detect_option.translation_norm_max;
  //

  if ((tmp_P - last_P).norm() > translation_threash_hold) {
    LOG(ERROR) << "Big translation !! " << (tmp_P - last_P).norm();
    return true;
  }
  //
  const double translation_z_threash_hold =
      options_.fail_detect_option.translation_z_max;
  //
  if (abs(tmp_P.z() - last_P.z()) > translation_z_threash_hold) {
    LOG(ERROR) << " Big z translation" << tmp_P.z() - last_P.z();
    return true;
  }

  Eigen::Matrix3d tmp_R = Rs[WINDOW_SIZE];
  Eigen::Matrix3d delta_R = tmp_R.transpose() * last_R;

  const double rotaion_threash_hold = options_.fail_detect_option.ratation_max;
  double delta_angle = common::RadToDeg(transform::GetAngle(
      transform::Rigid3d::Rotation(Eigen::Quaterniond(delta_R))));
  if (delta_angle > rotaion_threash_hold) {
    LOG(ERROR) << " Big delta_angle " << delta_angle;
    return true;
  }
  return false;
}

void Estimator::optimization() {
  TicToc t_whole, t_prepare;
  vector2double();

  ceres::Problem problem;
  ceres::LossFunction *loss_function;
  // loss_function = NULL;
  loss_function = new ceres::HuberLoss(1.0);
  ceres::ParameterBlockOrdering *ordering = new ceres::ParameterBlockOrdering();

  // loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
  // ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
  for (int i = 0; i < frame_count + 1; i++) {
    ceres::LocalParameterization *local_parameterization =
        new PoseLocalParameterization();
    ordering->AddElementToGroup(para_Pose[i], 1);
    problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
    if (options_.use_imu) {
      problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
      for (int j = 0; j < 6; j++) {
        problem.SetParameterLowerBound(para_SpeedBias[i], j + 3, -1);
        problem.SetParameterUpperBound(para_SpeedBias[i], j + 3, 1);
      }
      ordering->AddElementToGroup(para_SpeedBias[i], 1);
    }
  }
  if (!options_.use_imu) {
    problem.SetParameterBlockConstant(para_Pose[0]);
  }  
  // problem.SetParameterBlockConstant(para_Pose[0]);


  // is_velocity_updates_[frame_count] =true;
  problem.AddParameterBlock(para_Ex_Pose_Odom[0], SIZE_POSE,
                            new PoseLocalParameterization());
  // /
  ordering->AddElementToGroup(para_Ex_Pose_Odom[0], 1);
  // problem.SetParameterBlockConstant(para_Ex_Pose_Odom[0]);
  for (int i = 0; i < options_.use_cam_num; i++) {
    ceres::LocalParameterization *local_parameterization =
        new PoseLocalParameterization();

    problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE,
                              local_parameterization);

    ordering->AddElementToGroup(para_Ex_Pose[i], 1);
    if ((estimate_extrinsic_ && frame_count == WINDOW_SIZE &&
         Vs[0].norm() > 0.2) ||
        openExEstimation) {
      // ROS_INFO("estimate extinsic param");
      openExEstimation = 1;
    } else {
      // ROS_INFO("fix extinsic param");
      problem.SetParameterBlockConstant(para_Ex_Pose[i]);
    }
    if (!IsStereo()) break;
  }
  ordering->AddElementToGroup(para_Td[0], 1);
  problem.AddParameterBlock(para_Td[0], 1);
  problem.SetParameterLowerBound(para_Td[0], 0, -0.02);
  problem.SetParameterUpperBound(para_Td[0], 0,  0.02);
  //
  if (!options_.estimate_td || Vs[0].norm() < 0.2 || solver_flag == INITIAL) {
    problem.SetParameterBlockConstant(para_Td[0]);
  }
  LOG_IF(ERROR, fabs(para_Td[0][0]) > 0.020)
      << "Td estimate to large: " << para_Td[0][0];

  if (last_marginalization_info && last_marginalization_info->valid) {
    // construct new marginlization_factor
    MarginalizationFactor *marginalization_factor =
        new MarginalizationFactor(last_marginalization_info);
    problem.AddResidualBlock(marginalization_factor, NULL,
                             last_marginalization_parameter_blocks);
  }

  std::vector<ceres::ResidualBlockId> residual_block_id;
  if (options_.use_imu) {
    for (int i = 0; i < frame_count; i++) {
      int j = i + 1;
      // LOG(INFO)<<para_SpeedBias[j][0];
      // LOG(INFO)<<para_SpeedBias[j][1];
      // LOG(INFO)<<para_SpeedBias[j][2];
      // if (abs(Headers[i] - Headers[j]) > 4.0) {
      // }
      if (j == frame_count) {
        if (update_zero_velocity_) {
          if (is_velocity_updates_[j]) {
            //
            for (int k = 0; k < 7; k++) {
              para_Pose[j][k] = para_Pose[i][k];
            }
            update_zero_velocity_->AddToProblem(
                &problem, nullptr,
                std::array<double *, 3>{para_Pose[i], para_Pose[j],
                                        para_SpeedBias[i]});
          }
        }
      }
      if (options_.use_odom ) {
        odometry_factor_[j]->AddToProblem(
            &problem, nullptr,
            std::array<double *, 3>{para_Pose[i], para_Pose[j],
                                    para_Ex_Pose_Odom[0]});
      }

      //
      if (!pre_integrations[j]->IsValid()) {
        // problem.SetParameterBlockConstant(para_SpeedBias[i]);
        // problem.SetParameterBlockConstant(para_SpeedBias[j]);
        // problem.SetParameterBlockConstant(para_Ex_Pose[0]);
        // problem.SetParameterBlockConstant(para_Ex_Pose[1]);

        LOG(WARNING) << j << " Imu avalid..";
        continue;
      }

      IMUFactor *imu_factor = new IMUFactor(pre_integrations[j]);
      auto id = problem.AddResidualBlock(imu_factor, NULL, para_Pose[i],
                                         para_SpeedBias[i], para_Pose[j],
                                         para_SpeedBias[j]);
      residual_block_id.push_back(id);
    }
    // stringstream info;
    // info << "time: " << pre_integrations[frame_count - 1]->sum_dt
    //      << " DV: " << pre_integrations[frame_count - 1]->delta_v.transpose()
    //      << " DP: " << pre_integrations[frame_count - 1]->delta_p.transpose()
    //      << std::endl
    //      << " DR: " << pre_integrations[frame_count - 1]->delta_q;
    // LOG(INFO) << info.str();
    // cam_time_babg << info.str() << std::endl;
  }

  int f_m_cnt = 0;
  int feature_index = -1;
  std::stringstream info;
  const double cam_weight = optimizaion_cam_weight_;
  for (auto &it_per_id : f_manager->feature) {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (it_per_id.used_num < options_.convin_used_num) continue;

    ++feature_index;
    if(para_Feature[feature_index][0]<0)continue;
    info << 1.0/para_Feature[feature_index][0] << " ";
    int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

    Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;
    for (auto &it_per_frame : it_per_id.feature_per_frame) {
      imu_j++;

      if (imu_i != imu_j) {
        Eigen::Vector3d pts_j = it_per_frame.point;
        ProjectionTwoFrameOneCamFactor *f_td =
            new ProjectionTwoFrameOneCamFactor(
                pts_i, pts_j, it_per_id.feature_per_frame[0].velocity,
                it_per_frame.velocity, it_per_id.feature_per_frame[0].cur_td,
                it_per_frame.cur_td, cam_weight);
        auto id = problem.AddResidualBlock(
            f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j],
            para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);

        ordering->AddElementToGroup(para_Feature[feature_index], 0);
        residual_block_id.push_back(id);
      }

      if (IsStereo() && it_per_frame.is_stereo) {
        // LOG(INFO)<<"Use Stero optimizatin..";
        Eigen::Vector3d pts_j_right = it_per_frame.pointRight;
        if (imu_i != imu_j) {
          ProjectionTwoFrameTwoCamFactor *f =
              new ProjectionTwoFrameTwoCamFactor(
                  pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity,
                  it_per_frame.velocityRight,
                  it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
          problem.AddResidualBlock(f, loss_function, para_Pose[imu_i],
                                   para_Pose[imu_j], para_Ex_Pose[0],
                                   para_Ex_Pose[1], para_Feature[feature_index],
                                   para_Td[0]);
        } else {
          ProjectionOneFrameTwoCamFactor *f =
              new ProjectionOneFrameTwoCamFactor(
                  pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity,
                  it_per_frame.velocityRight,
                  it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
          problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0],
                                   para_Ex_Pose[1], para_Feature[feature_index],
                                   para_Td[0]);
        }
      }
      f_m_cnt++;
    }
  }

  VLOG(kGlogCostTimeLevel) << "visual measurement count: " << f_m_cnt;
  // printf("prepare for ceres: %f \n", t_prepare.toc());
  // LOG(INFO)<<info.str();
  ceres::Solver::Options options;
  options.linear_solver_ordering.reset(ordering);
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.num_threads = 1;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
  // options.dynamic_sparsity =true;
  options.use_explicit_schur_complement = true;
  // options.minimizer_progress_to_stdout = true;
  options.use_nonmonotonic_steps = true;

  // if (marginalization_flag == MARGIN_OLD)
  //   options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
  // else
  //   options.max_solver_time_in_seconds = SOLVER_TIME;
  options.max_num_iterations = optimization_max_num_iterations_;
  TicToc t_solver;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  VLOG(kGlogCeresLevel) <<summary.BriefReport();
  LOG_EVERY_N(INFO, 200) << "\n" << summary.FullReport();
  //
  final_cost_ = summary.final_cost;
  //
  // TicToc t_solver_ceres;
  // double cost;
  // double *residuals;
  // ceres::CRSMatrix priorJacobian_crs;
  // double** r;
  // // problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, &residuals,
  // //                  nullptr, &priorJacobian_crs);
  // //
  // for (auto id : residual_block_id) {
  //   problem.EvaluateResidualBlock(id, true, &cost, (double*)residuals,
  //                                 (double**)r);
  // }
  // printf("solver costs: %f \n", t_solver_ceres.toc());

  // static int count= 0;
  // CHECK(count++ <100);
  // std::stringstream info;
  auto tmp_Q = Eigen::Quaterniond(Rs[WINDOW_SIZE]);
  // info << std::setprecision(5) << std::fixed;
  // info << Headers[frame_count] << " cost_time " << t_solver.toc() << " T ";
  // info << Ps[WINDOW_SIZE].x() << " " << Ps[WINDOW_SIZE].y() << " "
  //      << Ps[WINDOW_SIZE].z() << " Q ";
  // info << tmp_Q.w() << " " << tmp_Q.x() << " " << tmp_Q.y() << " " <<
  // tmp_Q.z()
  //      << " ";
  // info << Vs[WINDOW_SIZE].x() << " " << Vs[WINDOW_SIZE].y() << " "
  //      << Vs[WINDOW_SIZE].z();
  // cam_time_babg << info.str() << std::endl;
  //
  // printf("solver costs: %f \n", t_solver.toc());

  // LOG(INFO)<<Ps[frame_count-1].transpose();
  // LOG(INFO)<<Ps[frame_count].transpose();
  double2vector();
  // printf("frame_count: %d \n", frame_count);
  // LOG(INFO)<<Ps[frame_count].transpose();
  if (frame_count < WINDOW_SIZE) return;

  TicToc t_whole_marginalization;
  if (marginalization_flag == MARGIN_OLD) {
    MarginalizationInfo *marginalization_info = new MarginalizationInfo();
    vector2double();

    if (last_marginalization_info && last_marginalization_info->valid) {
      std::vector<int> drop_set;
      for (int i = 0;
           i < static_cast<int>(last_marginalization_parameter_blocks.size());
           i++) {
        if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
            last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
          drop_set.push_back(i);
      }
      // construct new marginlization_factor
      MarginalizationFactor *marginalization_factor =
          new MarginalizationFactor(last_marginalization_info);
      ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
          marginalization_factor, NULL, last_marginalization_parameter_blocks,
          drop_set);
      marginalization_info->addResidualBlockInfo(residual_block_info);
    }
    // if (update_zero_velocity_) {
    //   if (is_velocity_updates_[1]) {
    //     ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
    //         update_zero_velocity_->CostFunction(), NULL,
    //         std::vector<double *>{para_Pose[0], para_Pose[1],
    //                               para_SpeedBias[0]},
    //         std::vector<int>{0,2});
    //     marginalization_info->addResidualBlockInfo(residual_block_info);
    //   }
    // }
    if (options_.use_odom) {
      ceres::CostFunction *cost_function = odometry_factor_[1]->CostFunction();
      if (cost_function) {
        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
            cost_function, NULL,
            std::vector<double *>{para_Pose[0], para_Pose[1],
                                  para_Ex_Pose_Odom[0]},
            std::vector<int>{0});
        marginalization_info->addResidualBlockInfo(residual_block_info);
      }
    }
    if (options_.use_imu) {
      if (pre_integrations[1]->IsValid()) {
        IMUFactor *imu_factor = new IMUFactor(pre_integrations[1]);
        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
            imu_factor, NULL,
            std::vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1],
                                  para_SpeedBias[1]},
            std::vector<int>{0, 1});
        marginalization_info->addResidualBlockInfo(residual_block_info);
      }
    }

    {
      int feature_index = -1;
      for (auto &it_per_id : f_manager->feature) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < options_.convin_used_num) continue;

        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        if (imu_i != 0) continue;

        Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame) {
          imu_j++;
          if (imu_i != imu_j) {
            Eigen::Vector3d pts_j = it_per_frame.point;
            ProjectionTwoFrameOneCamFactor *f_td =
                new ProjectionTwoFrameOneCamFactor(
                    pts_i, pts_j, it_per_id.feature_per_frame[0].velocity,
                    it_per_frame.velocity,
                    it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                    cam_weight);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                f_td, loss_function,
                std::vector<double *>{para_Pose[imu_i], para_Pose[imu_j],
                                      para_Ex_Pose[0],
                                      para_Feature[feature_index], para_Td[0]},
                std::vector<int>{0, 3});
            marginalization_info->addResidualBlockInfo(residual_block_info);
          }

          if (IsStereo() && it_per_frame.is_stereo) {
            Eigen::Vector3d pts_j_right = it_per_frame.pointRight;
            if (imu_i != imu_j) {
              ProjectionTwoFrameTwoCamFactor *f =
                  new ProjectionTwoFrameTwoCamFactor(
                      pts_i, pts_j_right,
                      it_per_id.feature_per_frame[0].velocity,
                      it_per_frame.velocityRight,
                      it_per_id.feature_per_frame[0].cur_td,
                      it_per_frame.cur_td);
              ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                  f, loss_function,
                  std::vector<double *>{
                      para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0],
                      para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                  std::vector<int>{0, 4});
              marginalization_info->addResidualBlockInfo(residual_block_info);
            } else {
              ProjectionOneFrameTwoCamFactor *f =
                  new ProjectionOneFrameTwoCamFactor(
                      pts_i, pts_j_right,
                      it_per_id.feature_per_frame[0].velocity,
                      it_per_frame.velocityRight,
                      it_per_id.feature_per_frame[0].cur_td,
                      it_per_frame.cur_td);
              ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                  f, loss_function,
                  std::vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1],
                                        para_Feature[feature_index],
                                        para_Td[0]},
                  std::vector<int>{2});
              marginalization_info->addResidualBlockInfo(residual_block_info);
            }
          }
        }
      }
    }

    TicToc t_pre_margin;
    marginalization_info->preMarginalize();
    VLOG(kGlogCostTimeLevel) << "pre marginalization " << t_pre_margin.toc();

    TicToc t_margin;
    marginalization_info->marginalize();
    VLOG(kGlogCostTimeLevel) << "marginalization " << t_margin.toc();
    // LOG(INFO)<<"marginalization "<<t_margin.toc()<<" ms ";

    std::unordered_map<long, double *> addr_shift;
    for (int i = 1; i <= WINDOW_SIZE; i++) {
      addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
      if (options_.use_imu) {
        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] =
            para_SpeedBias[i - 1];
      }
    }
    for (int i = 0; i < options_.use_cam_num; i++) {
      addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
    }
    addr_shift[reinterpret_cast<long>(para_Ex_Pose_Odom[0])] =
        para_Ex_Pose_Odom[0];
    addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

    std::vector<double *> parameter_blocks =
        marginalization_info->getParameterBlocks(addr_shift);

    if (last_marginalization_info) delete last_marginalization_info;
    last_marginalization_info = marginalization_info;
    last_marginalization_parameter_blocks = parameter_blocks;

  } else {
    if (last_marginalization_info &&
        std::count(std::begin(last_marginalization_parameter_blocks),
                   std::end(last_marginalization_parameter_blocks),
                   para_Pose[WINDOW_SIZE - 1])) {
      MarginalizationInfo *marginalization_info = new MarginalizationInfo();
      vector2double();
      if (last_marginalization_info && last_marginalization_info->valid) {
        std::vector<int> drop_set;
        for (int i = 0;
             i < static_cast<int>(last_marginalization_parameter_blocks.size());
             i++) {
          CHECK(last_marginalization_parameter_blocks[i] !=
                para_SpeedBias[WINDOW_SIZE - 1]);
          if (last_marginalization_parameter_blocks[i] ==
              para_Pose[WINDOW_SIZE - 1])
            drop_set.push_back(i);
        }
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor =
            new MarginalizationFactor(last_marginalization_info);
        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
            marginalization_factor, NULL, last_marginalization_parameter_blocks,
            drop_set);

        marginalization_info->addResidualBlockInfo(residual_block_info);
      }

      TicToc t_pre_margin;
      VLOG(kGlogCostTimeLevel) << "begin marginalization";
      marginalization_info->preMarginalize();
      VLOG(kGlogCostTimeLevel)
          << "end pre marginalization " << t_pre_margin.toc();

      TicToc t_margin;
      VLOG(kGlogCostTimeLevel) << "begin marginalization";
      marginalization_info->marginalize();
      VLOG(kGlogCostTimeLevel) << "end marginalization" << t_margin.toc();

      std::unordered_map<long, double *> addr_shift;
      for (int i = 0; i <= WINDOW_SIZE; i++) {
        if (i == WINDOW_SIZE - 1)
          continue;
        else if (i == WINDOW_SIZE) {
          addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
          if (options_.use_imu) {
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] =
                para_SpeedBias[i - 1];
          }

        } else {
          addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];

          if (options_.use_imu) {
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] =
                para_SpeedBias[i];
          }
        }
      }
      for (int i = 0; i < options_.use_cam_num; i++) {
        addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
      }
      addr_shift[reinterpret_cast<long>(para_Ex_Pose_Odom[0])] =
          para_Ex_Pose_Odom[0];
      addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

      std::vector<double *> parameter_blocks =
          marginalization_info->getParameterBlocks(addr_shift);
      if (last_marginalization_info) delete last_marginalization_info;
      last_marginalization_info = marginalization_info;
      last_marginalization_parameter_blocks = parameter_blocks;
    }
  }
  // printf("whole marginalization costs: %f \n",
  // t_whole_marginalization.toc()); printf("whole time for ceres: %f \n",
  // t_whole.toc());
}

void Estimator::slideWindow() {
  TicToc t_margin;
  if (marginalization_flag == MARGIN_OLD) {
    double t_0 = Headers[0];
    back_R0 = Rs[0];
    back_P0 = Ps[0];
    if (frame_count == WINDOW_SIZE) {
      for (int i = 0; i < WINDOW_SIZE; i++) {
        Headers[i] = Headers[i + 1];
        images_[i] = images_[i + 1];
        Rs[i].swap(Rs[i + 1]);
        Ps[i].swap(Ps[i + 1]);
        is_velocity_updates_[i] = is_velocity_updates_[i + 1];
        std::swap(odometry_factor_[i], odometry_factor_[i + 1]);
        if (options_.use_cam_num) {
          std::swap(pre_integrations[i], pre_integrations[i + 1]);

          dt_buf[i].swap(dt_buf[i + 1]);
          linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
          angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

          Vs[i].swap(Vs[i + 1]);
          Bas[i].swap(Bas[i + 1]);
          Bgs[i].swap(Bgs[i + 1]);
        }
      }
      Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
      images_[WINDOW_SIZE] = images_[WINDOW_SIZE - 1];
      Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
      Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];

      if (options_.use_cam_num) {
        Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
        Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
        Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];
        delete pre_integrations[WINDOW_SIZE];
        delete odometry_factor_[WINDOW_SIZE];
        odometry_factor_[WINDOW_SIZE] =
            new OdomFactor(options_.odom_factor_option, data_base_.get());
        pre_integrations[WINDOW_SIZE] =
            new IntegrationBase{options_.imu_option, acc_0, gyr_0,
                                Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

        dt_buf[WINDOW_SIZE].clear();
        linear_acceleration_buf[WINDOW_SIZE].clear();
        angular_velocity_buf[WINDOW_SIZE].clear();
      }

      if (true || solver_flag == INITIAL) {
        std::map<double, ImageFrame>::iterator it_0;
        // for(int i  =0;i< WINDOW_SIZE; i++){
        //   Bgs[i] = Eigen::Vector3d::Zero();
        // }
        it_0 = all_image_frame.find(t_0);
        if (it_0 != all_image_frame.end()) {
          if (it_0->second.pre_integration != nullptr) {
            delete it_0->second.pre_integration;
            it_0->second.pre_integration = nullptr;
          }
          all_image_frame.erase(all_image_frame.begin(), it_0);    
        }
  
      }
      slideWindowOld();
    }
  } else {
    if (frame_count == WINDOW_SIZE) {
      is_velocity_updates_[frame_count - 1] = is_velocity_updates_[frame_count];
      Headers[frame_count - 1] = Headers[frame_count];
      Ps[frame_count - 1] = Ps[frame_count];
      Rs[frame_count - 1] = Rs[frame_count];
      images_[frame_count - 1] = (images_[frame_count]);
      if (options_.use_cam_num) {
        for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++) {
          double tmp_dt = dt_buf[frame_count][i];
          Eigen::Vector3d tmp_linear_acceleration =
              linear_acceleration_buf[frame_count][i];
          Eigen::Vector3d tmp_angular_velocity =
              angular_velocity_buf[frame_count][i];

          pre_integrations[frame_count - 1]->push_back(
              tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

          dt_buf[frame_count - 1].push_back(tmp_dt);
          linear_acceleration_buf[frame_count - 1].push_back(
              tmp_linear_acceleration);
          angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
        }

        Vs[frame_count - 1] = Vs[frame_count];
        Bas[frame_count - 1] = Bas[frame_count];
        Bgs[frame_count - 1] = Bgs[frame_count];
        odometry_factor_[frame_count - 1]->Merge(
            *odometry_factor_[frame_count]);
        delete odometry_factor_[WINDOW_SIZE];
        odometry_factor_[WINDOW_SIZE] =
            new OdomFactor(options_.odom_factor_option, data_base_.get());
        delete pre_integrations[WINDOW_SIZE];
        pre_integrations[WINDOW_SIZE] =
            new IntegrationBase{options_.imu_option, acc_0, gyr_0,
                                Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

        dt_buf[WINDOW_SIZE].clear();
        linear_acceleration_buf[WINDOW_SIZE].clear();
        angular_velocity_buf[WINDOW_SIZE].clear();
      }
      slideWindowNew();
    }
  }
}

void Estimator::slideWindowNew() {
  sum_of_front++;
  f_manager->removeFront(frame_count);
}

void Estimator::slideWindowOld() {
  sum_of_back++;

  bool shift_depth = solver_flag == NON_LINEAR ? true : false;
  if (shift_depth) {
    Eigen::Matrix3d R0, R1;
    Eigen::Vector3d P0, P1;
    R0 = back_R0 * ric[0];
    R1 = Rs[0] * ric[0];
    P0 = back_P0 + back_R0 * tic[0];
    P1 = Ps[0] + Rs[0] * tic[0];
    f_manager->removeBackShiftDepth(R0, P0, R1, P1);
  } else
    f_manager->removeBack();
}

void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T) {
  T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = Rs[frame_count];
  T.block<3, 1>(0, 3) = Ps[frame_count];
}

void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T) {
  T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = Rs[index];
  T.block<3, 1>(0, 3) = Ps[index];
}

void Estimator::predictPtsInNextFrame() {
  // printf("predict pts in next frame\n");
  if (frame_count < 2) return;
  // predict next pose. Assume constant velocity motion
  Eigen::Matrix4d curT, prevT, nextT;
  getPoseInWorldFrame(curT);
  getPoseInWorldFrame(frame_count - 1, prevT);
  nextT = curT * (prevT.inverse() * curT);
  std::map<int, Eigen::Vector3d> predictPts;

  for (auto &it_per_id : f_manager->feature) {
    if (it_per_id.estimated_depth > 0) {
      int firstIndex = it_per_id.start_frame;
      int lastIndex =
          it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
      // printf("cur frame index  %d last frame index %d\n", frame_count,
      // lastIndex);
      if ((int)it_per_id.feature_per_frame.size() >= 2 &&
          lastIndex == frame_count) {
        double depth = it_per_id.estimated_depth;
        Eigen::Vector3d pts_j =
            ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
        Eigen::Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
        Eigen::Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() *
                                    (pts_w - nextT.block<3, 1>(0, 3));
        Eigen::Vector3d pts_cam = ric[0].transpose() * (pts_local - tic[0]);
        int ptsIndex = it_per_id.feature_id;
        predictPts[ptsIndex] = pts_cam;
      }
    }
  }
  feature_tracker_->setPrediction(predictPts);
  // printf("estimator output %d predict pts\n",(int)predictPts.size());
}

double Estimator::reprojectionError(Eigen::Matrix3d &Ri, Eigen::Vector3d &Pi,
                                    Eigen::Matrix3d &rici,
                                    Eigen::Vector3d &tici, Eigen::Matrix3d &Rj,
                                    Eigen::Vector3d &Pj, Eigen::Matrix3d &ricj,
                                    Eigen::Vector3d &ticj, double depth,
                                    Eigen::Vector3d &uvi,
                                    Eigen::Vector3d &uvj) {
  Eigen::Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
  Eigen::Vector3d pts_cj =
      ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
  Eigen::Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
  double rx = residual.x();
  double ry = residual.y();
  return sqrt(rx * rx + ry * ry);
}

void Estimator::outliersRejection(std::set<int> &removeIndex,const int convin_used_num) {
  // return;
  int feature_index = -1;
  for (auto &it_per_id : f_manager->feature) {
    double err = 0;
    int errCnt = 0;
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (it_per_id.used_num < convin_used_num) continue;
    feature_index++;
    int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
    Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;
    double depth = it_per_id.estimated_depth;
    for (auto &it_per_frame : it_per_id.feature_per_frame) {
      imu_j++;
      if (imu_i != imu_j) {
        Eigen::Vector3d pts_j = it_per_frame.point;
        double tmp_error =
            reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], Rs[imu_j],
                              Ps[imu_j], ric[0], tic[0], depth, pts_i, pts_j);
        err += tmp_error;
        errCnt++;
        // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
      }
        // need to rewrite projecton factor.........
        if (it_per_frame.is_stereo) {
          Eigen::Vector3d pts_j_right = it_per_frame.pointRight;
          if (imu_i != imu_j) {
            double tmp_error = reprojectionError(
                Rs[imu_i], Ps[imu_i], ric[0], tic[0], Rs[imu_j], Ps[imu_j],
                ric[1], tic[1], depth, pts_i, pts_j_right);
            err += tmp_error;
            errCnt++;
            // LOG(INFO)<<"tmp_error "<< FOCAL_LENGTH / 1.5 * tmp_error;
          } else {
            double tmp_error = reprojectionError(
                Rs[imu_i], Ps[imu_i], ric[0], tic[0], Rs[imu_j], Ps[imu_j],
                ric[1], tic[1], depth, pts_i, pts_j_right);
            err += tmp_error;
            errCnt++;
            // LOG(INFO)<<"tmp_error "<< FOCAL_LENGTH / 1.5 * tmp_error;
          }
        }
    }
    double ave_err = err / errCnt;
    if (ave_err * FOCAL_LENGTH > options_.optimazation_outliers_rejection_th ||
        depth < 0 ||
        depth > options_.rejection_points_depth_max_th) {
      removeIndex.insert(it_per_id.feature_id);
    }
  }
  // LOG(INFO)<<removeIndex.size();
}

void Estimator::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration,
                               Eigen::Vector3d angular_velocity) {
  // double dt = t - latest_time;
  // latest_time = t;
  // Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
  // Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) -
  // latest_Bg; latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
  // Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) -
  // g; Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1); latest_P =
  // latest_P + dt * latest_V + 0.5 * dt * dt * un_acc; latest_V = latest_V + dt
  // * un_acc; latest_acc_0 = linear_acceleration; latest_gyr_0 =
  // angular_velocity;
}

void Estimator::updateLatestStates() {
  // mPropagate.lock();
  // latest_time = Headers[frame_count] + td;
  // latest_P = Ps[frame_count];
  // latest_Q = Rs[frame_count];
  // latest_V = Vs[frame_count];
  // latest_Ba = Bas[frame_count];
  // latest_Bg = Bgs[frame_count];
  // latest_acc_0 = acc_0;
  // latest_gyr_0 = gyr_0;
  // mBuf.lock();
  // std::queue<std::pair<double, Eigen::Vector3d>> tmp_accBuf = accBuf;
  // // imu_extrapolator_->AddState(
  // //     latest_time,
  // //     ImuState{transform::Rigid3d(Ps[frame_count], Eigen::Quaterniond(
  // //     Rs[frame_count])),
  // //              Vs[frame_count], Bas[frame_count], Bgs[frame_count]});
  // // //
  // std::queue<std::pair<double, Eigen::Vector3d>> tmp_gyrBuf = gyrBuf;
  // mBuf.unlock();
  // while (!tmp_accBuf.empty()) {
  //   double t = tmp_accBuf.front().first;
  //   Eigen::Vector3d acc = tmp_accBuf.front().second;
  //   Eigen::Vector3d gyr = tmp_gyrBuf.front().second;
  //   fastPredictIMU(t, acc, gyr);
  //   tmp_accBuf.pop();
  //   tmp_gyrBuf.pop();
  // }
  // mPropagate.unlock();
}

//
std::unique_ptr<Estimator> TrackerFactory(const std::string &config_file) {
  EstimatorOption option;

  ParseYAMLOption(config_file, &option);
  return std::make_unique<Estimator>(option);
}

}  // namespace estimator
}  // namespace jarvis
