

#include "jarvis/estimator/estimator.h"

#include "ceres/tiny_solver.h"
#include "ceres/tiny_solver_autodiff_function.h"
#include "glog/logging.h"
#include "jarvis/common/time.h"
#include "jarvis/estimator/initial/initialization_stero_imu.h"
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
namespace {}
// ofstream cam_time_babg("/tmp/cam_time_babg.txt");
//
// Estimator(const EstimatorOption &options);

Estimator::Estimator(const EstimatorOption &options) : options_(options) {
  LOG(INFO) << options_.calibrate_option.extric_camera_to_imu[0];

  //
  f_manager = std::make_shared<FeatureManager>(options_.feature_manager_option);
  //

  data_base_ = std::make_unique<DataBase>(options_.data_base_lenth);

  //
  //
  initializer_ = std::make_unique<SteroImuInitialization>(
      SteroImuInitializationOption{
          WINDOW_SIZE, options_.imu_option,
          options_.calibrate_option.extric_camera_to_imu,
          options_.feature_manager_option},
      data_base_.get());
  //
  stereo_sample_ = std::make_unique<common::FixedRatioSampler>(
      options_.use_stereo_sample_ration);

  pose_predit_ = std::make_unique<PosePredit>();
  // options_.calibrate_option.extric_camera_to_imu[0] *
  // options_.calibrate_option.extric_camera_to_robot.inverse();

  // imu_extrapolator_ = std::make_unique<ImuExtrapolator>();

  feature_tracker_ =
      std::make_unique<FeatureTracker>(options_.feature_track_option);
  //
}

Estimator::~Estimator() {
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
    CHECK(feature_result.data->tracker_features_num.count(p.first));
    result.data->key_points.back().octave =
        feature_result.data->tracker_features_num[p.first];
  }

  // result.data->image =
  //     std::make_shared<cv::Mat>(feature_result.data->images[0].clone());
  return std::make_unique<TrackingData>(result);
}
}  // namespace
//
std::unique_ptr<TrackingData> Estimator::AddImageData(
    const sensor::ImageData &images) {
  TicToc add_image_data_cost;
  //
  ImageFeatureTrackerData featureFrame;
  std::map<int, int> track_num;
  //  FrameData::FeatureData featureFrame;
  common::Time cur_time = images.time + common::FromSeconds(estimator_td_);
  TrackState state = TrackState::INIT;
  if (slide_wondows_) {
    featureFrame = feature_tracker_->TrackImage(images.time, *images.image[0],
                                                cv::Mat(), &track_num);

    imu_state_ = pose_predit_->PreditDataBase(imu_state_, data_base_.get(),
                                              last_time_, images.time);

    FrameData frame_data = FrameData{std::make_shared<FrameData::Data>(
        FrameData::Data{images.time,
                        frame_id_,
                        imu_state_,
                        {{0, FrameData::FeatureData{featureFrame}}}})};

    frame_data = slide_wondows_->AddFeatureData(frame_data);
    imu_state_ = frame_data.data->imu_state;
    state = TrackState::TRACKING;
  } else {
    featureFrame = feature_tracker_->TrackImage(images.time, *images.image[0],
                                                *images.image[1], &track_num);
    auto init_result = initializer_->AddFeatureData(featureFrame);
    if (init_result) {
      SlideWindowOption option;
      option.extric_camera_to_imu =
          options_.calibrate_option.extric_camera_to_imu;
      //
      option.imu_option = options_.imu_option;
      slide_wondows_ = std::make_unique<SlideWindow>(option, data_base_.get(),
                                                     std::move(init_result));
      //
      imu_state_ = init_result->states.back();
    }
    state = TrackState::INIT;
  }
  last_time_ = cur_time;
  frame_id_++;
  auto tracking_data = ExtractKeyFrameMapPoints(*this, featureFrame);
  tracking_data->data->imu_state = imu_state_;
  tracking_data->status = state;
  tracking_data->data->image =
      std::make_shared<cv::Mat>(images.image[0]->clone());
  return tracking_data;
}
//
void Estimator::AddImuData(const sensor::ImuData &imu_data) {
  double d_time = common::ToSeconds(imu_data.time - common::FromUniversal(0));
  // if (update_zero_velocity_) {
  //   update_zero_velocity_->AddImu(imu_data);
  // }
  data_base_->AddImu(imu_data);
}

void Estimator::AddOdometryData(const sensor::OdometryData &odometry_data) {
  data_base_->AddOdometry(
      sensor::OdometryData{odometry_data.time, odometry_data.pose});
}

//
std::unique_ptr<Estimator> TrackerFactory(const std::string &config_file) {
  EstimatorOption option;

  ParseYAMLOption(config_file, &option);
  return std::make_unique<Estimator>(option);
}

}  // namespace estimator
}  // namespace jarvis
