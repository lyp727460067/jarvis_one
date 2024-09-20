

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
namespace estimator {
namespace {
std::array<int,3> KimageIndex{0, 2, 3};
}
// ofstream cam_time_babg("/tmp/cam_time_babg.txt");
//
// Estimator(const EstimatorOption &options);

Estimator::Estimator(const EstimatorOption &options) : options_(options) {
  data_base_ = std::make_unique<DataBase>(options_.data_base_lenth);
  for (int i = 0; i < options_.track_cam_num; i++) {
    LOG(INFO)
        << options_.feature_track_options[i].feature_detect_option.imag_size;
    feature_trackers_.emplace(
        i, std::make_unique<FeatureTracker>(options_.feature_track_options[i]));
  }
  if (options_.use_stero) {
    LOG(INFO)<<options_.stero_imu_init_option.imu_option.DebugInfo();
    initials_.emplace(0, std::make_unique<SteroImuInitialization>(
                             options_.stero_imu_init_option, data_base_.get()));
  } else {
    CHECK(false) << "not construct code.";
  }
  //
  //
  //
  // CHECK(false);
  stereo_sample_ = std::make_unique<common::FixedRatioSampler>(
      options_.use_stereo_sample_ration);

  pose_predit_ = std::make_unique<PosePredit>();
}

Estimator::~Estimator() {
  //
}
//
cv::KeyPoint EigenToCv(const Eigen::Vector2d &p) {
  return cv::KeyPoint(p.x(), p.y(), 2);
}

namespace {
//
// 填充地图点，cv::Keypoints
//
void FillFrameData(const int cam_id,
                   const ImageFeatureTrackerData &feature_result,
                   FrameData *frame_data) {
  auto &cam_fature = frame_data->data->features_datas[cam_id];
  // cv::imshow("tes",
  //            frame_data->data->features_datas[cam_id].features.data->images[0]);
  // cv::waitKey(0);
  if (cam_fature.key_points.empty()) {
    for (auto feat : cam_fature.features.data->features) {
      cam_fature.key_points[feat.first] =
          cv::KeyPoint(feat.second.camera_features[0].uv.x(),
                       feat.second.camera_features[0].uv.y(), 2);
      cam_fature.key_points[feat.first].octave =
          cam_fature.features.data->tracker_features_num[feat.first];
    }

  } else {
    for (auto &feature : cam_fature.key_points) {
      feature.second.octave =
          feature_result.data->tracker_features_num[feature.first];
    }
  }
}
}  // namespace
//
std::unique_ptr<TrackingData> Estimator::AddImageData(
    const sensor::ImageData &images) {
  TicToc add_image_data_cost;
  //

  std::map<int, int> track_num;
  //  FrameData::FeatureData featureFrame;
  common::Time cur_time = images.time + common::FromSeconds(estimator_td_);
  TrackState state = TrackState::INIT;
  FrameData frame_data;

  if (slide_wondows_) {
    imu_state_ = pose_predit_->PreditDataBase(imu_state_, data_base_.get(),
                                              last_time_, images.time);
    frame_data = FrameData{std::make_shared<FrameData::Data>(FrameData::Data{
        images.time,
        frame_id_,
        imu_state_,
    })};

    for (int i = 0; i < options_.track_cam_num; i++) {
      ImageFeatureTrackerData featureFrame = feature_trackers_[i]->TrackImage(
          images.time, images.image[KimageIndex[i]], cv::Mat(), &track_num);
      frame_data.data->features_datas.emplace(
          i, FrameData::FeatureData{featureFrame});
    }
    slide_wondows_->AddFeatureData(frame_data);
    imu_state_ = frame_data.data->imu_state;
    frame_data.status = TrackState::TRACKING;
  } else {
    ImageFeatureTrackerData featureFrame = feature_trackers_[0]->TrackImage(
        images.time, images.image[0], images.image[1], &track_num);
    auto init_result = initials_[0]->AddFeatureData(featureFrame);
    if (init_result) {
      slide_wondows_ = std::make_unique<SlideWindow>(
          options_.slide_windows_option, data_base_.get(),
          std::move(init_result));
      //
      imu_state_ = init_result->states.back();
    }
    frame_data = FrameData{std::make_shared<FrameData::Data>(
        FrameData::Data{images.time,
                        frame_id_,
                        imu_state_,
                        {{0, FrameData::FeatureData{featureFrame}}}})};

    frame_data.status = TrackState::INIT;
  }
  //
  last_time_ = cur_time;
  frame_id_++;
  for (auto &frame : frame_data.data->features_datas) {
    LOG(INFO)<<frame.first;
    FillFrameData(frame.first, frame.second.features, &frame_data);
  }
  data_base_->TrimData(cur_time);

  return std::make_unique<FrameData>(frame_data);
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
