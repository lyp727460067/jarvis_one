

#include "jarvis/estimator/estimator.h"

#include "ceres/tiny_solver.h"
#include "ceres/tiny_solver_autodiff_function.h"
#include "glog/logging.h"
#include "jarvis/common/time.h"
#include "jarvis/estimator/initial/initialization_stero_imu.h"
#include "jarvis/estimator/parameters.h"
#include "jarvis/option_parse.h"
namespace jarvis {
namespace estimator {
namespace {
std::array<int, 3> KimageIndex{0, 2, 3};
}

Estimator::Estimator(const EstimatorOption &options) : options_(options) {
  data_base_ = std::make_unique<DataBase>(options_.data_base_lenth);
  for (size_t i = 0; i < options_.track_sequence.size(); i++) {
    feature_trackers_.emplace(
        i, std::make_unique<FeatureTracker>(options_.feature_track_options[i]));
    LOG(INFO)<<options_.feature_track_options[i].feature_detect_option.imag_size;
    LOG(INFO)<<options_.feature_track_options[i].feature_detect_option.grid_size;
    LOG(INFO)<<options_.feature_track_options[i].mask.size();
  }
  if (options_.use_stero) {
    LOG(INFO) << options_.stero_imu_init_option.imu_option.DebugInfo();
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
  failure_detect_ =
      std::make_unique<FailureDetect>(options_.fail_detect_option);
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

  //  FrameData::FeatureData featureFrame;
  common::Time cur_time = images.time + common::FromSeconds(estimator_td_);
  TrackState state = TrackState::INIT;
  FrameData frame_data;

  if (slide_wondows_) {
    imu_state_ = pose_predit_->PreditDataBase(imu_state_, data_base_.get(),
                                              last_time_, images.time);
    LOG(INFO)<<imu_state_;
    frame_data = FrameData{std::make_shared<FrameData::Data>(FrameData::Data{
        images.time,
        frame_id_,
        imu_state_,
    })};

    for (size_t i = 0; i < options_.track_sequence.size(); i++) {
      //
      // if (options_.feature_track_options[i].pyramid_image.empty()) {
      //   for (size_t j = 0;
      //        j < options_.feature_track_options[i].pyramid_image.size(); j++)
      //     options_.feature_track_options[i].pyramid_image[j]->Build(cv::Mat());
      // }
      //
      CHECK(!images.image[options_.track_sequence[i][0]].empty());
      ImageFeatureTrackerData featureFrame = feature_trackers_[i]->TrackImage(
          images.time, images.image[options_.track_sequence[i][0]]);
      frame_data.data->features_datas.emplace(
          i, FrameData::FeatureData{featureFrame});
    }
    std::unique_ptr<SlideWindowResult> slie_result =
        slide_wondows_->AddFeatureData(frame_data);
    //
    frame_data = slie_result->frame_data;
    imu_state_ = frame_data.data->imu_state;
    frame_data.status = TrackState::TRACKING;
    auto rejection_outliers = slide_wondows_->RejectionOutliers();
    for (size_t i = 0; i < options_.track_sequence.size(); i++) {
      feature_trackers_[i]->RemoveOutliers(rejection_outliers[i]);
    }
    if (failure_detect_->Detect(*slie_result)) {
      frame_data.status = TrackState::LOST;
    }

  } else {
    //
    // if (options_.feature_track_options[0].pyramid_image.empty()) {
    //   for (size_t j = 0;
    //        j < options_.feature_track_options[0].pyramid_image.size(); j++)
    //     options_.feature_track_options[0].pyramid_image[j]->Build(cv::Mat());
    // }

    ImageFeatureTrackerData featureFrame = feature_trackers_[0]->TrackImage(
        images.time, images.image[0], images.image[1]);
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
    FillFrameData(frame.first, frame.second.features, &frame_data);
  }
  //

  data_base_->TrimData(cur_time);

  return std::make_unique<FrameData>(frame_data);
}
//
void Estimator::PredictPtsInNextFrame(const FrameData &frame_data,
                                      const transform::Rigid3d &predit_pose) {
  // std::map<int, Eigen::Vector3d> predictPts;

  // for (auto &it_per_id : f_manager->feature) {
  //   if (it_per_id.estimated_depth > 0) {
  //     int firstIndex = it_per_id.start_frame;
  //     int lastIndex =
  //         it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
  //     // printf("cur frame index  %d last frame index %d\n", frame_count,
  //     // lastIndex);
  //     if ((int)it_per_id.feature_per_frame.size() >= 2 &&
  //         lastIndex == frame_count) {
  //       double depth = it_per_id.estimated_depth;
  //       Eigen::Vector3d pts_j =
  //           ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
  //       Eigen::Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
  //       Eigen::Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() *
  //                                   (pts_w - nextT.block<3, 1>(0, 3));
  //       Eigen::Vector3d pts_cam = ric[0].transpose() * (pts_local - tic[0]);
  //       int ptsIndex = it_per_id.feature_id;
  //       predictPts[ptsIndex] = pts_cam;
  //     }
  //   }
  // }
  // feature_tracker_->setPrediction(predictPts);
  // printf("estimator output %d predict pts\n",(int)predictPts.size());
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

EstimatorOption ParseEstimatorOption(const std::string &config_file) {
  EstimatorOption option;

  ParseYAMLOption(config_file, &option);
  return option;
}

// std::unique_ptr<Estimator> TrackerFactory(const std::string &config_file) {

//   return std::make_unique<Estimator>(option);
// }

}  // namespace estimator
}  // namespace jarvis
