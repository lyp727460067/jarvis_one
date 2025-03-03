

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

Estimator::Estimator(const EstimatorOption &options)
    : options_(options), thread_pool_(options.thread_pool) {
  data_base_ = std::make_unique<DataBase>(options_.data_base_lenth);
  for (size_t i = 0; i < options_.track_sequence.size(); i++) {
    feature_trackers_.emplace(
        i, std::make_unique<FeatureTracker>(options_.feature_track_options[i]));
  }
  if (options_.use_stero) {
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
  when_done_task_ = std::make_unique<common::Task>();
  options_.slide_windows_option.thread_pool = options.thread_pool;
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

std::unique_ptr<EstimatorResult> Estimator::AddImageData(
    const sensor::ImageData &images) {
  TicToc add_image_data_cost;
  //

  //  FrameData::FeatureData featureFrame;
  common::Time cur_time = images.time + common::FromSeconds(estimator_td_);
  TrackState state = TrackState::INIT;
  FrameData frame_data;
  EstimatorResult result;
  if (slide_wondows_) {
      TicToc t_t;
    imu_state_ = pose_predit_->PreditDataBase(imu_state_, data_base_.get(),
                                              last_time_, cur_time);
    imu_state_.time = cur_time;
    //
    VLOG(kGlogCostTimeLevel) << "predit costs" << t_t.toc() << " ms";
    frame_data = FrameData{std::make_shared<FrameData::Data>(FrameData::Data{
        images.time,
        frame_id_,
        imu_state_,
    })};
    
    frame_data.data->images  = images;
    TicToc track_t_t;

    // 光流跟踪,使用thread_pool管理,不同相机的跟踪多线程进行
    for (size_t i = 0; i < options_.track_sequence.size(); i++) {
      CHECK(!images.image[options_.track_sequence[i][0]].empty());
      // if (options_.track_sequence[i].size() == 2 &&  stereo_sample_->Pulse()
      // ) {
      //   ImageFeatureTrackerData featureFrame =
      //   feature_trackers_[i]->TrackImage(
      //       images.time, images.image[options_.track_sequence[i][0]],
      //       images.image[options_.track_sequence[i][1]]);
      //   frame_data.data->features_datas.emplace(
      //       i, FrameData::FeatureData{featureFrame});
      //   LOG(INFO)<<"use stereo ..";
      // } else {
      frame_data.data->features_datas[i];
      auto track_task = std::make_unique<common::Task>();
      const int index =  i;
      track_task->SetWorkItem([&, index]() {
        //
        const auto predict_points =
            slide_wondows_->PredictNextFrame(imu_state_.Pose(), index);
        feature_trackers_[index]->SetPrediction(predict_points );
        ImageFeatureTrackerData featureFrame = feature_trackers_[index]->TrackImage(
            images.time, images.image[options_.track_sequence[index][0]]);
        frame_data.data->features_datas[index] =
            FrameData::FeatureData{featureFrame};
      });
      auto track_task_handle = thread_pool_->Schedule(std::move(track_task));
      when_done_task_->AddDependency(track_task_handle);

      // }
    }
    std::mutex mutex;
    std::condition_variable condtion;
    bool match_finish = false;
    when_done_task_->SetWorkItem([&] {
      std::lock_guard<std::mutex> lock(mutex);
      match_finish = true;
      condtion.notify_all();
    });
    thread_pool_->Schedule(std::move(when_done_task_));
    {
      std::unique_lock<std::mutex> locker(mutex);
      condtion.wait(locker, [&]() { return match_finish; });
    }
    when_done_task_ = std::make_unique<common::Task>();
    //
 

    VLOG(kGlogCostTimeLevel) << "track costs " << track_t_t.toc() << " ms";

    TicToc slide_t_t;
    std::unique_ptr<SlideWindowResult> slie_result =
        slide_wondows_->AddFeatureData(frame_data);
    //

    VLOG(kGlogCostTimeLevel) << "side costs " << slide_t_t.toc() << " ms";
    frame_data = slie_result->frame_data;
    imu_state_ = frame_data.data->imu_state;
    if (init_delay_state_num_ < 10) {
      init_delay_state_num_++;
      frame_data.status = TrackState::INIT;
    } else {
      frame_data.status = TrackState::TRACKING;
    }
    //
    result.slide_out_data = slie_result->slide_out_data;
    //
    TicToc other_t_t;
    auto rejection_outliers = slide_wondows_->RejectionOutliers();
    for (size_t i = 0; i < options_.track_sequence.size(); i++) {
      feature_trackers_[i]->RemoveOutliers(rejection_outliers[i]);
    }
    // LOG(INFO)<<slie_result->frame_data.data->extric_camera_to_imu.size();
    for (size_t i = 0;
         i < slie_result->frame_data.data->extric_camera_to_imu.size(); i++) {
      transform::Rigid3d &ext =
          slie_result->frame_data.data->extric_camera_to_imu[i];
      //
      std::stringstream info;
      info <<"Cam: "<<i<< " imucham:"<<transform::Rot2ypr(
                  options_.slide_windows_option.extric_camera_to_imu[i]
                      .rotation()
                      .toRotationMatrix())
                  .transpose();

      info << " op:"<<transform::Rot2ypr(ext.rotation().toRotationMatrix()).transpose();
      LOG_EVERY_N(INFO, 10) <<  info.str();
      //
      if (abs(options_.slide_windows_option.extric_camera_to_imu[i]
                  .translation()
                  .norm() -
              ext.translation().norm()) > 0.5) {


        LOG(ERROR) <<"camera "<<i<<  " opti ex error,lost." << ext;
        frame_data.status = TrackState::LOST;
      }
    }
    // if (lost_num_test_++ > 1000) {
    //   frame_data.status = TrackState::LOST;
    // }
    if (failure_detect_->Detect(*slie_result)) {
      frame_data.status = TrackState::LOST;
    }
    VLOG(kGlogCostTimeLevel) << "other costs " << other_t_t.toc() << " ms";
  } else {
    ImageFeatureTrackerData featureFrame = feature_trackers_[0]->TrackImage(
        images.time, images.image[0], images.image[1],true);
    auto init_result = initials_[0]->AddFeatureData(featureFrame);




    if (init_result) {
      slide_wondows_ = std::make_unique<SlideWindow>(
          options_.slide_windows_option, data_base_.get(),
          std::move(init_result),prior_factor_);
      //
      imu_state_ = init_result->states.back();
    }
    frame_data = FrameData{std::make_shared<FrameData::Data>(
        FrameData::Data{images.time,
                        frame_id_,
                        imu_state_,
                        {{0, FrameData::FeatureData{featureFrame}}}})};

    frame_data.status = TrackState::INIT;
    frame_data.data->images  = images;
  }
  //
  TicToc transform_t_t;
  last_time_ = cur_time;
  frame_id_++;
  for (auto &frame : frame_data.data->features_datas) {
    FillFrameData(frame.first, frame.second.features, &frame_data);
  }
  //

  data_base_->TrimData(cur_time);
  VLOG(kGlogCostTimeLevel) << "transform costs " << transform_t_t.toc()
                           << " ms";
  result.front_data = frame_data;
  // if (result.slide_out_data.data) {
  //   result.slide_out_data.data->extric_camera_to_imu =
  //       options_.slide_windows_option.extric_camera_to_imu;
  // }

  // result.front_data.data->images =  images;
  return std::make_unique<EstimatorResult>(result);
  // return std::make_unique<FrameData>(frame_data);
}
//


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

}  // namespace estimator
}  // namespace jarvis
