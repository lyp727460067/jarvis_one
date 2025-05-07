#include "jarvis/trajectory_builder.h"

#include "camera_models/camera_models/camera.h"
#include "camera_models/camera_models/camera_factory.h"
#include "jarvis/estimator/estimator.h"
namespace jarvis {

using namespace mapping;

TrajectorBuilder::TrajectorBuilder(const TrajectorBuilderOption &option,
                                   CallBack call_back)
    : options_(option), call_back_(call_back) {
  //
  thread_pool_ =
      std::make_unique<common::ThreadPool>(options_.esti_option.thread_num);
  //
  options_.esti_option.thread_pool = thread_pool_.get();
  options_.mapping_option.local_map_track_option.thread_pool =
      thread_pool_.get();
  tracker_ = std::make_unique<estimator::Estimator>(options_.esti_option);
  if (option.mapping_option.enable_loop_closure) {
    voc_ = std::make_unique<dbow::Vocabulary>(
        dbow::GetVocabulary(0, option.mapping_option.vocabulary_filebrif));
  }
  // if (option.mapping_option.enable_loop_closure) {
  //   CHECK(option.mapping_option.construct_use_des_match)
  //       << "Enable loop closure must set mapping.yaml "
  //          "construct_use_des_match=1";
  // }

  map_builder_ =
      std::make_unique<MappingBuilder>(options_.mapping_option, voc_.get());
  if (options_.mapping_option.enable_local_track) {
    tracker_->SetPriorFactorFunction(
        [&](const TrackingData &track_data)
            -> std::shared_ptr<LocalMapMatchResult> {
          if (track_data.data) {
            return map_builder_->TrackLocalMap(track_data);
          }
          return nullptr;
        });
  }
}
//

void TrajectorBuilder::ReSet(bool f) {
  tracker_ = std::make_unique<estimator::Estimator>(options_.esti_option);
  //等上了后端的时候map_builder_就不需要重新启动了
  if (!f) {
    map_builder_ =
        std::make_unique<MappingBuilder>(options_.mapping_option, voc_.get());
  } else {
    map_builder_->ResetActiveLocalMap(trajector_);
    trajector_++;
  }
  if (options_.mapping_option.enable_local_track) {
    tracker_->SetPriorFactorFunction(
        [&](const TrackingData &track_data)
            -> std::shared_ptr<LocalMapMatchResult> {
          if (track_data.data) {
            return map_builder_->TrackLocalMap(track_data);
          }
          return nullptr;
        });
  }
}
void TrajectorBuilder::AddImageData(const sensor::ImageData &images) {
  auto tracking_data = tracker_->AddImageData(images);
  
  if (call_back_) {
    call_back_(tracking_data->front_data);
  }
  if (estimator_state_ == 2 && tracking_data->front_data.status == 0) {
    Relocation();
    ReComputeTrajectorId();
  }

  estimator_state_ = tracking_data->front_data.status;
  if (tracking_data->front_data.status == 0) {
    LOG(ERROR) << "Lost ....restart ..";
    ReSet(map_builder_!=nullptr);
   
  }
  if (tracking_data->front_data.status == 2) {
    if (map_builder_) {
      if (tracking_data->slide_out_data.data) {
        map_builder_->AddTrackingData(trajector_,
                                      tracking_data->slide_out_data);
      }
    }
  }
}
//
void TrajectorBuilder::AddImuData(const sensor::ImuData &imu_data) {
  tracker_->AddImuData(imu_data);
  if (map_builder_) {
    map_builder_->AddImuData(imu_data);
  }
}
void TrajectorBuilder::AddOdometryData(
    const sensor::OdometryData &odometry_data) {
  tracker_->AddOdometryData(odometry_data);
  if (map_builder_) {
    map_builder_->AddOdometryData(odometry_data);
  }
}
//
//

void TrajectorBuilder::AddFixData(const sensor::FixedFramePoseData &fix_data) {
  if (map_builder_) {
    map_builder_->AddFixData(fix_data);
  }
}
//

std::vector<Eigen::Vector3d> TrajectorBuilder::GetMapPoints() {
  if (map_builder_) {
    return map_builder_->GetAllMapPoints();
  }
  return {};
}

std::map<KeyFrameId, transform::TimestampedTransform>
TrajectorBuilder::GetKeyFrameGlobalPose() {
  return map_builder_->GetAllKeyFramePose();
}
//
std::vector<Eigen::Vector3d> TrajectorBuilder::GetLocalMapPoints() {
  if (map_builder_->GetLocalMap() == nullptr) return {};
  std::vector<Eigen::Vector3d> result;
  const auto &all_map_points = map_builder_->GetLocalMap()->AllMapPoints();
  transform::Rigid3d local_map_pose = map_builder_->GetLocalMap()->LocalPose();
  for (const auto &mp_point : all_map_points) {
    result.push_back(local_map_pose * mp_point.data.data->pos);
  }
  return result;
}
//
std::vector<transform::Rigid3d> TrajectorBuilder::GetLocalKeyFramePose() {
  if (map_builder_->GetLocalMap() == nullptr) return {};
  std::vector<transform::Rigid3d> result;

  const auto &all_kf_re_pose =
      map_builder_->GetLocalMap()->AllKeyFrameRefPose();

  transform::Rigid3d local_map_pose = map_builder_->GetLocalMap()->LocalPose();
  for (const auto &re_pose : all_kf_re_pose) {
    result.push_back(local_map_pose * re_pose.second);
  }
  return result;
}

//
transform::Rigid3d TrajectorBuilder::GetLocalToGlobalTransform() {
  return transform::Rigid3d::Identity();
  //  return map_builder_->GetLocalToGlobleTransfrom();
}

TrajectorBuilder::~TrajectorBuilder() {}
}  // namespace jarvis