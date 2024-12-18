#include "jarvis/trajectory_builder.h"

#include "camera_models/camera_models/camera.h"
#include "camera_models/camera_models/camera_factory.h"
#include "jarvis/estimator/estimator.h"
namespace jarvis {

using namespace mapping;

TrajectorBuilder::TrajectorBuilder(const TrajectorBuilderOption &option,
                                   CallBack call_back)
    : options_(option),
      tracker_(std::make_unique<estimator::Estimator>(options_.esti_option)),
      call_back_(call_back) {
  map_builder_ = std::make_unique<MappingBuilder>(options_.mapping_option);
  if (options_.mapping_option.enable_local_track) {
    tracker_->SetPriorFactorFunction(
        [&](const TrackingData &track_data)
            -> std::unique_ptr<transform::Rigid3d> {
          if (track_data.data) {
            return map_builder_->TrackLocalMap(track_data);
          }
          return nullptr;
        });
  }
}
//

void TrajectorBuilder::ReSet() {
  tracker_ = std::make_unique<estimator::Estimator>(options_.esti_option);
  map_builder_ = std::make_unique<MappingBuilder>(options_.mapping_option);
  if (options_.mapping_option.enable_local_track) {
    tracker_->SetPriorFactorFunction(
        [&](const TrackingData &track_data)
            -> std::unique_ptr<transform::Rigid3d> {
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
  if (tracking_data->front_data.status == 0) {
    LOG(ERROR) << "Lost ....restart ..";
    ReSet(); 
  }
  if (tracking_data->front_data.status == 2) {
    if (map_builder_) {
      if (tracking_data->slide_out_data.data) {
        map_builder_->AddTrackingData(trajector_,
                                      tracking_data->slide_out_data);
      }
    }
  } else {
    Relocation();
    ReComputeTrajectorId();
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
  return map_builder_->GetAllMapPoints();
}

std::map<KeyFrameId, transform::TimestampedTransform>
TrajectorBuilder::GetKeyFrameGlobalPose() {
  return map_builder_->GetAllKeyFramePose();
}
//
std::vector<Eigen::Vector3d> TrajectorBuilder::GetLocalMapPoints() {
  if (map_builder_->GetLocalMapTrack() == nullptr) return {};
  return map_builder_->GetLocalMapTrack()->GetMapPoints();
}
//
std::vector<transform::Rigid3d> TrajectorBuilder::GetLocalKeyFramePose() {
  if (map_builder_->GetLocalMapTrack() == nullptr) return {};
  return map_builder_->GetLocalMapTrack()->GetKfPose();
}

//
transform::Rigid3d TrajectorBuilder::GetLocalToGlobalTransform() {
  return transform::Rigid3d::Identity();
  //  return map_builder_->GetLocalToGlobleTransfrom();
}

TrajectorBuilder::~TrajectorBuilder() {}
}  // namespace jarvis