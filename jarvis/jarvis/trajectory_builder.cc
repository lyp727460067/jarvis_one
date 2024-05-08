#include "camera_models/camera_models/camera.h"
#include "camera_models/camera_models/camera_factory.h"
#include "jarvis/trajectory_builder.h"
#include "jarvis/estimator/estimator.h"
namespace jarvis {

// namespace{
// class  CameraModules : public CameraBase {
//  public:
//   CameraModules(const std::string &config_file) {
//     cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
//     int pn = config_file.find_last_of('/');
//     std::string configPath = config_file.substr(0, pn);
//     std::string cam0Calib;
//     fsSettings["cam0_calib"] >> cam0Calib;
//     std::string cam0Path = configPath + "/" + cam0Calib;
//     camera_ =
//         camera_models::CameraFactory::instance()->generateCameraFromYamlFile(
//             cam0Path);
//   }
//   std::vector<Eigen::Vector3d> UndistortPointsNormal(
//       const std::vector<cv::KeyPoint> &key_points) const {
//     std::vector<Eigen::Vector3d> result;
//     for (const auto &points : key_points) {
//       Eigen::Vector3d b;
//       camera_->liftProjective(Eigen::Vector2d{points.pt.x, points.pt.y}, b);
//       result.push_back(b / b.z());
//     }
//     return result;
//   }

//   Eigen::Vector2d Project(const Eigen::Vector3d &point) const {
//     Eigen::Vector2d result;
//     camera_->spaceToPlane(point, result);
//     return result;
//   }

//   //
//  private:
//  camera_models::CameraPtr camera_;
// };

// }

TrajectorBuilder::TrajectorBuilder(const std::string &config,
                                   CallBack call_back)
    : tracker_(estimator::TrackerFactory(config)),
      call_back_(std::move(call_back)) {
  //  cv::FileStorage fsSettings(config, cv::FileStorage::READ);
  //  int pn = config.find_last_of('/');
  //  std::string configPath = config.substr(0, pn);
  //  std::string map_builder_file;
  //  fsSettings["map_builder_option"] >> map_builder_file;
  //  YAML::Node map_builder_paras = YAML::LoadFile(configPath +"/"+
  //  map_builder_file); UrdfOption urdf_option; urdf_option.camera_num = 1;
  //  CameraOption camera_option;
  //  int image_width;
  //  int image_height;
  //  fsSettings["image_width"] >> image_width;
  //  fsSettings["image_height"] >> image_height;
  //  camera_option.resolution.x() = image_width;
  //  camera_option.resolution.y() = image_height;
  //  urdf_option.camera_options.push_back(camera_option);
  //  const MappingBuilderOption map_builder_option =
  //      ParseYAMLOption(map_builder_paras, urdf_option);

  //  map_builder_ =
  //      std::make_unique<MetaBoundsVIO::mapping::internal::MappingBuilder>(
  //          map_builder_option, std::make_unique<CameraModules>(config));
  //  map_builder_ = CreateMapBuilder(config);
}
//
void TrajectorBuilder::AddImageData(const sensor::ImageData &images) {
  auto tracking_data = tracker_->AddImageData(images);
  if (tracking_data != nullptr && tracking_data->tracking_result) {
    if (!tracking_data->tracking_result->data->tracking_map_points.empty()) {
      // map_builder_->AddTrackingData(0, *tracking_data->tracking_result);
      if (call_back_) {
        call_back_(*tracking_data->tracking_result);
      }
    }
  } else if (tracking_data != nullptr) {
    TrackingData data{
        std::make_shared<TrackingData::Data>(
            TrackingData::Data{tracking_data->feature_result.time,
                               tracking_data->feature_result.pose,
                               {},
                               tracking_data->feature_result.key_points,
                               tracking_data->feature_result.images,
                               {},
                               nullptr,
                               {0}})};
    if (call_back_) {
      call_back_(data);
    }
  };
}
//
void TrajectorBuilder::AddImuData(const   sensor::ImuData &imu_data) {
  tracker_->AddImuData(imu_data);
}

TrajectorBuilder::~TrajectorBuilder() {}

std::vector<Eigen::Vector3d> TrajectorBuilder::GetMapPoints() {
  //  const auto all_map_points = map_builder_->GetAllMapPoints();
  //  std::vector<Eigen::Vector3d> map_points;
  //  for (const auto point : all_map_points) {
  //     map_points.push_back(point.data.global_pose);
  //  }
  // LOG(INFO) << "All map points size: " << map_points.size();
  //  return map_points;
  return {};
}

std::map<KeyFrameId, transform::TimestampedTransform>
TrajectorBuilder::GetKeyFrameGlobalPose() {
  return {};
  //  return map_builder_->GetAllKeyFramePose();
}
//
std::vector<std::pair<KeyFrameId, KeyFrameId>>
TrajectorBuilder::ConstraintId() {
  //  std::vector<std::pair<KeyFrameId, KeyFrameId>> result;
  //  auto constraints = map_builder_->GetConstraints();
  //  for (const auto &constraint : constraints) {
  //     result.emplace_back(constraint.kf_id_i, constraint.kf_id_j);
  //  }
  //  return result;
  return {};
}
//
transform::Rigid3d TrajectorBuilder::GetLocalToGlobalTransform() {
  return transform::Rigid3d::Identity();
  //  return map_builder_->GetLocalToGlobleTransfrom();
}
}  // namespace jarvis