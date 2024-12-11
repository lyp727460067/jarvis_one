#include "jarvis/object/object_interface.h"

#include "jarvis/object/object_process.h"

////

namespace jarvis {
namespace object {
//

class ObjectInterface::ObjectImpl {
 public:
  ObjectImpl(const camera_models::CameraPtr came_base,
             ObjectInterface *parent) {
    if (parent->map_builder_) {
      // map_builder_ =
          // dynamic_cast<internal::MappingBuilder *>(parent->map_builder_);
    }

    object_image_pross_ = std::make_unique<ObjectImageProcess>(
        ObjectImageProcessOption{}, came_base);

    wap_pose_object_detect_ = std::make_unique<WapObjectDetect>(
        std::make_unique<ObjectDetect>(ObjectDetectOption{}, came_base));
    //
  }
  std::vector<ObjectImageResult> Detect(const uint64_t &time,
                                        const cv::Mat &image,
                                        const transform::Rigid3d &pose,
                                        const transform::Rigid3d &imu_to_cam) {
    transform::Rigid3d cam_pose = pose * imu_to_cam;
    auto mark_with_poses = wap_pose_object_detect_->AddImage(
        common::FromUniversal(time), std::make_shared<cv::Mat>(image),
        cam_pose);
    std::vector<ObjectImageResult> object_result;
    for (const auto &mark : mark_with_poses) {
      // if (pose_temp.empty()) break;
      object_process_.AddLandMark(
          mark,
          {KeyFrameId{0, time}, transform::TimestampedTransform{
                                    common::FromUniversal(time), cam_pose}});
      //
      ObjectImageResult mark_project = object_image_pross_->ProjectObject(
          cam_pose, SimplePoseToObject(mark, cam_pose));
      object_result.push_back(mark_project);
    }

    if (!object_process_.GetObjectData(0).empty()) {
      auto global_objects = object_process_.GetObjectData(0);
      for (const auto &object : global_objects) {
        object_result.push_back(
            object_image_pross_->ProjectObject(cam_pose, object.second, true));
      }
    }
    return object_result;
  }
  void UpdateGloblePose() {
    if (map_builder_ == nullptr) return;
    // pose_temp = map_builder_->GetAllKeyFramePose();
  }

 private:
  std::unique_ptr<ObjectImageProcess> object_image_pross_;
  std::unique_ptr<WapObjectDetect> wap_pose_object_detect_;
  ObjectProcess object_process_;
  //
  std::map<KeyFrameId, transform::TimestampedTransform> pose_temp;
  MapBuilderInterface *map_builder_;
  std::vector<ObejectDataPose> online_mark_with_poses_;

  //
};

//
ObjectInterface::ObjectInterface(const camera_models::CameraPtr came_base,
                                 MapBuilderInterface *map_builder)
    : map_builder_(map_builder),
      object_impl_(std::make_unique<ObjectImpl>(came_base, this)) {}

//
void ObjectInterface::UpdateGloblePose() {
  if (!map_builder_) return;
  // local_to_globle_transform_ = map_builder_->GetLocalToGlobleTransfrom();
  object_impl_->UpdateGloblePose();
}
//
std::vector<ObjectImageResult> ObjectInterface::Detect(
    const uint64_t &time, const cv::Mat &image,
    const transform::Rigid3d &pose,const transform::Rigid3d &imu_to_cam) {
  return object_impl_->Detect(time, image, pose, imu_to_cam);
}
//
//

ObjectInterface::~ObjectInterface() {}
}  // namespace object
}  // namespace jarvis