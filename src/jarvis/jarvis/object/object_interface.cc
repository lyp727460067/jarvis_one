#include "jarvis/object/object_interface.h"

#include "jarvis/object/object_process.h"

////

namespace jarvis {
namespace object {
//

class ObjectInterface::ObjectImpl {
 public:
  ObjectImpl(ObjectInterface *parent) {
    // map_builder_ =
    //     dynamic_cast<internal::MappingBuilder *>(parent->map_builder_);
    // if (map_builder_) {
    //   object_image_pross_ = std::make_unique<ObjectImageProcess>(
    //       ObjectImageProcessOption{}, map_builder_->GetCameBase());

    //   wap_pose_object_detect_ =
    //       std::make_unique<WapObjectDetect>(std::make_unique<ObjectDetect>(
    //           ObjectDetectOption{}, map_builder_->GetCameBase()));
    // } else {
    //   LOG(ERROR) << "map_builder_ is nullptr. ";
    // }
    //
    //
  }
  std::vector<ObjectImageResult> Detect(const uint64_t &time,
                                        const cv::Mat &image,
                                        const transform::Rigid3d &pose) {
    if (map_builder_ == nullptr) return {};
    auto mark_with_poses = wap_pose_object_detect_->AddImage(
        common::FromUniversal(time), std::make_shared<cv::Mat>(image), pose);

    std::vector<ObjectImageResult> object_result;
    for (const auto &mark : mark_with_poses) {
      if (pose_temp.empty()) break;
      object_process_.AddLandMark(mark, *std::prev(pose_temp.end()));
      //
      ObjectImageResult mark_project = object_image_pross_->ProjectObject(
          pose, SimplePoseToObject(mark, pose));
      object_result.push_back(mark_project);
    }

    if (!object_process_.GetObjectData(0).empty()) {
      auto global_objects = object_process_.GetObjectData(0);
      for (const auto object : global_objects) {
        object_result.push_back(
            object_image_pross_->ProjectObject(pose, object.second, true));
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
ObjectInterface::ObjectInterface(MapBuilderInterface *map_builder)
    : map_builder_(map_builder),
      object_impl_(std::make_unique<ObjectImpl>(this)) {}
//
//
void ObjectInterface::UpdateGloblePose() {
  // 有可能内部不跑后端
  if (!map_builder_) return;
  // local_to_globle_transform_ = map_builder_->GetLocalToGlobleTransfrom();
  object_impl_->UpdateGloblePose();
}
//
std::vector<ObjectImageResult> ObjectInterface::Detect(
    const uint64_t &time, const cv::Mat &image,
    const transform::Rigid3d &pose) {
  if (!map_builder_) return {};
  return object_impl_->Detect(time, image, local_to_globle_transform_ * pose);
}
//
//

ObjectInterface::~ObjectInterface() {}
}  // namespace object
}