#include "jarvis/object/object_interface.h"

#include "jarvis/object/object_process.h"
#include "jarvis/transform/transform.h"

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
    // std::vector<ObejectDataPose> 符合要求的在世界坐标系下的arUco码
    auto mark_with_poses = wap_pose_object_detect_->AddImage(
        common::FromUniversal(time), std::make_shared<cv::Mat>(image),
        cam_pose);
    LOG(INFO) << imu_to_cam;
    std::vector<ObjectImageResult> object_result;
    for (const auto &mark : mark_with_poses) {
      // if (pose_temp.empty()) break;
      // 锚定arUco码
      object_process_.AddLandMark(
          mark,
          {KeyFrameId{0, time}, transform::TimestampedTransform{
                                    common::FromUniversal(time), cam_pose}});

      // 将ObejectDataPose 改为 ObjectImageResult格式
      ObjectImageResult mark_project = object_image_pross_->ProjectObject(
          cam_pose, SimplePoseToObject(mark, cam_pose));
      object_result.push_back(mark_project);
    }

    // 获取所有锚定arUco码信息
    if (!object_process_.GetObjectData(0).empty()) {
      auto global_objects = object_process_.GetObjectData(0);
      for (const auto &object : global_objects) {
        object_result.push_back(
            object_image_pross_->ProjectObject(cam_pose, object.second, true));
      }
    }
    return object_result;
  }

  std::vector<ObjectImageResult> ComputeError(const uint64_t &time,
                                              const cv::Mat &image,
                                              const transform::Rigid3d &pose,
                                              const transform::Rigid3d &imu_to_cam) {
      transform::Rigid3d cam_pose = pose * imu_to_cam;
      // std::vector<ObejectDataPose> 符合要求的在世界坐标系下的arUco码
      auto mark_with_poses = wap_pose_object_detect_->AddImage(
          common::FromUniversal(time), std::make_shared<cv::Mat>(image),
          cam_pose);
      // LOG(INFO) << imu_to_cam;
      std::vector<ObjectImageResult> object_result;
      for (const auto &mark : mark_with_poses) {
          // 将ObejectDataPose 改为 ObjectImageResult格式
          ObjectImageResult mark_project = object_image_pross_->ProjectObject(
              cam_pose, SimplePoseToObject(mark, cam_pose));
          object_result.push_back(mark_project);
      }

      // 获取所有锚定arUco码信息
      if (!object_process_.GetObjectData(0).empty()) {
          auto global_objects = object_process_.GetObjectData(0);
          for (const auto &object : global_objects) {
              object_result.push_back(
                  object_image_pross_->ProjectObject(cam_pose, object.second, true));
          }
      }

      // 根据id匹配arUco码
      std::map<int, std::vector<object::ObjectImageResult>> same_marks;
      if (!object_result.empty()) {
          for (const auto &result : object_result) {
              same_marks[result.id].push_back(result);
          }
      }

      // 计算误差
      if (!same_marks.empty()) {
          for (const auto &pair : same_marks) {
              if (pair.second.size() < 2) continue;

              auto delta_pose = pair.second[0].global_pose_cam.inverse() *
                                pair.second[1].global_pose_cam;
              float error_dis = delta_pose.translation().norm();
              float error_angle = common::RadToDeg(transform::GetAngle(delta_pose));

              if (max_error_.find(pair.first) != max_error_.end()) {
                  if (error_dis > max_error_[pair.first].first)
                      max_error_[pair.first].first = error_dis;
                  if (error_angle > max_error_[pair.first].second)
                      max_error_[pair.first].second = error_angle;
              } else {
                  max_error_[pair.first] = std::make_pair(error_dis, error_angle);
              }

              std::cout << "arUco id: " << pair.first << ", dis: " << error_dis << " / "
                        << max_error_[pair.first].first << ", angle: " << error_angle
                        << " / " << max_error_[pair.first].second << std::endl;
          }
      }

      return object_result;
  }

  jarvis::object::VslamFactoryResult GetFinalResult() {
      jarvis::object::VslamFactoryResult factory_result;

      std::cout << "max error list" << std::endl;
      int i = 0;
      for (auto it = max_error_.begin(); it != max_error_.end() && i < 4;
           it++, i++) {
          factory_result.err_dis[i] = it->second.first;
          factory_result.err_angle[i] = it->second.second;
          std::cout << it->first << ": " << it->second.first << ", " << it->second.second
                    << std::endl;
      }

      if (!object_process_.GetObjectData(0).empty()) {
          auto global_objects = object_process_.GetObjectData(0);
          if (global_objects.size() != 4) {
              // 第一圈锚定码数量不为4,设置狀态为255
              factory_result.status = 255;
              std::cout << "arUco num is not equal to 4: " << global_objects.size() << std::endl;
          }
          else if(max_error_.size() != 4) {
              // 第一圈观察数量不为4,设置狀态为254
              factory_result.status = 255;
              std::cout << "arUco num is not equal to 4: " << global_objects.size() << std::endl;
          }
          else {
              // 一切正常,设置狀态为0
              factory_result.status = 0;
          }
      } else {
          // 异常狀态,没有标码
          factory_result.status = 253;
          LOG(ERROR) << "object process empty";
      }

      return factory_result;
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
  std::map<int, std::pair<float, float>> max_error_; // <dis, angle>

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
std::vector<ObjectImageResult> ObjectInterface::ComputeError(
    const uint64_t &time, const cv::Mat &image,
    const transform::Rigid3d &pose,const transform::Rigid3d &imu_to_cam) {
  return object_impl_->ComputeError(time, image, pose, imu_to_cam);
}
//
object::VslamFactoryResult ObjectInterface::GetFinalResult() {
    return object_impl_->GetFinalResult();
}
//
ObjectInterface::~ObjectInterface() {}
}  // namespace object
}  // namespace jarvis