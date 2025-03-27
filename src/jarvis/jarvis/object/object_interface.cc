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
    //  LOG(ERROR)<< "imu pose in object: " << pose << std::endl;  
    transform::Rigid3d cam_pose = pose * imu_to_cam;
    // std::vector<ObejectDataPose> 符合要求的在世界坐标系下的arUco码
    auto mark_with_poses = wap_pose_object_detect_->AddImage(
        common::FromUniversal(time), std::make_shared<cv::Mat>(image),
        cam_pose);
    // LOG(INFO) << imu_to_cam;
    // std::cout << "imu_to_cam: " << std::endl << imu_to_cam << std::endl;
    // Eigen::Quaterniond q_raw = imu_to_cam.rotation();
    // Eigen::Vector3d t_raw = imu_to_cam.translation();
    // Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(1.0 / 180.0 * 3.14159, Eigen::Vector3d::UnitZ()));
    // Eigen::Matrix3d rotation_matrix = q_raw.toRotationMatrix() * yawAngle.toRotationMatrix();

    // std::cout << "before rotation: " << std::endl << "rotation: " << q_raw.toRotationMatrix()
    //           << std::endl << "tanslation: " << t_raw << std::endl;
    // std::cout << "after rotation: " << std::endl << "rotation: " << rotation_matrix
    //           << std::endl << "tanslation: " << t_raw << std::endl;


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
      // LOG(ERROR)<< "imu pose in object: " << pose << std::endl;
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
                //   if (error_dis > max_error_[pair.first].first || error_angle > max_error_[pair.first].second){
                //     std::cout << "error change" << std::endl;
                //     std::cout << "vio pose: " << pose << std::endl;
                //     std::cout << "imu to cam: " << imu_to_cam << std::endl;
                //     std::cout << "cam pose: " << cam_pose << std::endl;
                //     std::cout << "cur pose: " << pair.second[0].global_pose_cam << std::endl;
                //     std::cout << "mark pose: " << pair.second[1].global_pose_cam << std::endl;
                //   }

                  if (error_dis > max_error_[pair.first].first)
                      max_error_[pair.first].first = error_dis;
                  if (error_angle > max_error_[pair.first].second)
                      max_error_[pair.first].second = error_angle;
              } else {
                  max_error_[pair.first] = std::make_pair(error_dis, error_angle);
                  LOG(INFO) << "add detect: " << pair.first;
                  // std::cout << "error change" << std::endl;
                  // std::cout << "vio pose: " << pose << std::endl;
                  // std::cout << "imu to cam: " << imu_to_cam << std::endl;
                  // std::cout << "cam pose: " << cam_pose << std::endl;
                  // std::cout << "cur pose: " << pair.second[0].global_pose_cam << std::endl;
                  // std::cout << "mark pose: " << pair.second[1].global_pose_cam << std::endl;
              }

              // std::cout << "arUco id: " << pair.first << ", dis: " << error_dis << " / "
              //           << max_error_[pair.first].first << ", angle: " << error_angle
              //           << " / " << max_error_[pair.first].second << std::endl;
          }
      }

    //   std::cout << "error" << std::endl;
    //   for(auto it = max_error_.begin(); it != max_error_.end(); it++){
    //     std::cout << it->first << ": " << it->second.first << " / " << it->second.second << std::endl;
    //   }

      return object_result;
  }
//
//   VslamFactoryResult GetFinalResult() {
//       VslamFactoryResult factory_result;

//       std::cout << "max error list" << std::endl;
//       int i = 0;
//       for (auto it = max_error_.begin(); it != max_error_.end() && i < 4;
//            it++, i++) {
//           factory_result.err_dis[i] = it->second.first;
//           factory_result.err_angle[i] = it->second.second;
//           std::cout << it->first << ": " << it->second.first << ", " << it->second.second
//                     << std::endl;
//       }

//       float dis_thr = 0.2;
//       float angle_thr = 5.0;
//       if (!object_process_.GetObjectData(0).empty()) {
//           auto global_objects = object_process_.GetObjectData(0);
//           if (global_objects.size() != 4) {
//               // 第一圈锚定码数量不为4,设置狀态为255
//               factory_result.status = 255;
//               LOG(ERROR) << "arUco num is not equal to 4: " << global_objects.size();
//           } else if (max_error_.size() != 4) {
//               // 第二圈观察数量不为4,设置狀态为254
//               factory_result.status = 254;
//               LOG(ERROR) << "arUco num in the second round is not equal to 4: " << max_error_.size();
//           } else {
//               // 一切正常,判断是否符合要求,0为合格,1为不合格
//               float max_dis = .0;
//               float max_angle = .0;
//               for (int i = 0; i < 4; i++) {
//                   max_dis = (max_dis > factory_result.err_dis[i]) ? max_dis : factory_result.err_dis[i];
//                   max_angle = (max_angle > factory_result.err_dis[i]) ? max_angle : factory_result.err_angle[i];
//               }

//               if (max_dis > dis_thr || max_angle > angle_thr)
//                   factory_result.status = 1;
//               else
//                   factory_result.status = 0;
//           }
//       } else {
//           // 异常狀态,没有标码
//           factory_result.status = 253;
//           LOG(ERROR) << "object process empty";
//       }

//       return factory_result;
//   }
//

  void GetFinalResult(uint8_t &status, float (&err_dis)[4], float (&err_angle)[4],
                      float dis_avg_thr, float dis_max_thr, float angle_avg_thr, float angle_max_thr) {

      LOG(INFO) << "max error list";
      int i = 0;
      for (int i = 0; i < 4; i++) {
          err_dis[i] = 0;
          err_angle[i] = 0;
      }
      for (auto it = max_error_.begin(); it != max_error_.end() && i < 4;
           it++, i++) {
          err_dis[it->first] = it->second.first;
          err_angle[it->first] = it->second.second;
          LOG(INFO) << it->first << ": " << it->second.first << ", " << it->second.second;
      }

      float dis_thr = 0.2;
      float angle_thr = 5.0;
      if (!object_process_.GetObjectData(0).empty()) {
          auto global_objects = object_process_.GetObjectData(0);
          if (global_objects.size() != 4) {
              // 第一圈锚定码数量不为4,设置狀态为255
              status = 255;
              LOG(ERROR) << "arUco num is not equal to 4: " << global_objects.size();
          } else if (max_error_.size() != 4) {
              // 第二圈观察数量不为4,设置狀态为254
              status = 254;
              LOG(ERROR) << "arUco num in the second round is not equal to 4: " << max_error_.size();
          } else {
              // 一切正常,判断是否符合要求,0为合格,1为不合格
              float max_dis = .0;
              float max_angle = .0;
              float avg_dis = .0;
              float avg_angle = .0;
              for (int i = 0; i < 4; i++) {
                  max_dis = (max_dis > err_dis[i]) ? max_dis : err_dis[i];
                  max_angle = (max_angle > err_angle[i]) ? max_angle : err_angle[i];
                  avg_dis += err_dis[i];
                  avg_angle += err_angle[i];
              }

              avg_dis = avg_dis / 4.0;
              avg_angle = avg_angle / 4.0;

              if (max_dis > dis_max_thr || max_angle > angle_max_thr || avg_dis > dis_avg_thr || avg_angle > angle_avg_thr)
                  status = 1;
              else
                  status = 0;
          }
      } else {
          // 异常狀态,没有标码
          status = 253;
          LOG(ERROR) << "object process empty";
      }
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
                                 const std::string config_path,
                                 MapBuilderInterface *map_builder)
    : map_builder_(map_builder),
      object_impl_(std::make_unique<ObjectImpl>(came_base, this)) {
    err_dis_avg_thr_ = 0.20;
    err_dis_max_thr_ = 0.30;
    err_angle_avg_thr_ = 8.0;
    err_angle_max_thr_ = 24.0;
    // cv::FileStorage fsSettings(config_path, cv::FileStorage::READ);
    // fsSettings["err_dis_avg_thr"] >> err_dis_avg_thr_;
    // fsSettings["err_dis_max_thr"] >> err_dis_max_thr_;
    // fsSettings["err_angle_avg_thr"] >> err_angle_avg_thr_;
    // fsSettings["err_angle_max_thr"] >> err_angle_max_thr_;
    // LOG(INFO) << "error param: " << err_dis_avg_thr_ << ", " << err_dis_max_thr_ << ", " << err_angle_avg_thr_ << ", " << err_angle_max_thr_;
}

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
//   LOG_EVERY_N(INFO, 10) << "error param: " << err_dis_avg_thr_ << ", " << err_dis_max_thr_ << ", " << err_angle_avg_thr_ << ", " << err_angle_max_thr_;
  return object_impl_->Detect(time, image, pose, imu_to_cam);
}
//
std::vector<ObjectImageResult> ObjectInterface::ComputeError(
    const uint64_t &time, const cv::Mat &image,
    const transform::Rigid3d &pose,const transform::Rigid3d &imu_to_cam) {
  return object_impl_->ComputeError(time, image, pose, imu_to_cam);
}
//
// VslamFactoryResult ObjectInterface::GetFinalResult() {
//     return object_impl_->GetFinalResult();
// }
//
void ObjectInterface::GetFinalResult(uint8_t &status, float (&err_dis)[4],
                                     float (&err_angle)[4]) {
    return object_impl_->GetFinalResult(status, err_dis, err_angle, err_dis_avg_thr_,
                                        err_dis_max_thr_, err_angle_avg_thr_, err_angle_max_thr_);
}

ObjectInterface::~ObjectInterface() {}
}  // namespace object
}  // namespace jarvis