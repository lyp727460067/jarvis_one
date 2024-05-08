#ifndef __JARVIS_VIO_OBJECT_PROCESS_H
#define __JARVIS_VIO_OBJECT_PROCESS_H
#include <memory>
#include <vector>

#include "common/id.h"
#include "common/time.h"
#include "jarvis/camera_models/camera_base_interface.h"
#include "jarvis/object/object_detect.h"
#include "jarvis/object/object_interface.h"
#include "transform/timestamped_transform.h"
#include "transform/transform_interpolation_buffer.h"
//
//
// pose 全在相机坐标系下
namespace jarvis {
namespace object {
//
struct ObejectDataPose {
  uint64_t local_id;
  transform::Rigid3d local_pose;
  std::shared_ptr<const ObejectData> data;
};
//
struct ObjectPhysics {
  struct Decoration {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::AlignedBox3f bound_box;
  };
  ObejectDataPose object;
  transform::Rigid3d relative_kf_pose;
  transform::Rigid3d global_pose;
};
//
inline ObjectPhysics SimplePoseToObject(const transform::Rigid3d& pose) {
  return ObjectPhysics{ObejectDataPose{5555, pose, nullptr},
                       transform::Rigid3d::Identity(), pose};
}
//
inline ObjectPhysics SimplePoseToObject(const ObejectDataPose& object_pose_data,
                                        const transform::Rigid3d& pose) {
  return ObjectPhysics{object_pose_data, transform::Rigid3d::Identity(),
                       pose * object_pose_data.data->pose};
}
//

class WapObjectDetect {
 public:
  WapObjectDetect(std::unique_ptr<ObjectDetect> detect);
  std::vector<ObejectDataPose> AddImage(const common::Time& time,
                                        const std::shared_ptr<cv::Mat>& image,
                                        const transform::Rigid3d& cam_pose);

 private:
  std::unique_ptr<ObjectDetect> detect_;
  uint32_t local_id_counter_ = 0;
};
//
class ObjectProcess {
 public:
  //
  ObjectProcess();
  void AddLandMark(
      const ObejectDataPose&,
      const std::pair<KeyFrameId, transform::TimestampedTransform>& kf_data);
  //
  void Update(const std::map<KeyFrameId, transform::TimestampedTransform>&
                  optimazation_pose);
  void Clear(int local_id = -1);
  //
  std::map<int, ObjectPhysics> GetObjectData(int trajector);
  //
  struct ObjectsPhysics {
    common::Time time;
    std::vector<ObjectPhysics> objects;
  };
  using ObjectsPhysicsData = std::shared_ptr<ObjectsPhysics>;
  void Serialization(const std::string& mark_file);
  void Load(const std::string& mark_file);

 private:
  //
  std::unique_ptr<transform::TransformInterpolationBuffer>
      transformin_terpolation_buffer_;
  MapById<KeyFrameId, ObjectsPhysicsData> object_datas_;
  std::set<uint64_t> insert_local_ids_;
};

struct ObjectImageProcessOption {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d cube_min{-0.154 / 2, -0.154 / 2, 0.0};
  Eigen::Vector3d cube_max{0.154 / 2, 0.154 / 2, -0.154};
};
//
class ObjectImageProcess {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit ObjectImageProcess(const ObjectImageProcessOption& option,
                              const camera_models::CameraBase* came_base);
  ObjectImageResult ProjectObject(const transform::Rigid3d& pose,
                                  const ObjectPhysics& object,
                                  bool global = false);

 private:
  ObjectImageResult ProjectObjectFaceMark(const transform::Rigid3d& pose,
                                          const ObjectPhysics& object,
                                          bool limit = true);

  const camera_models::CameraBase* cam_base_;
  Eigen::AlignedBox3d bbox_;
  //   const Eigen::AlignedBox2d image_box_;
};

}  // namespace object
//
}  // namespace jarvis
#endif