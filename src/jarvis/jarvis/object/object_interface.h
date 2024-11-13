#ifndef JARVIS_OBJECT_OBJECT_INTERFACE_
#define JARVIS_OBJECT_OBJECT_INTERFACE_
#include <string>
#include <vector>

#include "jarvis/transform/timestamped_transform.h"
#include "opencv2/opencv.hpp"

#include "jarvis/camera_models/camera_models/camera.h"
namespace jarvis {

namespace object {
class MapBuilderInterface;  

struct ObjectImageResult {
  std::vector<Eigen::Vector2d> coners;
  std::vector<Eigen::Vector2d> direction;
  transform::Rigid3d global_pose_cam;
  transform::Rigid3d local_pose_cam;
  std::string type;
  uint64_t id;
};

//
class ObjectInterface {
 public:
  ObjectInterface(const camera_models::CameraPtr came_base,
                  MapBuilderInterface *map_builder = nullptr);
  //
  std::vector<ObjectImageResult> Detect(const uint64_t &time,
                                        const cv::Mat &image,
                                        const transform::Rigid3d &pose,
                                        const transform::Rigid3d &imu_to_cam);

  void UpdateGloblePose();

  ~ObjectInterface();

 private:
  class ObjectImpl;
  friend class ObjectImpl;
  MapBuilderInterface *map_builder_;
  std::unique_ptr<ObjectImpl> object_impl_;
  transform::Rigid3d local_to_globle_transform_;
};
//
}  // namespace object
}  // namespace jarvis
#endif