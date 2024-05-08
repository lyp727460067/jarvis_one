#ifndef JARVIS_OBJECT_OBJECT_INTERFACE_
#define JARVIS_OBJECT_OBJECT_INTERFACE_
#include <string>
#include <vector>

#include "jarvis/transform/timestamped_transform.h"
#include "opencv2/opencv.hpp"

namespace jarvis {

namespace object {
class MapBuilderInterface;  

struct ObjectImageResult {
  std::vector<Eigen::Vector2d> coners;
  std::vector<Eigen::Vector2d> direction;
  transform::Rigid3d global_pose_cam;
  std::string type;
  uint64_t id;
};

//
class ObjectInterface {
 public:
  ObjectInterface(MapBuilderInterface *map_builder);
  //
  // 主要调用这个函数
  std::vector<ObjectImageResult> Detect(const uint64_t &time,
                                        const cv::Mat &image,
                                        const transform::Rigid3d &pose);

  // 因为后端pose数据有可能比较大，可以频率低点调用一下这个函数去跟新一下所有的关键帧的pose
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