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

// 产测模式输出用于可视化结果
struct ObjectImageResult {
  std::vector<Eigen::Vector2d> coners; // 
  std::vector<Eigen::Vector2d> direction;
  transform::Rigid3d global_pose_cam;
  transform::Rigid3d local_pose_cam;
  std::string type;
  uint64_t id;
};

// #pragma pack(push, 1)
// // 产测模式输出最终结果
// struct VslamFactoryResult
// {
//   uint8_t status;  
//   float err_dis[4];
//   float err_angle[4];
// };

// #pragma pack(pop)

//
class ObjectInterface {
 public:
  ObjectInterface(const camera_models::CameraPtr came_base,
                  const std::string config_path,
                  MapBuilderInterface *map_builder = nullptr);
  //
  std::vector<ObjectImageResult> Detect(const uint64_t &time,
                                        const cv::Mat &image,
                                        const transform::Rigid3d &pose,
                                        const transform::Rigid3d &imu_to_cam);

  std::vector<ObjectImageResult> ComputeError(const uint64_t &time,
                                              const cv::Mat &image,
                                              const transform::Rigid3d &pose,
                                              const transform::Rigid3d &imu_to_cam);

  // VslamFactoryResult GetFinalResult();

  void GetFinalResult(uint8_t &status, float (&err_dis)[4], float (&err_angle)[4]);

  void UpdateGloblePose();

  ~ObjectInterface();

 private:
  class ObjectImpl; //嵌套类可以访问外围类的成员(包括private),反之不行,需要通过嵌套类的对象,此处用作类名的声明
  // friend class ObjectImpl; // 声明ObjectImpl是ObjectInterface的友元类
  //友元类和友元函数都不是类的一部分,但可以访问此类的数据成员(包括private 和protect)
  MapBuilderInterface *map_builder_;
  std::unique_ptr<ObjectImpl> object_impl_;
  transform::Rigid3d local_to_globle_transform_;
  float err_dis_avg_thr_ = 0.28;  // 0.15 0.20
  float err_dis_max_thr_ = 0.40;  // 0.20 0.25
  float err_angle_avg_thr_ = 8.0; // 5.0  7.0
  float err_angle_max_thr_ = 24.0; // 8.0 10.0
};
//
}  // namespace object
}  // namespace jarvis
#endif