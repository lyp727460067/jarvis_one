#ifndef JARVIS_GRID_MAP_H
#define JARVIS_GRID_MAP_H
#include <memory>
#include <optional>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Eigen"

namespace jarvis {
namespace grid_map {
// 相对于当前pose的zhi
using PointCloud = std::vector<Eigen::Vector3f>;
struct RigidPose {
  Eigen::Vector3d tanslation;  // m
  Eigen::Quaterniond rotaion;  //

};

//
struct AiObject {
  uint64_t time;
  RigidPose pose;
  std::map<uint8_t, PointCloud> points_clouds;  // 10类的数据

  
};
//
//
struct GridMapOption {
  float point_votex = 0.05;//当点云大于200个时候内部会降采样
  float resolution = 0.05;  // grid 的分辨率 建议0.05或者0.1 太大没用耗费时间
  int max_node_num = 200;          // 选择维护多少个节点的数据 //没有用
  bool insert_free_space = false;  // free space 是否要插图
  bool insert_free_sector_space = false;  //扇形插入 
  float insert_free_min_distance =0.0; //小于这个值的区域不要去减概率
  double max_distance = 2;         // 超过3米距离的点直接不插
  float hit_probability = 0.85;    // hit 每次概率插入多大
  float miss_probability = 0.45;   //每次miss消除的概率（快慢）
  uint8_t min_probability = 70;  ////没有用
  //在使用insert_free_space=true的时候 max_angle  min_angle 决定
  //投影在平面视角的大小，多少度的分辨率angle_size
  float max_angle = 50;
  float min_angle = -50;
  float angle_size = 0.5;
  //map的带小取决与 min_x_map_size*resolution  xy都满足这个条件后
  int min_x_map_size =400;  
  int min_y_map_size =400;
  //
  // bool compute_x_z_angle = false;
  // bool compute_max_z = false;
};
//
struct ObResultValue {
  uint8_t p;  //概率*255
  std::optional<double> z;
  std::optional<double> slop;
};
//
class GridMap {
 public:
  virtual void Insert(const AiObject& object) = 0;
  //// 暂定 XY和机器人当前pose的xy对齐的索引，
  virtual std::map<uint8_t, ObResultValue> IndexValue(
      const Eigen::Vector2f& index) = 0;
  //
  virtual PointCloud Votex(const PointCloud& point_clous,float size) = 0;
  //
  virtual bool HasValue(const Eigen::Vector2i& index)=0;
  //
  virtual ~GridMap() {}
  static std::unique_ptr<GridMap> Create(
      const std::map<int, GridMapOption>& option);
};
}  // namespace grid_map
}  // namespace jarvis
#endif
