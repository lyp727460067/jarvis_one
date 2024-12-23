#ifndef JARVIS_GRID_IMPL_MAP_H
#define JARVIS_GRID_IMPL_MAP_H
#include <memory>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Eigen"
#include "jarvis/grid_map/2d/submap_2d.h"
#include "jarvis/grid_map/grid_interface.h"
#include "jarvis/grid_map/2d/submap_2d.h"
namespace jarvis {
namespace grid_map {
//
class GridImpl : public GridMap {
 public:
  GridImpl(const std::map<int, GridMapOption>& option);
  void Insert(const AiObject& object);
  //
  //
  std::map<uint8_t, ObResultValue> IndexValue(const Eigen::Vector2f& index);
  //
  //
  bool HasValue(const Eigen::Vector2i& index);
  ~GridImpl();
  //
  PointCloud Votex(const PointCloud& point_clous,float size=0.5) ;
  void ToPgn(const std::string& dir);
  //
 private:
  std::map<int, GridMapOption> options_;
  //
  transform::Rigid2f last_pose_;
  std::map<uint8_t, std::unique_ptr<ActiveSubmaps2D>> active_submaps_;
};
}  // namespace grid_map
}  // namespace jarvis
#endif
