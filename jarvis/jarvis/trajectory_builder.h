#ifndef _MAPPING_TRAJECTORY_BUILDER_H
#define _MAPPING_TRAJECTORY_BUILDER_H
#include <functional>
#include <map>
#include <memory>
#include <vector>

#include "sensor/image_data.h"
#include "sensor/imu_data.h"
//
#include "jarvis/key_frame_data.h"
namespace jarvis {
namespace estimator{
  class Estimator;
}
using CallBack = std::function<void(const TrackingData &)>;
class TrajectorBuilder {
 public:
  TrajectorBuilder(const std::string &config, CallBack call_back);
  void AddImageData(const sensor::ImageData &images);
  //
  void AddImuData(const sensor::ImuData &imu_data);
  // MapBuilderInterface *GetMapBuilder() { return map_builder_.get(); }

  std::vector<Eigen::Vector3d> GetMapPoints();
  //
  std::map<KeyFrameId, transform::TimestampedTransform>
  GetKeyFrameGlobalPose(); std::vector<std::pair<KeyFrameId, KeyFrameId>>
  ConstraintId(); transform::Rigid3d GetLocalToGlobalTransform();
  ~TrajectorBuilder();

 private:
  // std::unique_ptr<tracking::TrackingInterface> tracker_;
  std::unique_ptr<estimator::Estimator> tracker_;
  // std::unique_ptr<MapBuilderInterface> map_builder_;
  CallBack call_back_;
};
}  // namespace jarvis

#endif
