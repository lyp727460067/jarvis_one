#ifndef _MAPPING_TRAJECTORY_BUILDER_H
#define _MAPPING_TRAJECTORY_BUILDER_H
#include <functional>
#include <map>
#include <memory>
#include <vector>
#include "jarvis/estimator/estimator.h"
#include "sensor/image_data.h"
#include "sensor/imu_data.h"
//
#include "jarvis/key_frame_data.h"
#include "sensor/odometry_data.h"
#include "jarvis/mapping/map_builder.h"
namespace jarvis {
namespace estimator{
  class Estimator;
}
using CallBack = std::function<void(const TrackingData &)>;
//
struct TrajectorBuilderOption {
  estimator::EstimatorOption esti_option;
  mapping::MapBuilderOption mapping_option;
};

class TrajectorBuilder {
 public:
  // TrajectorBuilder(){}
  TrajectorBuilder(const TrajectorBuilderOption& option, CallBack call_back);
  virtual void AddImageData(const sensor::ImageData &images);
  virtual void AddImuData(const sensor::ImuData &imu_data);
  virtual void AddOdometryData(const sensor::OdometryData& odometry_data);
  virtual void AddFixData(const sensor::FixedFramePoseData& fix_data);
  //
  mapping::MappingBuilder *GetMapBuilder() { return map_builder_.get(); }
  void Relocation(){};

  void ReComputeTrajectorId() {}
  //
  std::vector<Eigen::Vector3d> GetMapPoints();
  std::map<KeyFrameId, transform::TimestampedTransform> GetKeyFrameGlobalPose();

  std::vector<Eigen::Vector3d> GetLocalMapPoints();
  std::vector<transform::Rigid3d > GetLocalKeyFramePose();

  transform::Rigid3d GetLocalToGlobalTransform();
  // /
  virtual ~TrajectorBuilder();
 private:
  void ReSet();
  std::unique_ptr<mapping::dbow::Vocabulary> voc_;
  TrajectorBuilderOption options_;
  std::unique_ptr<mapping::MappingBuilder> map_builder_;
  std::unique_ptr<estimator::Estimator> tracker_;
  CallBack call_back_;
  int trajector_ =0;
  int estimator_state_ =0;
};
}  // namespace jarvis

#endif
