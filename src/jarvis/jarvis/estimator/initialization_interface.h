#ifndef JARVIS_INITIALIZETION_INTERFACE_H
#define JARVIS_INITIALIZETION_INTERFACE_H
#include "jarvis/estimator/initial/initial_alignment.h"
#include "jarvis/sensor/imu_data.h"
#include "jarvis/sensor/odometry_data.h"
#include "key_frame_data.h"
#include "optimization.h"
#include "jarvis/common/time.h"
namespace jarvis {
namespace estimator {

struct InitializationOption {
  OptimizationOption opti_option;
  int win_size;
};

//
struct InitializationResult {
  int cam_id;
  std::vector<ImuState> states;
  std::shared_ptr<FeatureManager> feat_manager;
  std::vector<transform::Rigid3d> extric_camera_to_imu;
  common::Time time;
  std::vector<std::shared_ptr<IntegrationBase>> integration_base;
};

//
class InitializationInterface {
 public:
  virtual void AddImuData(const sensor::ImuData& imu_data) = 0;
  virtual void AddOdometryData(const sensor::OdometryData& odometry_data) = 0;
  //
  virtual std::unique_ptr<InitializationResult> AddFeatureData(
      const ImageFeatureTrackerData&) = 0;
  //
  virtual ~InitializationInterface() {};
};
//

class InitializationImu : public InitializationInterface {
 public:
  InitializationImu(DataBase* data_base);
  virtual void AddImuData(const sensor::ImuData& imu_data);
  virtual void AddOdometryData(const sensor::OdometryData& odometry_data);
  //
  virtual std::unique_ptr<InitializationResult> AddFeatureData(
      const ImageFeatureTrackerData&) = 0;
  //
  virtual ~InitializationImu() {}

 protected:
  std::unique_ptr<Eigen::Quaterniond> InitImuRotaion(const common::Time& time);
  std::unique_ptr<Eigen::Quaterniond> InitImuRotaion(const common::Time& s_time,
                                                     const common::Time& e_time);

  DataBase* data_base_;
};

}  // namespace estimator
}  // namespace jarvis
#endif