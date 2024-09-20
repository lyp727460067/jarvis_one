#ifndef JARVIS_STERO_IMU_INITIALIZATION_H
#define JARVIS_STERO_IMU_INITIALIZATION_H
#include "jarvis/estimator/initial/initial_alignment.h"
#include "jarvis/estimator/initialization_interface.h"
#include "jarvis/estimator/optimization.h"
//
#include "jarvis/estimator/optimization.h"
namespace jarvis {
namespace estimator {
//

struct SteroImuInitializationOption {
  int sw_size=6;
  ImuOption imu_option;
  std::vector<transform::Rigid3d> extric_camera_to_imu;
  FeatureManagerOption feature_manager_option;
  OptimizationOption opti_option;
  double init_bg_th = 0.2;
  double init_ba_th = 0.2;
};

class SteroImuInitialization : public InitializationImu {
 public:
  SteroImuInitialization(const SteroImuInitializationOption& option,
                         DataBase* data_base);
  std::unique_ptr<InitializationResult> AddFeatureData(
      const ImageFeatureTrackerData&);
  //
 private:
  void RemoveBack();
  std::unique_ptr<InitializationResult> OptimizationResult();
  SteroImuInitializationOption options_;
  std::shared_ptr<FeatureManager> feature_manager_;
  std::vector<bool> init_pnp_states_;
  std::vector<transform::Rigid3d> sw_pose_;
  std::vector<ImageFrame> image_frames_;
  std::vector<std::shared_ptr<IntegrationBase>> integration_bases_;
  std::vector<bool> frames_continuously_track_num ;
  common::Time last_time;
  //
  Eigen::Vector3d init_bgs_;
  std::unique_ptr<InitialAlignment> initial_alignment_;
  std::optional<Eigen::Quaterniond> init_imu_rotation_; 
  std::unique_ptr<Optimization> optimization_;
};
//
}  // namespace estimator
}  // namespace jarvis

#endif