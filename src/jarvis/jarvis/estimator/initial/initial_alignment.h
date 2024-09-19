#ifndef _JARVIS_ESTIMATOR_INITIAL_INITIAL_ALIGNMENT_H
#define  _JARVIS_ESTIMATOR_INITIAL_INITIAL_ALIGNMENT_H

#include "Eigen/Dense"
#include <iostream>
#include <map>
#include "jarvis/estimator/feature_manager.h"
#include "jarvis/utility/utility.h"
#include "jarvis/common/time.h"
#include "jarvis/estimator/factor/integration_base.h"
namespace jarvis {
namespace estimator {
//
struct ImageFrame {
  common::Time time;
  transform::Rigid3d p;
  ImageFeatureTrackerData track_data;
  IntegrationBase* pre_integration;
  // std::shared_ptr<IntegrationBase> pre_integration=nullptr;
  bool is_key_frame = false;
};
struct InitialAlignmentOption {
  transform::Rigid3d cam_to_imu;
  Eigen::Vector3d gravity;
};
class InitialAlignment {
 public:
  explicit InitialAlignment(const InitialAlignmentOption &option)
      : options_(option) {}
  // /
  Eigen::Vector3d SolveGyroscopeBias(const std::vector<ImageFrame> &all_image_frame);
  //
  bool VisualIMUAlignment(const std::vector<ImageFrame> &all_image_frame,
                          Eigen::Vector3d *Bgs, Eigen::Vector3d &g,
                          Eigen::VectorXd &x);

 private:
  InitialAlignmentOption options_;
  void RefineGravity(const std::vector<ImageFrame> &all_image_frame,
                     Eigen::Vector3d &g, Eigen::VectorXd &x);
  bool LinearAlignment(const std::vector<ImageFrame> &all_image_frame,
                       Eigen::Vector3d &g, Eigen::VectorXd &x);
};

}  // namespace estimator
}  // namespace jarvis
#endif