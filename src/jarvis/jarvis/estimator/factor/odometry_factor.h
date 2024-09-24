#ifndef JARVIS_ESTIMATION_FACTOR_ODOM_FACTOR_H
#define JARVIS_ESTIMATION_FACTOR_ODOM_FACTOR_H

#include <ceres/ceres.h>

#include "Eigen/Dense"
#include <deque>
#include <iostream>
#include <optional>

#include "jarvis/estimator/data_base.h"
#include "jarvis/estimator/parameters.h"
#include "jarvis/utility/utility.h"
#include "jarvis/sensor/odometry_data.h"
namespace jarvis {
namespace estimator {

struct OdomFactorOption {
  transform::Rigid3d  transform_imu_to_robot;
  double optimize_weight = 1e4;
  double angle_threas_hold =5.5;
};
//
class OdomFactor {
 public:
  OdomFactor(const OdomFactorOption &option, const DataBase *data_base);
  //
  void ComputeObserve(const common::Time start_time, const common::Time &time);
  void AddToProblem(ceres::Problem *problem, ceres::LossFunction *loss_function,
                    std::array<double *, 3> pqe) const;
  void Merge(const OdomFactor &odom_factor);
  //
  std::optional<double> GetObserveDistance();
  //
  ceres::CostFunction* CostFunction() const;
 private:
  const OdomFactorOption option_;
  const DataBase *data_base_=nullptr;
  std::optional<transform::Rigid3d> odom_observe_;
  transform::Rigid3d  start_pose_;
};

}  // namespace estimator
}  // namespace jarvis
#endif