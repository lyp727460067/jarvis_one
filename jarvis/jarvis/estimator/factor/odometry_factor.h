#ifndef JARVIS_ESTIMATION_FACTOR_ODOM_FACTOR_H
#define JARVIS_ESTIMATION_FACTOR_ODOM_FACTOR_H

#include <ceres/ceres.h>

#include <Eigen/Dense>
#include <deque>
#include <iostream>
#include <optional>

#include "jarvis/estimator/data_base.h"
#include "jarvis/estimator/parameters.h"
#include "jarvis/utility/utility.h"
#include "sensor/odometry_data.h"
namespace jarvis {
namespace estimator {

struct OdomFactorOption {
  double optimize_weight = 20;
};
//
class OdomFactor {
 public:
  OdomFactor(const OdomFactorOption &option, const DataBase *data_base);
  //
  void ComputeObserve(const common::Time start_time, const common::Time &time);
  void AddToProblem(ceres::Problem *problem, ceres::LossFunction *loss_function,
                    std::array<double *, 2> pq) const;
  void Merge(const OdomFactor &odom_factor);
  ceres::CostFunction* CostFunction() const;
 private:
  const OdomFactorOption option_;
  const DataBase *data_base_;
  common::Time start_time_;
  common::Time end_time_;
  std::optional<Eigen::Vector3d> translation_observe_;
};

}  // namespace estimator
}  // namespace jarvis
#endif