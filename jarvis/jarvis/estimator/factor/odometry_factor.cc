

#include "odometry_factor.h"

#include <Eigen/Dense>
#include <iostream>

#include "jarvis/utility/utility.h"
#include "sensor/odometry_data.h"

namespace jarvis {
namespace estimator {
namespace {
#define residuals_block_size 3
class OdomCostFuction
    : public ceres::SizedCostFunction<residuals_block_size, 7, 7> {
 public:
  OdomCostFuction(const double& weight,
                  const Eigen::Vector3d& translation_observe)
      : weight_(weight), translation_observe(translation_observe) {}
  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const {
    //

    Eigen::Vector3d p_a(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Vector3d p_b(parameters[1][0], parameters[1][1], parameters[1][2]);

    Eigen::Vector3d delta_t = translation_observe - (p_b - p_a);
    //
    Eigen::Matrix<double, residuals_block_size, residuals_block_size>
        sqrt_info = weight_ * Eigen::Matrix<double, residuals_block_size,
                                            residuals_block_size>::Identity();
    //
    Eigen::Map<Eigen::Matrix<double, residuals_block_size, 1>> residual(
        residuals);
    residual << delta_t;
    residual = sqrt_info * residual;
    // LOG(INFO)<< residual ;
    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<
            Eigen::Matrix<double, residuals_block_size, 7, Eigen::RowMajor>>
            jacobians_(jacobians[0]);
        jacobians_.setZero();
        jacobians_.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();
        jacobians_ = sqrt_info * jacobians_;
      }
      if (jacobians[1]) {
        Eigen::Map<
            Eigen::Matrix<double, residuals_block_size, 7, Eigen::RowMajor>>
            jacobians_(jacobians[1]);
        jacobians_.setZero();
        jacobians_.block<3, 3>(0, 0) = -Eigen::Matrix<double, 3, 3>::Identity();
        jacobians_ = sqrt_info * jacobians_;
      }
    }
    return true;
  }
  ~OdomCostFuction() {}

 private:
  const double weight_;
  const Eigen::Vector3d translation_observe;
};
}  // namespace
OdomFactor::OdomFactor(const OdomFactorOption& option,
                       const DataBase* data_base)
    : option_(option), data_base_(data_base) {}
//

void OdomFactor::ComputeObserve(common::Time start_time,
                                const common::Time& time) {
  //

  if (!data_base_->HasOdometryData(start_time)) return;
  const sensor::OdometryData start_data =
      data_base_->InterpolateOdometry(start_time);
  //
  sensor::OdometryData end_data;
  if (!data_base_->HasOdometryData(time)) {
    end_data = data_base_->InterpolateOdometryUseLastData(time);
  } else {
    end_data = data_base_->InterpolateOdometry(time);
  }
  translation_observe_ =
      end_data.pose.translation() - start_data.pose.translation();
  //
  LOG(INFO) << translation_observe_.value().transpose();
}
//
void OdomFactor::AddToProblem(ceres::Problem* problem,
                              ceres::LossFunction* loss_function,
                              std::array<double*, 2> pq) const {
  if (!translation_observe_.has_value()) {
    LOG(WARNING) << "odom factor invalid...";
    return;
  }
  ceres::CostFunction* cons_function = new OdomCostFuction(
      option_.optimize_weight, translation_observe_.value());
  problem->AddResidualBlock(cons_function, loss_function, pq[0], pq[1]);
}

}  // namespace estimator
}  // namespace jarvis