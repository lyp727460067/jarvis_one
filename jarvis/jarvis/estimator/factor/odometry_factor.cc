

#include "odometry_factor.h"

#include <Eigen/Dense>
#include <iostream>

#include "jarvis/utility/utility.h"
#include "sensor/odometry_data.h"

namespace jarvis {
namespace estimator {
namespace {
inline Eigen::Matrix<double, 3, 3> Skew(const Eigen::Matrix<double, 3, 1>& w) {
  Eigen::Matrix<double, 3, 3> w_x;
  w_x << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
  return w_x;
}
#define residuals_block_size 3
class OdomCostFuction
    : public ceres::SizedCostFunction<residuals_block_size, 7, 7, 7> {
 public:
  OdomCostFuction(const double& weight,
                  const Eigen::Vector3d& translation_observe)
      : weight_(weight), translation_observe_(translation_observe) {}
  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const {
    //

    Eigen::Vector3d p_a(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond q_a(parameters[0][6], parameters[0][3], parameters[0][4],
                           parameters[0][5]);
    Eigen::Vector3d p_b(parameters[1][0], parameters[1][1], parameters[1][2]);

    Eigen::Vector3d p_e(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond q_e(parameters[2][6], parameters[2][3], parameters[2][4],
                           parameters[2][5]);
    Eigen::Vector3d temp = q_e * translation_observe_;
    Eigen::Vector3d delta_t = p_b - p_a - q_a * (temp);
    //
    // delta_t.z() = 0;
    // LOG(INFO) <<( p_b - p_a).transpose();

    Eigen::Matrix<double, residuals_block_size, residuals_block_size>
        sqrt_info = weight_ * Eigen::Matrix<double, residuals_block_size,
                                            residuals_block_size>::Identity();
    //
    Eigen::Map<Eigen::Matrix<double, residuals_block_size, 1>> residual(
        residuals);
    residual << delta_t;
    residual = sqrt_info * residual;
    // LOG(INFO)<< residual.transpose() ;
    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<
            Eigen::Matrix<double, residuals_block_size, 7, Eigen::RowMajor>>
            jacobians_(jacobians[0]);

        jacobians_.setZero();
        jacobians_.block<3, 3>(0, 0) = -Eigen::Matrix<double, 3, 3>::Identity();
        jacobians_.block<3, 3>(0, 3) = q_a.toRotationMatrix() * Skew(temp);
        jacobians_ = sqrt_info * jacobians_;
      }
      if (jacobians[1]) {
        Eigen::Map<
            Eigen::Matrix<double, residuals_block_size, 7, Eigen::RowMajor>>
            jacobians_(jacobians[1]);
        jacobians_.setZero();
        jacobians_.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();
        jacobians_ = sqrt_info * jacobians_;
      }
      if (jacobians[2]) {
        Eigen::Map<
            Eigen::Matrix<double, residuals_block_size, 7, Eigen::RowMajor>>
            jacobians_(jacobians[2]);
        jacobians_.setZero();
        // jacobians_.block<3, 3>(0, 0) = -Eigen::Matrix<double, 3,
        // 3>::Identity();
        jacobians_.block<3, 3>(0, 3) =
            (q_a * q_e).toRotationMatrix() * Skew(translation_observe_);
        jacobians_ = sqrt_info * jacobians_;
        // LOG(INFO)<<jacobians_ ;
      }
    }
    return true;
  }
  ~OdomCostFuction() {}

 private:
  const double weight_;
  const Eigen::Vector3d translation_observe_;
};
}  // namespace
OdomFactor::OdomFactor(const OdomFactorOption& option,
                       const DataBase* data_base)
    : option_(option), data_base_(data_base) {}
//

void OdomFactor::ComputeObserve(common::Time start_time,
                                const common::Time& time) {
  //
  CHECK(data_base_ != nullptr);

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
  //
  const transform::Rigid3d delta_pose =
      start_data.pose.inverse() * end_data.pose;
  if (abs(common::RadToDeg(transform::GetAngle(delta_pose))) >
      option_.angle_threas_hold)
    return;
  odom_observe_ = delta_pose;
  start_pose_ = start_data.pose;
}
//
void OdomFactor::Merge(const OdomFactor& odom_factor) {
  if (!odom_factor.odom_observe_.has_value()) {
    odom_observe_.reset();
  }
  if (!odom_observe_.has_value()) return;
  //
  odom_observe_ = odom_observe_.value() * odom_factor.odom_observe_.value();
  //
}
//
ceres::CostFunction* OdomFactor::CostFunction() const {
  if (!odom_observe_.has_value()) {
    LOG(WARNING) << "odom factor invalid...";
    return nullptr;
  }
  Eigen::Vector3d translation_observe =
      (option_.transform_imu_to_robot.inverse() * odom_observe_.value() *
       option_.transform_imu_to_robot)
          .translation();
  // Eigen::Vector3d translation_observe =
  //     (odom_observe_.value() * option_.transform_imu_to_robot).translation();

  // translation_observe.x() *= 1.1;
  return new OdomCostFuction(option_.optimize_weight, translation_observe);
}
void OdomFactor::AddToProblem(ceres::Problem* problem,
                              ceres::LossFunction* loss_function,
                              std::array<double*, 3> pqe) const {
  ceres::CostFunction* cons_function = CostFunction();
  if (cons_function == nullptr) return;
  problem->AddResidualBlock(cons_function, loss_function, pqe[0], pqe[1],
                            pqe[2]);
}

}  // namespace estimator
}  // namespace jarvis