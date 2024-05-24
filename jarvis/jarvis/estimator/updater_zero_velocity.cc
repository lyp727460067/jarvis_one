#include "updater_zero_velocity.h"

#include <boost/math/distributions/chi_squared.hpp>
#include <random>

#include "glog/logging.h"
#include "transform/rigid_transform.h"
#include "transform/transform.h"

namespace jarvis {
namespace estimator {
//
using namespace mapping;
namespace {
//
inline Eigen::Matrix<double, 3, 3> Skew(const Eigen::Matrix<double, 3, 1>& w) {
  Eigen::Matrix<double, 3, 3> w_x;
  w_x << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
  return w_x;
}
//
Eigen::Matrix<double, 6, 6> Adj(const Eigen::Vector3d& traslation,
                                const Eigen::Quaterniond& rotation) {
  Eigen::Matrix<double, 6, 6> result;
  const Eigen::Matrix3d r = rotation.toRotationMatrix();
  result.block(0, 0, 3, 3) = r;
  result.block(3, 3, 3, 3) = r;
  result.block(0, 3, 3, 3) = Skew(traslation) * r;
  result.block(3, 0, 3, 3) = Eigen::Matrix3d::Identity();
  return result;
}
//

class AutoZeroVelocityCostFuction {
 public:
  AutoZeroVelocityCostFuction(const std::array<double, 2>& weight,
                              const Eigen::Vector3d& average_acc,
                              const Eigen::Vector3d& average_gry)
      : weight_(weight), average_acc_(average_acc), average_gry_(average_gry) {}

  template <typename T>
  bool operator()(const T* const p_a_, const T* const p_b_, const T* const v_a_,
                  T* residuals) const {
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_);
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> q_a_angle(p_a_ + 3);
    const Eigen::Quaternion<T> q_a =
        transform::AngleAxisVectorToRotationQuaternion(
            Eigen::Matrix<T, 3, 1>(q_a_angle));
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_);
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> q_b_angle(p_b_ + 3);
    const Eigen::Quaternion<T> q_b =
        transform::AngleAxisVectorToRotationQuaternion(
            Eigen::Matrix<T, 3, 1>(q_b_angle));
    //
    Eigen::Quaternion<T> delta_q = q_a.conjugate() * q_b;
    const Eigen::Matrix<T, 3, 1> delta_t = p_b - p_a;
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> v_a(v_a_);
    //
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> acc_bias(v_a_ + 3);
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> gry_bias(v_a_ + 6);
    const Eigen::Matrix<T, 3, 1> gravity =
        Eigen::Matrix<T, 3, 1>::UnitZ() * T(9.81);
    //
    Eigen::Matrix<T, 3, 1> acc_bias_err =
        average_acc_.template cast<T>() - q_a.conjugate() * gravity - acc_bias;
    Eigen::Matrix<T, 3, 1> gry_bias_err =
        average_gry_.template cast<T>() - gry_bias;
    //-0.0799841  -0.178106 -0.0305463
    Eigen::Map<Eigen::Matrix<T, 9, 1>, Eigen::RowMajor> residual(residuals);
    residual << -T(weight_[0]) * T(2.0) * delta_q.vec(),
        -T(weight_[0]) * delta_t, -T(weight_[0]) * v_a,
        -T(weight_[1]) * acc_bias_err, -T(weight_[1]) * gry_bias_err;
    return true;
  }

  static ceres::CostFunction* Create(std::array<double, 2> weight,
                                     const Eigen::Vector3d& average_acc,
                                     const Eigen::Vector3d& average_gry) {
    return new ceres::AutoDiffCostFunction<AutoZeroVelocityCostFuction, 15, 6,
                                           6, 9>(
        new AutoZeroVelocityCostFuction(weight, average_acc, average_gry));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  const std::array<double, 2> weight_;
  const Eigen::Vector3d average_acc_;
  const Eigen::Vector3d average_gry_;
};

// p0 p1 v_bais_bias  (bias ignore)
constexpr uint32_t residuals_block_size = 15;
class ZeroVelocityCostFuction
    : public ceres::SizedCostFunction<residuals_block_size, 6, 6, 9> {
 public:
  ZeroVelocityCostFuction(const std::array<double, 2>& weight,
                          const Eigen::Vector3d& average_acc,
                          const Eigen::Vector3d& average_gry)
      : weight_(weight), average_acc_(average_acc), average_gry_(average_gry) {}
  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const {
    //

    const Eigen::Map<const Eigen::Vector3d> p_a(parameters[0]);
    const Eigen::Map<const Eigen::Vector3d> q_a_angle(parameters[0] + 3);
    const Eigen::Quaterniond q_a =
        transform::AngleAxisVectorToRotationQuaternion(
            Eigen::Vector3d(q_a_angle));

    const Eigen::Map<const Eigen::Vector3d> p_b(parameters[1]);
    const Eigen::Map<const Eigen::Vector3d> q_b_angle(parameters[1] + 3);
    const Eigen::Quaterniond q_b =
        transform::AngleAxisVectorToRotationQuaternion(
            Eigen::Vector3d(q_b_angle));
    //
    const Eigen::Map<const Eigen::Vector3d> v_a(parameters[2]);
    //
    Eigen::Quaterniond delta_q = q_a.conjugate() * q_b;
    // LOG(INFO)<<-LogSo3(q_a.toRotationMatrix()*q_b.conjugate().toRotationMatrix());
    Eigen::Vector3d delta_t = p_b - p_a;
    //

    Eigen::Map<const Eigen::Vector3d> acc_bias(parameters[2] + 3);
    Eigen::Map<const Eigen::Vector3d> gry_bias(parameters[2] + 6);
    const Eigen::Vector3d gravity = Eigen::Vector3d::UnitZ() * 9.81;
    Eigen::Vector3d acc_bias_err =
        average_acc_ - q_a.conjugate() * gravity - acc_bias;
    Eigen::Vector3d gry_bias_err = average_gry_ - gry_bias;
    //
    Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
    residual << (weight_[0]) * delta_t, (weight_[0]) * 2 * delta_q.vec(),
        (weight_[0]) * v_a, (weight_[1]) * acc_bias_err,
        (weight_[1]) * gry_bias_err;
    //
    // order is  t0,t1,v0
    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<
            Eigen::Matrix<double, residuals_block_size, 6, Eigen::RowMajor>>
            jacobians_(jacobians[0]);
        jacobians_.setZero();
        jacobians_.block<3, 3>(0, 0) =
            -weight_[0] * Eigen::Matrix<double, 3, 3>::Identity();
        jacobians_.block<3, 3>(3, 3) =
            -weight_[0] * Eigen::Matrix<double, 3, 3>::Identity();
        jacobians_.block<3, 3>(9, 3) =
            -weight_[1] * Skew(q_a.conjugate() * gravity);
      }
      if (jacobians[1]) {
        Eigen::Map<
            Eigen::Matrix<double, residuals_block_size, 6, Eigen::RowMajor>>
            jacobians_(jacobians[1]);
        jacobians_.setZero();
        //
        jacobians_.block<3, 3>(0, 0) =
            weight_[0] * Eigen::Matrix<double, 3, 3>::Identity();
        jacobians_.block<3, 3>(3, 3) =
            weight_[0] * Eigen::Matrix<double, 3, 3>::Identity();
      }
      if (jacobians[2]) {
        Eigen::Map<
            Eigen::Matrix<double, residuals_block_size, 9, Eigen::RowMajor>>
            jacobians_(jacobians[2]);
        jacobians_.setZero();
        jacobians_.block<3, 3>(6, 0) =
            weight_[0] * Eigen::Matrix<double, 3, 3>::Identity();
        jacobians_.block<3, 3>(9, 0) =
            -weight_[1] * Eigen::Matrix<double, 3, 3>::Identity();
        jacobians_.block<3, 3>(12, 0) =
            -weight_[1] * Eigen::Matrix<double, 3, 3>::Identity();
      }
    }
    return true;
  }
  ~ZeroVelocityCostFuction() {}

 private:
  const std::array<double, 2> weight_;
  const Eigen::Vector3d average_acc_;
  const Eigen::Vector3d average_gry_;
};

};  // namespace
//
//
bool ImuZeroVelocityDetect::IsZeroVelocity() {
  CHECK(!data_base_.empty());
  if (data_base_.size() < 2) return false;
  Eigen::Vector3d sum_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_gry = Eigen::Vector3d::Zero();
  for (const auto& imu_data : data_base_) {
    // LOG(INFO)<<imu_data.linear_acceleration.transpose();
    // LOG(INFO)<<imu_data.angular_velocity.transpose();
    sum_acc += imu_data.linear_acceleration;
    sum_gry += imu_data.angular_velocity;
  }
  Eigen::Vector3d expect_acc = sum_acc / data_base_.size();
  Eigen::Vector3d expect_gry = sum_gry / data_base_.size();
  double sum_covi = 0;
  for (const auto& imu_data : data_base_) {
    Eigen::Vector3d imu_acc = (imu_data.linear_acceleration - expect_acc);
    Eigen::Vector3d imu_gry = (imu_data.linear_acceleration - expect_gry);
    sum_covi += imu_acc.transpose() * imu_acc;
    sum_covi += imu_gry.transpose() * imu_gry;
  }
  const double cov_avg = sum_covi / data_base_.size();
  LOG(INFO) << cov_avg;
  if (cov_avg < 0.1) return true;
  return false;
}

bool OpenVinsZeroVelocityDetect::IsZeroVelocity() {
  // Large final matrices used for update
  if (data_base_.size() < 2) {
#ifdef __DEBUG__
    LOG(INFO)
        << "zupt failed - OpenVINS Inertial-based Detection (data_base_.size() "
        << data_base_.size() << " < 2 )";
#endif
    return false;
  }
  if (state_.linear_velocity.norm() > options_.zupt_max_velocity) {
#ifdef __DEBUG__
    LOG(INFO) << "zupt failed - OpenVINS Inertial-based Detection (vel "
              << state_.linear_velocity.norm() << " > max zupt vel "
              << options_.zupt_max_velocity << ")";
#endif
    return false;
  }
  int h_size = (options_.integrated_accel_constraint) ? 12 : 9;
  int m_size = 6 * ((int)data_base_.size() - 1);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m_size, h_size);
  Eigen::VectorXd res = Eigen::VectorXd::Zero(m_size);
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(m_size, m_size);

  // Loop through all our IMU and construct the residual and Jacobian
  // State order is: [q_GtoI, bg, ba, v_IinG]
  // Measurement order is: [w_true = 0, a_true = 0 or v_k+1 = 0]
  // w_true = w_m - bw - nw
  // a_true = a_m - ba - R*g - na
  // v_true = v_k - g*dt + R^T*(a_m - ba - na)*dt
  double dt_summed = 0;
  for (size_t i = 0; i < data_base_.size() - 1; i++) {
    // Precomputed values
    double dt =
        common::ToSeconds(data_base_.at(i + 1).time - data_base_.at(i).time);
    Eigen::Vector3d a_hat =
        data_base_.at(i).linear_acceleration - state_.linear_acceleration_bias;
    // LOG(INFO)<<a_hat.transpose();
    // Measurement residual (true value is zero)
    res.block(6 * i + 0, 0, 3, 1) =
        -(data_base_.at(i).angular_velocity - state_.angular_velocity_bias);

    if (!options_.integrated_accel_constraint) {
      res.block(6 * i + 3, 0, 3, 1) =
          -(a_hat - state_.pose.inverse().rotation() *
                        (options_.const_gravity * Eigen::Vector3d::UnitZ()));
    } else {
      res.block(6 * i + 3, 0, 3, 1) =
          -(state_.linear_velocity -
            (options_.const_gravity * Eigen::Vector3d::UnitZ() * dt) +
            state_.pose.rotation() * a_hat * dt);
    }

    // Measurement Jacobian
    Eigen::Matrix3d R_GtoI_jacob =
        state_.pose.rotation().toRotationMatrix().transpose();
    H.block(6 * i + 0, 3, 3, 3) = -Eigen::Matrix3d::Identity();
    if (!options_.integrated_accel_constraint) {
      H.block(6 * i + 3, 0, 3, 3) = Skew(R_GtoI_jacob * options_.const_gravity *
                                         Eigen::Vector3d::UnitZ());
      H.block(6 * i + 3, 6, 3, 3) = -Eigen::Matrix3d::Identity();
    } else {
      H.block(6 * i + 3, 0, 3, 3) =
          -R_GtoI_jacob.transpose() * Skew(a_hat) * dt;
      H.block(6 * i + 3, 6, 3, 3) = -R_GtoI_jacob.transpose() * dt;
      H.block(6 * i + 3, 9, 3, 3) = Eigen::Matrix3d::Identity();
    }
    // Measurement noise (convert from continuous to discrete)
    // Note the dt time might be different if we have "cut" any imu measurements
    R.block(6 * i + 0, 6 * i + 0, 3, 3) *= options_.angular_velocity_wnc / dt;
    if (!options_.integrated_accel_constraint) {
      R.block(6 * i + 3, 6 * i + 3, 3, 3) *= options_.accelerometer_wnc / dt;
    } else {
      R.block(6 * i + 3, 6 * i + 3, 3, 3) *= options_.accelerometer_wnc * dt;
    }
    dt_summed += dt;
  }

  // Multiply our noise matrix by a fixed amount
  // We typically need to treat the IMU as being "worst" to detect / not become
  // over confident
  R *= options_.zupt_noise_multiplier;

  // Next propagate the biases forward in time
  // NOTE: G*Qd*G^t = dt*Qd*dt = dt*Qc
  Eigen::MatrixXd Q_bias = Eigen::MatrixXd::Identity(6, 6);
  Q_bias.block(0, 0, 3, 3) *= dt_summed * options_.angular_velocity_random_walk;
  Q_bias.block(3, 3, 3, 3) *= dt_summed * options_.accelerometer_random_walk;

  // Chi2 distance check
  // NOTE: we also append the propagation we "would do before the update" if
  // this was to be accepted NOTE: we don't propagate first since if we fail the
  // chi2 then we just want to return and do normal logic
  Eigen::MatrixXd P_marg = Eigen::MatrixXd::Identity(h_size, h_size);
  //

  P_marg.block(3, 3, 6, 6) += Q_bias;
  Eigen::MatrixXd S = H * P_marg * H.transpose() + R;
  double chi2 = res.dot(S.llt().solve(res));
  boost::math::chi_squared chi_squared_dist(res.rows());
  auto chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
  if (chi2 < chi2_check * options_.zupt_chi2_multipler) {
#ifdef __DEBUG__
    LOG(INFO) << "zupt accepted - OpenVINS Inertial-based Detection (chi2 "
              << chi2 << " < chi2_check "
              << chi2_check * options_.zupt_chi2_multipler << ")";
#endif
    return true;
  }
#ifdef __DEBUG__
  LOG(INFO) << "zupt failed - OpenVINS Inertial-based Detection (chi2 " << chi2
            << " >= chi2_check " << chi2_check * options_.zupt_chi2_multipler
            << ")";
#endif
  return false;
}

bool ImageZeroVelocityDetect::IsZeroVelocity() {
  double disparity = 0;
  std::vector<float> disparitys;
  for (auto const key_points : data_base_) {
    if (key_points.second.size() >= 2) {
      disparitys.push_back((key_points.second.begin()->second -
                            key_points.second.rbegin()->second)
                               .norm());
    }
  }
  if (disparitys.size() < options_.min_disparity_num) {
#ifdef __DEBUG__
    LOG(INFO) << "zupt failed - Disparity-based Detection (disparitys.size() "
              << disparitys.size() << " < min_disparity_num "
              << options_.min_disparity_num << ")";
#endif
    return false;
  }
  disparity = std::accumulate(disparitys.begin(), disparitys.end(), 0.0) /
              disparitys.size();

  if (disparity > options_.max_disparity) {
#ifdef __DEBUG__
    LOG(INFO) << "zupt failed - Disparity-based Detection (disparity "
              << disparity << " > max_disparity " << options_.max_disparity
              << ", " << disparitys.size() << " features)";
#endif
    return false;
  }
#ifdef __DEBUG__
  LOG(INFO) << "zupt accepted - Disparity-based Detection (disparity "
            << disparity << " <= max_disparity " << options_.max_disparity
            << ", " << disparitys.size() << " features)";
#endif
  return true;
}
//
//
void UpdataZeroVelocity::AddImageKeyPoints(
    const common::Time& time, const std::vector<cv::KeyPoint>& points) {
  //
  for (auto const& point : points) {
    key_point_datas_[static_cast<uint64_t>(point.class_id)].emplace(
        time, Eigen::Vector2f{point.pt.x, point.pt.y});
  }
}
//
void UpdataZeroVelocity::AddImu(const ImuData& imu_data) {
  imu_datas_.push_back(imu_data);
}
//
UpdataZeroVelocity::UpdataZeroVelocity(const UpdataZeroVelocityOption& option)
    : options_(option) {
  //   for (int i = 1; i < 100; ++i) {
  //     boost::math::chi_squared chi_squared_dist(i);
  //     chi_squared_test_table[i] = boost::math::quantile(chi_squared_dist,
  //     0.05);
  //   }
}
//

UpdataZeroVelocity* UpdataZeroVelocity::AtState(const StateType& state) {
  std::vector<std::unique_ptr<ZeroVelocityDetect>> detect;
  //
  detect.emplace_back(std::make_unique<ImageZeroVelocityDetect>(
      options_.imag_disparity_option, key_point_datas_));
  detect.emplace_back(std::make_unique<OpenVinsZeroVelocityDetect>(
      options_.imu_velocity_option, imu_datas_, state));
  //
  zero_velocity_detects_ = std::move(detect);
  return this;
}
//
//
template <typename T>
void UpdataZeroVelocity::DropData(const common::Time& time,
                                  std::deque<T>* deque) {
  while (!deque->empty() && deque->front().time < time) {
    deque->pop_front();
  }
}
//

void UpdataZeroVelocity::DropData(const common::Time& time,
                                  ZeroVelocityDetect::KeyPointData* deque) {
  for (auto it = deque->begin(); it != deque->end();) {
    if (it->second.empty()) {
      it = deque->erase(it);
    } else {
      it->second.erase(it->second.begin(), it->second.lower_bound(time));
      ++it;
    }
  }
}
//

std::vector<uint64_t> UpdataZeroVelocity::GetOutlierPointClassId() {
  //
  if (!last_zupt_) return {};
  if (outlier_key_points_temp_) {
    return outlier_key_points_temp_.value();
  }
  std::vector<uint64_t> result;
  for (auto const key_points : key_point_datas_) {
    // LOG(INFO)<<key_points.first;
    if (key_points.second.size() >= 2) {
      auto norm = (key_points.second.begin()->second -
                   key_points.second.rbegin()->second)
                      .norm();
      // LOG(INFO)<<norm;

      if (norm >= options_.outlier_max_disparity) {
        result.push_back(key_points.first);
      }
    }
  }
  outlier_key_points_temp_ = std::move(result);
  return outlier_key_points_temp_.value();
}
//
bool UpdataZeroVelocity::IsZeroVelocity(const common::Time& time) {
  DropData(time - common::FromSeconds(options_.que_time_duration), &imu_datas_);
  DropData(time - common::FromSeconds(options_.que_time_duration),
           &key_point_datas_);
  bool is_zero_velocity = false;
  outlier_key_points_temp_.reset();
  for (auto& detect : zero_velocity_detects_) {
    if (detect->IsZeroVelocity()) {
      is_zero_velocity = true;
      break;
    }
  }
  if (is_zero_velocity) {
    Eigen::Vector3d acc_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d gry_sum = Eigen::Vector3d::Zero();
    for (const auto& imu_data : imu_datas_) {
      acc_sum += imu_data.linear_acceleration;
      gry_sum += imu_data.angular_velocity;
    }
    last_average_acc_ = acc_sum / imu_datas_.size();
    last_average_gry_ = gry_sum / imu_datas_.size();
  }

  if (last_zupt_ == is_zero_velocity) {
    state_count_++;
  } else {
    state_count_ = 0;
  }

  last_zupt_ = is_zero_velocity;

  if (last_state_ == 0 && state_count_ < options_.zupt_delay_frames) {
#ifdef __DEBUG__
    LOG(INFO) << "return last_state_ " << last_state_ << ", current state "
              << is_zero_velocity << ", state_count_ " << state_count_
              << " < zupt_delay_frames " << options_.zupt_delay_frames;
#endif
    return last_state_;
  }
  last_state_ = is_zero_velocity;
  return is_zero_velocity;
}
//
ceres::CostFunction* UpdataZeroVelocity::CostFunction() const {
  // return AutoZeroVelocityCostFuction::Create(
  //     {options_.optimize_weight, options_.optimize_bias_weight},
  //     last_average_acc_, last_average_gry_);
  return new ZeroVelocityCostFuction(
      {options_.optimize_weight, options_.optimize_bias_weight},
      last_average_acc_, last_average_gry_);
};
//

void UpdataZeroVelocity::AddToProblem(ceres::Problem* problem,
                                      std::array<double*, 3> pqv) const {
  LOG(INFO) << "Add ZeroVelocity factor.";
  gravity_ = options_.imu_velocity_option.const_gravity;
  problem->AddResidualBlock(CostFunction(), nullptr, pqv[0], pqv[1], pqv[2]);
}
//
}  // namespace estimator
}  // namespace jarvis