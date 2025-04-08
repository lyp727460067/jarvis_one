#include "estimator/updater_zero_velocity.h"

#include <unistd.h>
#include <random>

#include "glog/logging.h"
#include "transform/rigid_transform.h"
#include "transform/transform.h"
#include <random>
namespace jarvis {
namespace estimator {
//
namespace {
//
Eigen::MatrixXd Matrixmult(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) {
  CHECK_EQ(a.cols(), b.rows());
  Eigen::MatrixXd resutl = Eigen::MatrixXd::Zero(a.rows(), b.cols());
  for (int i = 0; i < a.rows(); i++) {
    for (int j = 0; j < b.cols(); j++) {
      resutl(i, j) = a.row(i) * b.col(j);
    }
  }
  return resutl;
};
const std::vector<double> kChi2095 = {
    3.841,   5.991,   7.815,   9.488,   11.070,  12.592,  14.067,  15.507,
    16.919,  18.307,  19.675,  21.026,  22.362,  23.685,  24.996,  26.296,
    27.587,  28.869,  30.144,  31.410,  32.671,  33.924,  35.172,  36.415,
    37.652,  38.885,  40.113,  41.337,  42.557,  43.773,  44.985,  46.194,
    47.400,  48.602,  49.802,  50.998,  52.192,  53.384,  54.572,  55.758,
    56.942,  58.124,  59.304,  60.481,  61.656,  62.830,  64.001,  65.171,
    66.339,  67.505,  68.669,  69.832,  70.993,  72.153,  73.311,  74.468,
    75.624,  76.778,  77.931,  79.082,  80.232,  81.381,  82.529,  83.675,
    84.821,  85.965,  87.108,  88.250,  89.391,  90.531,  91.670,  92.808,
    93.945,  95.081,  96.217,  97.351,  98.484,  99.617,  100.749, 101.879,
    103.010, 104.139, 105.267, 106.394, 107.521, 108.647, 109.773, 110.897,
    112.021, 113.145, 114.268, 115.390, 116.511, 117.632, 118.752, 119.872,
    120.991, 122.110, 123.228, 124.345};

double normal_quantile(double p) {
  if (p <= 0 || p >= 1) return std::numeric_limits<double>::quiet_NaN();

  // 近似正态分布分位数（Beasley-Springer-Moro算法）
  static const double a[] = {2.50662823884, -18.61500062529, 41.39119773534,
                             -25.44106049637};
  static const double b[] = {-8.4735109309, 23.08336743743, -21.06224101826,
                             3.13082909833};
  static const double c[] = {
      0.3374754822726147, 0.9761690190917186, 0.1607979714918209,
      0.0276438810333863, 0.0038405729373609, 0.0003951896511919,
      0.0000321767881768, 0.0000002888167364, 0.0000003960315187};

  double q = p - 0.5, r;

  if (std::abs(q) <= 0.42) {  // 中心区间
    r = q * q;
    return q * (((a[3] * r + a[2]) * r + a[1]) * r + a[0]) /
           ((((b[3] * r + b[2]) * r + b[1]) * r + b[0]) * r + 1.0);
  }

  r = p < 0.5 ? p : 1.0 - p;
  r = std::sqrt(-std::log(r));

  double val = (((c[8] * r + c[7]) * r + c[6]) * r + c[5]) * r + c[4];
  val = (((val * r + c[3]) * r + c[2]) * r + c[1]) * r + c[0];
  return (p < 0.5 ? -val : val);
}

// Wilson-Hilferty 近似计算卡方分布 0.95 分位数
double chi_squared_quantile(int df, double p) {
  if (df <= 0 || p <= 0 || p >= 1)
    return std::numeric_limits<double>::quiet_NaN();

  if (df > 30) {  // Wilson-Hilferty 近似
    double z = normal_quantile(p);
    double a = 2.0 / (9.0 * df);
    return df * std::pow(1 - a + z * std::sqrt(a), 3);
  } else {
    if (df <= 100) return kChi2095[df - 1];

    return std::numeric_limits<double>::quiet_NaN();  // 超出查表范围
  }
}

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
    const Eigen::Quaternion<T> q_a(p_a_[6], p_a_[3], p_a_[4], p_a_[5]);
    //
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_);
    const Eigen::Quaternion<T> q_b(p_b_[6], p_b_[3], p_b_[4], p_b_[5]);

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
    Eigen::Map<Eigen::Matrix<T, 9, 1>> residual(residuals);
    residual << T(weight_[0]) * delta_t,T(weight_[0]) * T(2.0) * delta_q.vec(),
         T(weight_[0]) * v_a;
        // -T(weight_[1]) * acc_bias_err, -T(weight_[1]) * gry_bias_err;
    return true;
  }

  static ceres::CostFunction* Create(std::array<double, 2> weight,
                                     const Eigen::Vector3d& average_acc,
                                     const Eigen::Vector3d& average_gry) {
    return new ceres::AutoDiffCostFunction<AutoZeroVelocityCostFuction, 9, 7,
                                           7, 9>(
        new AutoZeroVelocityCostFuction(weight, average_acc, average_gry));
  }

 private:
  const std::array<double, 2> weight_;
  const Eigen::Vector3d average_acc_;
  const Eigen::Vector3d average_gry_;
};

// p0 p1 v_bais_bias  (bias ignore)
constexpr uint32_t residuals_block_size = 9;
class ZeroVelocityCostFuction
    : public ceres::SizedCostFunction<residuals_block_size, 7, 7, 9> {
 public:
  ZeroVelocityCostFuction(const std::array<double, 2>& weight,
                          const Eigen::Vector3d& average_acc,
                          const Eigen::Vector3d& average_gry)
      : weight_(weight), average_acc_(average_acc), average_gry_(average_gry) {}
  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const {
    //

    Eigen::Vector3d p_a(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond q_a(parameters[0][6], parameters[0][3], parameters[0][4],
                          parameters[0][5]);

    Eigen::Vector3d p_b(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond q_b(parameters[1][6], parameters[1][3], parameters[1][4],
                          parameters[1][5]);
    //
    
    Eigen::Vector3d v_a(parameters[2][0], parameters[2][1], parameters[2][2]);
    //
    Eigen::Quaterniond delta_q = q_a.conjugate() * q_b;
    // LOG(INFO)<<-LogSo3(q_a.toRotationMatrix()*q_b.conjugate().toRotationMatrix());
    Eigen::Vector3d delta_t = p_b - p_a;
    //
    // LOG(INFO)<<delta_t.transpose();
    Eigen::Map<const Eigen::Vector3d> acc_bias(parameters[2] + 3);
    Eigen::Map<const Eigen::Vector3d> gry_bias(parameters[2] + 6);
    const Eigen::Vector3d gravity = Eigen::Vector3d::UnitZ() *  average_acc_.norm();
    Eigen::Vector3d acc_bias_err =
        acc_bias - q_a.conjugate() * gravity - average_acc_;
    Eigen::Vector3d gry_bias_err =  gry_bias- average_gry_;
    //
    Eigen::Matrix<double, residuals_block_size, residuals_block_size>
        sqrt_info =
            weight_[0] * Eigen::Matrix<double, residuals_block_size,
                                       residuals_block_size>::Identity();
    Eigen::Map<Eigen::Matrix<double, residuals_block_size, 1>> residual(
        residuals);
    residual << delta_t, 2 * delta_q.vec(), v_a;/*,-acc_bias_err,-gry_bias_err*/;
    // LOG(INFO)<<residual;
    // LOG(INFO)<<v_a;

    residual = sqrt_info * residual;
    //
    // order is  t0,t1,v0
    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<
            Eigen::Matrix<double, residuals_block_size, 7, Eigen::RowMajor>>
            jacobians_(jacobians[0]);
        jacobians_.setZero();
        jacobians_.block<3, 3>(0, 0) = -Eigen::Matrix<double, 3, 3>::Identity();
        jacobians_.block<3, 3>(3, 3) =
        -Utility::Qright(q_a.conjugate() * q_b).bottomRightCorner<3, 3>();
        //
        // jacobians_.block<3, 3>(9, 3) =
        //     q_a.conjugate().toRotationMatrix() * Skew(gravity);
        jacobians_ = sqrt_info *jacobians_;

      }
      if (jacobians[1]) {
        Eigen::Map<
            Eigen::Matrix<double, residuals_block_size, 7, Eigen::RowMajor>>
            jacobians_(jacobians[1]);
        jacobians_.setZero();
        jacobians_.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();
        jacobians_.block<3, 3>(3, 3) =
            Utility::Qleft(q_a.conjugate() * q_b).bottomRightCorner<3, 3>();
        jacobians_ = sqrt_info *jacobians_;
      }
      if (jacobians[2]) {
        Eigen::Map<
            Eigen::Matrix<double, residuals_block_size, 9, Eigen::RowMajor>>
            jacobians_(jacobians[2]);
        jacobians_.setZero();
        jacobians_.block<3, 3>(6, 0) = Eigen::Matrix<double, 3, 3>::Identity();
        // jacobians_.block<3, 3>(9, 3) = -Eigen::Matrix<double, 3, 3>::Identity();
        // jacobians_.block<3, 3>(12, 6) = -Eigen::Matrix<double, 3, 3>::Identity();
        jacobians_ = sqrt_info * jacobians_;
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
bool ImuZeroVelocityDetect::IsZeroVelocity(const common::Time& time) {
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
    if(imu_data.time>=time)break;
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

bool OpenVinsZeroVelocityDetect::IsZeroVelocity(const common::Time& time) {
  // Large final matrices used for update
  // LOG(INFO)<<state_.linear_velocity.norm();
  if (state_.linear_velocity.norm() > options_.zupt_max_velocity) {
    VLOG(kGlogLevel) << "zupt failed - OpenVINS Inertial-based Detection (vel"
                     << state_.linear_velocity.norm() << " > max zupt vel "
                     << options_.zupt_max_velocity << ")";
    return false;
  }
  int valid_data_size = 0;
  for (int i = 0; i < int(data_base_.size()); i++) {
    if (data_base_[i].time >= time) {
      valid_data_size = i + 1;
      break;
    }
  }
  if (valid_data_size < 2) {
    VLOG(kGlogLevel)
        << "zupt failed - OpenVINS Inertial-based Detection(data_base_.size() "
        << valid_data_size << " < 2 )";
    return false;
  }
  int h_size = (options_.integrated_accel_constraint) ? 12 : 9;
  int m_size = 6 * (valid_data_size - 1);
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
  for (int i = 0; i < valid_data_size - 1; i++) {
    // Precomputed values
    double dt =
        common::ToSeconds(data_base_.at(i + 1).time - data_base_.at(i).time);
    Eigen::Vector3d a_hat =
        data_base_.at(i).linear_acceleration -
        state_.linear_acceleration_bias;
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
      H.block(6 * i + 3, 0, 3, 3) = Skew(R_GtoI_jacob *
      options_.const_gravity *
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
  // We typically need to treat the IMU as being "worst" to detect / not
  // become
  // over confident
  R *= options_.zupt_noise_multiplier;

  // Next propagate the biases forward in time
  // NOTE: G*Qd*G^t = dt*Qd*dt = dt*Qc
  Eigen::MatrixXd Q_bias = Eigen::MatrixXd::Identity(6, 6);
  Q_bias.block(0, 0, 3, 3) *= dt_summed * options_.angular_velocity_random_walk;
  Q_bias.block(3, 3, 3, 3) *= dt_summed * options_.accelerometer_random_walk;

  // Chi2 distance check
  // NOTE: we also append the propagation we "would do before the update" if
  // this was to be accepted NOTE: we don't propagate first since if we fail
  // the
  // chi2 then we just want to return and do normal logic
  Eigen::MatrixXd P_marg = Eigen::MatrixXd::Identity(h_size, h_size);
  //

  P_marg.block(3, 3, 6, 6) += Q_bias;
  // Eigen::MatrixXd S = H * P_marg * H.transpose() + R;
  Eigen::MatrixXd S = Matrixmult(Matrixmult(H, P_marg), H.transpose()) + R;
  // usleep(50000);
  Eigen::VectorXd lltdot = S.llt().solve(res);
  double chi2 = res.dot(lltdot);
  // std::chi_squared_distribution<float> chi_squared_dist(res.rows());
  // CHECK(size_t(res.rows())<chi_squared_0_95_quantiles.size());
  auto chi2_check = chi_squared_quantile(res.rows(), 0.95);
  if (chi2 < chi2_check * options_.zupt_chi2_multipler) {
    LOG(INFO) << "zupt accepted - OpenVINS Inertial-based Detection (chi2 "
              << chi2 << " < chi2_check "
              << chi2_check * options_.zupt_chi2_multipler << ")";
    return true;
  }
  VLOG(kGlogLevel) << "zupt failed - OpenVINS Inertial-based Detection (chi2"
                   << chi2 << " >= chi2_check "
                   << chi2_check * options_.zupt_chi2_multipler << ")";
  return false;
}

bool ImageZeroVelocityDetect::IsZeroVelocity(const common::Time& time) {
  double disparity = 0;
  std::vector<float> disparitys;
  for (auto const& key_points : data_base_) {
    if (key_points.second.size() >= 2) {
      //
      const double parity = (key_points.second.begin()->second -
                             key_points.second.rbegin()->second)
                                .norm();
      if (parity < options_.max_disparity * 2) {
        disparitys.push_back(parity);
      }
    }
  }
  if (disparitys.size() < size_t(options_.min_disparity_num)) {
    VLOG(kGlogLevel)
        << "zupt failed - Disparity-based Detection (disparitys.size() "
        << disparitys.size() << " < min_disparity_num "
        << options_.min_disparity_num << ")";
    return false;
  }
  disparity = std::accumulate(disparitys.begin(), disparitys.end(), 0.0) /
              disparitys.size();

  if (disparity > options_.max_disparity) {
    VLOG(kGlogLevel) << "zupt failed - Disparity-based Detection (disparity "
                     << disparity << " > max_disparity "
                     << options_.max_disparity << ", " << disparitys.size()
                     << " features)";
    return false;
  }
  LOG(INFO) << "Image zupt accepted - Disparity-based Detection (disparity "
            << disparity << " <= max_disparity " << options_.max_disparity
            << ", " << disparitys.size() << " features)";
  return true;
}
//
//
void UpdataZeroVelocity::AddImageKeyPoints(
    const common::Time& time, const ImageFeatureTrackerData& feature_frame) {
  //
  lates_time_ = time;
  for (auto const& point : feature_frame.data->features) {
    key_point_datas_[static_cast<uint64_t>(point.first)].emplace(
        time, point.second.camera_features[0].uv);
  }
}
//
void UpdataZeroVelocity::AddImu(const sensor::ImuData& imu_data) {
  imu_datas_.push_back(imu_data);
  if (imu_datas_.size() >= 6000) {
    imu_datas_.pop_front();
  }
}
//
UpdataZeroVelocity::UpdataZeroVelocity(const UpdataZeroVelocityOption& option)
    : options_(option) {
  zero_velocity_detects_.emplace_back(std::make_unique<ImageZeroVelocityDetect>(
      options_.imag_disparity_option, key_point_datas_));
  zero_velocity_detects_.emplace_back(
      std::make_unique<OpenVinsZeroVelocityDetect>(options_.imu_velocity_option,
                                                   imu_datas_));
  //   for (int i = 1; i < 100; ++i) {
  //     boost::math::chi_squared chi_squared_dist(i);
  //     chi_squared_test_table[i] = boost::math::quantile(chi_squared_dist,
  //     0.05);
  //   }
}
//

UpdataZeroVelocity* UpdataZeroVelocity::AtState(const StateType& state) {
  static_cast<OpenVinsZeroVelocityDetect*>(zero_velocity_detects_[1].get())
      ->UpdateState(state);
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
  std::stringstream info;
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
  for (auto const &key_points : key_point_datas_) {
    // LOG(INFO)<<key_points.first;
    if (key_points.second.size() >= 2) {
      auto norm = (key_points.second.begin()->second -
                   key_points.second.rbegin()->second)
                      .norm();
      if (norm >= options_.outlier_max_disparity) {
        result.push_back(key_points.first);
      }
    }
  }
  outlier_key_points_temp_ = std::move(result);
  return outlier_key_points_temp_.value();
}
//

// bool UpdataZeroVelocity::IsZeroVelocity() {
//   return IsZeroVelocity(lates_time_);
// }
bool UpdataZeroVelocity::IsZeroVelocity(const common::Time& time) {
  DropData(time - common::FromSeconds(options_.que_time_duration), &imu_datas_);
  DropData(time - common::FromSeconds(options_.que_time_duration),
           &key_point_datas_);
  bool is_zero_velocity = false;
  outlier_key_points_temp_.reset();
  for (auto& detect : zero_velocity_detects_) {
    if (detect->IsZeroVelocity(time)) {
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
    LOG(INFO) << "return last_state_ " << last_state_
                     << ", current state " << is_zero_velocity
                     << ", state_count_ " << state_count_
                     << " < zupt_delay_frames " << options_.zupt_delay_frames;
    return last_state_;
  }
  last_state_ = is_zero_velocity;
  return is_zero_velocity;
}
//
ceres::CostFunction* UpdataZeroVelocity::CostFunction() const {
  // return AutoZeroVelocityCostFuction::Create(
      // {options_.optimize_weight, options_.optimize_bias_weight},
      // last_average_acc_, last_average_gry_);
  // LOG(INFO) << last_average_acc_.transpose() << " "
  //           << last_average_gry_.transpose();
  return new ZeroVelocityCostFuction(
      {options_.optimize_weight, options_.optimize_bias_weight},
      last_average_acc_, last_average_gry_);
};
//

void UpdataZeroVelocity::AddToProblem(ceres::Problem* problem,
                                      ceres::LossFunction* loss_function,
                                      const std::array<double*, 3>&  pqv) const {
  // LOG(INFO) << "Add ZeroVelocity factor.";
  gravity_ = options_.imu_velocity_option.const_gravity;
  problem->AddResidualBlock(CostFunction(), loss_function, pqv[0], pqv[1],
                            pqv[2]);
}
//
}  // namespace estimator
}  // namespace jarvis
