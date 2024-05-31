#ifndef _IMU_EXTRAPOLATOR_H
#define _IMU_EXTRAPOLATOR_H
#include "jarvis/transform/transform.h"
#include "jarvis/common/time.h"
#include "jarvis/sensor/imu_data.h"
#include <deque>
#include  <optional>
namespace jarvis {
namespace estimator {
//
struct ImuState {
  struct Data {
    transform::Rigid3d pose;
    Eigen::Vector3d velocity{0, 0, 0};
    Eigen::Vector3d linear_acceleration_bias{0, 0, 0};
    Eigen::Vector3d angular_velocity_bias{0, 0, 0};
    Eigen::Vector3d gravity{0, 0, 9.8};
    std::optional<Eigen::Matrix<double, 15, 15>> jacobian;
    std::optional<Eigen::Matrix<double, 15, 15>> covariance;
  };
  std::shared_ptr<Data> data;
};
//
struct ImuExtrapolatorOption {
  bool exrapolate_jaobic;
};
//
class ImuExtrapolator {
 public:
  ImuExtrapolator();
  ~ImuExtrapolator();
  void AddState(const double& t, const ImuState& state);
  void AddImu(const sensor::ImuData& imu_data);
  ImuState Exrapolate(const double& time);

 private:
  void TrimImuData(const double &t);
  class ImuIntegral;
  std::unique_ptr<ImuIntegral> imu_intergral_;

  std::deque<sensor::ImuData> imu_datas_;
  std::deque<ImuState> imu_state_;
  ImuState imu_intergral_state_;
};
//
}  // namespace estimator
}  // namespace jarvis

#endif
