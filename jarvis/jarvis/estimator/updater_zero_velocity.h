#ifndef __METABOUNDS_VIO_UPDATA_ZER0_VELOCITY_H
#define __METABOUNDS_VIO_UPDATA_ZER0_VELOCITY_H
#include <deque>
#include <memory>
#include <optional>

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "key_frame_data.h"

namespace jarvis {
namespace estimator {
//

class ZeroVelocityDetect {
 public:
  using KeyPointData =
      std::unordered_map<uint64_t, std::map<common::Time, Eigen::Vector2f>>;
  virtual bool IsZeroVelocity() = 0;
  virtual ~ZeroVelocityDetect() {}
};
//
struct ImuZeroVelocityDetectOption {
  bool integrated_accel_constraint;
};
class ImuZeroVelocityDetect : public ZeroVelocityDetect {
 public:
  ImuZeroVelocityDetect(const ImuZeroVelocityDetectOption& option,
                        const std::deque<mapping::ImuData>& data_base)
      : options_(option), data_base_(data_base) {}
  virtual bool IsZeroVelocity() override;

 protected:
  const ImuZeroVelocityDetectOption options_;
  const std::deque<mapping::ImuData>& data_base_;
};
//
struct StateType {
  common::Time time;
  transform::Rigid3d pose;
  Eigen::Vector3d linear_velocity;
  Eigen::Vector3d linear_acceleration_bias;
  Eigen::Vector3d angular_velocity_bias;
};
struct OpenVinsZeroVelocityDetectOption {
  ImuZeroVelocityDetectOption base_option;
  bool integrated_accel_constraint;
  //

  double angular_velocity_wnc;
  double accelerometer_wnc;

  double angular_velocity_random_walk;
  double accelerometer_random_walk;
  //
  float zupt_noise_multiplier = 10;
  double const_gravity = 9.81;
  float zupt_chi2_multipler = 1;
  float zupt_max_velocity = 0.1;
};
//
class OpenVinsZeroVelocityDetect : public ImuZeroVelocityDetect {
 public:
  OpenVinsZeroVelocityDetect(const OpenVinsZeroVelocityDetectOption& option,
                             const std::deque<mapping::ImuData>& data_base,
                             const StateType& state)
      : ImuZeroVelocityDetect(option.base_option, data_base),
        options_(option),
        state_(state) {}
  bool IsZeroVelocity() override;

 private:
  const OpenVinsZeroVelocityDetectOption options_;
  const StateType& state_;
};

//
struct ImageZeroVelocityDetectOption {
  int min_disparity_num = 20;
  float max_disparity = 1;
};
//

class ImageZeroVelocityDetect : public ZeroVelocityDetect {
 public:
  ImageZeroVelocityDetect(const ImageZeroVelocityDetectOption& option,
                          const KeyPointData& data_base)
      : options_(option), data_base_(data_base) {}
  //
  bool IsZeroVelocity() override;

 private:
  const ImageZeroVelocityDetectOption options_;
  const KeyPointData& data_base_;
};
//
struct UpdataZeroVelocityOption {
  OpenVinsZeroVelocityDetectOption imu_velocity_option;
  ImageZeroVelocityDetectOption imag_disparity_option;
  double optimize_weight = 100;
  double optimize_bias_weight = 0;
  float que_time_duration = 1;
  float outlier_max_disparity = 0.5;
  float zupt_max_velocity = 0.1;
  unsigned int zupt_delay_frames = 0;
};
//
class UpdataZeroVelocity {
 public:
  UpdataZeroVelocity(const UpdataZeroVelocityOption& option);
  bool IsZeroVelocity(const common::Time& time);
  //
  UpdataZeroVelocity* AtState(const StateType&);
  ceres::CostFunction* CostFunction() const;
  // order is T0,T1,V0
  void AddToProblem(ceres::Problem* problem, std::array<double*, 3> pqv) const;
  //
  void AddImu(const mapping::ImuData& imu_data);
  void AddImageKeyPoints(const common::Time& time,
                         const std::vector<cv::KeyPoint>& points);
  std::vector<uint64_t> GetOutlierPointClassId();

  const std::deque<mapping::ImuData>& getImuData() const { return imu_datas_; }

 private:
  template <typename T>
  void DropData(const common::Time& time, std::deque<T>* deque);
  void DropData(const common::Time& time,
                ZeroVelocityDetect::KeyPointData* deque);
  std::optional<std::vector<uint64_t>> outlier_key_points_temp_;
  std::deque<mapping::ImuData> imu_datas_;
  //
  ZeroVelocityDetect::KeyPointData key_point_datas_;
  Eigen::Vector3d last_average_acc_;
  Eigen::Vector3d last_average_gry_;
  bool last_zupt_ = false;
  bool last_state_ = 1;  // 1: zupt, 0: motion
  int state_count_ = 0;
  mutable double gravity_ = 9.81;
  //
  std::vector<std::unique_ptr<ZeroVelocityDetect>> zero_velocity_detects_;
  UpdataZeroVelocityOption options_;
};
//
}  // namespace estimator
}  // namespace jarvis
#endif