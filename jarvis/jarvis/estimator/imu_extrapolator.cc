#include "imu_extrapolator.h"

namespace jarvis {
namespace estimator {

class ImuExtrapolator::ImuIntegral {
 public:
  explicit ImuIntegral(const sensor::ImuData& imu) : last_imu_(imu) {}
   common::Time Time() {return  last_imu_.time; }
  //



  ImuState Advance(const sensor::ImuData& imu, const ImuState& state_) {
    //
    const auto& state = *state_.data;
    const double dt = common::ToSeconds(imu.time - last_imu_.time);
    const Eigen::Vector3d av_angular_velocity =
        0.5 * (last_imu_.angular_velocity + imu.angular_velocity) -
        state.angular_velocity_bias;
    //
    const Eigen::Quaterniond delta_q =
        transform::AngleAxisVectorToRotationQuaternion(
            Eigen::Vector3d(av_angular_velocity * dt));
    //
    const Eigen::Quaterniond new_quatation = (state.pose.rotation() * delta_q).normalized();
    //

    const Eigen::Vector3d av_g_acceleration =
        state.pose.rotation() * (0.5 * (last_imu_.linear_acceleration +
                                        delta_q * imu.linear_acceleration) -
                                 state.linear_acceleration_bias)-state.gravity;
    LOG(INFO)<<av_g_acceleration; 
    const Eigen::Vector3d new_translation = state.pose.translation() +
                                            state.velocity * dt +
                                            0.5 * av_g_acceleration * dt * dt;
    const Eigen::Vector3d new_velocity =
        state.velocity + dt * av_g_acceleration;

    if (state.covariance.has_value()) {
    }
    if (state.jacobian.has_value()) {
    }
    last_imu_ = imu;
    return ImuState{std::make_shared<ImuState::Data>(ImuState::Data{
        transform::Rigid3d(new_translation, new_quatation), new_velocity})};
  }

 private:
  sensor::ImuData last_imu_;
};

ImuExtrapolator::ImuExtrapolator() {}

ImuState ImuExtrapolator::Exrapolate(const common::Time& time) {
  //
  if (imu_state_.size() != 2 || imu_intergral_ == nullptr) return ImuState{};
  //
  CHECK_GE(time, imu_datas_.front().time);
  auto it = std::lower_bound(
      imu_datas_.begin(), imu_datas_.end(), imu_intergral_->Time(),
      [](const sensor::ImuData& imu_data, const common::Time & time) {
        return imu_data.time < time;
      });
  ++it;
  //
  while (it != imu_datas_.end() && it->time < time) {
    LOG(INFO)<<it->time;
    imu_intergral_state_ = imu_intergral_->Advance(*it, imu_intergral_state_);
    ++it;
  }
  return imu_intergral_state_;
}
//
void ImuExtrapolator::TrimImuData(const common::Time & t) {
  while (!imu_datas_.empty()&&(imu_datas_.front().time) < t) {
    imu_datas_.pop_front();
  }
}
//
void ImuExtrapolator::AddImu(const sensor::ImuData& imu_data) {
  imu_datas_.push_back(imu_data);
}
//
//
void ImuExtrapolator::AddState(const common::Time & t, const ImuState& state) {
  //
  imu_state_.push_back(state);
  if (imu_state_.size() > 2) {
    imu_state_.pop_front();
  }
  //
  if (imu_datas_.size() < 1) {
    LOG(WARNING) << "Imu date empyt..";
    return;
  }
  auto last_imu_data = imu_datas_.front();
  while (!imu_datas_.empty()&&(imu_datas_.front().time) < t) {
    LOG(INFO)<<imu_datas_.front().time;
    LOG(INFO)<<t;
    last_imu_data = imu_datas_.front();
    imu_datas_.pop_front();
  }
  LOG(INFO)<<imu_datas_.size();
  imu_intergral_ = std::make_unique<ImuIntegral>(last_imu_data);
  auto last_state = state;
  auto it = imu_datas_.begin();
  while (it!=imu_datas_.end()&&  it->time < t) {
      last_state = imu_intergral_->Advance(*it, last_state);
      ++it;
  }
  //
  LOG(INFO)<<imu_datas_.size();
  imu_intergral_state_= last_state;


}

ImuExtrapolator::~ImuExtrapolator() {}
//
}  // namespace estimator
}  // namespace jarvis
