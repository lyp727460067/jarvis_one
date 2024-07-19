#include "jarvis/estimator/imu_extrapolator.h"

namespace jarvis {
namespace estimator {

class ImuExtrapolator::ImuIntegral {
 public:
  explicit ImuIntegral(const ImuState state, common::Time& time)
      : state_(state), time_(time) {}
  void Reset(const common::Time& time, const ImuState state) {
    time_ = time;
    state_ = state;
    last_imu_ =
        sensor::ImuData{time, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  }

  common::Time Time()const { return  time_; }
  //
  void AddLastImuObservation(const sensor::ImuData& imu) { last_imu_ = imu; }
  void AddImuObservation(const sensor::ImuData& imu) { imu_observation_ = imu; }
  //
  ImuState State() { return state_; }
  //
  void Advance(const common::Time& time) {
    //
    time_ = time;
    const ImuState& state = state_;
    const sensor::ImuData& imu = imu_observation_;
    //
    const double dt = common::ToSeconds(time_ - last_imu_.time);
    const Eigen::Vector3d av_angular_velocity =
        0.5 * (last_imu_.angular_velocity + imu.angular_velocity) -
        state.angular_velocity_bias;
    //
    const Eigen::Quaterniond delta_q =
        transform::AngleAxisVectorToRotationQuaternion(
            Eigen::Vector3d(av_angular_velocity * dt));
    //
    const Eigen::Quaterniond new_quatation =
        (state.pose.rotation() * delta_q).normalized();
    //

    const Eigen::Vector3d av_g_acceleration =
        state.pose.rotation() * (0.5 * (last_imu_.linear_acceleration +
                                        delta_q * imu.linear_acceleration) -
                                 state.linear_acceleration_bias) -
        state.gravity;
    // LOG(INFO)<<av_g_acceleration;
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
    state_ = ImuState{transform::Rigid3d(new_translation, new_quatation),
                      new_velocity};
  }

 private:
  ImuState state_;
  common::Time time_;
  sensor::ImuData last_imu_;
  sensor::ImuData imu_observation_;
};

ImuExtrapolator::ImuExtrapolator():imu_intergral_(nullptr) {
}

ImuState ImuExtrapolator::Exrapolate(const common::Time& time) {
  //
  if(imu_intergral_==nullptr) return ImuState{};
  if (imu_datas_.empty() || time < imu_datas_.front().time) {
    imu_intergral_->AddImuObservation(sensor::ImuData{
        time, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()});
    imu_intergral_->Advance(time);
    return imu_intergral_->State();
  }

  //
  if (imu_intergral_->Time() < imu_datas_.front().time) {
    imu_intergral_->AddLastImuObservation(imu_datas_.front());
  }
  //
  //
  auto it = std::lower_bound(
      imu_datas_.begin(), imu_datas_.end(), imu_intergral_->Time(),
      [](const sensor::ImuData& imu_data, const common::Time& time) {
        return imu_data.time < time;
      });
  //
  if(it != imu_datas_.end()){
    imu_intergral_->AddLastImuObservation(*it);
    ++it;
  }
  while (it != imu_datas_.end() &&
         (it->time < time)) {
    imu_intergral_->AddImuObservation(*it);
    imu_intergral_->Advance(it->time);
    ++it;
  }

  return imu_intergral_->State();
}
//
void ImuExtrapolator::TrimImuData(const common::Time& t) {
  while (!imu_datas_.empty() &&
         (imu_datas_.front().time) < t) {
    imu_datas_.pop_front();
  }
}
//
void ImuExtrapolator::AddImu(const sensor::ImuData& imu_data) {
  imu_datas_.push_back(imu_data);
}
//
//
void ImuExtrapolator::AddState(const common::Time& time,
                               const ImuState& state) {
  //
  if (imu_intergral_ == nullptr) {
    common::Time tracker_start = time;
    if (!imu_datas_.empty()) {
      tracker_start = std::min(tracker_start, imu_datas_.front().time);
    }
    imu_intergral_ = std::make_unique<ImuIntegral>(state, tracker_start);
  } else {
    imu_intergral_->Reset(time, state);
  }
  //
  //
  TrimImuData(time);
  LOG_EVERY_N(WARNING, 60) <<"imu date behind .. "<<imu_datas_.size();
}

ImuExtrapolator::~ImuExtrapolator() {}
//
}  // namespace estimator
}  // namespace jarvis
