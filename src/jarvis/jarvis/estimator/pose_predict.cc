#include "jarvis/estimator/pose_predict.h"
#include "jarvis/transform/transform.h"
namespace jarvis {
namespace estimator {
//
constexpr int KmaxImuNum = 1000;

// 没有预测只有积分//database 近来的数据进行了插值
class PosePredit::IntegratorImpl {
 public:
  //
  IntegratorImpl(const common::Time& time) : time_(time) {}
  //
  void Advance(ImuState* state_, const common::Time& time) {
    //
    const sensor::ImuData& imu = imu_observation_;
    const ImuState state = *state_;
    const double dt = common::ToSeconds(time - last_imu_.time);
    //
    const Eigen::Vector3d av_angular_velocity =
        0.5 * (last_imu_.angular_velocity + imu.angular_velocity) - state.bg;
    //
    
    const Eigen::Quaterniond delta_q =
        transform::AngleAxisVectorToRotationQuaternion(
            Eigen::Vector3d(av_angular_velocity * dt));
    //
    const Eigen::Quaterniond new_quatation = (state.q * delta_q).normalized();
    //
    //
    const Eigen::Vector3d av_g_acceleration =
        state.q * (0.5 * (last_imu_.linear_acceleration - state.ba +
                          delta_q * (imu.linear_acceleration - state.ba))) -
        state.g;
    state_->p = state.p + state.v * dt + 0.5 * av_g_acceleration * dt * dt;
    // LOG(INFO)<<new_translation.transpose();
    state_->v = state.v + dt * av_g_acceleration;
    state_->q = new_quatation;
    time_ = time;
    last_imu_ = imu_observation_;
  }
  //
  void AddLastImuObservation(const sensor::ImuData& imu) { last_imu_ = imu; }
  void AddImuObservation(const sensor::ImuData& imu) { imu_observation_ = imu; }

  //
  common::Time Time() { return time_; }

 private:
  common::Time time_;
  sensor::ImuData last_imu_;
  sensor::ImuData imu_observation_;
};

///
void PosePredit::AddState(common::Time& time, const ImuState& state) {
  if (imu_intergral_ == nullptr) {
    if (!imu_datas_.empty()) {
      ImuState state_tmp;
      common::Time tracker_start = time;
      if (!imu_datas_.empty()) {
        tracker_start = std::min(tracker_start, imu_datas_.front().time);
      }
      //
      imu_intergral_ = std::make_unique<IntegratorImpl>(tracker_start);
      //
      //
      while (imu_intergral_->Time() < time && !imu_datas_.empty()) {
        imu_intergral_->AddImuObservation(imu_datas_.back());
        imu_intergral_->Advance(&state_tmp, imu_datas_.back().time);
        imu_datas_.pop_front();
      }
    }
  }
  sensor::ImuData last_imu_data{time, Eigen::Vector3d{0, 0, 9.81},
                                Eigen::Vector3d::Zero()};
  while (!imu_datas_.empty() && (imu_datas_.front().time) < time) {
    last_imu_data = imu_datas_.front();
    imu_datas_.pop_front();
  }
  if (imu_intergral_) {
    imu_intergral_->AddImuObservation(last_imu_data);
    imu_intergral_->Advance(&state_, time);
  }
  state_ = state;
}
//
//
// /
ImuState PosePredit::PreditDataBase(const ImuState& imu_stat,
                                    DataBase* data_base,
                                    const common::Time& start_time,
                                    const common::Time& end_time) {
  ImuState state = imu_stat;




  const auto imu_data = data_base->GetImuIntervalData(start_time, end_time);

  LOG(INFO) << "image interval [" << start_time << "," << end_time
            << ",peri: " << common::ToSeconds(end_time - start_time)
            << "],imu num: " << imu_data.size();
  //
  if (imu_data.empty()) {
    LOG(WARNING) << "Imu data empyt.[" << start_time << "," << end_time << "]";
    return imu_stat;
  }
  IntegratorImpl imu_integer(imu_data.begin()->time);
  imu_integer.AddLastImuObservation(*imu_data.begin());
  for (int i = 1; i < imu_data.size(); i++) {
    imu_integer.AddImuObservation(imu_data[i]);
    imu_integer.Advance(&state, imu_data[i].time);
  }
  return state;
}
//
//
ImuState PosePredit::Predit(const common::Time& time) {
  // if(imu_intergral_==nullptr) return ImuState{};
  // //
  // if (imu_datas_.empty() || time < imu_datas_.front().time) {
  //   imu_intergral_->AddImuObservation(sensor::ImuData{
  //       time, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()});
  //   imu_intergral_->Advance(&state_, time);
  //   return state_;
  // }

  // if (time < imu_intergral_->Time()) {
  //   LOG(WANRING) << "Predit time < imu_intergral_-> time";
  //   return state_;
  // }
  // //
  // if (imu_intergral_->Time() < imu_datas_.front().time) {
  //   imu_intergral_->AddImuObservation(imu_datas_.front());
  //   imu_intergral_->Advance(&state_, time);
  // }
  // //

  // //
  // auto it = std::lower_bound(
  //     imu_datas_.begin(), imu_datas_.end(), imu_intergral_->Time(),
  //     [](const sensor::ImuData& imu_data, const common::Time& time) {
  //       return imu_data.time < time;
  //     });
  // //
  // if(it != imu_datas_.end()){
  //   imu_intergral_->AddLastImuObservation(*it);
  //   ++it;
  // }
  // while (it != imu_datas_.end() &&
  //        (it->time < time)) {
  //   imu_intergral_->AddImuObservation(*it);
  //   imu_intergral_->Advance(it->time);
  //   ++it;
  // }

  // return imu_intergral_->State();
}
//
void PosePredit::AddOdomData(const sensor::OdometryData& odom) {
  odom_datas_.push_back(odom);
  if (odom_datas_.size() > KmaxImuNum) {
    LOG_EVERY_N(WARNING, 10) << "Odom data size too big.";
    odom_datas_.pop_front();
  }
}
//
void PosePredit::AddImuData(const sensor::ImuData& imu) {
  imu_datas_.push_back(imu);
  if (imu_datas_.size() > KmaxImuNum) {
    imu_datas_.erase(imu_datas_.begin());
    LOG_EVERY_N(WARNING, 10)
        << "Imu data too big,maby need add state..[" << imu_datas_.begin()->time
        << "-" << imu_datas_.back().time << "]";
  }
}
//
void PosePredit::TrimImuData(const common::Time& t) {
  while (!imu_datas_.empty() && (imu_datas_.front().time) < t) {
    imu_datas_.pop_front();
  }
}
//

PosePredit::PosePredit() {}
PosePredit::~PosePredit() {}

}  // namespace estimator
}  // namespace jarvis