#ifndef JARVIS_ESTIMATOR_POSE_PREDICT_H
#define JARVIS_ESTIMATOR_POSE_PREDICT_H
//
#include <deque>

#include "jarvis/estimator/data_base.h"
#include "jarvis/sensor/odometry_data.h"
#include "jarvis/estimator/factor/integration_base.h"
namespace jarvis {
namespace estimator {


class PosePredit {
 public:
  PosePredit() ;
  void AddState(common::Time& time, const ImuState& state);
  void AddOdomData(const sensor::OdometryData& odom);
  void AddImuData(const sensor::ImuData& imu);
  ImuState Predit(const common::Time& time);
  ImuState PreditDataBase(const ImuState& imu_stat, DataBase* data_base,
                          const common::Time& start_time,
                          const common::Time& end_time);
  ~PosePredit();

 private:
  class IntegratorImpl;
  std::unique_ptr<IntegratorImpl> imu_intergral_;
  void TrimImuData(const common::Time& t);
  std::deque<sensor::ImuData> imu_datas_;
  std::deque<sensor::OdometryData> odom_datas_;
  ImuState state_;
};

}  // namespace estimator
}  // namespace jarvis

#endif