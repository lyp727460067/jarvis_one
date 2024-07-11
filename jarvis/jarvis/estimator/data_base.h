#ifndef JARVIS_ESTIMATOR_IMU_DATA_BASE_
#define JARVIS_ESTIMATOR_IMU_DATA_BASE_
#include "jarvis/sensor/imu_data.h"
#include <map>
#include "common/time.h"
namespace jarvis {

namespace estimator {

class DataBase {
 public:
  void AddImuData(const sensor::ImuData &imu_data);
  std::vector<sensor::ImuData> GetImuIntervalData(
      const common::Time &first_time, const common::Time &sencond_time);
  
 private:
  std::map<common::Time, sensor::ImuData> imu_datas_;
};
}
}  // namespace jarvis

#endif