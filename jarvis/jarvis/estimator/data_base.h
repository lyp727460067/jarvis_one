#ifndef JARVIS_ESTIMATOR_IMU_DATA_BASE_
#define JARVIS_ESTIMATOR_IMU_DATA_BASE_
#include <deque>
#include <map>

#include "common/time.h"
#include "jarvis/sensor/imu_data.h"
#include "memory.h"
#include "sensor/odometry_data.h"
#include "transform/transform_interpolation_buffer.h"
namespace jarvis {

namespace estimator {

//
sensor::OdometryData Interpolate(const sensor::OdometryData &start,
                                 const sensor::OdometryData &end,
                                 const common::Time &time);
sensor::ImuData Interpolate(const sensor::ImuData &start,
                            const sensor::ImuData &end,
                            const common::Time &time);
class SensorBase {
 public:
  virtual common::Time Time() const = 0;
  virtual ~SensorBase() {}
};
template <typename Sensor>
class DataType : public SensorBase {
 public:
  DataType(const Sensor &sensor) : data_(sensor) {}
  ~DataType(){};
  //
  virtual common::Time Time() { return data_.time; }
  const Sensor Data() { return data_; }

 private:
  const Sensor &data_;
};
// 注意在数据传入系统的时候可以把odom和imu的数据认为延迟100ms在补回去
// 保证IMU间隔数据大于图像的间隔
class DataBase {
 public:
  DataBase(const double data_duration);
  //
  void TrimData(const common::Time &time);
  //

  void AddOdometry(const sensor::OdometryData &odom);
  void AddImu(const sensor::ImuData &imu);

  //
  bool HasImuData(const common::Time &time) const;
  bool HasOdometryData(const common::Time &time) const;

  sensor::OdometryData InterpolateOdometry(const common::Time &time) const;
  sensor::ImuData InterpolateImu(const common::Time &time) const;
  //
  sensor::OdometryData InterpolateOdometryUseLastData(
      const common::Time &time) const;
  //
  //
  std::vector<sensor::ImuData> GetImuIntervalData(
      const common::Time &first_time, const common::Time &end_time);
  //
  // std::vector<sensor::OdometryData> GetOdomeIntervalData(
  //     const common::Time &first_time, const common::Time &end_time);
  //

 private:
  const double data_duration_;
  common::Time start_time_;
  //
  std::deque<sensor::OdometryData> odometry_data_;
  std::deque<sensor::ImuData> imu_data_;
};
}  // namespace estimator
}  // namespace jarvis

#endif