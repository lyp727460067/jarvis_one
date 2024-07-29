#include "data_base.h"

namespace jarvis {

namespace estimator {
inline sensor::OdometryData Interpolate(const sensor::OdometryData &start,
                                        const sensor::OdometryData &end,
                                        const common::Time &time) {

  const transform::TimestampedTransform slert = transform::Interpolate(
      transform::TimestampedTransform{start.time, start.pose},
      transform::TimestampedTransform{end.time, end.pose}, time);
  LOG(INFO)<<start.pose<<" "<<start.time;
  LOG(INFO)<<end.pose<<" "<<end.time;
  LOG(INFO)<<slert.transform<<" "<<time;
  return sensor::OdometryData{time, slert.transform};
}

//
sensor::ImuData Interpolate(const sensor::ImuData &start,
                            const sensor::ImuData &end,
                            const common::Time &time) {
  const double duration = common::ToSeconds(end.time - start.time);
  const double factor = common::ToSeconds(time - start.time) / duration;
  const Eigen::Vector3d linear_acceleration =
      (1 - factor) * start.linear_acceleration +
      end.linear_acceleration * factor;
  const Eigen::Vector3d angular_velocity =
      (1 - factor) * start.angular_velocity + end.angular_velocity * factor;
  return sensor::ImuData{time, linear_acceleration, angular_velocity};
}

template <typename Sensor>
bool HasDataForTime(const std::deque<Sensor> *data_base,
                    const common::Time &time) {
  return data_base->size() > 2 && data_base->front().time < time &&
         time < data_base->back().time;
}

template <typename Sensor>
Sensor InterpolateSensor(const std::deque<Sensor> *data_base,
                         const common::Time &time) {
  auto data =
      std::upper_bound(data_base->begin(), data_base->end(), time,
                       [](const common::Time &time, const Sensor &data) {
                         return time < data.time;
                       });
  return Interpolate(*std::prev(data), *data, time);
}
//

sensor::OdometryData DataBase::InterpolateOdometryUseLastData(
    const common::Time &time) const {
  CHECK(odometry_data_.size() > 2) << "Odom size <2";
  return Interpolate(*std::prev(odometry_data_.end(), 2),
                     *std::prev(odometry_data_.end(), 1), time);
}
//

//
sensor::OdometryData DataBase::InterpolateOdometry(
    const common::Time &time) const {
  return InterpolateSensor(&odometry_data_, time);
}

//
sensor::ImuData DataBase::InterpolateImu(const common::Time &time) const {
  return InterpolateSensor(&imu_data_, time);
}
//
sensor::ImuData DataBase::InterpolateImuUseLastData(
    const common::Time &time) const {
  CHECK(imu_data_.size() > 2) << "imu size <2";
  return Interpolate(*std::prev(imu_data_.end(), 2),
                     *std::prev(imu_data_.end(), 1), time);
}
//
bool DataBase::HasImuData(const common::Time &time) const {
  return HasDataForTime(&imu_data_, time);
}
//
bool DataBase::HasOdometryData(const common::Time &time) const {
  return HasDataForTime(&odometry_data_, time);
}
//
//
std::vector<sensor::ImuData> DataBase::GetImuIntervalData(
    const common::Time &first_time, const common::Time &end_time) {
  //
  if (!HasImuData(first_time)) {
    LOG(WARNING) << "start _time not in deque" << first_time<<imu_data_.front().time;
    return {};
  }
  auto data = std::upper_bound(
      imu_data_.begin(), imu_data_.end(), first_time,
      [](const common::Time &time, const sensor::ImuData &imu_data) {
        return time < imu_data.time;
      });
  std::vector<sensor::ImuData> result;
  result.push_back(InterpolateImu(first_time));
  for (; data != imu_data_.end() && data->time < end_time; ++data) {
    result.push_back(*data);
  }
  if (!HasImuData(end_time)) {
    LOG(WARNING) << "HasImuData false";
    // result.push_back(InterpolateImuUseLastData(end_time));
  } else {
    result.push_back(InterpolateImu(end_time));
  }
  return result;
}

//
DataBase::DataBase(const double data_duration)
    : data_duration_(data_duration) {}
//

void DataBase::TrimData(const common::Time &time) {
  while(odometry_data_.size() > 2 &&
      odometry_data_.front().time <
          time - common::FromSeconds(data_duration_)) {
    odometry_data_.pop_front();
  }
  while(imu_data_.size() > 2 &&
      imu_data_.front().time < time - common::FromSeconds(data_duration_)) {
    imu_data_.pop_front();
  }
  //
}

void DataBase::AddOdometry(const sensor::OdometryData &odom) {
  odometry_data_.push_back(odom);
}
void DataBase::AddImu(const sensor::ImuData &imu) {
  imu_data_.push_back(imu);
}

}  // namespace estimator
}  // namespace jarvis
