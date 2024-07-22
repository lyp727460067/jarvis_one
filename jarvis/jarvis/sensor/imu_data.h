#ifndef JARVIS_SENSOR_IMU_DATA_H
#define JARVIS_SENSOR_IMU_DATA_H
#include <Eigen/Core>
#include "jarvis/common/time.h"
namespace jarvis {
namespace sensor {
struct ImuData {
  common::Time time;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;
};

}  // namespace sensor

}  // namespace jarvis

#endif