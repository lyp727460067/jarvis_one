#ifndef JARVIS_SENSOR_ODOM_DATA_H
#define JARVIS_SENSOR_ODOM_DATA_H
#include "Eigen/Core"
#include "jarvis/transform/rigid_transform.h"
#include "jarvis/common/time.h"
namespace jarvis {
namespace sensor {
struct OdometryData {
  common::Time time;
  transform::Rigid3d pose;
  static std::string TypeName() { return "odom"; }
};
}  // namespace sensor

}  // namespace jarvis

#endif