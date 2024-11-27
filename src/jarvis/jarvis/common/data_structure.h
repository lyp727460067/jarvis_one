#ifndef JARVIS_VIO_COMMON_DATA_STRUCTURE_H_
#define JARVIS_VIO_COMMON_DATA_STRUCTURE_H_

#include "Eigen/Dense"
#include "transform/transform.h"
#include "transform/rigid_transform.h"

namespace jarvis {
namespace common {

struct Pose3d {
  Eigen::Vector3d p;
  Eigen::Quaterniond q;
  transform::Rigid3d Pose() const{
    return transform::Rigid3d(p, q);
  }
};
struct EulerPose3d {
  Eigen::Vector3d p;
  Eigen::Vector3d euler;
  transform::Rigid3d Pose() const {
    return transform::Rigid3d(
               p, transform::RollPitchYaw(common::DegToRad(euler.z()),
                                          common::DegToRad(euler.y()),
                                          common::DegToRad(euler.x()))) ;
  }

  static transform::Rigid3d imu_to_cam_extric_;
};

}  // namespace common
}  // namespace jarvis

#endif  //JARVIS_VIO_COMMON_DATA_STRUCTURE_H_
