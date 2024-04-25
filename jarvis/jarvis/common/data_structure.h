/*** 
 * @Author: liyanyan liyanyan@meta-bounds.com
 * @Date: 2023-02-17 06:10:17
 * @LastEditTime: 2023-03-01 02:27:50
 * @LastEditors: liyanyan liyanyan@meta-bounds.com
 * @Description: 
 * @FilePath: /metaboundsstereovio/src/common/data_structure.h
 */
#ifndef JARVIS_VIO_COMMON_DATA_STRUCTURE_H_
#define JARVIS_VIO_COMMON_DATA_STRUCTURE_H_

#include <Eigen/Dense>
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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace common
}  // namespace jarvis

#endif  //JARVIS_VIO_COMMON_DATA_STRUCTURE_H_
