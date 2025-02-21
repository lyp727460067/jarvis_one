#ifndef _JARVIS_MAPPING_AUTO_PORSE_GRAPHE_H
#define _JARVIS_MAPPING_AUTO_PORSE_GRAPHE_H
#include <map>
#include <vector>

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "jarvis/transform/rigid_transform.h"
namespace jarvis {
namespace mapping {

struct FourRePoseGraphErr {
 public:
  FourRePoseGraphErr(const Eigen::Vector3d& relative_t, double relative_yaw,
                     double roll1, double pitch1, double weith)
      : relative_t_(relative_t),
        relative_yaw_(relative_yaw),
        pith_roll_rotation_(transform::RollPitchYaw(roll1, pitch1, 0.0)),
        factor_(weith) {}
  //
  template <typename T>
  bool operator()(const T* t1_, const T* q1_, const T* t2_, const T* q2_,
                  T* residul) const {
    //
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t1(t1_);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t2(t1_);
    const Eigen::Matrix<T, 3, 1> detal_t = t2 - t1;
    const Eigen::Quaternion<T> q1 =
        Eigen::AngleAxis<T>(q1_[0], Eigen::Matrix<T, 3, 1>::UnitZ()) *
        pith_roll_rotation_.cast<T>();
    //
    const Eigen::Matrix<T, 3, 1> relative_t1 = q1.conjugate() * detal_t;
    const T relative_yaw = NormalizeAngle(q2_[0] - q1_[0]);
    residul[0] = T(factor_) * (relative_t1[0] - T(relative_t_[0]));
    residul[1] = T(factor_) * (relative_t1[1] - T(relative_t_[1]));
    residul[2] = T(factor_) * (relative_t1[2] - T(relative_t_[2]));
    residul[3] = T(factor_) * NormalizeAngle(relative_yaw - T(relative_yaw_));
    return true;
  }
  //
  static ceres::CostFunction* Creat(const Eigen::Vector3d& relative_t,
                                    double relative_yaw, double roll1,
                                    double pitch1, double weith) {
    return new ceres::AutoDiffCostFunction<FourRePoseGraphErr, 4, 3, 1, 3, 1 >(
        new FourRePoseGraphErr(relative_t, relative_yaw, roll1, pitch1, weith));
  }

 private:
  const Eigen::Vector3d relative_t_;
  double relative_yaw_;
  const Eigen::Quaterniond pith_roll_rotation_;
  double factor_;
};
}  // namespace mapping

}  // namespace jarvis

#endif
