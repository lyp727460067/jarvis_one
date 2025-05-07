#ifndef _JARVIS_MAPPING_AUTO_RE_PROJECTTION_H
#define _JARVIS_MAPPING_AUTO_RE_PROJECTTION_H
#include <map>
#include <vector>

#include "Eigen/Core"
#include "ceres/ceres.h"

namespace jarvis {
namespace mapping {

// template <typename T>
// void YawPitchRollToRotationMatrix(const T yaw, const T pitch, const T roll,
//                                   T R[9]) {
//   T y = yaw / T(180.0) * T(M_PI);
//   T p = pitch / T(180.0) * T(M_PI);
//   T r = roll / T(180.0) * T(M_PI);

//   R[0] = cos(y) * cos(p);
//   R[1] = -sin(y) * cos(r) + cos(y) * sin(p) * sin(r);
//   R[2] = sin(y) * sin(r) + cos(y) * sin(p) * cos(r);
//   R[3] = sin(y) * cos(p);
//   R[4] = cos(y) * cos(r) + sin(y) * sin(p) * sin(r);
//   R[5] = -cos(y) * sin(r) + sin(y) * sin(p) * cos(r);
//   R[6] = -sin(p);
//   R[7] = cos(p) * sin(r);
//   R[8] = cos(p) * cos(r);
// };

struct ReProjectionErr {
 public:
  ReProjectionErr(const Eigen::Vector2d& nor_poit,
                  const Eigen::Vector3d& map_point, const double& factor)
      : nor_point_(nor_poit), map_point_(map_point), factor_(factor) {}

  template <typename T>
  bool operator()(const T* t1_, const T* q1_, const T* te_, const T* qe_,
                  T* residul) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t1(t1_);
    Eigen::Map<const Eigen::Quaternion<T>> q1(q1_);

    Eigen::Map<const Eigen::Matrix<T, 3, 1>> te(te_);
    Eigen::Map<const Eigen::Quaternion<T>> qe(qe_);
    Eigen::Matrix<T, 3, 1> project_p =
        qe * q1 * map_point_.template cast<T>() + qe * t1 + te;
    T x_normal = project_p[0] / project_p[2];
    T y_normal = project_p[1] / project_p[2];
    residul[0] = T(factor_) * (x_normal - T(nor_point_.x()));
    residul[1] = T(factor_) * (y_normal - T(nor_point_.y()));
    return true;
  }
  static ceres::CostFunction* Creat(const Eigen::Vector2d& nor_poit,
                                    const Eigen::Vector3d& map_point,
                                    double factor) {
    return new ceres::AutoDiffCostFunction<ReProjectionErr, 2, 3, 4, 3, 4>(
        new ReProjectionErr(nor_poit.head<2>(), map_point, factor));
  }

 private:
  const Eigen::Vector2d nor_point_;
  const Eigen::Vector3d map_point_;
  const double factor_;
};

struct ReProjectionBaErr {
 public:
  ReProjectionBaErr(const Eigen::Vector2d& nor_poit, const double& factor)
      : nor_point_(nor_poit), factor_(factor) {}

  template <typename T>
  bool operator()(const T* t1_, const T* q1_, const T* te_, const T* qe_,
                  const T* point, const T* local_map_pose_, T* residul) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t1(t1_);
    Eigen::Map<const Eigen::Quaternion<T>> q1(q1_);

    Eigen::Map<const Eigen::Matrix<T, 3, 1>> te(te_);
    Eigen::Map<const Eigen::Quaternion<T>> qe(qe_);

    // 局部地图之间的旋转量是统一的
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> map_point_temp(point);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> local_map_pose(local_map_pose_);
    const Eigen::Matrix<T, 3, 1> map_point = map_point_temp + local_map_pose;

    Eigen::Matrix<T, 3, 1> project_p =
        qe.inverse() * (q1.inverse() * (map_point.template cast<T>() - t1) - te);
    T x_normal = project_p[0] / project_p[2];
    T y_normal = project_p[1] / project_p[2];
    residul[0] = T(factor_) * (x_normal - T(nor_point_.x()));
    residul[1] = T(factor_) * (y_normal - T(nor_point_.y()));
    return true;
  }
  static ceres::CostFunction* Create(const Eigen::Vector3d& nor_poit,
                                     double factor) {
    return new ceres::AutoDiffCostFunction<ReProjectionBaErr, 2, 3, 4, 3, 4, 3, 3>(
        new ReProjectionBaErr(nor_poit.head<2>(), factor));
  }

 private:
  const Eigen::Vector2d nor_point_;
  const double factor_;
};

struct FourReProjectionErr {
 public:
  FourReProjectionErr(const Eigen::Vector2d& nor_poit,
                      const Eigen::Vector3d& map_point, const double& roll,
                      const double& pitch, const double& factor)
      : nor_point_(nor_poit),
        map_point_(map_point),
        factor_(factor),
        pith_roll_rotation_(transform::RollPitchYaw(roll, pitch, 0.0)) {}

  template <typename T>
  bool operator()(const T* t1_, const T* q1_, const T* te_, const T* qe_,
                  T* residul) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t1(t1_);
    // Eigen::Map<const Eigen::Quaternion<T>> q1(q1_);
    //
    const Eigen::Quaternion<T> q1 =
        Eigen::AngleAxis<T>(q1_[0], Eigen::Matrix<T, 3, 1>::UnitZ()) *
        pith_roll_rotation_.cast<T>();
    //
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> te(te_);
    Eigen::Map<const Eigen::Quaternion<T>> qe(qe_);
    Eigen::Matrix<T, 3, 1> project_p =
        qe * q1 * map_point_.template cast<T>() + qe * t1 + te;
    T x_normal = project_p[0] / project_p[2];
    T y_normal = project_p[1] / project_p[2];
    residul[0] = T(factor_) * (x_normal - T(nor_point_.x()));
    residul[1] = T(factor_) * (y_normal - T(nor_point_.y()));
    return true;
  }
  static ceres::CostFunction* Creat(const Eigen::Vector2d& nor_poit,
                                    const Eigen::Vector3d& map_point,
                                    const double& roll, const double& pitch,
                                    double factor) {
    return new ceres::AutoDiffCostFunction<FourReProjectionErr, 2, 3, 1, 3, 4>(
        new FourReProjectionErr(nor_poit.head<2>(), map_point, roll, pitch,
                                factor));
  }

 private:
  const Eigen::Vector2d nor_point_;
  const Eigen::Vector3d map_point_;
  const double factor_;
  const Eigen::Quaterniond pith_roll_rotation_;
};

struct FourReProjectionBaErr {
 public:
  FourReProjectionBaErr(const Eigen::Vector2d& nor_poit, const double& roll,
                        const double& pitch, const double& factor)
      : nor_point_(nor_poit),
        factor_(factor),
        pith_roll_rotation_(transform::RollPitchYaw(roll, pitch, 0.0)) {}

  template <typename T>
  bool operator()(const T* t1_, const T* q1_, const T* te_, const T* qe_,
                  const T* point, T* residul) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t1(t1_);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> map_point(point);
    //
    //
    const Eigen::Quaternion<T> q1 =
        Eigen::AngleAxis<T>(q1_[0], Eigen::Matrix<T, 3, 1>::UnitZ()) *
        pith_roll_rotation_.cast<T>();
    //
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> te(te_);
    Eigen::Map<const Eigen::Quaternion<T>> qe(qe_);
    Eigen::Matrix<T, 3, 1> pts_pose = q1.inverse() * (map_point - t1);
    Eigen::Matrix<T, 3, 1> project_p = qe.inverse() * (pts_pose - te);

    T x_normal = project_p[0] / project_p[2];
    T y_normal = project_p[1] / project_p[2];
    residul[0] = T(factor_) * (x_normal - T(nor_point_.x()));
    residul[1] = T(factor_) * (y_normal - T(nor_point_.y()));
    return true;
  }
  static ceres::CostFunction* Creat(const Eigen::Vector3d& nor_poit,
                                    const double& roll, const double& pitch,
                                    double factor) {
    return new ceres::AutoDiffCostFunction<FourReProjectionBaErr, 2, 3, 1, 3, 4,
                                           3>(
        new FourReProjectionBaErr(nor_poit.head<2>(), roll, pitch, factor));
  }

 private:
  const Eigen::Vector2d nor_point_;
  const Eigen::Vector3d map_point_;
  const double factor_;
  const Eigen::Quaterniond pith_roll_rotation_;
};

struct FourTwoReProjectionBaErr {
 public:
  FourTwoReProjectionBaErr(const Eigen::Vector2d& nor_poit, const double& roll,
                        const double& pitch, const double& factor)
      : nor_point_(nor_poit),
        factor_(factor),
        pith_roll_rotation_(transform::RollPitchYaw(roll, pitch, 0.0)) {}

  template <typename T>
  bool operator()(const T* t1_, const T* q1_, const T* te_, const T* qe_,
                  const T* t2_, const T* q2_, const T* te2_, const T* qe2_,
                  const T* point, const T* local_map_pose_, T* residul) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t1(t1_);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> map_point_temp(point);
    //
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> local_map_pose(local_map_pose_);
    //
    const Eigen::Matrix<T, 3, 1> map_point = map_point_temp ;
    const Eigen::Quaternion<T> q1 =
        Eigen::AngleAxis<T>(q1_[0], Eigen::Matrix<T, 3, 1>::UnitZ()) *
        pith_roll_rotation_.cast<T>();
    //
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> te(te_);
    Eigen::Map<const Eigen::Quaternion<T>> qe(qe_);
    Eigen::Matrix<T, 3, 1> pts_pose = q1.inverse() * (map_point - t1);
    Eigen::Matrix<T, 3, 1> project_p = qe.inverse() * (pts_pose - te);

    T x_normal = project_p[0] / project_p[2];
    T y_normal = project_p[1] / project_p[2];
    residul[0] = T(factor_) * (x_normal - T(nor_point_.x()));
    residul[1] = T(factor_) * (y_normal - T(nor_point_.y()));
    return true;
  }
  static ceres::CostFunction* Creat(const Eigen::Vector3d& nor_poit,
                                    const double& roll, const double& pitch,
                                    double factor) {
    return new ceres::AutoDiffCostFunction<FourTwoReProjectionBaErr, 4, 3, 1, 3,
                                           4, 3, 1, 3, 4, 3, 3>(
        new FourTwoReProjectionBaErr(nor_poit.head<2>(), roll, pitch, factor));
  }

 private:
  const Eigen::Vector2d nor_point_;
  const Eigen::Vector3d map_point_;
  const double factor_;
  const Eigen::Quaterniond pith_roll_rotation_;
};

struct RepeatMapPointErr {
 public:
  RepeatMapPointErr(const double& factor) : factor_(factor) {}

  template <typename T>
  bool operator()(const T* mp1_, const T* mp2_, const T* t1_, const T* q1_,
                  T* residul) const {

    Eigen::Map<const Eigen::Matrix<T, 3, 1>>map_point_1(mp1_);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> map_point_2(mp2_);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t1(t1_);
    const Eigen::Quaternion<T> q1 = Eigen::Quaternion<T>(
        Eigen::AngleAxis<T>(q1_[0], Eigen::Matrix<T, 3, 1>::UnitZ()));
    //
    const Eigen::Matrix<T, 3, 1> map_point_2_ref = q1 * map_point_2 + t1;
    //
    Eigen::Map<Eigen::Matrix<T, 3, 1>> r(residul);
    r = T(factor_) * (map_point_2_ref - map_point_1);
     return true;
  }
  static ceres::CostFunction* Creat(double factor) {
    return new ceres::AutoDiffCostFunction<RepeatMapPointErr, 3, 3, 3, 3, 1>(
        new RepeatMapPointErr(factor));
  }

 private:
  const double factor_;
};
struct RepeatMapPose {
 public:
  RepeatMapPose(const double& factor) : factor_(factor) {}

  template <typename T>
  bool operator()(const T* t0_, const T* q0_, const T* t1_, const T* q1_,
                  const T* mt1_, const T* mq1_, T* residul) const {

    //
    const Eigen::Quaternion<T> mq1 = Eigen::Quaternion<T>(
        Eigen::AngleAxis<T>(mq1_[0], Eigen::Matrix<T, 3, 1>::UnitZ()));
    //
    //
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t1(t1_);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t0(t0_);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> mt0(t0_);
    Eigen::Matrix<T, 3, 1> q1_in_t0   = mq1 *t1+ mt0;
  //
    T yaw =  mq1_[0]+q1_[0];
    T r_yaw =  NormalizeAngle( q0_[0]-yaw); 
    //
    residul[0]  =T(factor_)* t0.x()-q1_in_t0.x();
    residul[1]  =T(factor_)* t0.y()-q1_in_t0.y();
    residul[2]  =T(factor_)* t0.z()-q1_in_t0.z();
    residul[4]  =T(factor_)* r_yaw ;
     return true;
  }
  static ceres::CostFunction* Creat(double factor) {
    return new ceres::AutoDiffCostFunction<RepeatMapPose, 4, 3, 1,3, 1,3,1>(
        new RepeatMapPose(factor));
  }

 private:
  const double factor_;
};
}  // namespace mapping

}  // namespace jarvis

#endif
