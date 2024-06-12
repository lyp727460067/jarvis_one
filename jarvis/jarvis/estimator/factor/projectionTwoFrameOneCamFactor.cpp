/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "projectionTwoFrameOneCamFactor.h"
#include "rtm/impl/matrix_affine_common.h"
#include "rtm/matrix3x3d.h"
#include "rtm/matrix3x3f.h"
namespace jarvis {
namespace estimator {
// Eigen::Matrix2d ProjectionTwoFrameOneCamFactor::sqrt_info =
//     Eigen::Matrix2d::Identity();
double ProjectionTwoFrameOneCamFactor::sum_t = 0.0;
#define __USE_SIMD__
namespace {
#ifdef __USE_SIMD__
  using _Scalar = double;
  using _simd_scalar = rtm::scalard;
  using _simd_vector4 = rtm::vector4d;
  using _simd_matrix3 = rtm::matrix3x3d;
  using _simd_quaternion = rtm::quatd;
#endif
#ifdef __USE_SIMD__
  _simd_matrix3 _simd_hat_from_eigen(const Eigen::Vector3<_Scalar> &vec) {
    _simd_vector4 _simd_x_axis, _simd_y_axis, _simd_z_axis;
    _Scalar x_axis[4] = {0, vec.z(), -vec.y(), 0};
    _Scalar y_axis[4] = {-vec.z(), 0, vec.x(), 0};
    _Scalar z_axis[4] = {vec.y(), -vec.x(), 0, 0};
    _simd_x_axis = rtm::vector_load(x_axis);
    _simd_y_axis = rtm::vector_load(y_axis);
    _simd_z_axis = rtm::vector_load(z_axis);
    return _simd_matrix3{_simd_x_axis, _simd_y_axis, _simd_z_axis};
  }
  Eigen::Vector3d ToEigen(const rtm::vector4d &rtm_vector) {
    return Eigen::Vector3d{rtm::vector_get_x(rtm_vector),
                           rtm::vector_get_y(rtm_vector),
                           rtm::vector_get_z(rtm_vector)};
  }

  Eigen::Matrix3d ToEigen(const rtm::matrix3x3d &rtm_matrix) {
    Eigen::Matrix3d result;
    result << ToEigen(rtm_matrix.x_axis), ToEigen(rtm_matrix.y_axis),
        ToEigen(rtm_matrix.z_axis);
    return result;
  }
#endif

}


ProjectionTwoFrameOneCamFactor::ProjectionTwoFrameOneCamFactor(
    const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j,
    const Eigen::Vector2d &_velocity_i, const Eigen::Vector2d &_velocity_j,
    const double _td_i, const double _td_j,double weight)
    : pts_i(_pts_i), pts_j(_pts_j), td_i(_td_i), td_j(_td_j) {
  velocity_i.x() = _velocity_i.x();
  velocity_i.y() = _velocity_i.y();
  velocity_i.z() = 0;
  velocity_j.x() = _velocity_j.x();
  velocity_j.y() = _velocity_j.y();
  velocity_j.z() = 0;
  sqrt_info = weight * Eigen::Matrix2d::Identity();

#ifdef UNIT_SPHERE_ERROR
  Eigen::Vector3d b1, b2;
  Eigen::Vector3d a = pts_j.normalized();
  Eigen::Vector3d tmp(0, 0, 1);
  if (a == tmp) tmp << 1, 0, 0;
  b1 = (tmp - a * (a.transpose() * tmp)).normalized();
  b2 = a.cross(b1);
  tangent_base.block<1, 3>(0, 0) = b1.transpose();
  tangent_base.block<1, 3>(1, 0) = b2.transpose();
#endif
};
bool ProjectionTwoFrameOneCamFactor::Evaluate(double const *const *parameters,
                                              double *residuals,
                                              double **jacobians) const {
  return EvaluateSIMD(parameters, residuals, jacobians);
}
bool ProjectionTwoFrameOneCamFactor::EvaluateNormal(double const *const *parameters,
                                              double *residuals,
                                              double **jacobians) const {
  TicToc tic_toc;
  Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4],
                        parameters[0][5]);

  Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
  Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4],
                        parameters[1][5]);

  Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
  Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4],
                         parameters[2][5]);

  const double &inv_dep_i = parameters[3][0];

  const double &td = parameters[4][0];
  Eigen::Vector3d pts_i_td = pts_i - (td - td_i) * velocity_i; 
  Eigen::Vector3d pts_j_td = pts_j - (td - td_j) * velocity_j;
  Eigen::Vector3d pts_camera_i = pts_i_td / inv_dep_i;
  Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
  Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
  Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
  Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
  Eigen::Map<Eigen::Vector2d> residual(residuals);
  double dep_j = pts_camera_j.z();
  residual = (pts_camera_j / dep_j).head<2>() - pts_j_td.head<2>();
  residual = sqrt_info * residual;

  if (jacobians) {
    Eigen::Matrix3d Ri = Qi.toRotationMatrix();
    Eigen::Matrix3d Rj = Qj.toRotationMatrix();
    Eigen::Matrix3d ric = qic.toRotationMatrix();
    Eigen::Matrix<double, 2, 3> reduce(2, 3);
#ifdef UNIT_SPHERE_ERROR
    double norm = pts_camera_j.norm();
    Eigen::Matrix3d norm_jaco;
    double x1, x2, x3;
    x1 = pts_camera_j(0);
    x2 = pts_camera_j(1);
    x3 = pts_camera_j(2);
    norm_jaco << 1.0 / norm - x1 * x1 / pow(norm, 3), -x1 * x2 / pow(norm, 3),
        -x1 * x3 / pow(norm, 3), -x1 * x2 / pow(norm, 3),
        1.0 / norm - x2 * x2 / pow(norm, 3), -x2 * x3 / pow(norm, 3),
        -x1 * x3 / pow(norm, 3), -x2 * x3 / pow(norm, 3),
        1.0 / norm - x3 * x3 / pow(norm, 3);
    reduce = tangent_base * norm_jaco;
#else
    reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j), 0, 1. / dep_j,
        -pts_camera_j(1) / (dep_j * dep_j);
#endif
    reduce = sqrt_info * reduce;

    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(
          jacobians[0]);

      Eigen::Matrix<double, 3, 6> jaco_i;
      jaco_i.leftCols<3>() = ric.transpose() * Rj.transpose();
      jaco_i.rightCols<3>() = ric.transpose() * Rj.transpose() * Ri *
                              -Utility::skewSymmetric(pts_imu_i);

      jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
      jacobian_pose_i.rightCols<1>().setZero();
    }

    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(
          jacobians[1]);

      Eigen::Matrix<double, 3, 6> jaco_j;
      jaco_j.leftCols<3>() = ric.transpose() * -Rj.transpose();
      jaco_j.rightCols<3>() =
          ric.transpose() * Utility::skewSymmetric(pts_imu_j);

      jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
      jacobian_pose_j.rightCols<1>().setZero();
    }
    if (jacobians[2]) {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(
          jacobians[2]);
      Eigen::Matrix<double, 3, 6> jaco_ex;
      jaco_ex.leftCols<3>() =
          ric.transpose() * (Rj.transpose() * Ri - Eigen::Matrix3d::Identity());
      Eigen::Matrix3d tmp_r = ric.transpose() * Rj.transpose() * Ri * ric;
      jaco_ex.rightCols<3>() =
          -tmp_r * Utility::skewSymmetric(pts_camera_i) +
          Utility::skewSymmetric(tmp_r * pts_camera_i) +
          Utility::skewSymmetric(ric.transpose() *
                                 (Rj.transpose() * (Ri * tic + Pi - Pj) - tic));
      jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
      jacobian_ex_pose.rightCols<1>().setZero();
    }
    if (jacobians[3]) {
      Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[3]);
      jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri * ric *
                         pts_i_td * -1.0 / (inv_dep_i * inv_dep_i);
    }
    if (jacobians[4]) {
      Eigen::Map<Eigen::Vector2d> jacobian_td(jacobians[4]);
      jacobian_td = reduce * ric.transpose() * Rj.transpose() * Ri * ric *
                        velocity_i / inv_dep_i * -1.0 +
                    sqrt_info * velocity_j.head(2);
    }
  }
  sum_t += tic_toc.toc();

  return true;
}
bool ProjectionTwoFrameOneCamFactor::EvaluateSIMD(double const *const *parameters,
                                              double *residuals,
                                              double **jacobians) const {
  //




  // Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  // Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4],
  //                       parameters[0][5]);

  // Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
  // Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4],
  //                       parameters[1][5]);

  // Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
  // Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4],
  //                        parameters[2][5]);
  // Eigen::Vector3d pts_i_td = pts_i - (td - td_i) * velocity_i; 
  // Eigen::Vector3d pts_j_td = pts_j - (td - td_j) * velocity_j;
  // Eigen::Vector3d pts_camera_i = pts_i_td / inv_dep_i;



  TicToc tic_toc;
  rtm::vector4d rtm_Pi =
      rtm::vector_set(parameters[0][0], parameters[0][1], parameters[0][2], 0);
  rtm::quatd rtm_Qi = rtm::quat_set(parameters[0][3], parameters[0][4],
                        parameters[0][5],parameters[0][6] );
  //
  //
  // Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  // Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4],
  //                       parameters[0][5]);

  //
  rtm::vector4d rtm_Pj =
      rtm::vector_set(parameters[1][0], parameters[1][1], parameters[1][2], 0);
  rtm::quatd rtm_Qj = rtm::quat_set(parameters[1][3], parameters[1][4],
                                        parameters[1][5], parameters[1][6]);
  //
  rtm::quatd rtm_Qj_inv = rtm::quat_set(-parameters[1][3], -parameters[1][4],
                                        -parameters[1][5], parameters[1][6]);

  // Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
  // Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4],
  //                       parameters[1][5]);

  //
  rtm::vector4d rtm_tic =
      rtm::vector_set(parameters[2][0], parameters[2][1], parameters[2][2], 0);
  rtm::quatd rtm_qic = rtm::quat_set(parameters[2][3], parameters[2][4],
                                         parameters[2][5], parameters[2][6]);
  rtm::quatd rtm_qic_inv = rtm::quat_set(-parameters[2][3], -parameters[2][4],
                                         -parameters[2][5], parameters[2][6]);

  // Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
  // Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4],
  //                        parameters[2][5]);

  const double &inv_dep_i = parameters[3][0];
  const double &td = parameters[4][0];
  Eigen::Vector3d pts_i_td = pts_i - (td - td_i) * velocity_i;
  Eigen::Vector3d pts_j_td = pts_j - (td - td_j) * velocity_j;
  //
  //
  //
  Eigen::Vector3d pts_camera_i = pts_i_td / inv_dep_i;
  //
  //////////
  // Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
  // Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
  // Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
  ///////////
  rtm::vector4d rtm_pts_camera_i = rtm::vector_set(pts_camera_i.x(),
       pts_camera_i.y(),pts_camera_i.z(),0);
  auto rtm_pts_imu_i = rtm::vector_add(
      rtm::quat_mul_vector3(rtm_pts_camera_i, rtm_qic), rtm_tic);
  //
  // Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;

    auto rtm_pts_w = rtm::vector_add(
      rtm::quat_mul_vector3(rtm_pts_imu_i, rtm_Qi), rtm_Pi);
  // Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
    auto rtm_pts_imu_j =
        rtm::quat_mul_vector3(rtm::vector_sub(rtm_pts_w, rtm_Pj), rtm_Qj_inv);


    // Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
    auto rtm_pts_camera_j = rtm::quat_mul_vector3(
        rtm::vector_sub(rtm_pts_imu_j, rtm_tic), rtm_qic_inv);
    // Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
    //

    Eigen::Map<Eigen::Vector2d> residual(residuals);
    const double dep_j_inv = 1.0/rtm::vector_get_z(rtm_pts_camera_j);
     // double dep_j = pts_camera_j.z();
    Eigen::Vector3d pts_camera_j{rtm::vector_get_x(rtm_pts_camera_j),
                                 rtm::vector_get_y(rtm_pts_camera_j),
                                 rtm::vector_get_z(rtm_pts_camera_j)};
    residual = (pts_camera_j *dep_j_inv).head<2>() - pts_j_td.head<2>();
    // residual = (pts_camera_j / dep_j).head<2>() - pts_j_td.head<2>();
    residual = sqrt_info * residual;

    if (jacobians) {
      // Eigen::Matrix3d Ri = Qi.toRotationMatrix();
      // Eigen::Matrix3d Rj = Qj.toRotationMatrix();
      // Eigen::Matrix3d ric = qic.toRotationMatrix();
      Eigen::Matrix<double, 2, 3> reduce(2, 3);
      reduce << dep_j_inv, 0, -pts_camera_j(0) * dep_j_inv * dep_j_inv,0,
          dep_j_inv, -pts_camera_j(1) * dep_j_inv * dep_j_inv;
      reduce = sqrt_info * reduce;
      rtm::matrix3x3d rtm_m_ric_inv = rtm::matrix_from_quat(rtm_qic_inv);
      rtm::matrix3x3d rtm_m_ric = rtm::matrix_from_quat(rtm_qic);
      rtm::matrix3x3d rtm_m_Rj_inv = rtm::matrix_from_quat(rtm_Qj_inv);
      rtm::matrix3x3d rtm_m_Ri = rtm::matrix_from_quat(rtm_Qi);
      auto ric_t_mul_rj_inv_mul_ri_ric = rtm::matrix_mul(rtm_m_ric,
        rtm::matrix_mul(rtm_m_Ri,rtm::matrix_mul(rtm_m_Rj_inv,rtm_m_ric_inv)));

      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
            jacobian_pose_i(jacobians[0]);

        Eigen::Matrix<double, 3, 6> jaco_i;
        auto tmp =  rtm::matrix_mul(rtm_m_Rj_inv,rtm_m_ric_inv);
        jaco_i.leftCols<3>() = ToEigen(tmp);
        // LOG(INFO) << "\n" << ric.transpose() * Rj.transpose();
        // LOG(INFO) << "\n" << ToEigen(tmp);

        jaco_i.rightCols<3>() = ToEigen(
                         rtm::matrix_mul(rtm_m_Ri, tmp))* -Utility::skewSymmetric(ToEigen(rtm_pts_imu_i));

        // jaco_i.rightCols<3>() = ric.transpose() * Rj.transpose() * Ri *
        //                         -Utility::skewSymmetric(pts_imu_i);

        jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
        jacobian_pose_i.rightCols<1>().setZero();
    }

    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(
          jacobians[1]);
      Eigen::Matrix<double, 3, 6> jaco_j;
      auto tmp =  rtm::matrix_mul(rtm_m_Rj_inv,rtm_m_ric_inv);
      jaco_j.leftCols<3>() = -ToEigen(tmp);
      // jaco_j.leftCols<3>() = ric.transpose() * -Rj.transpose();
      jaco_j.rightCols<3>() = ToEigen(
          rtm_m_ric_inv)* Utility::skewSymmetric(ToEigen(rtm_pts_imu_j));
      // jaco_j.rightCols<3>() =
      //     ric.transpose() * Utility::skewSymmetric(pts_imu_j);
      jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
      jacobian_pose_j.rightCols<1>().setZero();
    }
    if (jacobians[2]) {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(
          jacobians[2]);
      Eigen::Matrix<double, 3, 6> jaco_ex;

      auto rj_inve_ri = rtm::matrix_mul(rtm_m_Ri,rtm_m_Rj_inv);
      auto rtm_tmp_r = rtm::matrix_mul(rj_inve_ri,rtm_m_ric_inv);
      jaco_ex.leftCols<3>() = ToEigen(rtm_tmp_r) - ToEigen(rtm_m_ric_inv);

       Eigen::Matrix3d tmp_r =   ToEigen(rtm_tmp_r);
       //
       auto r_mul_tmp = rtm::matrix_mul_vector3(
           rtm::vector_sub(
               rtm::matrix_mul_vector3(
                   rtm::vector_sub(
                       rtm::vector_add(
                           rtm::matrix_mul_vector3(rtm_tic, rtm_m_Ri), rtm_Pi),
                       rtm_Pj),
                   rtm_m_Rj_inv),
               rtm_tic),
           rtm_m_ric_inv);
       //
       jaco_ex.rightCols<3>() = -tmp_r * Utility::skewSymmetric(pts_camera_i) +
                                Utility::skewSymmetric(tmp_r * pts_camera_i) +
                                Utility::skewSymmetric(ToEigen(r_mul_tmp));
       // jaco_ex.rightCols<3>() =
       //     -tmp_r * Utility::skewSymmetric(pts_camera_i) +
       //     Utility::skewSymmetric(tmp_r * pts_camera_i) +
       //     Utility::skewSymmetric(ric.transpose() *
       //                            (Rj.transpose() * (Ri * tic + Pi - Pj) -
       //                            tic));
       jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
       jacobian_ex_pose.rightCols<1>().setZero();
    }
    if (jacobians[3]) {
      Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[3]);


      jacobian_feature =
          reduce * ToEigen(ric_t_mul_rj_inv_mul_ri_ric) * pts_i_td * -1.0 / (inv_dep_i * inv_dep_i);

      // jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri * ric *
      //                    pts_i_td * -1.0 / (inv_dep_i * inv_dep_i);
    }
    if (jacobians[4]) {
      Eigen::Map<Eigen::Vector2d> jacobian_td(jacobians[4]);
      jacobian_td = reduce * ToEigen(ric_t_mul_rj_inv_mul_ri_ric) * velocity_i /
                        inv_dep_i * -1.0 +
                    sqrt_info * velocity_j.head(2);

      // jacobian_td = reduce * ric.transpose() * Rj.transpose() * Ri * ric *
      //                   velocity_i / inv_dep_i * -1.0 +
      //               sqrt_info * velocity_j.head(2);
    }
  }
  sum_t += tic_toc.toc();

  return true;
}
void ProjectionTwoFrameOneCamFactor::check(double **parameters) {
  double *res = new double[2];
  double **jaco = new double *[5];
  jaco[0] = new double[2 * 7];
  jaco[1] = new double[2 * 7];
  jaco[2] = new double[2 * 7];
  jaco[3] = new double[2 * 1];
  jaco[4] = new double[2 * 1];
  Evaluate(parameters, res, jaco);
  puts("check begins");

  puts("my");

  std::cout << Eigen::Map<Eigen::Matrix<double, 2, 1>>(res).transpose()
            << std::endl
            << std::endl;
  std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[0])
            << std::endl
            << std::endl;
  std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[1])
            << std::endl
            << std::endl;
  std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[2])
            << std::endl
            << std::endl;
  std::cout << Eigen::Map<Eigen::Vector2d>(jaco[3]) << std::endl << std::endl;
  std::cout << Eigen::Map<Eigen::Vector2d>(jaco[4]) << std::endl << std::endl;

  Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4],
                        parameters[0][5]);

  Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
  Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4],
                        parameters[1][5]);

  Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
  Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4],
                         parameters[2][5]);
  double inv_dep_i = parameters[3][0];
  double td = parameters[4][0];

  Eigen::Vector3d pts_i_td, pts_j_td;
  pts_i_td = pts_i - (td - td_i) * velocity_i;
  pts_j_td = pts_j - (td - td_j) * velocity_j;
  Eigen::Vector3d pts_camera_i = pts_i_td / inv_dep_i;
  Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
  Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
  Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
  Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
  Eigen::Vector2d residual;

#ifdef UNIT_SPHERE_ERROR
  residual = tangent_base * (pts_camera_j.normalized() - pts_j_td.normalized());
#else
  double dep_j = pts_camera_j.z();
  residual = (pts_camera_j / dep_j).head<2>() - pts_j_td.head<2>();
#endif
  residual = sqrt_info * residual;

  puts("num");
  std::cout << residual.transpose() << std::endl;

  const double eps = 1e-6;
  Eigen::Matrix<double, 2, 20> num_jacobian;
  for (int k = 0; k < 20; k++) {
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4],
                          parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4],
                          parameters[1][5]);

    Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4],
                           parameters[2][5]);
    double inv_dep_i = parameters[3][0];
    double td = parameters[4][0];

    int a = k / 3, b = k % 3;
    Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

    if (a == 0)
      Pi += delta;
    else if (a == 1)
      Qi = Qi * Utility::deltaQ(delta);
    else if (a == 2)
      Pj += delta;
    else if (a == 3)
      Qj = Qj * Utility::deltaQ(delta);
    else if (a == 4)
      tic += delta;
    else if (a == 5)
      qic = qic * Utility::deltaQ(delta);
    else if (a == 6 && b == 0)
      inv_dep_i += delta.x();
    else if (a == 6 && b == 1)
      td += delta.y();

    Eigen::Vector3d pts_i_td, pts_j_td;
    pts_i_td = pts_i - (td - td_i) * velocity_i;
    pts_j_td = pts_j - (td - td_j) * velocity_j;
    Eigen::Vector3d pts_camera_i = pts_i_td / inv_dep_i;
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
    Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
    Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
    Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
    Eigen::Vector2d tmp_residual;

#ifdef UNIT_SPHERE_ERROR
    tmp_residual =
        tangent_base * (pts_camera_j.normalized() - pts_j_td.normalized());
#else
    double dep_j = pts_camera_j.z();
    tmp_residual = (pts_camera_j / dep_j).head<2>() - pts_j_td.head<2>();
#endif
    tmp_residual = sqrt_info * tmp_residual;

    num_jacobian.col(k) = (tmp_residual - residual) / eps;
  }
  std::cout << num_jacobian.block<2, 6>(0, 0) << std::endl;
  std::cout << num_jacobian.block<2, 6>(0, 6) << std::endl;
  std::cout << num_jacobian.block<2, 6>(0, 12) << std::endl;
  std::cout << num_jacobian.block<2, 1>(0, 18) << std::endl;
  std::cout << num_jacobian.block<2, 1>(0, 19) << std::endl;
}



}  // namespace estimator
}  // namespace jarvis
   // namespace jarvis