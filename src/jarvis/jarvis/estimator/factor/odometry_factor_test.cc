#include "odometry_factor.h"

#include <random>

#include "gtest/gtest.h"
#include "jarvis/estimator/factor/pose_local_parameterization.h"
namespace jarvis {
namespace estimator {
namespace {

//
}  // namespace
void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0,
                      Eigen::Matrix<double, 3, 4> &Pose1,
                      Eigen::Vector2d &point0, Eigen::Vector2d &point1,
                      Eigen::Vector3d &point_3d) {
  Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
  design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
  design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
  design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
  design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
  Eigen::Vector4d triangulated_point;
  triangulated_point =
      design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
  point_3d(0) = triangulated_point(0) / triangulated_point(3);
  point_3d(1) = triangulated_point(1) / triangulated_point(3);
  point_3d(2) = triangulated_point(2) / triangulated_point(3);
}
TEST(ZeroVelocityDetectTestOptimazation1, ZeroVelocityDetectTestOptimazation3) {
  Eigen::Matrix<double, 3, 4> pose0;
  Eigen::Matrix<double, 3, 4> pose1;
  pose0.leftCols<3>() = Eigen::Matrix3d::Identity();
  //
  pose1.leftCols<3>() = Eigen::Matrix3d::Identity();
  pose1.rightCols<1>() = Eigen::Vector3d(-2, 0, 0);
  //
  Eigen::Vector2d point0(0.5, 0.5);
  Eigen::Vector2d point1(-0.5, 0.5);
  Eigen::Vector3d point3d;
  triangulatePoint(pose0, pose1, point0, point1, point3d);
  LOG(INFO) << point3d.transpose();
}
TEST(OdomFactor, OdomFactorOptimazation1) {
  DataBase data_base(1);
  for (double i = 0; i < 1.2; i += 0.1) {
    sensor::OdometryData data{
        common::Time(common::FromSeconds(i)),
        transform::Rigid3d::Translation(Eigen::Vector3d(i, 0, 0))};
    data_base.AddOdometry(data);
  }
  OdomFactor odom_factor(OdomFactorOption{}, &data_base);

  std::array<double, 7> pose1{1,           1, 1,          sin(M_PI_4),
                              sin(M_PI_4), 0, cos(M_PI_4)};
  std::array<double, 7> pose0{0, 0, 0, 0, 0, 0, 1};
  ceres::LocalParameterization *local_parameterization =
      new PoseLocalParameterization();
  ceres::Problem problem;
  problem.AddParameterBlock(pose0.data(), 7, local_parameterization);
  problem.AddParameterBlock(pose1.data(), 7, local_parameterization);
  //
  odom_factor.ComputeObserve(common::Time(common::FromSeconds(0.001)),
                             common::Time(common::FromSeconds(1)));
  odom_factor.AddToProblem(&problem, nullptr,
                           std::array<double *, 3>{pose0.data(), pose1.data()});
  //
  problem.SetParameterBlockConstant(pose0.data());
  ceres::Solver::Summary summary;
  ceres::Solver::Options options;
  // //
  options.max_num_iterations = 3;
  ceres::Solve(options, &problem, &summary);
  std::stringstream info;
  for (int i = 0; i < 7; i++) {
    info << pose0[i] << " ";
  }
  info << "\n";
  for (int i = 0; i < 7; i++) {
    info << pose1[i] << " ";
  }
  LOG(INFO) << "\n" << info.str();

  // for (int i = 0; i < 9; i++) {
  //   LOG(INFO) << bias[i];
  // }
  // LOG(INFO) << summary.FullReport();
  // for (int i = 0; i < 7; i++) {
  //   CHECK_NEAR(pose1[i], pose0[i], 1e-6);
  // }
};

}  // namespace estimator
//
}  // namespace jarvis