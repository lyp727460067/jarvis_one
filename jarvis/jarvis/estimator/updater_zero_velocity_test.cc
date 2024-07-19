#include "estimator/updater_zero_velocity.h"

#include <random>

#include "gtest/gtest.h"
#include "jarvis/estimator/factor/pose_local_parameterization.h"
namespace jarvis {
namespace estimator {
namespace {

//
}  // namespace

TEST(ZeroVelocityDetectTestOptimazation, ZeroVelocityDetectTestOptimazation1) {
  UpdataZeroVelocity updata_zero_velocity({{}, {}, 10});
  std::array<double, 7> pose0{10, 1, 1, sin(M_PI_4),sin(M_PI_4),0,cos(M_PI_4)};
  std::array<double, 7> pose1{0, 0, 0, 0, 0, 0,1};
  std::array<double, 9> bias{1, 10, 10, 0, 0, 0, 0, 0, 0};
  ceres::LocalParameterization* local_parameterization =
      new PoseLocalParameterization();
  ceres::Problem problem;      
  problem.AddParameterBlock(pose0.data(), 7, local_parameterization);
  problem.AddParameterBlock(pose1.data(), 7, local_parameterization);


  updata_zero_velocity.AddToProblem(
      &problem, nullptr,
      std::array<double*, 3>{pose0.data(), pose1.data(), bias.data()});
  //

  problem.SetParameterBlockConstant(pose0.data());
  ceres::Solver::Summary summary;
  ceres::Solver::Options options;
  //
  options.max_num_iterations = 2;
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

  for (int i = 0; i < 9; i++) {
    LOG(INFO) << bias[i];
  }
  // LOG(INFO) << summary.FullReport();
  // for (int i = 0; i < 7; i++) {
  //   CHECK_NEAR(pose1[i], pose0[i], 1e-6);
  // }
};

TEST(ZeroVelocityDetectTestOptimazation, ZeroVelocityDetectTestOptimazation2) {
  UpdataZeroVelocity updata_zero_velocity({{}, {}, 100});
  std::array<double, 7> pose1{1, 1, 1, sin(M_PI_4),sin(M_PI_4),0,cos(M_PI_4)};
  std::array<double, 7> pose0{0, 0, 0, 0, 0, 0,1};
  std::array<double, 9> bias{0, 0, 0, 0, 0, 0, 0, 0, 0};
  ceres::LocalParameterization* local_parameterization =
      new PoseLocalParameterization();
   ceres::Problem problem;     
  problem.AddParameterBlock(pose0.data(), 7, local_parameterization);
  problem.AddParameterBlock(pose1.data(), 7, local_parameterization);

  updata_zero_velocity.AddToProblem(
      &problem, nullptr,
      std::array<double*, 3>{pose0.data(), pose1.data(), bias.data()});
  //


  problem.SetParameterBlockConstant(pose1.data());
  ceres::Solver::Summary summary;
  ceres::Solver::Options options;
  //
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

  for (int i = 0; i < 9; i++) {
    LOG(INFO) << bias[i];
  }
  LOG(INFO) << summary.FullReport();
  for (int i = 0; i < 7; i++) {
    CHECK_NEAR(pose1[i], pose0[i], 1e-6);
  }
};

}  // namespace estimator
//
}  // namespace jarvis