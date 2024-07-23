#include "jarvis/estimator/data_base.h"

#include <random>

#include "gtest/gtest.h"
namespace jarvis {
namespace estimator {
namespace {

//
}  // namespace

TEST(DataBase, DataBaseCase) {
  DataBase data_base(1);
  int num = 100;
  for (double i = 0; i < 1.2; i += 0.1) {
    sensor::OdometryData data{common::Time(common::FromSeconds(i))};
    data_base.AddOdometry(data);
    sensor::ImuData imu{common::Time(common::FromSeconds(i)),
                        Eigen::Vector3d::Identity(),
                        Eigen::Vector3d::Identity()};
    LOG(INFO) << i;
    data_base.AddImu(imu);
  }
  auto data_imu =
      data_base.GetImuIntervalData(common::Time(common::FromSeconds(0.1)),
                                   common::Time(common::FromSeconds(1)));
  //
  auto odom =
      data_base.InterpolateOdometry(common::Time(common::FromSeconds(0.1)));
  CHECK_EQ(data_base.HasImuData(common::Time(common::FromSeconds(0.1))), true);
  for (const auto &imu : data_imu) {
    LOG(INFO) << imu.time;
  }
  // LOG(INFO) << data_imu.size();
};

}  // namespace estimator
}  // namespace jarvis