#include "initialization_interface.h"
namespace jarvis {
namespace estimator {

void InitializationImu::AddImuData(const sensor::ImuData& imu_data) {
  data_base_->AddImu(imu_data);
}
//
void InitializationImu::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  data_base_->AddOdometry(odometry_data);
}
//

InitializationImu::InitializationImu(DataBase* data_base)
    : data_base_(data_base) {
  CHECK_NOTNULL(data_base);
}
//
std::unique_ptr<Eigen::Quaterniond> InitializationImu::InitImuRotaion(
    const common::Time& time) {
  const auto imu_datas = data_base_->GetEarlyImuData(time);

  if (imu_datas.size() < 20) {
    LOG(WARNING) << "Init imu data empty..";
    return nullptr;
  }
  Eigen::Vector3d aver_acc = Eigen::Vector3d::Zero();

  // return;
  for (size_t i = 0; i < imu_datas.size(); i++) {
    aver_acc += imu_datas[i].linear_acceleration;
  }
  aver_acc = aver_acc / imu_datas.size();
  LOG(INFO) << "averge acc : " << aver_acc.transpose();

  Eigen::Matrix3d R0 = Utility::g2R(aver_acc);
  double yaw = Utility::R2ypr(R0).x();
  R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;

  LOG(INFO) << "Init rpy:"<< Utility::R2ypr(R0).transpose()<<"  R:\n"
            << R0;
  return std::make_unique<Eigen::Quaterniond>(R0);

  //
}

std::unique_ptr<Eigen::Quaterniond> InitializationImu::InitImuRotaion(
    const common::Time& s_time, const common::Time& e_time) {
  const auto imu_datas = data_base_->GetImuIntervalData(s_time, e_time);

  if (imu_datas.size() < 20) {
    LOG(WARNING) << "Init imu data empty..";
    return nullptr;
  }
  Eigen::Vector3d aver_acc = Eigen::Vector3d::Zero();

  // return;
  for (size_t i = 0; i < imu_datas.size(); i++) {
    aver_acc += imu_datas[i].linear_acceleration;
  }
  aver_acc = aver_acc / imu_datas.size();
  LOG(INFO) << "averge acc : " << aver_acc.transpose();

  Eigen::Matrix3d R0 = Utility::g2R(aver_acc);
  double yaw = Utility::R2ypr(R0).x();
  R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;

  LOG(INFO) << "Init rpy:" << Utility::R2ypr(R0).transpose() << "  R:\n" << R0;
  return std::make_unique<Eigen::Quaterniond>(R0);
}

//
}  // namespace estimator
}  // namespace jarvis