#include "data_capture.h"

#include <array>
#include <chrono>
#include <vector>

namespace JarvisPic {
namespace {
constexpr double kAccUnit = 1.0 / 2048 * 9.81;  // 加速度单位
cv::Mat YuvBufToGrayMat(uint8_t* buf, long size, uint32_t width,
                        uint32_t height) {
  cv::Mat yuvMat(height + height / 2, width, CV_8UC1, (unsigned char*)buf);
  cv::Mat grayMat;
  cv::cvtColor(yuvMat, grayMat, cv::COLOR_YUV2GRAY_NV21);
  return grayMat;
}

}  // namespace
#define FRAME_MAX_LEN 64

DataCapture::DataCapture(const DataCaptureOption& option)
    : mem_ssq_(new ShmSensorQueue) {
  thread_ = std::thread([this]() {
    Run();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(option_.imu_durition));
  });
}
//
ImuData ToImuData(const ModSyncImuFb& imu,
                  const std::pair<uint64_t, uint64_t>& base_time) {
  return ImuData{base_time.first + imu.time_stamp - base_time.second,
                 Eigen::Vector3d{
                     imu.imu_data.accel_x * kAccUnit,
                     imu.imu_data.accel_y * kAccUnit,
                     imu.imu_data.accel_z * kAccUnit,
                 },
                 Eigen::Vector3d{
                     imu.imu_data.gyro_x * 0.001,
                     imu.imu_data.gyro_x * 0.001,
                     imu.imu_data.gyro_x * 0.001,
                 }};
}
//
void DataCapture::ProcessImu(const ModSyncImuFb& imu) {
  imu_catch_.push_back(imu);
  if (!sys_time_base_) {
    return;
  }
  while (imu_catch_.empty()) {
    const auto imu_data = ToImuData(imu, sys_time_base_.value());
    for (const auto& f : imu_call_backs_) {
      f(imu_data);
    }
  }
  imu_catch_.clear();
};
void DataCapture::Run() {
  ModSyncImuFb imudata;
  int32_t res = mem_ssq_->PopImuData(&imudata);

  if (res > 0 && last_imu_time_stamp_ != imudata.time_stamp) {
    last_imu_time_stamp_ = imudata.time_stamp;
    ProcessImu(imudata);
  }
  std::array<uint8_t, FRAME_MAX_LEN> read_buf;
  CameraFrame frame;
  frame.buf = read_buf.data();
  frame.max_len = FRAME_MAX_LEN;

  int ret_len = mem_ssq_->PopAllCameraData(IMAGE_RESIZE_HALF, frame);

  if (ret_len >= 0) {
    uint32_t frame_sys_count = frame.head.sys_count;
    if (last_frame_sys_count_ == frame_sys_count) return;
    last_frame_sys_count_ = frame_sys_count;
    SysPorocess(frame);
  }
}

void DataCapture::ProcessImag(const CameraFrame& frame) {
  cv::Mat grayImg = YuvBufToGrayMat(
      frame.buf + sizeof(CameraFrameHead) + (frame.head.len >> 2),
      (frame.head.len - sizeof(CameraFrameHead)) >> 1, option_.frame_width,
      option_.frame_hight);
  const Frame out_frame{0, frame.head.time_stamp, grayImg};
  for (auto& f : frame_call_backs_) {
    f(out_frame);
  }
}

void DataCapture::SysPorocess(const CameraFrame& frame) {
  if (sys_time_base_.has_value()) {
    return;
  }

  auto it = std::find_if(imu_catch_.begin(), imu_catch_.end(),
                         [frame](const ModSyncImuFb& imu) {
                           return (frame.head.sys_count == imu.sync_count);
                         });
  int seq = 0;
  auto next_it = std::next(it, option_.cam_durion_imu_cout - 1);
  if (next_it != imu_catch_.end() && next_it->sync_count == it->sync_count) {
    sys_time_base_ =
        std::pair<uint64_t, uint64_t>(frame.head.time_stamp, it->time_stamp);
    imu_catch_.erase(imu_catch_.begin(), it);
  }
}

}  // namespace JarvisPic