#include "data_capture.h"

#include <array>
#include <chrono>
#include <vector>

#include "SensorDataCapturer/DataCapturer.h"
#include "glog/logging.h"
//
namespace jarvis_pic {
namespace {
// #define FRAME_MAX_LEN (640 * 544 * 100)
std::array<uint8_t, FRAME_MAX_LEN> read_buf;
constexpr double kAccUnit = (1.0 / 2048 * 9.81);  // 加速度单位
//
cv::Mat YuvBufToGrayMat(uint8_t* buf, long size, uint32_t width,
                        uint32_t height) {
  cv::Mat yuvMat(height + height / 2, width, CV_8UC1, (unsigned char*)buf);
  cv::Mat grayMat;
  cv::cvtColor(yuvMat, grayMat, cv::COLOR_YUV2GRAY_NV21);
  return grayMat;
}

}  // namespace

DataCapture::DataCapture(const DataCaptureOption& option)
    : mem_ssq_(new ShmSensorQueue) {}
//
void DataCapture::Start() {
  thread_ = std::thread([this]() {
    while (!stop_) {
      Run();
      std::this_thread::sleep_for(
          std::chrono::milliseconds(option_.imu_durition));
    }
  });
}

void DataCapture::Stop() {
  stop_ = true;
  thread_.join();
}
DataCapture::~DataCapture() { Stop(); }
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
                     imu.imu_data.gyro_y * 0.001,
                     imu.imu_data.gyro_z * 0.001,
                 }};
}
//
void DataCapture::ProcessImu(const ModSyncImuFb& imu) {
  imu_catch_.push_back(imu);
  if (!sys_time_base_) {
    if (imu_catch_.size() > 200) {
      imu_catch_.erase(imu_catch_.begin());
    }
    return;
  }
  while (!imu_catch_.empty()) {
    const auto imu_data = ToImuData(imu_catch_.front(), sys_time_base_.value());
    for (const auto& f : imu_call_backs_) {
      f(imu_data);
    }
    imu_catch_.erase(imu_catch_.begin());
  }
}
//
void DataCapture::Run() {
  ModSyncImuFb imudata;
  int32_t res = mem_ssq_->PopImuData(&imudata);

  if (res > 0 && last_imu_time_stamp_ != imudata.time_stamp) {
    last_imu_time_stamp_ = imudata.time_stamp;
    ProcessImu(imudata);
  }
  CameraFrame frame;
  frame.buf = read_buf.data();
  frame.max_len = FRAME_MAX_LEN;

  int ret_len = mem_ssq_->PopAllCameraData(IMAGE_RESIZE_HALF, frame);

  if (ret_len >= 0) {
    uint32_t frame_sys_count = frame.head.sys_count;
    if (last_frame_sys_count_ == frame_sys_count) return;
    last_frame_sys_count_ = frame_sys_count;
    ProcessImag(frame);
  }
  //
  // ModRTKFB  rtk_data;
  SysPorocess();
}
//
//
Frame ToFrameData(const CameraFrame& frame, const DataCaptureOption& option) {
  cv::Mat grayImg = YuvBufToGrayMat(
      frame.buf + sizeof(CameraFrameHead) + (frame.head.len >> 2),
      (frame.head.len - sizeof(CameraFrameHead)) >> 1, option.frame_width,
      option.frame_hight);
  return {0, frame.head.time_stamp, grayImg};
}
//
uint64_t DataCapture::GetOrigImuTime(const uint64_t& time) {
  if (sys_time_base_.has_value()) {
    return time - sys_time_base_.value().first + sys_time_base_.value().second;
  }
  return 0;
}
//
//
void DataCapture::ProcessImag(const CameraFrame& frame) {
  image_catch_.push_back(
      std::make_pair(frame.head.sys_count, ToFrameData(frame, option_)));
  if (image_catch_.size() <= 2) {
    return;
  }
  image_catch_.erase(image_catch_.begin());
  if (sys_time_base_.has_value()) {
    for (auto& f : frame_call_backs_) {
      f(image_catch_.front().second);
    }
  }
}

void DataCapture::SysPorocess() {
  if (sys_time_base_.has_value() || image_catch_.size() != 2) return;
  auto& last_frame = image_catch_.front();

  auto it = std::find_if(imu_catch_.begin(), imu_catch_.end(),
                         [last_frame](const ModSyncImuFb& imu) {
                           return (last_frame.first == imu.sync_count);
                         });
  auto next_it = std::next(it, option_.cam_durion_imu_cout - 1);
  if (it != imu_catch_.end() &&
      std::distance(it, imu_catch_.end()) >= option_.cam_durion_imu_cout &&
      next_it->sync_count == it->sync_count) {
    sys_time_base_ = std::make_pair(last_frame.second.time, it->time_stamp);
    LOG(INFO) << "find same count: " << static_cast<int>(last_frame.first);
    imu_catch_.erase(imu_catch_.begin(), it);
  }
  if (sys_time_base_.has_value()) {
    LOG(INFO) << " Capture start cam time: " << sys_time_base_.value().first
              << " imu base: " << sys_time_base_.value().second;
  } else {
    LOG(WARNING) << "Imu base not sys." << " imu lenth: " << imu_catch_.size();
  }
}
//
std::unique_ptr<DataCapture> CreateDataCaputure(
    const DataCaptureOption& option) {
  if (option.use_method == 0) {
    return std::make_unique<DataCapture>(DataCaptureOption{});
  } else if (option.use_method == 1) {
    return std::make_unique<VSLAM::DataCapturer>(10, 200);
  } else {
    LOG(FATAL) << "Unsupport capture type...";
  }
  return nullptr;
}
//
}  // namespace jarvis_pic