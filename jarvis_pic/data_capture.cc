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

#define GET_BIT(var, bit) (((var) >> (bit)) & 0x01)
std::array<uint8_t, FRAME_MAX_LEN> read_buf;
constexpr double kGryUnit = 0.001;
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
  return ImuData{base_time.first +( imu.time_stamp - base_time.second),
                 Eigen::Vector3d{
                     imu.imu_data.accel_x * kAccUnit,
                     imu.imu_data.accel_y * kAccUnit,
                     imu.imu_data.accel_z * kAccUnit,
                 },
                 Eigen::Vector3d{
                     imu.imu_data.gyro_x * kGryUnit,
                     imu.imu_data.gyro_y * kGryUnit,
                     imu.imu_data.gyro_z * kGryUnit,
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
void DataCapture::ProcessOdom(const ModSyncChassisPosFb& odom) {
  odom_catch_.emplace_back(odom.sync_count,
                           EncoderData{
                               odom.time_stamp,
                               odom.chassis_pos.left_encoder_pos,
                               odom.chassis_pos.right_encoder_pos,
                           });

  if (!sys_odom_time_base_) {
    if (odom_catch_.size() > 200) {
      odom_catch_.erase(odom_catch_.begin());
    }
    return;
  }
  while (!odom_catch_.empty()) {
    auto odom_data = odom_catch_.front().second;
    odom_data.time = sys_odom_time_base_.value().first + odom_catch_.front().second.time -
                     sys_odom_time_base_.value().second;
    for (const auto& f : encoder_call_backs_) {
      f(odom_data);
    }
    odom_catch_.erase(odom_catch_.begin());
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
    LOG(INFO) << frame_sys_count;
    if (last_frame_sys_count_ == frame_sys_count) return;
    last_frame_sys_count_ = frame_sys_count;
    ProcessImag(frame);
  }

  ModSyncChassisPosFb odom_data;
  ret_len = mem_ssq_->PopEncodeData(&odom_data);
  if (ret_len > 0 && last_odom_time_stamp_ != odom_data.time_stamp) {
    last_odom_time_stamp_ = odom_data.time_stamp;
    ProcessOdom(odom_data);
  }
  //
  // ModRTKFB  rtk_data;
  SysPorocess();
  SysPorocessOdom();
}
//
//
Frame ToFrameData(const CameraFrame& frame, const DataCaptureOption& option) {
  //
  Frame result{frame.head.time_stamp, std::vector<cv::Mat>(2)};
  uint64_t camera_data_lenth =
      (option.frame_width * option.frame_hight * 3 * 2) >> 2;

  if (GET_BIT(frame.head.capture_flag, 1) == 1) {
    //
    cv::Mat grayImg = YuvBufToGrayMat(
        frame.buf + sizeof(CameraFrameHead) + camera_data_lenth,
        camera_data_lenth, option.frame_width, option.frame_hight);
    result.images[0] = grayImg;
  }
  if (GET_BIT(frame.head.capture_flag, 2) == 1) {
    cv::Mat grayImg = YuvBufToGrayMat(
        frame.buf + sizeof(CameraFrameHead) + camera_data_lenth * 2,
        camera_data_lenth, option.frame_width, option.frame_hight);

    result.images[1] = grayImg;
  }
  return result;
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
  const auto frame_data = ToFrameData(frame, option_);
  if (frame_data.images[0].empty()) {
    return;
  }
  image_catch_.push_back(std::make_pair(frame.head.sys_count, frame_data));
  if (image_catch_.size() <= 2) {
    return;
  }
  image_catch_.erase(image_catch_.begin());
  // if (sys_time_base_.has_value()) {
  for (auto& f : frame_call_backs_) {
    f(image_catch_.front().second);
  }
  // }
}

void DataCapture::SysPorocess() {
  if (sys_time_base_.has_value() || image_catch_.size() != 2) return;
  if(imu_catch_.empty())return;
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
              << " imu base: " << sys_time_base_.value().second << " imu lenth "
              << imu_catch_.size();
  } else {
    LOG(WARNING) << "Imu base not sys." << " imu lenth: " << imu_catch_.size();
  }
}
void DataCapture::SysPorocessOdom() {
  if (sys_odom_time_base_.has_value() || image_catch_.size() != 2) return;

  if(odom_catch_.empty())return;
  auto& last_frame = image_catch_.front();
  auto it =
      std::find_if(odom_catch_.begin(), odom_catch_.end(),
                   [last_frame](const std::pair<uint64_t, EncoderData>& odom) {
                     return (last_frame.first == odom.first);
                   });

  auto next_it = std::next(it, option_.cam_durion_odom_cout - 1);
  if (it != odom_catch_.end() &&
      std::distance(it, odom_catch_.end()) >= option_.cam_durion_odom_cout &&
      next_it->first == it->first) {
    sys_odom_time_base_ =
        std::make_pair(last_frame.second.time, it->second.time);
    LOG(INFO) << "find same count: " << static_cast<int>(last_frame.first);
    odom_catch_.erase(odom_catch_.begin(), it);
  }

  if (sys_odom_time_base_.has_value()) {
    LOG(INFO) << " Capture start cam time: "
              << sys_odom_time_base_.value().first
              << " odom base: " << sys_odom_time_base_.value().second;
  } else {
    LOG(WARNING) << "odom base not sys."
                 << " odom lenth: " << odom_catch_.size();
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