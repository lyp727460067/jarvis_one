#include "data_capture.h"

#include <array>
#include <chrono>
#include <vector>
#define FRAME_MAX_LEN (4116580)
// #include "SensorDataCapturer/DataCapturer.h"
#include "glog/logging.h"
#include "optional"
#include "jarvis/transform/rigid_transform.h"
//
// #define NEED_SYNC
namespace jarvis_pic {
namespace {
// #define FRAME_MAX_LEN (640 * 544 * 100)

constexpr double kWheelDistance = 0.37;
std::optional<Eigen::Vector2i> kLastEncoderData;
jarvis::transform::Rigid3d global_odom_ =
      jarvis::transform::Rigid3d::Identity();

void EncodeToOdom(const EncoderData& encode) {
  if (!kLastEncoderData.has_value()) {
    kLastEncoderData =
        Eigen::Vector2i(encode.left_encoder, encode.right_encoder);
  }
  const Eigen::Vector2i cur_encode{encode.left_encoder, encode.right_encoder};
  const Eigen::Vector2d delta_encode =
      0.001 * (cur_encode - kLastEncoderData.value()).cast<double>();

  kLastEncoderData = cur_encode;
  double delta_theta = (delta_encode.y() - delta_encode.x()) / kWheelDistance;
  double delta_translation = (delta_encode.y() + delta_encode.x()) / 2.0;
  jarvis::transform::Rigid3d delta_pose(
      Eigen::Vector3d(delta_translation, 0, 0),
      Eigen::Quaterniond(cos(delta_theta / 2), 0, 0, sin(delta_theta / 2)));
  global_odom_ = global_odom_ * delta_pose;

}

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
  return grayMat.clone();
}

}  // namespace

DataCapture::DataCapture(const DataCaptureOption& option)
    : mem_ssq_(new ShmSensorQueue), shm_mod_(new ShmMod()) {}
//
void DataCapture::Start() {
  threads_.emplace_back([this]() {
    while (!stop_) {
      
      ModUIBoardStatusFb mower_status;
      int s = shm_mod_->GetModByID(MOD_ID_UI_BOARD_STATUS_FB, &mower_status);
      if (s == sizeof(ModUIBoardStatusFb)) {
        // std::lock_guard<std::mutex> lock(mutex_);
        system_info_call_backs_({mower_status.MowerStatus});
      }

      ModSyncImuFb imudata;
      int32_t res = mem_ssq_->PopImuData(&imudata);
      while (res > 0) {
        if (res > 0 && last_imu_time_stamp_ != imudata.time_stamp) {
          last_imu_time_stamp_ = imudata.time_stamp;
          std::lock_guard<std::mutex> lock(mutex_);
          ProcessImu(imudata);
        }
        res = mem_ssq_->PopImuData(&imudata);
      }
      ModSyncChassisPosFb odom_data;
      int ret_len = mem_ssq_->PopEncodeData(&odom_data);
      while (ret_len > 0) {
        if (last_odom_time_stamp_ != odom_data.time_stamp) {
          last_odom_time_stamp_ = odom_data.time_stamp;
          std::lock_guard<std::mutex> lock(mutex_);
          ProcessOdom(odom_data);
        }
        ret_len = mem_ssq_->PopEncodeData(&odom_data);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  
  });
  threads_.emplace_back([this]() {
    while (!stop_) {
      
      CameraFrame frame;
      frame.buf = read_buf.data();
      frame.max_len = FRAME_MAX_LEN;

      int ret_len = mem_ssq_->PopAllCameraData(IMAGE_RESIZE_HALF, frame);

      while (ret_len >= 0) {
        uint32_t frame_sys_count = frame.head.sys_count;
        if (last_frame_sys_count_ != frame_sys_count) {
          static uint64_t last_time = frame.head.time_stamp;
          // LOG(INFO)<<frame.head.time_stamp-last_time;
          last_time = frame.head.time_stamp;
          last_frame_sys_count_ = frame_sys_count;
          std::lock_guard<std::mutex> lock(mutex_);
          ProcessImag(frame);
        }
        ret_len = mem_ssq_->PopAllCameraData(IMAGE_RESIZE_HALF, frame);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
  });
  // thread_ = std::thread([this]() {
  //   while (!stop_) {
  //     Run();
  //     std::this_thread::sleep_for(
  //         std::chrono::milliseconds(4));
  //   }
  // });
}

void DataCapture::Stop() {
  stop_ = true;
  for (size_t i = 0; i < threads_.size(); i++) {
    if (threads_[i].joinable()) {
      threads_[i].join();
    }
  }
  // thread_.join();
}
DataCapture::~DataCapture() { Stop(); }
//
ImuData ToImuData(const ModSyncImuFb& imu,
                  const std::pair<uint64_t, uint64_t>& base_time) {
#ifdef NEED_SYNC
  return ImuData{base_time.first + int64_t(imu.time_stamp - base_time.second),
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
#else
  return ImuData{imu.time_stamp,
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
#endif
}
//
void DataCapture::ProcessImu(const ModSyncImuFb& imu) {
#ifdef NEED_SYNC
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
      f.second(imu_data);
    }
    imu_catch_.erase(imu_catch_.begin());
  }
#else
  const auto imu_data = ToImuData(imu, {});
  for (const auto& f : imu_call_backs_) {
    f.second(imu_data);
  }
#endif
}
void DataCapture::ProcessOdom(const ModSyncChassisPosFb& odom) {
#ifdef NEED_SYNC
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
    odom_data.time = sys_odom_time_base_.value().first +
                     odom_catch_.front().second.time -
                     sys_odom_time_base_.value().second;
    for (const auto& f : encoder_call_backs_) {
      f.second(odom_data);
    }
    odom_catch_.erase(odom_catch_.begin());
  }
#else
  auto odom_data =
      EncoderData{odom.time_stamp, odom.chassis_pos.left_encoder_pos,
                  odom.chassis_pos.right_encoder_pos};
  EncodeToOdom(odom_data);
  //
  auto odom_data_tmp = OdomData{odom.time_stamp, global_odom_.translation(),
                                global_odom_.rotation()};
  //
  for (const auto& f : encoder_call_backs_) {
    f.second(odom_data_tmp);
  }
#endif
}
//
void DataCapture::Run() {
  
  //
  // ModRTKFB  rtk_data;
#ifdef NEED_SYNC
  SysPorocess();
  SysPorocessOdom();
#endif
}
//
//
Frame ToFrameData(const CameraFrame& frame, const DataCaptureOption& option) {
  //
  Frame result{frame.head.time_stamp, std::vector<cv::Mat>(2)};
  uint64_t camera_data_lenth =
      (option.frame_width * option.frame_hight * 3 * 2) >> 2;
  std::thread thread1([&]() {
    if (GET_BIT(frame.head.capture_flag, 1) == 1) {
      //
      cv::Mat grayImg = YuvBufToGrayMat(
          frame.buf + sizeof(CameraFrameHead) + camera_data_lenth,
          camera_data_lenth, option.frame_width, option.frame_hight);
      result.images[0] = grayImg;
    }
  });

  std::thread thread2([&]() {
    if (GET_BIT(frame.head.capture_flag, 2) == 1) {
      cv::Mat grayImg = YuvBufToGrayMat(
          frame.buf + sizeof(CameraFrameHead) + camera_data_lenth * 2,
          camera_data_lenth, option.frame_width, option.frame_hight);

      result.images[1] = grayImg;
    }
  });
  thread1.join();
  thread2.join();
  return result;
}
//
uint64_t DataCapture::GetOrigImuTime(const uint64_t& time) {
#ifdef NEED_SYNC
  if (sys_time_base_.has_value()) {
    return int64(time - sys_time_base_.value().first) +
           sys_time_base_.value().second;
  }
  return 0;
#else
  return time;
#endif
}
//
//
void DataCapture::ProcessImag(const CameraFrame& frame) {
  const auto frame_data = ToFrameData(frame, option_);
  if (frame_data.images[0].empty()) {
    return;
  }
#ifdef NEED_SYNC
  image_catch_.push_back(std::make_pair(frame.head.sys_count, frame_data));
  if (image_catch_.size() <= 2) {
    return;
  }
  image_catch_.erase(image_catch_.begin());
  // if (sys_time_base_.has_value()) {
  for (auto& f : frame_call_backs_) {
    f.second(image_catch_.front().second);
  }
#else
  for (auto& f : frame_call_backs_) {
    f.second(frame_data);
  }
#endif

  // }
}

void DataCapture::SysPorocess() {
  if (sys_time_base_.has_value() || image_catch_.size() != 2) return;
  if (imu_catch_.empty()) return;
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

  if (odom_catch_.empty()) return;
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
void DataCapture::RemoveCallBack(const std::string& id) {
  std::lock_guard<std::mutex> lock(mutex_);
  imu_call_backs_.erase(id);
  frame_call_backs_.erase(id);
  encoder_call_backs_.erase(id);
}

std::unique_ptr<DataCapture> CreateDataCaputure(
    const DataCaptureOption& option) {
  // if (option.use_method == 0) {
    return std::make_unique<DataCapture>(DataCaptureOption{});
  // } else if (option.use_method == 1) {
  //   return std::make_unique<VSLAM::DataCapturer>(10, 200);
  // } else {
  //   LOG(FATAL) << "Unsupport capture type...";
  // }
}
//
}  // namespace jarvis_pic