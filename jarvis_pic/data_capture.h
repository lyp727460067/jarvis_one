#ifndef _JARVIS_PIC_DATA_CAPTURE_H
#define _JARVIS_PIC_DATA_CAPTURE_H
#include <functional>
#include <memory>
#include <opencv2/opencv.hpp>
#include <optional>

#include "Eigen/Core"
#include "shm_mpmc_frame.h"
#include "shm_sensor_queue.h"
namespace jarvis_pic {
struct DataCaptureOption {
  int use_method = 0;
  uint32_t cam_durion_imu_cout = 20;
  uint32_t cam_durion_odom_cout = 20;
  int imu_durition = 4;  // ms
  int frame_width = 640;
  int frame_hight = 544;
};

struct Frame {
  uint64_t id;
  uint64_t time;
  cv::Mat image;
};
struct ImuData 
{
  uint64_t time;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;
};
struct EncoderData {
  uint64_t time;
  int32_t left_encoder;
  int32_t right_encoder;
};
class DataCapture {
 public:
  explicit DataCapture(const DataCaptureOption& option);
  explicit DataCapture(){};
  virtual ~DataCapture();
  void Rigister(std::function<void(const Frame&)> f) {
    frame_call_backs_.push_back(std::move(f));
  }
  void Rigister(std::function<void(const ImuData&) >f) {
    imu_call_backs_.push_back(std::move(f));
  }
  void Rigister(std::function<void(const EncoderData&) >f) {
    encoder_call_backs_.push_back(std::move(f));
  }
  virtual uint64_t GetOrigImuTime(const uint64_t& time);
  virtual void Start();
  virtual void Stop();

 protected:
  void Run();
  void SysPorocess();
  void SysPorocessOdom();
  void ProcessImu(const ModSyncImuFb& imu);
  void ProcessImag(const CameraFrame& frame);
  void ProcessOdom(const ModSyncChassisPosFb& frame);
  
  std::vector<std::function<void(const ImuData&)>> imu_call_backs_;
  std::vector<std::function<void(const Frame&)>> frame_call_backs_;
  std::vector<std::function<void(const EncoderData&)>> encoder_call_backs_;
  DataCaptureOption option_;
  std::unique_ptr<ShmSensorQueue> mem_ssq_;
  std::vector<ModSyncImuFb> imu_catch_;
  std::vector<std::pair<uint64_t,EncoderData>> odom_catch_;
  std::vector<std::pair<uint8_t,Frame>> image_catch_;
  std::thread thread_;
  std::optional<std::pair<uint64_t, uint64_t>> sys_time_base_;
  std::optional<std::pair<uint64_t, uint64_t>> sys_odom_time_base_;
  uint32_t last_frame_sys_count_ = 0;
  uint64_t last_imu_time_stamp_ = 0;
  uint64_t last_odom_time_stamp_ = 0;
  bool stop_ = false;

};
std::unique_ptr<DataCapture> CreateDataCaputure(
    const DataCaptureOption& option);
}  // namespace jarvis_pic
#endif
