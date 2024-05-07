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
  uint32_t cam_durion_imu_cout = 20;
  int imu_durition = 5;  // ms
  int frame_width = 640;
  int frame_hight = 544;
};

struct Frame {
  uint64_t id;
  uint64_t time;
  cv::Mat image;
};
struct ImuData {
  uint64_t time;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;
};

class DataCapture {
 public:
  explicit DataCapture(const DataCaptureOption& option);
  ~DataCapture();
  void Rigister(std::function<void(const Frame&)> f) {
    frame_call_backs_.push_back(std::move(f));
  }
  void Rigister(std::function<void(const ImuData&) >f) {
    imu_call_backs_.push_back(std::move(f));
  }
  void Start();
  void Stop();

 private:
  void Run();
  void SysPorocess();
  void ProcessImu(const ModSyncImuFb& imu);
  void ProcessImag(const CameraFrame& frame);
  
  std::vector<std::function<void(const ImuData&)>> imu_call_backs_;
  std::vector<std::function<void(const Frame&)>> frame_call_backs_;
  DataCaptureOption option_;
  std::unique_ptr<ShmSensorQueue> mem_ssq_;
  std::vector<ModSyncImuFb> imu_catch_;
  std::vector<std::pair<uint8_t,Frame>> image_catch_;
  std::thread thread_;
  std::optional<std::pair<uint64_t, uint64_t>> sys_time_base_;
  uint32_t last_frame_sys_count_ = 0;
  uint64_t last_imu_time_stamp_ = 0;
  bool stop_ = false;
};

}  // namespace jarvis_pic
#endif
