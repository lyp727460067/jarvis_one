#ifndef _JARVIS_PIC_DATA_RECORD_H
#define _JARVIS_PIC_DATA_RECORD_H
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "jarvis/transform/rigid_transform.h"
#include "data_capture.h"
#include "jarvis/common/time.h"
namespace jarvis_pic {
class DataRecord {
 public:
  explicit DataRecord(const std::string& data_path, bool record = false);
  ~DataRecord();
  void AddFrame(const Frame& frame);
  void AddImu(const ImuData& imu);
  void AddOdom(const OdomData& odom);
  void AddVioData(
      jarvis::common::Time& time, const jarvis::transform::Rigid3d& pose,
      bool slipe);

 private:
  void Run();
  void CreateDataDir();
  bool record_;
  std::string data_path_;
  std::mutex mutex_;
  std::thread thread_;
  bool kill_thread_ =false;
  std::queue<std::function<void(void)>> tasks_;
  std::string image_data_dir_;
  std::ofstream imu_file_;
  std::ofstream pose_file_;
  
};
}  // namespace jarvis_pic

#endif