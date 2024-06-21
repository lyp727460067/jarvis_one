#include <dirent.h>
#include <sys/types.h>

#include <condition_variable>
#include <map>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "data_capture.h"
#include "fstream"
#include "glog/logging.h"
#include "jarvis/sensor/data_process.h"
#include "jarvis/sensor/stereo_sync.h"
#include "jarvis/trajectory_builder.h"
#include "mutex"
#include "ostream"
#include "time.h"
#include "unistd.h"
#include "zmq_component.h"

//
namespace {

jarvis::TrackingData tracking_data_temp;
bool kill_thread_ = false;
constexpr char kImagTopic0[] = "/usb_cam_1/image_raw/compressed";
constexpr char kImagTopic1[] = "/usb_cam_2/image_raw/compressed";
constexpr char kImuTopic[] = "/imu";
constexpr char kOdomTopic[] = "/odom";
double image_sample = 1;
uint8_t kRecordFlag = 0;
uint8_t kDataCaputureType = 0;
std::ofstream kOImuFile;
std::ofstream kOPoseFile;
std::string image_dir;
void ParseOption(const std::string& config) {
  cv::FileStorage fsSettings(config, cv::FileStorage::READ);
  fsSettings["image_sample"] >> image_sample;
  fsSettings["record"] >> kRecordFlag;
}
}  // namespace
//
// using namespace jarvis;
namespace jarvis_pic {
class JarvisBrige {
 public:
  JarvisBrige(const std::string& config, jarvis::CallBack call_back)
      : data_capture_(CreateDataCaputure({kDataCaputureType})) {
    //

    LOG(INFO) << "Jarvis start...";

    if (kRecordFlag) {
      data_capture_->Rigister([&](const ImuData& imu) {
        std::stringstream info;
        info << "imu " << std::to_string(uint64_t(imu.time * 1e3)) << " "
             << imu.angular_velocity.x() << " " << imu.angular_velocity.y()
             << " " << imu.angular_velocity.z() << " "
             << imu.linear_acceleration.x() << " "
             << imu.linear_acceleration.y() << " "
             << imu.linear_acceleration.z();

        kOImuFile << info.str() << std::endl;
      });

      data_capture_->Rigister([&](const EncoderData& imu) {
        std::stringstream info;
        info << "odom " << std::to_string(uint64_t(imu.time * 1e3)) << " "
             << imu.left_encoder << " " << imu.right_encoder;
        kOImuFile << info.str() << std::endl;
      });

      data_capture_->Rigister([&](const Frame& frame) {
        cv::imwrite(
            image_dir + std::to_string(uint64_t(frame.time * 1e3)) + "_l_.png",
            frame.images[0]);
        cv::imwrite(
            image_dir + std::to_string(uint64_t(frame.time * 1e3)) + "_r_.png",
            frame.images[1]);
      });
    }
    data_capture_->Start();
  }
  DataCapture* GetDataCapture() { return data_capture_.get(); }

 private:
  jarvis_pic::MpcComponent mpc_;
  std::unique_ptr<DataCapture> data_capture_;
  std::unique_ptr<jarvis::sensor::OrderedMultiQueue> order_queue_;
  std::unique_ptr<jarvis::TrajectorBuilder> builder_;
};
}  // namespace jarvis_pic
std::string kDataDir  = "/mnt/UDISK/jarvis/";
void CreateDataDir() {
  if (kRecordFlag) {
    time_t now;
    struct tm* local;
    time(&now);
    local = localtime(&now);
    std::string name = std::to_string(local->tm_year + 1900) + "_" +
                       std::to_string(local->tm_mon + 1) + "_" +
                       std::to_string(local->tm_mday) + "_" +
                       std::to_string(local->tm_hour) + "_" +
                       std::to_string(local->tm_min);
    std::string data_d = kDataDir + "data/";
    if (access(data_d.c_str(), F_OK) == -1) {
      mkdir(data_d.c_str(), S_IRWXO | S_IRWXG | S_IRWXU);
    }
    const std::string data_dir = data_d + name + "/";
    if (access(data_dir.c_str(), F_OK) == -1) {
      mkdir(data_dir.c_str(), S_IRWXO | S_IRWXG | S_IRWXU);
    }

    image_dir = data_dir + "image/";
    if (access(image_dir.c_str(), F_OK) == -1) {
      mkdir(image_dir.c_str(), S_IRWXO | S_IRWXG | S_IRWXU);
    }
    LOG(INFO) << "Record data dir : " << data_dir;
    LOG(INFO) << "Record image_dir dir : " << image_dir;
    const std::string imu_file = data_dir + "imu.txt";
    const std::string pose_file = data_dir + "vio_odom.txt";
    kOImuFile.open(imu_file, std::ios::out);
    if (kOImuFile.good()) {
      LOG(INFO) << " open imu file : " << imu_file << " done.";
    }
    kOPoseFile.open(pose_file, std::ios::out);
    if (kOPoseFile.good()) {
      LOG(INFO) << " open pose file : " << pose_file << " done.";
    }
  }
}
//
int main(int argc, char* argv[]) {
  google::InitGoogleLogging("jarvis");
  FLAGS_log_dir = kDataDir;
  //
  if (access(FLAGS_log_dir.c_str(), F_OK) == -1) {
    mkdir(FLAGS_log_dir.c_str(), S_IRWXO | S_IRWXG | S_IRWXU);
  }
  //
  //
  const std::string config_file(
      "/oem/mowpack/vslam/configuration/simple_vo.yaml");
  //
  ParseOption(config_file);
  //
  //
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  //
  CreateDataDir();
  std::mutex jarvis_mutex;
  std::condition_variable con_variable;

  std::unique_ptr<jarvis_pic::JarvisBrige> jarvis_slam =
      std::make_unique<jarvis_pic::JarvisBrige>(
          std::string(config_file), [&](const jarvis::TrackingData& data) {
            {
              std::lock_guard<std::mutex> lock(jarvis_mutex);
              tracking_data_temp = data;
            }
             con_variable.notify_all();
          });

  while (!kill_thread_) {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  if (kRecordFlag) {
    kOPoseFile.close();
    kOImuFile.close();
  }
  LOG(INFO) << "Release jarvis...";
  kill_thread_ = false;
  jarvis_slam = nullptr;
  sleep(1);
  con_variable.notify_all();

  return 0;
}