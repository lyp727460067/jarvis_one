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
#include "jarvis/common/fixed_ratio_sampler.h"
#include "jarvis/sensor/data_process.h"
#include "jarvis/sensor/stereo_sync.h"
#include "jarvis/trajectory_builder.h"
#include "mutex"
#include "ostream"
#include "time.h"
#include "unistd.h"
#include "zmq_component.h"

#include "SensorDataCapturer/DataCapturer.h"
//
namespace {

constexpr char kImagTopic0[] = "/usb_cam_1/image_raw/compressed";
constexpr char kImagTopic1[] = "/usb_cam_2/image_raw/compressed";
constexpr char kImuTopic[] = "/imu";
double imu_cam_time_offset = 0;
double image_sample = 1;
uint8_t kRecordFlag = 0;
std::ofstream kOImuFile;
std::ofstream kOPoseFile;
std::string image_dir;

void ParseOption(const std::string& config) {
  cv::FileStorage fsSettings(config, cv::FileStorage::READ);
  fsSettings["imu_cam_time_offset"] >> imu_cam_time_offset;
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
      : data_capture_(new VSLAM::DataCapturer(10,200)) 
  {
    //
    image_sample_ =
        std::make_unique<jarvis::common::FixedRatioSampler>(image_sample);
    LOG(INFO) << "Jarvis start...";
    builder_ = std::make_unique<jarvis::TrajectorBuilder>(std::string(config),
                                                          std::move(call_back));
    //
    LOG(INFO) << "Orlder queue start..";
    order_queue_ = std::make_unique<jarvis::sensor::OrderedMultiQueue>();
    order_queue_->AddQueue(kImuTopic,
                           [&](const jarvis::sensor::ImuData& imu_data) {
                             // LOG(INFO) << std::to_string(imu_data.time);
                             // LOG(INFO)<<
                             //      imu_data.linear_acceleration.transpose()
                             //     << imu_data.angular_velocity.transpose();
                             builder_->AddImuData(imu_data);
                           });
    //
    order_queue_->AddQueue(
        kImagTopic0, [&](const jarvis::sensor::ImageData& imag_data) {
          // LOG(INFO) << std::to_string(imag_data.time);
          auto start = std::chrono::high_resolution_clock::now();
          builder_->AddImageData(imag_data);
          LOG(INFO) << "One frame cost: "
                    << std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::high_resolution_clock::now() - start)
                           .count();
        });

    LOG(INFO) << "Capture start..";

    data_capture_->Rigister([&](const ImuData& imu) {
      order_queue_->AddData(
          kImuTopic, std::make_unique<
                         jarvis::sensor::DispathcData<jarvis::sensor::ImuData>>(
                         jarvis::sensor::ImuData{
                             imu.time * 1e-6,
                             imu.linear_acceleration,
                             imu.angular_velocity,
                         }));
    });
    data_capture_->Rigister([&](const Frame& frame) {
      if (!image_sample_->Pulse()) return;
      auto temp = std::make_shared<cv::Mat>(frame.image.clone());
      order_queue_->AddData(
          kImagTopic0,
          std::make_unique<
              jarvis::sensor::DispathcData<jarvis::sensor::ImageData>>(
              jarvis::sensor::ImageData{frame.time * 1e-6 + imu_cam_time_offset,
                                        {temp, temp}}));
    });
    if (kRecordFlag) {
      data_capture_->Rigister([&](const ImuData& imu) {
        std::stringstream info;
        info << std::to_string(uint64_t(imu.time * 1e3)) << " "
             << imu.angular_velocity.x() << " " << imu.angular_velocity.y()
             << " " << imu.angular_velocity.z() << " "
             << imu.linear_acceleration.x() << " "
             << imu.linear_acceleration.y() << " "
             << imu.linear_acceleration.z();

        kOImuFile << info.str() << std::endl;
      });
      data_capture_->Rigister([&](const Frame& frame) {
        cv::imwrite(
            image_dir + std::to_string(uint64_t(frame.time * 1e3)) + ".png",
            frame.image);
      });
    }
    order_queue_->Start();
    data_capture_->Start();
  }

 private:
  std::unique_ptr<DataCapture> data_capture_;
  std::unique_ptr<jarvis::sensor::OrderedMultiQueue> order_queue_;
  std::unique_ptr<jarvis::TrajectorBuilder> builder_;
  std::unique_ptr<jarvis::common::FixedRatioSampler> image_sample_;
};
}  // namespace jarvis_pic

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
    if (access("/tmp/jarvis/data/", F_OK) == -1) {
      mkdir("/tmp/jarvis/data/", S_IRWXO | S_IRWXG | S_IRWXU);
    }
    const std::string data_dir = std::string("/tmp/jarvis/data/") + name + "/";
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
jarvis::TrackingData tracking_data_temp;
//
bool kill_thread_ = false;
int main(int argc, char* argv[]) {
  google::InitGoogleLogging("jarvis");
  FLAGS_log_dir = std::string("/tmp/jarvis/");
  //
  if (access(FLAGS_log_dir.c_str(), F_OK) == -1) {
    mkdir(FLAGS_log_dir.c_str(), S_IRWXO | S_IRWXG | S_IRWXU);
  }
  //
  ParseOption(std::string(argv[1]));
  //
  //
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  //
  CreateDataDir();
  std::mutex mutex;
  std::condition_variable cond;
  jarvis_pic::ZmqComponent zmq;
  jarvis_pic::MpcComponent mpc;
  std::unique_ptr<jarvis_pic::JarvisBrige> jarvis_slam =
      std::make_unique<jarvis_pic::JarvisBrige>(
          std::string(argv[1]), [&](const jarvis::TrackingData& data) {
            // LOG(INFO) << data.data->pose;
            {
              std::lock_guard<std::mutex> lock(mutex);
              tracking_data_temp = data;
              cond.notify_all();
            }
            std::stringstream info;
            info << std::to_string(uint64_t(
                        jarvis::common::ToUniversal(data.data->time) * 1e2))
                 << " " << data.data->pose.translation().x() << " "
                 << data.data->pose.translation().y() << " "
                 << data.data->pose.translation().z() << " "
                 << data.data->pose.rotation().w() << " "
                 << data.data->pose.rotation().x() << " "
                 << data.data->pose.rotation().y() << " "
                 << data.data->pose.rotation().z() << std::endl;
            kOPoseFile << info.str();
            // mpc.Write(data);
          });
#ifdef __ZMQ_ENABLAE__
  // jarvis_pic::ZmqComponent zmq;
  while (!kill_thread_) {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    jarvis::TrackingData tracking_data;
    {
      std::unique_lock<std::mutex> lock(mutex);
      cond.wait(lock);
      tracking_data = tracking_data_temp;
    }
    zmq.PubLocalData(tracking_data);
  }
#else
  while (!kill_thread_) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
#endif

  return 0;
}