#include <dirent.h>
#include <sys/types.h>

#include <condition_variable>
#include <map>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include "glog/logging.h"
#include "data_capture.h"
#include "fstream"
#include "jarvis/sensor/data_process.h"
#include "jarvis/sensor/stereo_sync.h"
#include "jarvis/trajectory_builder.h"
#include "mutex"
#include "unistd.h"
#include "zmq_component.h"
#include "jarvis/common/fixed_ratio_sampler.h"
//
namespace 
{

constexpr char kImagTopic0[] = "/usb_cam_1/image_raw/compressed";
constexpr char kImagTopic1[] = "/usb_cam_2/image_raw/compressed";
constexpr char kImuTopic[] = "/imu";
double imu_cam_time_offset = 0;
double image_sample =1;
void ParseOption(const std::string& config) {
  cv::FileStorage fsSettings(config, cv::FileStorage::READ);
  fsSettings["imu_cam_time_offset"] >> imu_cam_time_offset;
  fsSettings["image_sample"] >> image_sample;
}
}
//
// using namespace jarvis;
namespace jarvis_pic {
class JarvisBrige {
 public:
  JarvisBrige(const std::string& config, jarvis::CallBack call_back)
      : data_capture_(new DataCapture(DataCaptureOption{})) {
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
    order_queue_->AddQueue(kImagTopic0,
                           [&](const jarvis::sensor::ImageData& imag_data) {
                              // LOG(INFO) << std::to_string(imag_data.time);
                             builder_->AddImageData(imag_data);
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
      if(!image_sample_->Pulse())return;
      auto temp = std::make_shared<cv::Mat>(frame.image.clone());
      order_queue_->AddData(
          kImagTopic0,
          std::make_unique<
              jarvis::sensor::DispathcData<jarvis::sensor::ImageData>>(
              jarvis::sensor::ImageData{frame.time * 1e-6 + imu_cam_time_offset,
                                        {temp, temp}}));
    });
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

jarvis::TrackingData tracking_data_temp;
bool kill_thread_ = false;
int main(int argc, char* argv[]) {
  google::InitGoogleLogging("jarvis");
  FLAGS_log_dir = std::string("/tmp/jarvis/");
  if (access(FLAGS_log_dir.c_str(), F_OK) == -1) {
    mkdir(FLAGS_log_dir.c_str(), S_IRWXO | S_IRWXG | S_IRWXU);
  }

  //
  //
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  const std::string data_dir(argv[1]);

  LOG(INFO) << data_dir;
  std::cout << data_dir << std::endl;
  std::mutex mutex;
  std::condition_variable cond;
  jarvis_pic::ZmqComponent zmq;
  jarvis_pic::MpcComponent mpc;
  ParseOption(std::string(argv[1]));
  std::unique_ptr<jarvis_pic::JarvisBrige> jarvis_slam =
      std::make_unique<jarvis_pic::JarvisBrige>(
          std::string(argv[1]), [&](const jarvis::TrackingData& data) {
            // LOG(INFO) << data.data->pose;
            {
              std::lock_guard<std::mutex> lock(mutex);
              tracking_data_temp = data;
              cond.notify_all();
            }
            mpc.Write(data);
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