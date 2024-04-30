#include <dirent.h>
#include <sys/types.h>

#include <map>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "data_capture.h"
#include "fstream"
#include "jarvis/sensor/data_process.h"
#include "jarvis/sensor/stereo_sync.h"
#include "jarvis/trajectory_builder.h"
#include "unistd.h"
//

constexpr char kImagTopic0[] = "/usb_cam_1/image_raw/compressed";
constexpr char kImagTopic1[] = "/usb_cam_2/image_raw/compressed";
constexpr char kImuTopic[] = "/imu";

//
// using namespace jarvis;
namespace jarvis_pic {
class JarvisBrige {
 public:
  JarvisBrige(const std::string& config, jarvis::CallBack call_back)
      : data_capture_(new DataCapture(DataCaptureOption{})) {
    //
    LOG(INFO) << "Jarvis start...";
    builder_ = std::make_unique<jarvis::TrajectorBuilder>(
        std::string(config),
        [&](const jarvis::TrackingData& data) { call_back(data); });
    //
    LOG(INFO) << "Orlder queue start..";
    order_queue_ = std::make_unique<jarvis::sensor::OrderedMultiQueue>();
    order_queue_->AddQueue(kImuTopic,
                           [](const jarvis::sensor::ImuData& imu_data) {
                             // builder_->AddImuData(imu_data);
                           });
    //
    order_queue_->AddQueue(kImagTopic0,
                           [](const jarvis::sensor::ImageData& imag_data) {
                             // builder_->AddImageData(imag_data);
                           });

    LOG(INFO) << "Capture start..";
    data_capture_->Rigister([&](const ImuData& imu) {
      order_queue_->AddData(
          kImuTopic, std::make_unique<
                         jarvis::sensor::DispathcData<jarvis::sensor::ImuData>>(
                         jarvis::sensor::ImuData{
                             imu.time * 1e-9,
                             imu.linear_acceleration,
                             imu.angular_velocity,
                         }));
    });
    data_capture_->Rigister([&](const Frame& frame) {
      order_queue_->AddData(
          kImagTopic0,
          std::make_unique<
              jarvis::sensor::DispathcData<jarvis::sensor::ImageData>>(
              jarvis::sensor::ImageData{frame.time * 1e-9,
                                        {frame.image, frame.image}}));
    });

    data_capture_->Start();
  }

 private:
  std::unique_ptr<DataCapture> data_capture_;
  std::unique_ptr<jarvis::sensor::OrderedMultiQueue> order_queue_; 
  std::unique_ptr<jarvis::TrajectorBuilder> builder_  ;
};
}  // namespace jarvis_pic
bool kill_thread_ = false;
int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  //
  //
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  const std::string data_dir(argv[2]);
  std::unique_ptr<jarvis_pic::JarvisBrige> jarvis_slam =
      std::make_unique<jarvis_pic::JarvisBrige>(
          std::string(argv[1]), [](const jarvis::TrackingData& data) {
            LOG(INFO) << data.data->pose;
          });
  while (kill_thread_) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return 0;
}