#include "jarvis_brige.h"

#include "data_capture.h"
namespace jarvis_pic {
using namespace jarvis;
constexpr double kImuOdomPrvCamOffTime =0.04;
constexpr char kImagTopic0[] = "/usb_cam_1/image_raw/compressed";
constexpr char kImagTopic1[] = "/usb_cam_2/image_raw/compressed";
constexpr char kImuTopic[] = "/imu";
constexpr char kOdomTopic[] = "/odom";
double image_sample = 1;
double imu_cam_time_offset = 0;
void ParseOption(const std::string& config) {
  cv::FileStorage fsSettings(config, cv::FileStorage::READ);
  fsSettings["image_sample"] >> image_sample;
  fsSettings["imu_cam_time_offset"] >> imu_cam_time_offset;
  LOG(INFO)<< image_sample;
  LOG(INFO)<<imu_cam_time_offset ;
}

JarvisBrige::JarvisBrige(const std::string& config, DataCapture* data_capture,
                         std::function<void(const TrackingData&)> call_back)
    : data_capture_(data_capture) {
  //
  LOG(INFO) << "Jarvis start...";
  ParseOption(config);
  order_queue_ = std::make_unique<jarvis::sensor::OrderedMultiQueue>();
  image_sample_ =
      std::make_unique<jarvis::common::FixedRatioSampler>(image_sample);
  builder_ = std::make_unique<jarvis::TrajectorBuilder>(std::string(config),
                                                        std::move(call_back));

  order_queue_->AddQueue(kImuTopic,
                         [&](const jarvis::sensor::ImuData& imu_data) {
                          // LOG(INFO) << imu_data.time;
                           builder_->AddImuData(jarvis::sensor::ImuData{
                               imu_data.time + common::FromSeconds(kImuOdomPrvCamOffTime),
                               imu_data.linear_acceleration,
                               imu_data.angular_velocity,
                           });
                         });
  //
  order_queue_->AddQueue(
      kOdomTopic, [&](const jarvis::sensor::OdometryData& odom_data) {
        // LOG(INFO) << odom_data.time;
        builder_->AddOdometryData(jarvis::sensor::OdometryData{
            odom_data.time + common::FromSeconds(kImuOdomPrvCamOffTime),
            odom_data.pose
        });
      });
  order_queue_->AddQueue(
      kImagTopic0, [&](const jarvis::sensor::ImageData& imag_data) {

        if (imag_data.image[0]->empty() || imag_data.image[1]->empty()) {
          LOG(WARNING) << "Input Image empty..";
          return;
        }
        auto start = std::chrono::high_resolution_clock::now();
        builder_->AddImageData(imag_data);
        // LOG(INFO) << "One frame cost: "
        //           << std::chrono::duration_cast<std::chrono::milliseconds>(
        //                  std::chrono::high_resolution_clock::now() - start)
        //                  .count();
      });

  data_capture_->Rigister(class_name_, [&](const Frame& frame) {
    if (frame.time == 0) return;
    static uint64_t last_time = frame.time;
    int64_t delta_t = frame.time-   last_time;
    CHECK(delta_t >= 0) << delta_t;
    if (delta_t >= 50636) {
      LOG(WARNING) << frame.time << " " << delta_t;
    }
    last_time = frame.time;
    if (!image_sample_->Pulse()) return;
    //  cv::Mat temp1;
    // //  cv::equalizeHist( frame.image, temp1);
    //  static cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(10.0, cv::Size(8,
    //  8)); clahe->apply( frame.image, temp1);
    // auto temp = std::make_shared<cv::Mat>(temp1.clone());
    order_queue_->AddData(
        kImagTopic0,
        std::make_unique<
            jarvis::sensor::DispathcData<jarvis::sensor::ImageData>>(
            jarvis::sensor::ImageData{
                jarvis::common::FromUniversal(frame.time * 10) +
                    jarvis::common::FromSeconds(imu_cam_time_offset),
                {std::make_shared<cv::Mat>(frame.images[0]),
                 std::make_shared<cv::Mat>(frame.images[1])}}));
  });

  LOG(INFO) << "Capture start..";

  data_capture_->Rigister(class_name_, [&](const ImuData& imu) {
    static uint64_t last_time = imu.time;
    int64_t  delta_t = imu.time-   last_time;
    CHECK(delta_t >= 0) << delta_t;
    if (delta_t > 10001) {
      LOG(WARNING) << imu.time << " " << delta_t;
    }
    last_time = imu.time;
    // LOG(INFO)<<imu.linear_acceleration.transpose();
    order_queue_->AddData(
        kImuTopic,
        std::make_unique<jarvis::sensor::DispathcData<jarvis::sensor::ImuData>>(
            jarvis::sensor::ImuData{
                jarvis::common::FromUniversal(imu.time * 10) -
                    common::FromSeconds(kImuOdomPrvCamOffTime),
                imu.linear_acceleration,
                imu.angular_velocity,
            }));
  });

  data_capture_->Rigister(class_name_, [&](const OdomData& odom) {
    static uint64_t last_time = odom.time;
    int64_t delta_t = odom.time - last_time;
    CHECK(delta_t >= 0) << delta_t;
    if (delta_t > 16001) {
      LOG(WARNING) << odom.time << " " << delta_t;
    }
    last_time =odom.time;
    order_queue_->AddData(
        kOdomTopic,
        std::make_unique<
            jarvis::sensor::DispathcData<jarvis::sensor::OdometryData>>(
            jarvis::sensor::OdometryData{
                jarvis::common::FromUniversal(odom.time * 10) -
                    common::FromSeconds(kImuOdomPrvCamOffTime),
                transform::Rigid3d(odom.translation, odom.rotaion)}));
  });

  order_queue_->Start();
}
//
JarvisBrige::~JarvisBrige() {
  data_capture_->RemoveCallBack(class_name_);
  order_queue_->Stop();
}
}  // namespace jarvis_pic