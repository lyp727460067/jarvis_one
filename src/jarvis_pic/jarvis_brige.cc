#include "jarvis_brige.h"

#include "data_capture.h"
#include "gpu_pyramid_compute.h"
#include "opencl_handler.h"
namespace jarvis_pic {
using namespace jarvis;
constexpr double kImuOdomPrvCamOffTime = 0.1;
constexpr char kImagTopic0[] = "/usb_cam_1/image_raw/compressed";
constexpr char kImagTopic1[] = "/usb_cam_2/image_raw/compressed";
constexpr char kImuTopic[] = "/imu";
constexpr char kOdomTopic[] = "/odom";
Eigen::Vector3d gry_bise{0.0100714, 0.00475531, 0.00846774};
double image_sample = 1;
double imu_cam_time_offset = 0;
int kuse_gpu = 0;
void ParseOption(const std::string& config) {
  cv::FileStorage fsSettings(config, cv::FileStorage::READ);
  fsSettings["image_sample"] >> image_sample;
  fsSettings["imu_cam_time_offset"] >> imu_cam_time_offset;
  LOG(INFO) << image_sample;
  LOG(INFO) << imu_cam_time_offset;
  fsSettings["use_gpu"] >> kuse_gpu;
}

JarvisBrige::JarvisBrige(const std::string& config, DataCapture* data_capture,
                         std::function<void(const TrackingData&)> call_back)
    : data_capture_(data_capture) {
  //
  LOG(INFO) << "Jarvis start...";
  opencl_handler_ = std::make_unique<OpenCLHandler>();
  ParseOption(config);
  order_queue_ = std::make_unique<jarvis::sensor::OrderedMultiQueue>();
  image_sample_ =
      std::make_unique<jarvis::common::FixedRatioSampler>(image_sample);
  low_image_sample_ =
      std::make_unique<jarvis::common::FixedRatioSampler>(image_sample / 2);

  //
  esit_option_ = estimator::ParseEstimatorOption(std::string(config));

  if (kuse_gpu) {
    for (size_t i = 0; i < esit_option_.track_sequence.size(); i++) {
      for (size_t j = 0; j < esit_option_.track_sequence[i].size(); j++) {
        esit_option_.feature_track_options[i].pyramid_image.push_back(
            std::make_shared<jarvis::estimator::ExtendPyramidImage>(
                esit_option_.feature_track_options[i].pyrmid_option));
      }
    }
    //
  }
  builder_ = std::make_unique<jarvis::TrajectorBuilder>(esit_option_,
                                                        std::move(call_back));

  imu_cam_time_offset  = jarvis::GetTimeShiftCamImu();
  LOG(INFO)<<"imu_cam_time_offset   "<<imu_cam_time_offset  ;
  order_queue_->AddQueue(
      kImuTopic, [&](const jarvis::sensor::ImuData& imu_data) {
        // LOG(INFO) << imu_data.time;
        // LOG(INFO)<<imu_data.angular_velocity.transpose();
        // LOG(INFO)<<imu_data.linear_acceleration.transpose();
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
            odom_data.pose});
      });
  order_queue_->AddQueue(kImagTopic0, [&](const jarvis::sensor::ImageData&
                                              imag_data) {
    if (imag_data.image[0].empty() || imag_data.image[1].empty() /*||
        imag_data.image[2].empty() || imag_data.image[3].empty()*/) {
      LOG(WARNING) << "Input Image empty..";
      return;
    }

    if (kuse_gpu) {
      if (image_datas_pyra_.size() >= 1) {
        if (pyramid_thread_.joinable()) {
          auto start = std::chrono::high_resolution_clock::now();
          pyramid_thread_.join();
          VLOG(1) << "join frame cost: "
                    << std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::high_resolution_clock::now() - start)
                           .count();
        }

        //
        if (image_datas_pyra_.size() == 2) {
          image_datas_pyra_.erase(image_datas_pyra_.begin());
        }
        
        image_datas_pyra_.push_back({false, imag_data});
        image_datas_pyra_.back().second.pyramid_derive.resize(
            image_datas_pyra_.back().second.image.size());
        //
        pyramid_thread_ = std::thread([this, imag_data]() {
          for (size_t i = 0; i < esit_option_.track_sequence.size(); i++) {
            for (size_t j = 0; j < esit_option_.track_sequence[i].size(); j++) {

              std::vector<cv::Mat> pyrmd_drev = BuildPyramidsUsingGPU(
                  imag_data.image[esit_option_.track_sequence[i][j]],
                  opencl_handler_.get(),
                  &esit_option_.feature_track_options[i].pyrmid_option,
                  esit_option_.feature_track_options[i].klt_type);
              image_datas_pyra_.back()
                  .second.pyramid_derive[esit_option_.track_sequence[i][j]] =
                  pyrmd_drev;
            }
          }
          image_datas_pyra_.back().first = true;
        });

        for (size_t i = 0; i < esit_option_.track_sequence.size(); i++) {
          for (size_t j = 0; j < esit_option_.track_sequence[i].size(); j++) {
            esit_option_.feature_track_options[i]
                .pyramid_image[j]
                ->SetCurrPyram(
                    image_datas_pyra_[0]
                        .second
                        .pyramid_derive[esit_option_.track_sequence[i][j]]);
          }
        }
        auto start = std::chrono::high_resolution_clock::now();
        builder_->AddImageData(image_datas_pyra_[0].second);
        VLOG(1) << "One frame cost: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::high_resolution_clock::now() - start)
                         .count();
      } else {
        image_datas_pyra_.push_back({false, imag_data});
        image_datas_pyra_.back().second.pyramid_derive.resize(
            image_datas_pyra_.back().second.image.size());
        pyramid_thread_ = std::thread([this, imag_data]() {
          for (size_t i = 0; i < esit_option_.track_sequence.size(); i++) {
            for (size_t j = 0; j < esit_option_.track_sequence[i].size(); j++) {
              std::vector<cv::Mat> pyrmd_drev = BuildPyramidsUsingGPU(
                  imag_data.image[esit_option_.track_sequence[i][j]],
                  opencl_handler_.get(),
                  &esit_option_.feature_track_options[i].pyrmid_option,
                  esit_option_.feature_track_options[i].klt_type);
              image_datas_pyra_.back()
                  .second.pyramid_derive[esit_option_.track_sequence[i][j]] =
                  pyrmd_drev;
            }
          }
          image_datas_pyra_.back().first = true;
        });
      }
    } else {
      // auto start = std::chrono::high_resolution_clock::now();
      builder_->AddImageData(imag_data);
      // LOG(INFO) << "One frame cost: "
                // << std::chrono::duration_cast<std::chrono::milliseconds>(
                      //  std::chrono::high_resolution_clock::now() - start)
                      //  .count();
    }
    // auto start = std::chrono::high_resolution_clock::now();
    // builder_->AddImageData(imag_data);
    // LOG(INFO) << "One frame cost: "
    //           << std::chrono::duration_cast<std::chrono::milliseconds>(
    //                  std::chrono::high_resolution_clock::now() - start)
    //                  .count();
  });

  data_capture_->Rigister(class_name_, [&](const Frame& frame) {
    if (frame.time == 0) return;
    newst_frame_time_ = frame.time;
    static uint64_t last_time = frame.time;
    int64_t delta_t = frame.time - last_time;
    if (delta_t <= 0) {
      LOG(WARNING) << "image time reorde.." << delta_t << " cur: " << frame.time
                   << " last: " << last_time;
      last_time = frame.time;
      return;
    }
    // CHECK(delta_t >= 0) << delta_t;
    if (delta_t >= 80636) {
      LOG(WARNING) << "image .. " << frame.time << " " << delta_t
                   << " last: " << frame.time;
    }
    last_time = frame.time;

    //  cv::Mat temp1;
    // //  cv::equalizeHist( frame.image, temp1);
    //  static cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(10.0, cv::Size(8,
    //  8)); clahe->apply( frame.image, temp1);
    // auto temp = std::make_shared<cv::Mat>(temp1.clone());


   if (!image_sample_->Pulse()) return;
    order_queue_->AddData(
        kImagTopic0,
        std::make_unique<
            jarvis::sensor::DispathcData<jarvis::sensor::ImageData>>(
            jarvis::sensor::ImageData{
                jarvis::common::FromUniversal(frame.time * 10) +
                    jarvis::common::FromSeconds(imu_cam_time_offset),
                {frame.images[0], frame.images[1], frame.images[2],
                 frame.images[3]}}));
  });

  LOG(INFO) << "Capture start..";

  data_capture_->Rigister(class_name_, [&](const ImuData& imu) {
    static uint64_t last_time = imu.time;
    newst_imu_time_ = imu.time;
    //

    if (newst_frame_time_.has_value() && newst_imu_time_.has_value()) {
      const int64_t delta_time =
          newst_frame_time_.value() - newst_imu_time_.value();
      LOG_EVERY_N(WARNING, 1000) << "Frame behind imu " << delta_time
                                 << " frame: " << newst_frame_time_.value()
                                 << " imu:" << newst_imu_time_.value();
      LOG(WARNING) << (abs(delta_time) < 2000000)
                   << "The camera is too delayed IMU " << delta_time
                   << "frame: " << newst_frame_time_.value()
                   << "imu: " << newst_imu_time_.value();

      //
    }

    // /
    //
    int64_t delta_t = imu.time - last_time;
    if (delta_t <= 0) {
      LOG(WARNING) << "imu time reorde.." << delta_t << " cur: " << imu.time
                   << " last: " << last_time;
      last_time = imu.time;
      return;
    }
    // CHECK(delta_t >= 0) << delta_t;
    if (delta_t > 22001) {
      LOG(WARNING) << "imu lost: " << imu.time << " " << delta_t
                   << " last: " << last_time;
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
    //
    if (delta_t <= 0) {
      LOG(WARNING) << "odom time reorde.." << delta_t << " cur: " << odom.time
                   << " last : " << last_time;
      last_time = odom.time;
      return;
    }
    // CHECK(delta_t >= 0) << delta_t;
    if (delta_t > 46001) {
      LOG(WARNING) << "odo lost: " << odom.time << " " << delta_t
                   << " last: " << last_time;
      if (delta_t > 3600000000) {
        LOG(WARNING) << "odo jump too big..";
        last_time = odom.time;
        return;
      }
    }
    last_time = odom.time;
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

  if (pyramid_thread_.joinable()) {
    pyramid_thread_.join();
  }

}
}  // namespace jarvis_pic