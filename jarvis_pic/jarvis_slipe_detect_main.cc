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
#include "slip_detection/slip_detect.h"
#include "time.h"
#include "unistd.h"
#include "zmq_component.h"
#include "jarvis_brige.h"
#include "jarvis/common/time.h"
//
namespace {

jarvis::TrackingData tracking_data_temp;
bool kill_thread_ = false;
constexpr char kImagTopic0[] = "/usb_cam_1/image_raw/compressed";
constexpr char kImagTopic1[] = "/usb_cam_2/image_raw/compressed";
constexpr char kImuTopic[] = "/imu";
constexpr char kOdomTopic[] = "/odom";
int kGLOG_v = 0;
int kVioState = 0;
bool kSlipeState = 0;
uint8_t kRecordFlag = 0;
uint8_t kEnableSlipDetect = 0;
uint8_t kDataCaputureType = 0;
std::ofstream kOImuFile;
std::ofstream kOPoseFile;
std::string image_dir;

void ParseOption(const std::string& config) {
  cv::FileStorage fsSettings(config, cv::FileStorage::READ);
  fsSettings["record"] >> kRecordFlag;
  fsSettings["slip_detect"] >> kEnableSlipDetect;
  fsSettings["GLOG_v"] >> kGLOG_v;
}
}  // namespace
//
using namespace jarvis;

namespace jarvis_pic {
struct jarvis_pic_call_back_data {
  bool slip_flag;
  jarvis::TrackingData data;
};



class JarvisBuilder {
 public:
  JarvisBuilder(const std::string& config,
              std::function<void(const jarvis_pic_call_back_data&)> call_back)
      : config_path_( config),data_capture_(CreateDataCaputure({kDataCaputureType})),
        call_back_(std::move(call_back)) {
    //
 
    // if (kEnableSlipDetect) {
    //   slip_detect_ = jarvis::slip_detect::FactorSlipDetect(config);
    // }

    LOG(INFO) << "Capture start..";

    data_capture_->Rigister("imu_extrapolator", [&](const ImuData& imu) {
      // LOG(INFO)<<jarvis::common::FromUniversal(imu.time * 10);
      // LOG(INFO)<<imu.linear_acceleration.transpose()<<"
      // "<<imu.angular_velocity.transpose();
      jarvis::estimator::ImuState state;
      {
        if (imu_extrapolator_) {
          std::lock_guard<std::mutex> lock(mutex_);
          imu_extrapolator_->AddImu(jarvis::sensor::ImuData{
              jarvis::common::FromUniversal(imu.time * 10),
              imu.linear_acceleration,
              imu.angular_velocity,
          });
          state = imu_extrapolator_->Exrapolate(
              jarvis::common::FromUniversal(imu.time * 10) +
              common::FromSeconds(0.001));
        }
      }
      if (slip_detect_) {
        jarvis::TrackingData data{
            std::make_shared<jarvis::TrackingData::Data>(
                jarvis::TrackingData::Data{
                    jarvis::common::FromUniversal(imu.time * 10), state}),
            kVioState};
        transform::Rigid3d slipe_alignment_pose =
            slip_detect_->ToPoseInOdom(state.pose);
        mpc_.Write(slipe_alignment_pose, data,
                   GetDataCapture()->GetOrigImuTime(imu.time), kSlipeState);
      }
      // auto pose = jarvis::GetGlobleImuExtrapolatorPose();

    });

    data_capture_->Rigister("slip_detect",[&](const OdomData& encode) {
      if(!slip_detect_)return ;
      if (!last_odom_data_.has_value()) {
        last_odom_data_ = encode;
      }
      //
      const Eigen::Vector3d delta_encode =
          encode.translation - last_odom_data_.value().translation;
      last_odom_data_ = encode;
      //
      // LOG(INFO)<<encode.translation.transpose()<< " "<<  delta_encode.transpose();
      if (abs(delta_encode.x()) < 0.001 && abs(delta_encode.y() < 0.001)) {
        if (slip_detect_) {
          slip_detect_->ClearData();
        }
      }
      //

      if (slip_detect_) {
        slip_detect_->AddOdometry(sensor::OdometryData{
            jarvis::common::FromUniversal(encode.time * 10),
            transform::Rigid3d(encode.translation, encode.rotaion)});
      }

    });

    if (kRecordFlag) {
      data_capture_->Rigister("data_record",[&](const ImuData& imu) {
        std::stringstream info;
        static uint64_t last_time = imu.time * 1e3;
        if (uint64_t(imu.time * 1e3) - last_time > 6000000) {
          LOG(INFO) << uint64_t(imu.time * 1e3) - last_time;
        }
        last_time = imu.time * 1e3;
        info << "imu " << std::to_string(uint64_t(imu.time * 1e3)) << " "
             << imu.angular_velocity.x() << " " << imu.angular_velocity.y()
             << " " << imu.angular_velocity.z() << " "
             << imu.linear_acceleration.x() << " "
             << imu.linear_acceleration.y() << " "
             << imu.linear_acceleration.z();
        kOImuFile << info.str() << std::endl;
      });

      data_capture_->Rigister("data_record", [&](const OdomData& odom) {
        std::stringstream info;
        info << "odom " << std::to_string(uint64_t(odom.time * 1e3)) << " "
             << odom.translation.x() << " " << odom.translation.y() << " "
             << odom.translation.z() << " " << odom.rotaion.w() << " "
             << odom.rotaion.x() << " " << odom.rotaion.x() << " "
             << odom.rotaion.z();
        kOImuFile << info.str() << std::endl;
      });
      data_capture_->Rigister("data_record",[&](const Frame& frame) {
        std::thread write_thread([=]() {
          cv::imwrite(image_dir + std::to_string(uint64_t(frame.time * 1e3)) +
                          "_l_.png",
                      frame.images[0]);
          cv::imwrite(image_dir + std::to_string(uint64_t(frame.time * 1e3)) +
                          "_r_.png",
                      frame.images[1]);
        });
        write_thread.detach();
      });
    }
    data_capture_->Rigister([&](const SystmeInfo& state) {
      if (system_state_ != state.state) {
        LOG(WARNING) << "System status change. frome " << int(system_state_) << " to "
                     << int(state.state);
        system_state_ = state.state;
        if (system_state_ == MowStatus::MS_CHARGING ||
            system_state_ == MowStatus::MS_SLEEP) {
          LOG(WARNING) << "Rest jarvis brige...";
          jarvis_brige_.reset(nullptr);
          imu_extrapolator_.reset(nullptr);
          slip_detect_.reset(nullptr);
          // global_odom_= transform::Rigid3d::Identity();
          kVioState= 0;
          }
        else {
          if(!jarvis_brige_){
            CreateJarvisBrige();
          }
        }
      } 
    });

    data_capture_->Start();
  }


  //
  //
  DataCapture* GetDataCapture() { return data_capture_.get(); }
  void CreateJarvisBrige() {
    imu_extrapolator_ = std::make_unique<jarvis::estimator::ImuExtrapolator>();
    slip_detect_ = jarvis::slip_detect::FactorSlipDetect(config_path_);
    jarvis_brige_ = std::make_unique<JarvisBrige>(
        config_path_, data_capture_.get(),
        [&](const jarvis::TrackingData& data) {
          bool slip_flag = false;
          if (slip_detect_) {
            if (data.status != 2) {
              slip_detect_->ClearData();
            } else {
              slip_detect_->AddPose(slip_detect::TimePose{
                  data.data->time, data.data->imu_state.pose});
              slip_flag = slip_detect_->Detect(data.data->time);
            }
          }
          kSlipeState = slip_flag;
          kVioState = data.status;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            imu_extrapolator_->AddState(data.data->time, data.data->imu_state);
          }
          call_back_(jarvis_pic_call_back_data{slip_flag, data});
        });
        
  }

 private:
  
  const std::string config_path_;
  std::optional<OdomData > last_odom_data_;
  
  jarvis_pic::MpcComponent mpc_;
  std::unique_ptr<DataCapture> data_capture_;
  std::unique_ptr<JarvisBrige> jarvis_brige_;
  std::unique_ptr<jarvis::slip_detect::SlipDetect> slip_detect_;
  std::function<void(const jarvis_pic_call_back_data&)> call_back_;
  
  uint8_t system_state_ = 0xff;
  std::mutex mutex_;
  std::unique_ptr<jarvis::estimator::ImuExtrapolator> imu_extrapolator_;  //=
  
};
}  // namespace jarvis_pic
std::string kDataDir = "/mnt/UDISK/jarvis/";
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
  const std::string config_file("/oem/mowpack/ai_model/vslam.yaml");
  //
  //
  //
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  //
 
  ParseOption(config_file);
  CreateDataDir();
  FLAGS_v = kGLOG_v;
  std::mutex jarvis_mutex;
  std::condition_variable con_variable;
  uint8_t slip_flag = 0;
  std::unique_ptr<jarvis_pic::JarvisBuilder> jarvis_slam =
      std::make_unique<jarvis_pic::JarvisBuilder>(
          std::string(config_file),
          [&](const jarvis_pic::jarvis_pic_call_back_data& data) {
            {
              std::lock_guard<std::mutex> lock(jarvis_mutex);
              tracking_data_temp = data.data;
              slip_flag = data.slip_flag;
            }
            con_variable.notify_all();
          });

  jarvis_pic::ZmqComponent zmq;
  jarvis_pic::MpcComponent mpc;
  while (!kill_thread_) {
    uint8_t flag = 0;
    jarvis::TrackingData tracking_data;
    {
      std::unique_lock<std::mutex> lock(jarvis_mutex);
      con_variable.wait(lock);
      tracking_data = tracking_data_temp;
      flag = slip_flag;
    }
    LOG_EVERY_N(WARNING, 60) << tracking_data.data->imu_state.pose;
    if (kRecordFlag) {
      std::stringstream info;
      info << std::to_string(uint64_t(
                  jarvis::common::ToUniversal(tracking_data.data->time) * 1e2))
           << " " << tracking_data.data->imu_state.pose.translation().x() << " "
           << tracking_data.data->imu_state.pose.translation().y() << " "
           << tracking_data.data->imu_state.pose.translation().z() << " "
           << tracking_data.data->imu_state.pose.rotation().w() << " "
           << tracking_data.data->imu_state.pose.rotation().x() << " "
           << tracking_data.data->imu_state.pose.rotation().y() << " "
           << tracking_data.data->imu_state.pose.rotation().z() << " " << flag
           << std::endl;
      kOPoseFile << info.str();
    }

    //
    // mpc.Write(
    //     tracking_data,
    //     jarvis_slam->GetDataCapture()->GetOrigImuTime(static_cast<uint64_t>(
    //         jarvis::common::ToUniversal(tracking_data.data->time) / 10)));
    //
#ifdef __ZMQ_ENABLAE__
    if (tracking_data.status == 2) {
      zmq.PubLocalData(tracking_data, flag);
    }
#endif
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