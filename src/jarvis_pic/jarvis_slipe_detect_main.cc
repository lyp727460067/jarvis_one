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
#include "jarvis/common/time.h"
#include "jarvis/sensor/data_process.h"
#include "jarvis/sensor/stereo_sync.h"
#include "jarvis/trajectory_builder.h"
#include "jarvis_brige.h"
#include "mutex"
#include "ostream"
#include "slip_detection/slip_detect.h"
#include "time.h"
#include "unistd.h"
#include "zmq_component.h"
#include "data_record.h"
#include "glog_sink.h"
//

namespace {
jarvis::TrackingData tracking_data_temp;
bool kill_thread_ = false;

std::unique_ptr<jarvis_pic::DataRecord> data_record_ = nullptr;
constexpr char kImagTopic0[] = "/usb_cam_1/image_raw/compressed";
constexpr char kImagTopic1[] = "/usb_cam_2/image_raw/compressed";
constexpr char kImuTopic[] = "/imu";
constexpr char kOdomTopic[] = "/odom";
int kGLOG_v = 0;
int kVioState = 0;
int kWriteMpcPoseType = 0;

bool kSlipeState = 0;
uint8_t kRecordFlag = 0;
uint8_t kEnableSlipDetect = 0;
uint8_t kDataCaputureType = 0;

std::string test_ip;

void ParseOption(const std::string& config) {
  cv::FileStorage fsSettings(config, cv::FileStorage::READ);
  fsSettings["record"] >> kRecordFlag;
  fsSettings["slip_detect"] >> kEnableSlipDetect;
  fsSettings["GLOG_v"] >> kGLOG_v;
  fsSettings["test_ip"] >>test_ip;
  fsSettings["write_mpc_pose_type"] >>kWriteMpcPoseType;
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
      : config_path_(config),
        data_capture_(CreateDataCaputure({kDataCaputureType})),
        call_back_(std::move(call_back)) {
    //

    // if (kEnableSlipDetect) {
    //   slip_detect_ = jarvis::slip_detect::FactorSlipDetect(config);
    // }

    LOG(INFO) << "Capture start..";

    data_capture_->Rigister("slip_detect", [this](const OdomData& encode) {
      if (!slip_detect_) return;
      if (!last_odom_data_.has_value()) {
        last_odom_data_ = encode;
      }
      //
      const Eigen::Vector3d delta_encode =
          encode.translation - last_odom_data_.value().translation;
      last_odom_data_ = encode;
      //
      // LOG(INFO)<<encode.translation.transpose()<< " "<<
      // delta_encode.transpose();
      // if (abs(delta_encode.x()) < 0.001 && abs(delta_encode.y() < 0.001)) {
      //   if (slip_detect_) {
      //     slip_detect_->ClearData();
      //   }
      // }
      //

      if (slip_detect_) {
        slip_detect_->AddOdometry(sensor::OdometryData{
            jarvis::common::FromUniversal(encode.time * 10),
            transform::Rigid3d(encode.translation, encode.rotaion)});
      }
    });

    data_capture_->Rigister([this](const SystmeInfo& state) {
      
      if (system_state_ != state.state) {
        LOG(WARNING) << "System status change. frome " << int(system_state_)
                     << " to " << int(state.state);
        system_state_ = state.state;
        if (event_dark_ == 1) {
          LOG(WARNING) << "Not revive envet dark off.";
          return;
        }
        if (system_state_ == MowStatus::MS_CHARGING ||
            system_state_ == MowStatus::MS_SLEEP) {
          LOG(WARNING) << "Rest jarvis brige...";
          jarvis_brige_.reset(nullptr);
          LOG(WARNING) << "Rest  sliep detect...";
          slip_detect_.reset(nullptr);
          LOG(WARNING) << "Rest  sliep done...";
          // global_odom_= transform::Rigid3d::Identity();
          kVioState = 0;
        } else {
          if (!jarvis_brige_) {
            CreateJarvisBrige();
          }
        }


      }
    });
    //
    data_capture_->RigisterEvnt([this](const SystmeInfo& state) {
      event_dark_ = state.env_dark_data;
      LOG(INFO)<<int(event_dark_) ;
      if (event_dark_ ==1) {
        LOG(WARNING) << "event dark recive,delte jarvis."<<int(event_dark_);
        jarvis_brige_.reset(nullptr);
        slip_detect_.reset(nullptr);
        kVioState = 0;
      }
    });
    //
    //
    data_capture_->Rigister(
        "data_record", [this](const ImuData& imu) { data_record_->AddImu(imu); });

    data_capture_->Rigister("data_record", [this](const OdomData& odom) {
      data_record_->AddOdom(odom);
    });
    data_capture_->Rigister("data_record", [this](const Frame& frame) {
      data_record_->AddFrame(frame);
    });
    data_capture_->Rigister(
        "data_record", [this](const RtkData& rtk) { data_record_->AddRtk(rtk); });

    data_capture_->Start();
  }

  //
  //
  DataCapture* GetDataCapture() { return data_capture_.get(); }
  //
  void AddStateToImuExtrapolator(const jarvis::TrackingData& data) {
    // std::lock_guard<std::mutex> lock(mutex_);
    // if (!imu_extrapolator_ || kWriteMpcPoseType == 0) {
    //   return;
    // }
    // if (data.status != 2) {
    //   imu_extrapolator_->Rest();
    // }
    // imu_extrapolator_->AddState(data.data->time, data.data->imu_state);

    // if (imu_extrapolator_->GetImuNum() > 200) {
    //   LOG(WARNING) << "imu_extrapolator_ too much imu data is cached.";
    //   jarvis_brige_.reset(nullptr);
    //   CreateJarvisBrige();
    // }
  }
  //
  void CreateJarvisBrige() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // imu_extrapolator_ =
      //     std::make_unique<jarvis::estimator::ImuExtrapolator>();
    }

    slip_detect_ = jarvis::slip_detect::FactorSlipDetect(config_path_);
    jarvis_brige_ = std::make_unique<JarvisBrige>(
        config_path_, data_capture_.get(),
        [&](const jarvis::TrackingData& data) {
          data_record_->AddAtTimeFram(common::ToUniversal(
              data.data->time -
              common::FromSeconds(jarvis::GetTimeShiftCamImu())));
          bool slip_flag = false;
          if (slip_detect_) {
            if (data.status != 2) {
              slip_detect_->ClearData();
              // std::lock_guard<std::mutex> lock(mutex_);
              // imu_extrapolator_->Rest();
            } else {
              slip_detect_->AddPose(slip_detect::TimePose{
                  data.data->time, data.data->imu_state.Pose()});
              slip_flag = slip_detect_->Detect(data.data->time);
            }
            if (kWriteMpcPoseType == 0) {
              transform::Rigid3d slipe_alignment_pose =
                  slip_detect_->ToPoseInOdom(data.data->imu_state.Pose());
              mpc_.Write(slipe_alignment_pose, data, 0, slip_flag);
            }
          }
          //

          kSlipeState = slip_flag;
          kVioState = data.status;
          // if (kWriteMpcPoseType) {
          //   std::lock_guard<std::mutex> lock(mutex_);
          //   imu_extrapolator_->AddState(data.data->time, data.data->imu_state);
          // }
          call_back_(jarvis_pic_call_back_data{slip_flag, data});
        });
  }
  jarvis::slip_detect::SlipDetect* GetSlipDect() { return slip_detect_.get(); }

 private:
  const std::string config_path_;
  std::optional<OdomData> last_odom_data_;

  jarvis_pic::MpcComponent mpc_;
  std::unique_ptr<DataCapture> data_capture_;
  std::unique_ptr<JarvisBrige> jarvis_brige_;
  std::unique_ptr<jarvis::slip_detect::SlipDetect> slip_detect_;
  std::function<void(const jarvis_pic_call_back_data&)> call_back_;
  // std::unique_ptr<jarvis::estimator::ImuExtrapolator> imu_extrapolator_=nullptr;  //=
  uint8_t system_state_ = 0xff;
  std::mutex mutex_;
  uint8_t event_dark_=0;
};
}  // namespace jarvis_pic
std::string kDataDir = "/mnt/UDISK/jarvis/";

//
int main(int argc, char* argv[]) {
  //
  
  auto CreateDir = [](const std::string& dir) {
    if (access(dir.c_str(), F_OK) == -1) {
      mkdir(dir.c_str(), S_IRWXO | S_IRWXG | S_IRWXU);
    }
    if (access(dir.c_str(), F_OK) == -1) {
      mkdir(dir.c_str(), S_IRWXO | S_IRWXG | S_IRWXU);
    }
    return true;
  };
  google::InitGoogleLogging("jarvis");
  //
  
  LocalGlogSink glog_sink;
  google::AddLogSink(&glog_sink);
  CreateDir(kDataDir);

  //
  //
  const std::string config_file("/oem/mowpack/vslam_param/vslam.yaml");
  //
  //
  //
  ParseOption(config_file);
  if (kGLOG_v >= 0) {
    FLAGS_log_dir = kDataDir + "/log/";
    CreateDir(FLAGS_log_dir);
    FLAGS_v = kGLOG_v;
  } else {
    google::SetLogDestination(google::LogSeverity::GLOG_INFO, "");
    google::SetLogDestination(google::LogSeverity::GLOG_WARNING, "");
    google::SetLogDestination(google::LogSeverity::GLOG_ERROR, "");
    google::SetLogDestination(google::LogSeverity::FATAL, "");
  }

  //
  data_record_ =
      std::make_unique<jarvis_pic::DataRecord>(kDataDir, kRecordFlag);
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
              LOG_EVERY_N(INFO,100) << tracking_data_temp.data->imu_state
                        << " state: " << tracking_data_temp.status;
              con_variable.notify_all();
            }
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
    data_record_->AddVioData(tracking_data.data->time,
                               tracking_data.data->imu_state.Pose(), flag);
    LOG_EVERY_N(INFO,5) << "vio pose:" << tracking_data.data->imu_state;
    // jarvis_slam->AddStateToImuExtrapolator(tracking_data);
    // mpc.Write(
    //     tracking_data,
    //     jarvis_slam->GetDataCapture()->GetOrigImuTime(static_cast<uint64_t>(
    //         jarvis::common::ToUniversal(tracking_data.data->time) / 10)));
    
#ifdef __ZMQ_ENABLAE__
    if (tracking_data.status == 2) {
    //获取sliep的时候小心GetSlipDect可能在另外的线程被释放
    //  auto pose =   jarvis_slam->GetSlipDect()->ToPoseInOdom(
    //           tracking_data.data->imu_state.Pose());
      // tracking_data.data->imu_state.p =  pose.translation();
      // tracking_data.data->imu_state.q =  pose.rotation();
      zmq.PubLocalData(tracking_data, flag);
    }
#endif
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }


  LOG(INFO) << "Release jarvis...";
  kill_thread_ = false;
  jarvis_slam = nullptr;
  sleep(1);
  con_variable.notify_all();

  return 0;
}