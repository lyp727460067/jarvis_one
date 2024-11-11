#include "calibrate_data_record.h"
// #include "file_stream.h"
#include "key_board.h"
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <fstream>
#include <string>

#include "glog/logging.h"
#include "zmq_component.h"
//
//
namespace jarvis_pic {
void CalibrateDataRecord::CreateDataDir() {
  time_t now;
  struct tm* local;
  time(&now);
  local = localtime(&now);
  std::string name = std::to_string(local->tm_year + 1900) + "_" +
                     std::to_string(local->tm_mon + 1) + "_" +
                     std::to_string(local->tm_mday) + "_" +
                     std::to_string(local->tm_hour) + "_" +
                     std::to_string(local->tm_min);
  //
  std::string data_d = data_path_ + "data/";
  if (access(data_d.c_str(), F_OK) == -1) {
    mkdir(data_d.c_str(), S_IRWXO | S_IRWXG | S_IRWXU);
  }
  const std::string data_dir = data_d + name + "/";
  if (access(data_dir.c_str(), F_OK) == -1) {
    mkdir(data_dir.c_str(), S_IRWXO | S_IRWXG | S_IRWXU);
  }

  std::string image_dir = data_dir + "image/";
  if (access(image_dir.c_str(), F_OK) == -1) {
    mkdir(image_dir.c_str(), S_IRWXO | S_IRWXG | S_IRWXU);
  }
  image_data_dir_ = image_dir;
  for (int i = 0; i < 4; i++) {
    std::string image_dir = image_data_dir_ + "cam" + std::to_string(i) + "/";
    if (access(image_dir.c_str(), F_OK) == -1) {
      mkdir(image_dir.c_str(), S_IRWXO | S_IRWXG | S_IRWXU);
      LOG(INFO) << "create dir:" << image_dir;
    }
  }
  LOG(INFO) << "Record data dir : " << data_dir;
  LOG(INFO) << "Record image_dir dir : " << image_dir;
  const std::string imu_file = data_dir + "imu.txt";
  const std::string pose_file = data_dir + "vio_odom.txt";
  imu_file_.open(imu_file, std::ios::out);
  if (imu_file_.good()) {
    LOG(INFO) << " open imu file : " << imu_file << " done.";
  }
}

CalibrateDataRecord::CalibrateDataRecord(double sample_ration,
                                         const std::string& data_path)
    : data_path_(data_path) {
  CreateDataDir();
  image_sample_ =
      std::make_unique<jarvis::common::FixedRatioSampler>(sample_ration);
}
//
//
void CalibrateDataRecord::AddFrame(const Frame& frame) {
  if (image_sample_->Pulse())
    for (size_t i = 0; i < frame.images.size(); i++) {
      cv::imwrite(image_data_dir_ + "cam" + std::to_string(i) + "/" +
                      std::to_string(uint64_t(frame.time * 1e3)) + ".png",
                  frame.images[i]);
    }
}
//
void CalibrateDataRecord::AddImu(const ImuData& imu) {
  // std::lock_guard<std::mutex> lock(mutex_);
  // tasks_.push([=]() {
  std::stringstream info;
  info << "imu " << std::to_string(uint64_t(imu.time * 1e3)) << " "
       << imu.angular_velocity.x() << " " << imu.angular_velocity.y() << " "
       << imu.angular_velocity.z() << " " << imu.linear_acceleration.x() << " "
       << imu.linear_acceleration.y() << " " << imu.linear_acceleration.z();
  imu_file_ << info.str() << std::endl;
  // });
}
}  // namespace jarvis_pic
//
//
using namespace jarvis_pic;
std::string kDataDir = "/mnt/UDISK/jarvis/";
int main(int argc, char* argv[]) {
  auto CreateDir = [](const std::string& dir) {
    if (access(dir.c_str(), F_OK) == -1) {
      mkdir(dir.c_str(), S_IRWXO | S_IRWXG | S_IRWXU);
    }
    if (access(dir.c_str(), F_OK) == -1) {
      mkdir(dir.c_str(), S_IRWXO | S_IRWXG | S_IRWXU);
    }
    return true;
  };
  io::KeyBoard key_bord;
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  int n = std::stoi(argv[1]);
  int type = std::stoi(argv[2]);
  LOG(INFO)<<type;
  CreateDir(kDataDir);
  double sample = n / 10.;
  if(type ==0) sample =1.0;
  CalibrateDataRecord cali_data_record(sample, kDataDir);
  jarvis::common::FixedRatioSampler  image_sample(0.2); 
  std::unique_ptr<DataCapture> data_capture = CreateDataCaputure({0});
  jarvis_pic::ZmqComponent zmq;

  data_capture->Rigister("data_record", [&](const ImuData& imu) {
    LOG_EVERY_N(INFO, 100) << "Record imu data " << imu.time;
    cali_data_record.AddImu(imu);
  });
  bool start_record_ = false;
  data_capture->Rigister("data_record", [&](const Frame& frame) {
    jarvis::TrackingData tracking_data_temp;
    tracking_data_temp.data = std::make_shared<jarvis::TrackingData::Data>();
    if (frame.images[0].empty() || frame.images[1].empty() ||
        frame.images[2].empty() || frame.images[3].empty())
      return;
    if (type || start_record_) {
      LOG_EVERY_N(INFO, 1) << "Record frame data " << frame.time;
      cali_data_record.AddFrame(frame);
    }
    if (image_sample.Pulse()) {
      for (int i = 0; i < 4; i++) {
        tracking_data_temp.data->features_datas[i].features.data =
            std::make_shared<
                jarvis::estimator::ImageFeatureTrackerData::Data>();
        tracking_data_temp.data->features_datas[i]
            .features.data->images.push_back(frame.images[i]);
      }
      zmq.PubLocalData(tracking_data_temp, 0);
    }

    start_record_ = false;

  });
  data_capture->Start();

  while (true) {
    if (key_bord.GetKey()=='1') {
      start_record_ = true;
    }
    usleep(10000);
  }
  return 0;
}
