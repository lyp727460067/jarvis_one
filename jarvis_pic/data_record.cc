#include "data_record.h"

#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <fstream>
#include <string>

#include "glog/logging.h"
//
//
namespace jarvis_pic {
void DataRecord::CreateDataDir() {
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
  LOG(INFO) << "Record data dir : " << data_dir;
  LOG(INFO) << "Record image_dir dir : " << image_dir;
  const std::string imu_file = data_dir + "imu.txt";
  const std::string pose_file = data_dir + "vio_odom.txt";
  imu_file_.open(imu_file, std::ios::out);
  if (imu_file_.good()) {
    LOG(INFO) << " open imu file : " << imu_file << " done.";
  }
  pose_file_.open(pose_file, std::ios::out);
  if (pose_file_.good()) {
    LOG(INFO) << " open pose file : " << pose_file << " done.";
  }
}

DataRecord::DataRecord(const std::string& data_path, bool record)
    : record_(record),data_path_(data_path) {
  LOG(INFO)<< record_;
  if (record_) {
    CreateDataDir();
    thread_ = std::thread([this]() {
      while (!kill_thread_) {
        Run();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        // usleep(1000);
      }
    });
  }
}
//
void DataRecord::AddFrame(const Frame& frame) {
  if (!record_) return;
  static int i = 0;
  if ((++i) % 2) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  tasks_.push([=]() {
    cv::imwrite(
        image_data_dir_ + std::to_string(uint64_t(frame.time * 1e3)) + "_l_.png",
        frame.images[0]);
    cv::imwrite(
        image_data_dir_ + std::to_string(uint64_t(frame.time * 1e3)) + "_r_.png",
        frame.images[1]);
  });
}
void DataRecord::AddImu(const ImuData& imu) {
  if (!record_) return;
  // std::lock_guard<std::mutex> lock(mutex_);
  // tasks_.push([=]() {
    std::stringstream info;
    info << "imu " << std::to_string(uint64_t(imu.time * 1e3)) << " "
         << imu.angular_velocity.x() << " " << imu.angular_velocity.y() << " "
         << imu.angular_velocity.z() << " " << imu.linear_acceleration.x()
         << " " << imu.linear_acceleration.y() << " "
         << imu.linear_acceleration.z();
    imu_file_ << info.str() << std::endl;
  // });
}
//

void DataRecord::AddOdom(const OdomData& odom) {
  if (!record_) return;
  // std::lock_guard<std::mutex> lock(mutex_);
  // tasks_.push([=]() {
    std::stringstream info;
    info << "odom " << std::to_string(uint64_t(odom.time * 1e3)) << " "
         << odom.translation.x() << " " << odom.translation.y() << " "
         << odom.translation.z() << " " << odom.rotaion.w() << " "
         << odom.rotaion.x() << " " << odom.rotaion.x() << " "
         << odom.rotaion.z();
    imu_file_ << info.str() << std::endl;
  // });
}

void DataRecord::AddVioData(jarvis::common::Time& time,
                            const jarvis::transform::Rigid3d& pose,
                            bool flag) {
  // std::lock_guard<std::mutex> lock(mutex_);
  // tasks_.push([=]() {
    std::stringstream info;
    info << std::to_string(uint64_t(jarvis::common::ToUniversal(time) * 1e2))
         << " " <<pose.translation().x() << " " << pose.translation().y()
         << " " << pose.translation().z() << " " << pose.rotation().w() << " "
         << pose.rotation().x() << " " << pose.rotation().y() << " "
         << pose.rotation().z() << " " << flag << std::endl;
    pose_file_ << info.str();
  // });
}
void DataRecord::Run() {
  size_t task_size = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    task_size = tasks_.size();
  }
  std::function<void(void)> f;
  while (task_size != 0) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      f = std::move(tasks_.front());
      tasks_.pop();
      task_size = tasks_.size();
    }
    LOG_EVERY_N(INFO,100) << "Record task size " << task_size;
    f();
  }
};
DataRecord::~DataRecord() {
  kill_thread_ = true;
  if (thread_.joinable()) {
    thread_.join();
  }
  pose_file_.close();
  imu_file_.close();
}
}  // namespace jarvis_pic
