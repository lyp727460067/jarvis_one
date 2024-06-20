
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros_component.h"
#include <dirent.h>
#include <sys/types.h>

#include <map>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include "jarvis/common/fixed_ratio_sampler.h"
#include "fstream"
#include "jarvis/sensor/data_process.h"
#include "jarvis/sensor/stereo_sync.h"
#include "jarvis/trajectory_builder.h"
#include "unistd.h"
#include "slip_detection/slip_detect.h"
#include "slip_detection/simple_vo.h"
//
#include <glog/logging.h>
#include "jarvis/estimator/imu_extrapolator.h"
// #define CHECK_DATA
constexpr char kImagTopic0[] = "/usb_cam_1/image_raw/compressed";
constexpr char kImagTopic1[] = "/usb_cam_2/image_raw/compressed";
constexpr char kImuTopic[] = "/imu";
constexpr char kOdomTopic[] = "/odom";

//

namespace {
double imu_cam_time_offset = 0;
double image_sample = 1;
uint8_t kRecordFlag = 1;
uint8_t kDataCaputureType = 0;
std::ofstream kOImuFile;
std::ofstream kOPoseFile;
std::string image_dir;
void ParseOption(const std::string& config) {
  cv::FileStorage fsSettings(config, cv::FileStorage::READ);
  fsSettings["imu_cam_time_offset"] >> imu_cam_time_offset;
  fsSettings["image_sample"] >> image_sample;
  // fsSettings["record"] >> kRecordFlag;
  // fsSettings["data_capture"] >> kDataCaputureType;
}
using namespace jarvis;

std::unique_ptr<sensor::OrderedMultiQueue> order_queue_ = nullptr;
std::unique_ptr<TrajectorBuilder> builder_ = nullptr;

std::set<std::string> ReadFileFromDir(const std::string& path) {
  std::set<std::string> fp_set;
  DIR* dir = opendir(path.c_str());
  CHECK(dir);
  struct dirent* entry = nullptr;
  while ((entry = readdir(dir)) != nullptr) {
    if (std::string(entry->d_name) == ".") continue;
    if (std::string(entry->d_name) == "..") continue;
    std::string pic_name = path + std::string(entry->d_name);
    fp_set.emplace(pic_name);
  }
  closedir(dir);
  // //
  LOG(INFO) << "dir path has file size :" << fp_set.size();
  return fp_set;
  //
}

struct ImuData {
  uint64_t time;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;
  std::unique_ptr<sensor::Data> ToPatchData() {
    return std::make_unique<sensor::DispathcData<sensor::ImuData>>(
        sensor::ImuData{
            common::FromUniversal(time /100),
            linear_acceleration,
            angular_velocity,
        });
  }
  static std::string Name() { return kImuTopic; }
  static std::map<uint64_t, ImuData> Parse(const std::string& dir_file);
};
struct OdomData {
  uint64_t time;
  Eigen::Vector3d translation;
  Eigen::Quaterniond rotation;
  static std::string Name() { return kOdomTopic; }
  std::unique_ptr<sensor::Data> ToPatchData() {
    return std::make_unique<sensor::DispathcData<sensor::OdometryData>>(
        sensor::OdometryData{
          common::FromUniversal(time /100), transform::Rigid3d(translation,
                                                 rotation)
        });
  }
  static std::map<uint64_t, OdomData> Parse(const std::string& dir_file);
};

//
std::optional<std::pair<uint64_t, uint64_t>> init_imu_time;
std::istringstream& operator>>(std::istringstream& ifs, ImuData& imu_data) {
  uint64_t time;
  ifs >> time;
#ifdef CHECK_DATA
  static uint64_t last_imu_time = time;
  LOG(INFO) << (time - last_imu_time);
  last_imu_time = time;
#endif
  imu_data.time   = time;
  ifs >> imu_data.angular_velocity.x() >> imu_data.angular_velocity.y() >>
      imu_data.angular_velocity.z() >> imu_data.linear_acceleration.x() >>
      imu_data.linear_acceleration.y() >> imu_data.linear_acceleration.z();
  return ifs;
}
//
//
std::istringstream& operator>>(std::istringstream& ifs, OdomData& odom_data) {
  uint64_t time;
  ifs >> time;
  odom_data.time = time;
  ifs >> odom_data.translation.x() >> odom_data.translation.y() >>
      odom_data.translation.z() >> odom_data.rotation.w() >>
      odom_data.rotation.x() >>odom_data.rotation.y() >>
      odom_data.rotation.z();
  return ifs;
}

template <typename TypeName>
std::vector<TypeName> ReadFile(const std::string& txt) {
  std::ifstream file;
  file.open(txt);
  CHECK(file.good())<<txt;
  std::string line;
  std::vector<TypeName> result;
  std::getline(file, line);
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    TypeName data;
    iss >> data;
    // CHECK(file.good());
    result.push_back(data);
  }
  file.close();
  LOG(INFO) << "done";
  return result;
}
//
std::map<uint64_t, ImuData> ImuData::Parse(const std::string& file) {
  const auto imu_data = ReadFile<ImuData>(file);
  CHECK(!imu_data.empty());
  std::map<uint64_t, ImuData> result;
  for (const auto& imu : imu_data) {
    LOG_IF(ERROR, !result.emplace(imu.time, imu).second)
        << "Imu time duplicate..";

  }
  return result;
}

//
uint64_t GetTimeFromName(const std::string& name) {
  CHECK(!name.empty());
  auto it = name.find_last_of('/');
  std::string outdir = name.substr(0, it + 1);
  const std::string file_name =
      name.substr(it + 1, name.size() - outdir.size());
  auto it1 = file_name.find_last_of('.');
//   LOG(INFO)<<std::stol(file_name.substr(0, it1));
  return std::stol(file_name.substr(0, it1));
}
//

//
struct ImageData {
  uint64_t time;
  // cv::Mat images;
  std::string image_name;
  static std::map<uint64_t, ImageData> Parse(const std::string& dir_file) {
    const auto image_files_name = ReadFileFromDir(dir_file);
    CHECK(!image_files_name.empty()) << "Need Image file in dir..";
    std::map<uint64_t, ImageData> result;
    //

    for (const auto& file : image_files_name) {

#ifdef CHECK_DATA
      static uint64_t last_imu_time = GetTimeFromName(file);
      LOG(INFO) << (GetTimeFromName(file) - last_imu_time);      
      last_imu_time = GetTimeFromName(file);

#endif
    //   LOG(INFO) << "Read Image: " << file;
      LOG_IF(ERROR, !result
                         .emplace(GetTimeFromName(file),
                                  ImageData{GetTimeFromName(file), file})
                         .second)
          << "Image time duplicate..";
    }
    return result;
  }
};
//
//
template <typename Sensor>
void WriteImuData(uint64_t time, std::map<uint64_t, Sensor>& imu_datas) {
  auto it = imu_datas.upper_bound(time);
  for (auto itor = imu_datas.begin(); itor != it; ++itor) {
    order_queue_->AddData(Sensor::Name(), it->second.ToPatchData());
  }
  imu_datas.erase(imu_datas.begin(), it);
}

//
template <typename Sensor>
void Run(std::map<uint64_t, Sensor>& imu_datas,
         std::map<uint64_t, ImageData> images_datas) {
  LOG(INFO) << "Run start..";
  LOG(INFO) << "Write init befor image time imu data lenth: "
            << std::distance(
                   imu_datas.begin(),
                   imu_datas.upper_bound(images_datas.begin()->first));
  //

  for (const auto& image : images_datas) {
    //
    LOG(INFO) << "image time : " << image.second.time
              << " start imu t: " << imu_datas.begin()->first
              << ", end imu t: " << imu_datas.upper_bound(image.first)->first
              << " size:"
              << std::distance(imu_datas.begin(),
                               imu_datas.upper_bound(image.first));    
    WriteImuData( image.second.time, imu_datas);
    auto temp = std::make_shared<cv::Mat>(
        cv::imread(image.second.image_name, cv::IMREAD_GRAYSCALE).clone());
    order_queue_->AddData(
        kImagTopic0,
        std::make_unique<sensor::DispathcData<sensor::ImageData>>(
            sensor::ImageData{common::FromUniversal(image.first /100),
                              {temp, temp}}));
  }
  if (!imu_datas.empty()) {
    WriteImuData(UINT64_MAX, imu_datas);
  }
  CHECK(imu_datas.empty());
}

}  // namespace
bool kill_thread =false;
int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  //
  //
  //

  if (kRecordFlag) {
    kOPoseFile.open("/tmp/vio_pose.txt", std::ios::out);
  }
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("jarvis_ros2");

  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  const std::string data_dir(argv[2]);
  CHECK_EQ(argc, 3);
  LOG(INFO) << "input dir : " << data_dir;
  LOG(INFO) << "config file : " << argv[1];
  //
  ParseOption(argv[1]);
  std::unique_ptr<jarvis_ros::RosCompont> ros_compont =
      std::make_unique<jarvis_ros::RosCompont>(node.get());

  //
  auto slip_detect = std::make_unique<slip_detect::SlipDetect>(slip_detect::SlipDetectOption{});
  // /
  TrackingData tracking_data_temp;
  std::mutex mutex;
  std::condition_variable cond;

  const std::string image_file = data_dir + "image/";
  const std::string odom_file = data_dir + "odom.txt";
  //
  builder_ = std::make_unique<slip_detect::SimpleVo>(
     slip_detect::SimpleVoOption{std::string(argv[1])} , [&](const TrackingData& data) {
        std::lock_guard<std::mutex> lock(mutex);
        //
        tracking_data_temp = data;
        cond.notify_one();
      });
  //
  //

  //
  order_queue_ = std::make_unique<sensor::OrderedMultiQueue>();
  order_queue_->AddQueue(kOdomTopic, [&](const sensor::OdometryData& odom_data) {
    slip_detect->AddOdometry(odom_data);
  });
  //
  order_queue_->AddQueue(kImagTopic0, [&](const sensor::ImageData& imag_data) {
    builder_->AddImageData(imag_data);
  });
  
  LOG(INFO) << "Parse image dir: " << image_file;
  LOG(INFO) << "Parse imu dir: " << odom_file;
  auto image_datas = ImageData::Parse(image_file);
  auto odom_datas = ImuData::Parse(odom_file);
  //
  //
  LOG(INFO) << "Start run...";
  std::thread pub_map_points([&]() {
    while (!kill_thread) {
      TrackingData tracking_data;
      std::this_thread::sleep_for(std::chrono::milliseconds(30));

      //
      static uint8_t count = 0;
      // if (kReciveTempGoal)
      if (++count > 30) {
        count = 0;
        std::map<int, std::map<KeyFrameId, transform::TimestampedTransform>>
            poses;
      }
      {
        std::unique_lock<std::mutex> lock(mutex);
        cond.wait(lock);
        tracking_data = tracking_data_temp;
      }
      LOG(INFO)<<tracking_data.status;
      if (tracking_data.status == 2) {
        LOG(INFO) << tracking_data.data->imu_state.data->pose;
      //
      ros_compont->OnLocalTrackingResultCallback(
          tracking_data, nullptr, transform::Rigid3d::Identity());
      ros_compont->PosePub(tracking_data.data->imu_state.data->pose,
                           transform::Rigid3d::Identity());
      rclcpp::spin_some(node);
    }
    }
  });
  order_queue_->Start();
  Run(odom_datas, image_datas);
  order_queue_->Stop();
  builder_= nullptr;
  kill_thread = true;
  sleep(1);
  cond.notify_all();
  pub_map_points.join();
  if (kRecordFlag) {
    kOPoseFile.close();
  }

  LOG(INFO) << "Done";
  return 0;
}