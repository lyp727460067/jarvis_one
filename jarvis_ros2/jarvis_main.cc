
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

#include "fstream"
#include "jarvis/sensor/data_process.h"
#include "jarvis/sensor/stereo_sync.h"
#include "jarvis/trajectory_builder.h"
#include "unistd.h"
//
#include <glog/logging.h>

constexpr char kImagTopic0[] = "/usb_cam_1/image_raw/compressed";
constexpr char kImagTopic1[] = "/usb_cam_2/image_raw/compressed";
constexpr char kImuTopic[] = "/imu";

//

namespace {

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

  static std::map<uint64_t, ImuData> Parse(const std::string& dir_file);
};

//
std::optional<std::pair<uint64_t, uint64_t>> init_imu_time;
std::istringstream& operator>>(std::istringstream& ifs, ImuData& imu_data) {
  uint64_t time;
  ifs >> time;
  char unuse_char;
  int unuse_data;
  imu_data.time   = time;
  ifs >> unuse_char >> unuse_data>> unuse_char;
  ifs >> imu_data.angular_velocity.x() >> unuse_char >>
      imu_data.angular_velocity.y() >> unuse_char >>
      imu_data.angular_velocity.z() >> unuse_char >>
      imu_data.linear_acceleration.x() >> unuse_char >>
      imu_data.linear_acceleration.y() >> unuse_char >>
      imu_data.linear_acceleration.z();
  
  return ifs;
}
//

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
void WriteImuData(uint64_t time, std::map<uint64_t, ImuData>& imu_datas) {
  auto it = imu_datas.upper_bound(time);
  for (auto itor = imu_datas.begin(); itor != it; ++itor) {
    order_queue_->AddData(
        kImuTopic,
        std::make_unique<sensor::DispathcData<sensor::ImuData>>(sensor::ImuData{
            itor->first * 1e-9,
            itor->second.linear_acceleration,
            itor->second.angular_velocity,
        }));
  }
  imu_datas.erase(imu_datas.begin(), it);
}

//
void Run(std::map<uint64_t, ImuData>& imu_datas,
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
        kImagTopic0, std::make_unique<sensor::DispathcData<sensor::ImageData>>(
                         sensor::ImageData{image.first * 1e-9, {temp, temp}}));
  }
  if (!imu_datas.empty()) {
    WriteImuData(UINT64_MAX, imu_datas);
  }
  CHECK(imu_datas.empty());
}
}  // namespace

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  //
  //
  //

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("jarvis_ros2");

  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  const std::string data_dir(argv[2]);
  CHECK_EQ(argc, 3);
  LOG(INFO) << "input dir : " << data_dir;
  LOG(INFO) << "config file : " << argv[1];
  //
  std::unique_ptr<jarvis_ros::RosCompont> ros_compont =
      std::make_unique<jarvis_ros::RosCompont>(node.get());
  TrackingData tracking_data_temp;
  std::mutex mutex;
  std::condition_variable cond;

  const std::string image_file = data_dir + "/calib_save/cam0/";
  const std::string imu_file = data_dir + "/calib_save/" + "imu.txt";
  //
  builder_ = std::make_unique<TrajectorBuilder>(
      std::string(argv[1]), [&](const TrackingData& data) {
        std::unique_lock<std::mutex> lock(mutex);
        LOG(INFO)<<data.data->pose;
        LOG(INFO)<<jarvis::common::ToUniversal(data.data->time) ;
        tracking_data_temp = data;
        cond.notify_one();
      });
  //
  order_queue_ = std::make_unique<sensor::OrderedMultiQueue>();
  order_queue_->AddQueue(kImuTopic, [](const sensor::ImuData& imu_data) {
    builder_->AddImuData(imu_data);
  });
  //
  order_queue_->AddQueue(kImagTopic0, [](const sensor::ImageData& imag_data) {
    builder_->AddImageData(imag_data);
  });
  LOG(INFO) << "Parse image dir: " << image_file;
  LOG(INFO) << "Parse imu dir: " << imu_file;
  auto image_datas = ImageData::Parse(image_file);
  auto imu_datas = ImuData::Parse(imu_file);

  //
  //
  LOG(INFO) << "Start run...";
  std::thread pub_map_points([&]() {
    while (rclcpp::ok()) {
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
      ros_compont->OnLocalTrackingResultCallback(
          tracking_data, nullptr, transform::Rigid3d::Identity());
      ros_compont->PosePub(tracking_data.data->pose,
                           transform::Rigid3d::Identity());
    }
  });
  order_queue_->Start();
  Run(imu_datas, image_datas);
  pub_map_points.join();
  LOG(INFO) << "Done";
  return 0;
}