
#include <dirent.h>
#include <sys/types.h>

#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include "jarvis/imu_tracker.h"
#include "fstream"
#include "jarvis/common/fixed_ratio_sampler.h"
#include "jarvis/sensor/data_process.h"
#include "jarvis/sensor/stereo_sync.h"
#include "jarvis/trajectory_builder.h"
#include "jarvis/object/object_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "ros_component.h"
#include "slip_detection/simple_vo.h"
#include "slip_detection/slip_detect.h"
#include "std_msgs/msg/string.hpp"

#include "unistd.h"
//
#include <sensor_msgs/msg/imu.hpp>
#include "jarvis/estimator/featureTracker/pyramid_image.h"
#include <glog/logging.h>

// #include "jarvis/estimator/imu_extrapolator.h"
// #define CHECK_DATA
constexpr char kImagTopic0[] = "/usb_cam_1/image_raw/compressed";
constexpr char kImagTopic1[] = "/usb_cam_2/image_raw/compressed";
constexpr char kImuTopic[] = "/imu";
constexpr char kOdomTopic[] = "/odom";
std::unique_ptr<jarvis_ros::RosCompont> ros_compont;
//
std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub2_;
namespace {
double imu_cam_time_offset = 0;
double image_sample = 1;
int KStartImageTime = 0;
uint8_t kRecordFlag = 1;
uint8_t kDataCaputureType = 0;
std::ofstream kOImuFile;
std::ofstream kOPoseFile;
std::ofstream kSlipFile;
std::string image_dir;
int kuse_gpu = 0;
// std::unique_ptr<jarvis::estimator::ImuExtrapolator> KImuExtrapolator;
void ParseOption(const std::string& config) {
  cv::FileStorage fsSettings(config, cv::FileStorage::READ);
  fsSettings["imu_cam_time_offset"] >> imu_cam_time_offset;
  LOG(INFO) << imu_cam_time_offset;
  fsSettings["image_sample"] >> image_sample;
  fsSettings["start_image_time"] >> KStartImageTime;
    fsSettings["use_gpu"] >> kuse_gpu;
    LOG(INFO)<< kuse_gpu;
  // fsSettings["record"] >> kRecordFlag;
  // fsSettings["data_capture"] >> kDataCaputureType;
}
using namespace jarvis;

std::unique_ptr<sensor::OrderedMultiQueue> order_queue_ = nullptr;
std::unique_ptr<TrajectorBuilder> builder_ = nullptr;
std::unique_ptr<ImuTracker> imu_tracker_= nullptr;

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
    // KImuExtrapolator->AddImu(sensor::ImuData{
    //     common::FromUniversal(time / 100),
    //     linear_acceleration,
    //     angular_velocity,
    // });
    // LOG(INFO)<<common::FromUniversal(time / 100);
    // auto state = KImuExtrapolator->Exrapolate(
        // common::FromUniversal(time / 100) + common::FromSeconds(0.001));
    // LOG(INFO) << state.pose;
    // ros_compont->PushMark({{"imu_pose", state.Pose}}, false);
    // ros_compont->PosePub(state.pose, transform::Rigid3d::Identity());
    return std::make_unique<sensor::DispathcData<sensor::ImuData>>(
        sensor::ImuData{
            common::FromUniversal(time / 100) - common::FromSeconds(0.1),
            linear_acceleration,
            angular_velocity,
        });
  }
  static std::string Name() { return kImuTopic; }
  static std::map<uint64_t, ImuData> Parse(const std::string& dir_file);
};
jarvis::transform::Rigid3d kOdom = jarvis::transform::Rigid3d::Identity();
struct OdomData {
  uint64_t time;
  Eigen::Vector3d translation;
  Eigen::Quaterniond rotation;
  static std::string Name() { return kOdomTopic; }
  std::unique_ptr<sensor::Data> ToPatchData() {
    return std::make_unique<sensor::DispathcData<sensor::OdometryData>>(
        sensor::OdometryData{common::FromUniversal(time / 100)- common::FromSeconds(0.1),
                             transform::Rigid3d(translation, rotation)});
  }
  static std::map<uint64_t, OdomData> Parse(const std::string& dir_file);
};
constexpr double kGryUnit = 0.001;
constexpr double kAccUnit = (1.0 / 2048 * 9.81);  // 加速度单位
//
std::optional<std::pair<uint64_t, uint64_t>> init_imu_time;
std::istringstream& operator>>(std::istringstream& ifs, ImuData& imu_data) {
  std::string type;
  ifs >> type;
  if (type != "imu") throw "Not imu";
  uint64_t time;
  ifs >> time;

  static uint64_t last_time = time;
  if ((time - last_time) > 10000000) {
    // LOG(INFO) << "   " << time - last_time;
  }
  last_time = time;
#ifdef CHECK_DATA
  static uint64_t last_imu_time = time;
  LOG(INFO) << (time - last_imu_time);
  last_imu_time = time;
#endif

  uint64_t un_time;
  // ifs >> un_time;
  imu_data.time = time;
  //
  int un_count = 0;
  // ifs>>un_count;
  ifs >> imu_data.angular_velocity.x() >> imu_data.angular_velocity.y() >>
      imu_data.angular_velocity.z() >> imu_data.linear_acceleration.x() >>
      imu_data.linear_acceleration.y() >> imu_data.linear_acceleration.z();
  return ifs;
}
//
//
std::istringstream& operator>>(std::istringstream& ifs, OdomData& odom_data) {
  std::string type;
  ifs >> type;
  if (type != "odom") throw "Not odom";
  uint64_t time;
  ifs >> time;
  odom_data.time = time;
  ifs >> odom_data.translation.x() >> odom_data.translation.y() >>
      odom_data.translation.z() >> odom_data.rotation.w() >>
      odom_data.rotation.x() >> odom_data.rotation.y() >>
      odom_data.rotation.z();

  return ifs;
}

template <typename TypeName>
std::vector<TypeName> ReadFile(const std::string& txt) {
  std::ifstream file;
  file.open(txt);
  CHECK(file.good()) << txt;
  std::string line;
  std::vector<TypeName> result;
  std::getline(file, line);
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    try {
      TypeName data;
      iss >> data;
      // CHECK(file.good());
      result.push_back(data);
    } catch (...) {
    }
  }
  file.close();
  LOG(INFO) << "done";
  return result;
}
//
template <typename Sensor>
std::map<uint64_t, Sensor> SesorDataParse(const std::string& file) {
  const auto imu_data = ReadFile<Sensor>(file);
  CHECK(!imu_data.empty());
  std::map<uint64_t, Sensor> result;
  for (const auto& imu : imu_data) {
    LOG_IF(ERROR, !result.emplace(imu.time, imu).second)
        << "Sensor time duplicate..";
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
  auto it1 = file_name.find_last_of('.') ;
    // LOG(INFO)<<std::stol(file_name.substr(0, it1-2));
  return std::stol(file_name.substr(0, it1-2));
}
//
std::string GetFromName(const std::string& name) {
  CHECK(!name.empty());
  auto it1 = name.find_last_of('.') ;
    // LOG(INFO)<<(name.substr(0, it1-2));
  return name.substr(0, it1-2);
}
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
      LOG_IF_EVERY_N(
          ERROR,
          !result
               .emplace(GetTimeFromName(file),
                        ImageData{GetTimeFromName(file), GetFromName(file)})
               .second,
          1000)
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
    order_queue_->AddData(Sensor::Name(), itor->second.ToPatchData());
  }
  imu_datas.erase(imu_datas.begin(), it);
}
void WriteImuData(uint64_t time, std::map<uint64_t, ImuData>& imu_datas) {
  auto it = imu_datas.upper_bound(time);
  for (auto itor = imu_datas.begin(); itor != it; ++itor) {
      sensor_msgs::msg::Imu msg;
        // uint64_t time;
      msg.angular_velocity.set__x(itor->second.angular_velocity.x());
      msg.angular_velocity.set__y(itor->second.angular_velocity.y());
      msg.angular_velocity.set__z(itor->second.angular_velocity.z());
      msg.linear_acceleration.set__x(itor->second.linear_acceleration.x());
      msg.linear_acceleration.set__y(itor->second.linear_acceleration.y());
      msg.linear_acceleration.set__z(itor->second.linear_acceleration.z());
      msg.header.frame_id = "imu";
      msg.header.stamp =  rclcpp::Time(itor->second.time);
      if (imu_tracker_ == nullptr) {
        imu_tracker_ = std::make_unique<ImuTracker>(
            10, common::FromUniversal(itor->second.time / 100));
      }
      //

      imu_tracker_->AddImuData(sensor::ImuData{
          common::FromUniversal(itor->second.time / 100),
          itor->second.linear_acceleration, itor->second.angular_velocity});
      //
      imu_tracker_->Advance(common::FromUniversal(itor->second.time / 100));
      //
      const Eigen::Quaterniond q = imu_tracker_->orientation();
      //
      imu_pub2_->publish(msg);
      //
      ::geometry_msgs::msg::TransformStamped tf_trans;
      tf_trans.header.stamp = rclcpp::Time(itor->second.time);
      tf_trans.header.frame_id = "map";
      tf_trans.child_frame_id = "imu_link";
      tf_trans.transform.rotation.x = q.x();
      tf_trans.transform.rotation.y = q.y();
      tf_trans.transform.rotation.z = q.z();
      tf_trans.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(tf_trans);

      // LOG(INFO)<<"!";
      order_queue_->AddData(ImuData::Name(), itor->second.ToPatchData());
  }
  imu_datas.erase(imu_datas.begin(), it);
}
//
template <typename Sensor, typename OdomSensor>
void Run(std::map<uint64_t, Sensor>& imu_datas,
         std::map<uint64_t, OdomSensor>& odom_datas,
         std::map<uint64_t, ImageData> images_datas) {
  LOG(INFO) << "Run start..";
  // LOG(INFO) << "Write init befor image time imu data lenth: "
  //           << std::distance(
  //                  imu_datas.begin(),
  //                  imu_datas.upper_bound(images_datas.begin()->first));
  //
  int i = 0;

  uint64_t time = images_datas.begin()->first;
  for (const auto& image : images_datas) {
    //
    time = image.second.time;
    if(time<22709841922900)continue;
    // LOG(INFO) << "image time : " << image.second.time
    //           << " start imu t: " << imu_datas.begin()->first
    //           << ", end imu t: " << imu_datas.upper_bound(time)->first
    //           << " size:"
    //           << std::distance(imu_datas.begin(), imu_datas.upper_bound(time));

    // if(i++<120){
    //   time+=100*1000*1000;
    //   continue;
    // }
    WriteImuData(time, imu_datas);
    WriteImuData(time, odom_datas);
    // cv::Mat image_l= cv::imread(image.second.image_name +
    // "_l_.png",cv::IMREAD_GRAYSCALE); cv::Mat image_r=
    // cv::imread(image.second.image_name + "_r_.png",cv::IMREAD_GRAYSCALE);
    // cv::Mat temp1;
    //  cv::equalizeHist( image_l, temp1);
    // // static cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(10.0, cv::Size(8,
    // 8));
    // // clahe->apply(image_l, temp1);
    // auto temp = std::make_shared<cv::Mat>(temp1.clone());
    //  cv::equalizeHist( image_r, temp1);
    // // clahe->apply(image_r, temp1);
    // auto temp2 = std::make_shared<cv::Mat>(temp1.clone());

    // order_queue_->AddData(
    //     kImagTopic0,
    //     std::make_unique<sensor::DispathcData<sensor::ImageData>>(
    //         sensor::ImageData{
    //             common::FromUniversal(image.first / 100),
    //             {
    //                 temp,
    //                 temp2,

    //             }}));

  // try{
    const cv::Mat lr_image =
        cv::imread(image.second.image_name + "_0.jpg", cv::IMREAD_GRAYSCALE);
    const cv::Mat vr_image =
        cv::imread(image.second.image_name + "_1.jpg", cv::IMREAD_GRAYSCALE);
    //         const cv::Mat vr_image1 =
    //     cv::imread(image.second.image_name + "_2.jpg", cv::IMREAD_GRAYSCALE);
    // cv::imwrite("/home/lyp/mask.png",vr_image(cv::Rect(0, 0, 544, 640)).clone());
    // cv::imshow("l_image",lr_image);

    if(lr_image.empty()||vr_image.empty() )continue;
    // cv::imshow("l_image",lr_image(cv::Rect(640, 0, 640, 544)));
    // // cv::imshow("vr_image",vr_image);
    // cv::waitKey(0);
    // LOG(INFO)<<imu_cam_time_offset;

    order_queue_->AddData(
        kImagTopic0,
        std::make_unique<sensor::DispathcData<sensor::ImageData>>(
            sensor::ImageData{
                common::FromUniversal(time / 100) +
                    common::FromSeconds(imu_cam_time_offset),
                {
                  // lr_image,
                  // vr_image,
                  // vr_image1,
                  // vr_image1
                    lr_image(cv::Rect(0, 0, 640, 544)).clone(),
                    lr_image(cv::Rect(640, 0, 640, 544)).clone(),
                    vr_image(cv::Rect(0, 0, 544, 640)).clone(),
                    vr_image(cv::Rect(544, 0, 544, 640)).clone()
                }}));

  // }catch(cv::Exception){

  // }
    // time+=100*1000*1000;
  }
  if (!imu_datas.empty()) {
    WriteImuData(UINT64_MAX, imu_datas);
  }
  if (!odom_datas.empty()) {
    WriteImuData(UINT64_MAX, odom_datas);
  }
  CHECK(imu_datas.empty());
  CHECK(odom_datas.empty());
}
transform::Rigid3d Projectz(const transform::Rigid3d& pose) {
  return jarvis::transform::Rigid3d(
      Eigen::Vector3d(pose.translation().x(), pose.translation().y(), 0),
      pose.rotation());
  ;
}
}  // namespace
bool kill_thread = false;
int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  //
  // LocalGlogSink glog_sink;
  // google::AddLogSink(&glog_sink);

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("jarvis_ros2");
  if (kRecordFlag) {
    kOPoseFile.open("/tmp/vio_pose.txt", std::ios::out);
    kSlipFile.open("/tmp/slep_vio_pose.txt", std::ios::out);
  }
 
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  //
  // Eigen::Vector2d rtk_start(-17.538,-5.120);
  // Eigen::Vector2d odm_start(-27.193,-5.598);
  // Eigen::Vector2d vio_start(-7.175,-8.858);

  // Eigen::Vector2d rtk_end(-15.873,-4.996);
  // Eigen::Vector2d odm_end(-26.046,-7.033);
  // Eigen::Vector2d vio_end(-7.937,-9.761);




  // auto vio_lenth = (vio_end-vio_start).norm();
  // auto odo_lenth = (odm_end-odm_start).norm();
  // auto rth_lenth = (rtk_end-rtk_start).norm();
  // LOG(INFO) << "vio_lenth:" << vio_lenth << " " << "odo_lenth:" << odo_lenth
  //           << " ,rtk lenth:" << rth_lenth << " vio err:" << odo_lenth - vio_lenth
  //           << " rtk err:" << rth_lenth - odo_lenth;
  // KImuExtrapolator = std::make_unique<jarvis::estimator::ImuExtrapolator>();
  const std::string data_dir(argv[2]);
  CHECK_EQ(argc, 3);
  LOG(INFO) << "input dir : " << data_dir;
  LOG(INFO) << "config file : " << argv[1];
  //
  ParseOption(argv[1]);
  // std::unique_ptr<jarvis_ros::RosCompont> ros_compont =
  //     std::make_unique<jarvis_ros::RosCompont>(node.get());
  ros_compont = std::make_unique<jarvis_ros::RosCompont>(node.get());
  imu_pub2_ = node->create_publisher<sensor_msgs::msg::Imu>("imu", 1000);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  //
  // /
  TrackingData tracking_data_temp;
  std::mutex mutex;
  std::condition_variable cond;

  const std::string image_file = data_dir + "image/";
  const std::string odom_file = data_dir + "imu.txt";
  //
  const std::string vslam_yaml_file(argv[1]);

  auto slip_detect = slip_detect::FactorSlipDetect(vslam_yaml_file);
  //
  std::vector<bool> slip_states;


  TrajectorBuilderOption trajectorbuilder_option ;
  
  ParseYAMLOption(std::string(argv[1]),&trajectorbuilder_option );
  //
  estimator::EstimatorOption& option = trajectorbuilder_option.esti_option;

  //
  std::vector<std::shared_ptr<jarvis::estimator::PyramidImage>> pry_impls;
  auto& esit_option_ = option;
  imu_cam_time_offset = jarvis::GetTimeShiftCamImu();
  LOG(INFO)<<imu_cam_time_offset ;
  if (kuse_gpu) {
    for (size_t i = 0; i < esit_option_.track_sequence.size(); i++) {
      for (size_t j = 0; j < esit_option_.track_sequence[i].size(); j++) {
        esit_option_.feature_track_options[i].pyramid_image.push_back(
            std::make_shared<jarvis::estimator::ExtendPyramidImage>(
                esit_option_.feature_track_options[i].pyrmid_option));
        pry_impls.push_back(std::make_shared<jarvis::estimator::PyramidImage>(
            option.feature_track_options[i].pyrmid_option));
      }
    }
    //
  }
  // for (int i = 0; i < option.track_sequence.size(); i++) {
  //   option.feature_track_options[i].pyramid_image.push_back(
  //       std::make_shared<ExtendPyramidImage>
  //   );
  // }

std::unique_ptr<jarvis::object::ObjectInterface> object_interface  =nullptr;
//
builder_ = std::make_unique<TrajectorBuilder>(
    trajectorbuilder_option, [&](const TrackingData& data) {
      std::lock_guard<std::mutex> lock(mutex);
      //
      VLOG(kGlogLevel) << data.data->imu_state;
      CHECK(!isnan(data.data->imu_state.p.x()));
      auto tracking_data = data;
      Eigen::Matrix3d rotaion;
      rotaion << 0, 0, 1, -1, 0, 0, 0, -1, 0;
      // LOG(INFO) << rotaion;
      // auto extric =
      // transform::Rigid3d::Rotation(Eigen::Quaterniond(rotaion));
      // tracking_data_temp.data->imu_state.data->pose =
      //     extric * tracking_data_temp.data->imu_state.data->pose;
      // {
      //   std::unique_lock<std::mutex> lock(mutex);
      //   cond.wait(lock);
      //   tracking_data = tracking_data_temp;
      // }
      // if(tracking_data.status==2){
      //   KImuExtrapolator->AddState(data.data->time, data.data->imu_state);
      // }

      auto start = std::chrono::high_resolution_clock::now();
      auto slipe_alignment_pose = tracking_data.data->imu_state.Pose();
      if (slip_detect) {
        slip_detect->AddPose(slip_detect::TimePose{
            tracking_data.data->time, tracking_data.data->imu_state.Pose()});
        auto flag = slip_detect->Detect(tracking_data.data->time);
        kSlipFile << std::to_string(uint64_t(
                         jarvis::common::ToUniversal(tracking_data.data->time) *
                         1e2))
                  << " " << int(flag) << std::endl;

        ros_compont->PubBoolMsg(flag);
        slipe_alignment_pose =
            slip_detect->ToPoseInOdom((tracking_data.data->imu_state.Pose()));
      }
      // LOG(INFO) << tracking_data.data->imu_state;
      if (kRecordFlag) {
        const auto pose =
            slipe_alignment_pose;  // tracking_data.data->imu_state.Pose();
        std::stringstream info;
        info << std::to_string(uint64_t(
                    jarvis::common::ToUniversal(tracking_data.data->time) *
                    1e2))
             << " " << pose.translation().x() << " " << pose.translation().y()
             << " " << pose.translation().z() << " " << pose.rotation().w()
             << " " << pose.rotation().x() << " " << pose.rotation().y() << " "
             << pose.rotation().z();
        kOPoseFile << info.str() << std::endl;
      }

      ros_compont->PushMark({{"vo", slipe_alignment_pose}}, true);

      // ros_compont->PushMark({{"vo", tracking_data.data->imu_state.Pose()}},
      // true);
      //
      std::vector<jarvis::object::ObjectImageResult> object_result;
      if (tracking_data.status == 2) {
        if (object_interface == nullptr) {
          object_interface = std::make_unique<jarvis::object::ObjectInterface>(
              option.feature_track_options[0].cameras[0],"");
        }
        object_result = object_interface->Detect(
            common::ToUniversal(tracking_data.data->time),
            tracking_data.data->images.image[0],
            // tracking_data.data->features_datas[0].features.data->images[0],
            tracking_data.data->imu_state.Pose(),
            option.slide_windows_option.extric_camera_to_imu[0]);
        ros_compont->OnLocalTrackingResultCallback(
            tracking_data, &object_result, transform::Rigid3d::Identity());
      } else {
        object_interface = nullptr;
      }
      ros_compont->PosePub(tracking_data.data->imu_state.Pose(),
                           transform::Rigid3d::Identity());
      rclcpp::spin_some(node);
      cond.notify_one();
    });

//
order_queue_ = std::make_unique<sensor::OrderedMultiQueue>();
order_queue_->AddQueue(kOdomTopic, [&](const sensor::OdometryData& odom_data) {
  // LOG(INFO)<<odom_data.pose<<odom_data.time;;
  ros_compont->PushMark({{"odom", odom_data.pose}});
  builder_->AddOdometryData(jarvis::sensor::OdometryData{
      odom_data.time + common::FromSeconds(0.1), odom_data.pose});
  slip_detect->AddOdometry(odom_data);
});
//

std::vector<std::pair<bool, sensor::ImageData>> image_datas_pry;
std::mutex mutex_py;
  std::mutex pyramid_mutex_;
  std::thread  pyramid_thread_;
  std::vector<std::pair<bool, jarvis::sensor::ImageData> > image_datas_pyra_;
  
order_queue_->AddQueue(
    kImagTopic0,
    [&](const sensor::ImageData& imag_data) {
      // slip_detect->AddImage(imag_data);
      // auto flag = slip_detect->Detect(imag_data.time);
      // ros_compont->PubBoolMsg(flag);

      if (imag_data.image[0].empty() || imag_data.image[1].empty() ||
          imag_data.image[2].empty() || imag_data.image[3].empty()) {
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
        pyramid_thread_ = std::thread([&,imag_data]() {
          for (size_t i = 0; i < esit_option_.track_sequence.size(); i++) {
            for (size_t j = 0; j < esit_option_.track_sequence[i].size(); j++) {
              pry_impls[esit_option_.track_sequence[i][j]]->Build(
                  imag_data.image[esit_option_.track_sequence[i][j]]);
              std::vector<cv::Mat> pyrmd_drev =
                  pry_impls[esit_option_.track_sequence[i][j]]->CurrPyram();

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
         pyramid_thread_ = std::thread([&, imag_data]() {
          for (size_t i = 0; i < esit_option_.track_sequence.size(); i++) {
            for (size_t j = 0; j < esit_option_.track_sequence[i].size(); j++) {
              pry_impls[esit_option_.track_sequence[i][j]]->Build(
                  imag_data.image[esit_option_.track_sequence[i][j]]);
              std::vector<cv::Mat> pyrmd_drev =
                  pry_impls[esit_option_.track_sequence[i][j]]->PrePyram();

              image_datas_pyra_.back()
                  .second.pyramid_derive[esit_option_.track_sequence[i][j]] =
                  pyrmd_drev;
            }
          }
          image_datas_pyra_.back().first = true;
        });
      }

      } else {
        builder_->AddImageData(imag_data);
      }
    }

);
order_queue_->AddQueue(kImuTopic, [&](const sensor::ImuData& imu) {
  builder_->AddImuData(jarvis::sensor::ImuData{
      imu.time + common::FromSeconds(0.1),
      imu.linear_acceleration,
      imu.angular_velocity,
  });
});
LOG(INFO) << "Parse image dir: " << image_file;
LOG(INFO) << "Parse imu dir: " << odom_file;
auto image_datas = ImageData::Parse(image_file);
auto odom_datas = SesorDataParse<OdomData>(odom_file);
auto imu_datas = SesorDataParse<ImuData>(odom_file);
//
//
LOG(INFO) << "Start run...";
std::thread pub_map_points([&]() {
  while (!kill_thread) {
    TrackingData tracking_data;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //
    {
      ros_compont->PubMapPoints(builder_->GetMapPoints());
      auto global_pose = builder_->GetKeyFrameGlobalPose();
      std::map<std::string, std::vector<Eigen::Vector3d>> pub_poses;
      for (const auto& pose : global_pose) {
        pub_poses["trajctor_" + std::to_string(pose.first.trajectory_id)]
            .push_back(pose.second.transform.translation());
      }
      ros_compont->PubTrajectorPoseWithMark(pub_poses);
    }
    {
      ros_compont->PubLocalMapPoints(builder_->GetLocalMapPoints());
      auto local_track_pose = builder_->GetLocalKeyFramePose();

      std::map<std::string, std::vector<Eigen::Vector3d>> pub_poses;
      for (const auto& pose : local_track_pose) {
        pub_poses["trajctor_0"].push_back(pose.translation());
      }
      ros_compont->PubLocalTrajectorPoseWithMark(pub_poses);
    }

    // ros_compont->PushMark(
    //     {{"vo", slip_detect->ToPoseInOdom(
    //                   tracking_data.data->imu_state.data->pose)}},true);

    // rclcpp::spin_some(node);
    // }
  }
});
order_queue_->Start();
Run(imu_datas, odom_datas, image_datas);

order_queue_->Stop();
builder_ = nullptr;
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