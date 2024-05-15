#include <compressed_image_transport/compressed_publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <rosbag/view.h>
#include <signal.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <condition_variable>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <queue>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>

#include "geometry_msgs/PoseStamped.h"
#include "glog/logging.h"
#include "image_transport/image_transport.h"
#include "jarvis/key_frame_data.h"
#include "jarvis/sensor/data_process.h"
#include "jarvis/sensor/stereo_sync.h"
#include "jarvis/trajectory_builder.h"
#include "ros_component.h"
#include "ros_viewer.h"
#include "rosbag/bag.h"
#include "sensor_msgs/Imu.h"
#include "std_srvs/Empty.h"
//
#include "yaml-cpp/yaml.h"
// #include <ros/ros.h>
//

bool thread_kill_ = false;
void FunSig(int sig) {
  ros::shutdown();
  thread_kill_ = true;
  exit(0);
}
namespace {
double image_sample_ration = 1;
double cam0_cam1_time_offset_thresh_hold = 0;
double imu_cam_time_offset = 0.0;
double play_bag_duration = 0.3;
}  // namespace
using namespace jarvis;
transform::Rigid3d local_to_globle_transform = transform::Rigid3d::Identity();

class RosbagProscess {
 public:
  using pair_image_fuction = std::function<int(const sensor::ImageData &)>;
  using imu_fuction = std::function<int(const sensor::ImuData &)>;
  RosbagProscess(const std::string &bag_file) {
    try {
      in_bag_.open(bag_file, rosbag::bagmode::Read);
    } catch (...) {
      LOG(FATAL) << "open :" << bag_file << "faied.";
    }
    order_queue_ = std::make_unique<sensor::OrderedMultiQueue>();
  };
  //
  void initDataProcFunc(imu_fuction func, const std::string &topic) {
    // imu_call_back_.emplace(topic, func);
    order_queue_->AddQueue(
        topic,
        [this, func](const sensor::ImuData &imu_data) { func(imu_data); });
  }

  void initDataProcFunc(pair_image_fuction func, const std::string &topic,
                        bool is_base) {
    if (is_base) {
      stereo_sync_ = std::make_unique<sensor::StereoSync>(
          cam0_cam1_time_offset_thresh_hold, image_sample_ration,
          [&](const sensor::ImageData &data) {
            order_queue_->AddData(
                topic,
                std::make_unique<sensor::DispathcData<sensor::ImageData>>(
                    data));
          });
    }
    stereo_sync_->InitCacheImage(topic, is_base);
    if (is_base) {
      order_queue_->AddQueue(topic,
                             [this, func](const sensor::ImageData &imag_data) {
                               func(imag_data);
                             });
    }
  }
  const Eigen::Vector3d gry_offset{0, 0, 0};
  sensor::ImuData ToImuData(sensor_msgs::ImuConstPtr imu) {
    return sensor::ImuData{
        imu->header.stamp.toSec() + imu_cam_time_offset,
        Eigen::Vector3d{imu->linear_acceleration.x, imu->linear_acceleration.y,
                        imu->linear_acceleration.z},
        (Eigen::Vector3d{imu->angular_velocity.x, imu->angular_velocity.y,
                         imu->angular_velocity.z} -
         double(M_PI / 180.f) * gry_offset)};
  }
  void invokeProcess() {
    double last_topic_time = 0;
    bool last_topic_time_flag = false;
    // while (ros::ok()) {
    ros::Rate rate(1.0 / play_bag_duration);
    rosbag::View view(in_bag_);
    rosbag::View::const_iterator view_iterator = view.begin();
    for (auto view_iterator = view.begin(); view_iterator != view.end();
         view_iterator++) {
      rosbag::MessageInstance msg = *view_iterator;
      // KeyPause();

      ros::spinOnce();
      rate.sleep();
      // if (msg.getTime().toSec() < 1.403636580838555574e9) continue;
      if (msg.isType<sensor_msgs::Imu>()) {
        auto imu = msg.instantiate<sensor_msgs::Imu>();
        if (!last_topic_time_flag) {
          last_topic_time_flag = true;
          last_topic_time = imu->header.stamp.toSec();
        }
        if (fabs(last_topic_time - imu->header.stamp.toSec()) > 0.5) {
          return;
        }
        last_topic_time = imu->header.stamp.toSec();
        // CHECK(imu_call_back_.count(msg.getTopic()));
        order_queue_->AddData(
            msg.getTopic(),
            std::make_unique<sensor::DispathcData<sensor::ImuData>>(
                ToImuData(imu)));
      };
      if (msg.isType<sensor_msgs::Image>()) {
        //
        cv_bridge::CvImagePtr img =
            cv_bridge::toCvCopy(msg.instantiate<sensor_msgs::Image>());
        const sensor::StereoSync::LocalImageData image_msg{
            img->header.stamp.toSec(), msg.getTopic(),
            std::make_shared<cv::Mat>(img->image.clone())};

        stereo_sync_->ProcessImage(image_msg, msg.getTopic());
      }

      if (msg.isType<sensor_msgs::CompressedImage>()) {
        cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(
            msg.instantiate<sensor_msgs::CompressedImage>());
        if (!last_topic_time_flag) {
          last_topic_time_flag = true;
          last_topic_time = img->header.stamp.toSec();
        }
        if (fabs(last_topic_time - img->header.stamp.toSec()) > 0.5) {
          return;
        }
        last_topic_time = img->header.stamp.toSec();

        const sensor::StereoSync::LocalImageData image_msg{
            img->header.stamp.toSec(), msg.getTopic(),
            std::make_shared<cv::Mat>(img->image.clone())};

        stereo_sync_->ProcessImage(image_msg, msg.getTopic());
      }
    }
  }
  double last_topic_time = 0;
  bool last_topic_time_flag = false;

  rosbag::Bag in_bag_;
  std::unique_ptr<sensor::StereoSync> stereo_sync_;
  std::unique_ptr<sensor::OrderedMultiQueue> order_queue_;
};

std::unique_ptr<jarvis_ros::RosCompont> ros_compont;
std::mutex gMutex;
int main(int argc, char *argv[]) {
  if (argc != 2) {
    fprintf(stderr, "%s yaml_config_path\n", argv[0]);
    return -1;
  }
  //
  ros::init(argc, argv, "stero_test");

  YAML::Node paras = YAML::LoadFile(std::string(argv[1]));
  //
  std::string imu_topic = paras["imu_topic"].as<std::string>();
  std::string left_img_topic = paras["image0_topic"].as<std::string>();
  std::string right_img_topic = paras["image1_topic"].as<std::string>();
  //
  imu_cam_time_offset = paras["imu_cam_time_offset"].as<double>();
  cam0_cam1_time_offset_thresh_hold =
      paras["cam0_cam1_time_offset_thresh_hold"].as<double>();

  image_sample_ration = paras["image_sample"].as<double>();
  std::string rosbag_path = paras["data_folder"].as<std::string>();
  play_bag_duration = paras["offline_play_bag_duration"].as<double>();
  ros::NodeHandle nh;
  ros_compont = std::make_unique<jarvis_ros::RosCompont>(&nh);
  jarvis_ros::RosViewer ros_viwer(&nh, "/tmp/");
  TrackingData tracking_data_temp;
  std::mutex mutex;
  std::condition_variable cond;
  //
  //

  RosbagProscess rosbagin(rosbag_path);
  //
  std::unique_ptr<TrajectorBuilder> builder_(
      new TrajectorBuilder(std::string(argv[1]), [&](const TrackingData &data) {
        std::unique_lock<std::mutex> lock(mutex);
        tracking_data_temp = data;
        cond.notify_one();
      }));
  // object::ObjectInterface object_interface(builder_->GetMapBuilder());
  //
  rosbagin.initDataProcFunc(
      [&](const sensor::ImuData &imu) {
        std::lock_guard<std::mutex> lock(gMutex);
        builder_->AddImuData(imu);
        // LOG(INFO) << "imu " << std::to_string(imu.time) << " "
        //           << imu.angular_velocity.transpose() << " "
        //           << imu.linear_acceleration.transpose();
        return 0;
      },
      imu_topic);

  rosbagin.initDataProcFunc(
      [&](const sensor::ImageData &data) {
        std::lock_guard<std::mutex> lock(gMutex);
        // LOG(INFO) <<"cam "<< std::to_string(data.time);
        builder_->AddImageData(data);
        return 0;
      },
      left_img_topic, true);

  rosbagin.initDataProcFunc(nullptr, right_img_topic, false);
  std::map<KeyFrameId, transform::TimestampedTransform> pose_temp;
  std::thread pub_map_points([&]() {
    while (ros::ok()) {
      TrackingData tracking_data;
      std::this_thread::sleep_for(std::chrono::milliseconds(30));

      //
      static uint8_t count = 0;
      // if (kReciveTempGoal)
      if (++count > 30) {
        count = 0;
        auto points = builder_->GetMapPoints();
        std::map<int, std::map<KeyFrameId, transform::TimestampedTransform>>
            poses;

        ros_viwer.AddPairIds(builder_->ConstraintId());
        pose_temp = builder_->GetKeyFrameGlobalPose();
        // object_interface.UpdateGloblePose();
        // for (const auto pose_id : pose_temp) {
        //   poses[pose_id.first.trajectory_id_].insert(pose_id);
        // }
        // ros_viwer.AddPoses(poses[0], "global");
        // for (size_t i = 1; i < poses.size(); i++) {
        //   ros_viwer.AddPoses(poses[i], "global" + std::to_string(i));
        // }
        // ros_viwer.Viewer();
        // ros_compont->PubMapPoints(points);
        // local_to_globle_transform = builder_->GetLocalToGlobalTransform();
      }
      {
        std::unique_lock<std::mutex> lock(mutex);
        cond.wait(lock);
        tracking_data = tracking_data_temp;
      }
      // std::vector<mb::object::ObjectImageResult> object_result =
      //     object_interface.Detect(common::ToUniversal(tracking_data.data->time),
      //                             *tracking_data.data->image,
      //                             tracking_data.data->pose);

      //
      ros_compont->OnLocalTrackingResultCallback(tracking_data, nullptr,
                                                 local_to_globle_transform);

      ros_compont->PosePub(tracking_data.data->pose, local_to_globle_transform);
    }
  });

  rosbagin.invokeProcess();
  signal(SIGINT, FunSig);  // ctrl-c
  ros::start();
  ros::spin();

  pub_map_points.join();
}