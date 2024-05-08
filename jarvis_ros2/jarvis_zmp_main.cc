
#include <dirent.h>
#include <sys/types.h>

#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "fstream"
#include "jarvis/sensor/data_process.h"
#include "jarvis/sensor/stereo_sync.h"
#include "jarvis/trajectory_builder.h"
#include "rclcpp/rclcpp.hpp"
#include "ros_component.h"
#include "std_msgs/msg/string.hpp"
#include "unistd.h"
//
#include <glog/logging.h>

#include <zmqpp/zmqpp.hpp>
//
#include <cv_bridge/cv_bridge.hpp>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

namespace {

using namespace jarvis;

}  // namespace
#define SERVER_PORT		97555    //
#define SERVER_IP		"192.168.78.1"	//服务器IP地址
#define recv_Buf_Size  1500
struct PoseData {
  double x;
  double y;
  double z;
  double qw;
  double qx;
  double qy;
  double qz;
};
nav_msgs::msg::Path path_;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
void PosePub(const transform::Rigid3d &pose,
                         const transform::Rigid3d &local_to_global) {
  ::geometry_msgs::msg::TransformStamped tf_trans;
  ::geometry_msgs::msg::TransformStamped global_tf_trans;
  const Eigen::Quaterniond &q = pose.rotation();
  tf_trans.header.stamp = rclcpp::Time();
  tf_trans.header.frame_id = "map";
  tf_trans.child_frame_id = "camere_link";
  auto const &trans = pose.translation();
  tf_trans.transform.translation.x = trans[0];
  tf_trans.transform.translation.y = trans[1];
  tf_trans.transform.translation.z = trans[2];
  tf_trans.transform.rotation.x = q.x();
  tf_trans.transform.rotation.y = q.y();
  tf_trans.transform.rotation.z = q.z();
  tf_trans.transform.rotation.w = q.w();
  tf_broadcaster_->sendTransform(tf_trans);
  //

  const transform::Rigid3d global_pose = local_to_global * pose;

  global_tf_trans.header.stamp = rclcpp::Time();
  global_tf_trans.header.frame_id = "map";
  global_tf_trans.child_frame_id = "global_camera_link";
  global_tf_trans.transform.translation.x = global_pose.translation().x();
  global_tf_trans.transform.translation.y = global_pose.translation().y();
  global_tf_trans.transform.translation.z = global_pose.translation().z();
  global_tf_trans.transform.rotation.x = global_pose.rotation().x();
  global_tf_trans.transform.rotation.y = global_pose.rotation().y();
  global_tf_trans.transform.rotation.z = global_pose.rotation().z();
  global_tf_trans.transform.rotation.w = global_pose.rotation().w();
  tf_broadcaster_->sendTransform(global_tf_trans);
  //
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.stamp = rclcpp::Time();
  pose_stamped.header.frame_id = "map";
  pose_stamped.pose.position.x = tf_trans.transform.translation.x;
  pose_stamped.pose.position.y = tf_trans.transform.translation.y;
  pose_stamped.pose.position.z = tf_trans.transform.translation.z;

  path_.header.frame_id = "map";
  if (pub_path_->get_subscription_count() != 0) {
    path_.poses.push_back(pose_stamped);
    pub_path_->publish(path_);
  } else {
    path_.poses.clear();
  }
}
void CommpressedImagePub(const cv::Mat &image) {
  // if (compressed_image_pub_->get_subscription_count() == 0) return;

  // std_msgs::msg::Header header;
  // header.frame_id = "map";
  // header.stamp = rclcpp::Time();
  // cv_bridge::CvImage img_bridge =
  //     cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, image);
  // auto img_msg = img_bridge.toCompressedImageMsg();
  // compressed_image_pub_->publish(*img_msg);

  if (image_pub_->get_subscription_count() == 0) return;

  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp = rclcpp::Time();
  cv_bridge::CvImage img_bridge =
      cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image);
  auto img_msg = img_bridge.toImageMsg();
  image_pub_->publish(*img_msg);
}

void ProcessData(std::vector<uint8_t>&d)
{
  if(d.empty())return;
  uint8_t head[4] = {0xaa, 0x55, 0xaa, 0x5a};
  auto it = std::find(d.begin(), d.end(), 0xaa);
  if (it == d.end()) {
    d.clear();
    return;
  }
  //
  for (int i = 0; i < 4; i++) {
    if (std::next(it, i) == d.end()) {
      d.clear();
      return;
    }
    if (*std::next(it, i) != head[i]) {
      d.erase(d.begin(), std::next(it, i + 1));
      return;
    }
  }

  if (std::distance(it, d.end()) < 10) return;
  it = std::next(it, 4);
  for (int i = 0; i < 4; i++) {
    head[i] = *std::next(it, i);
  }
  uint32_t data_lenth = 0;
  memcpy((void*)(&data_lenth), (void*)head, 4);
  if (data_lenth == 0) d.clear(); 
  it = std::next(it, 4);
  if(std::distance(it,d.end())<data_lenth)return;

  std::vector<uint8_t> image_data(it, std::next(it, data_lenth - sizeof(PoseData)));
  cv::Mat  image=  cv::imdecode(image_data, cv::IMREAD_COLOR);
  if (!image.empty()) {
    CommpressedImagePub(image);
  }
  // cv::imshow("iii",image);
  // cv::waitKey(1);
  // it = std::next(it, data_lenth - sizeof(PoseData));
  //

  std::vector<uint8_t> pose(it + data_lenth - sizeof(PoseData),
                            it + data_lenth);

  PoseData p;
  memcpy((void *)(&p), (void *)pose.data(), sizeof(PoseData));

  d.erase(d.begin(), std::next(it, data_lenth));
  //
  jarvis::transform::Rigid3d pose1{Eigen::Vector3d{p.x, p.y, p.z},
                                  Eigen::Quaterniond{p.qw, p.qx, p.qy, p.qz}};
  PosePub(pose1, transform::Rigid3d::Identity());
}

std::vector<uint8_t> datas;
int main(int argc, char* argv[]) {
  //
  rclcpp::init(argc, argv);
  auto nh_ = rclcpp::Node::make_shared("jarvis_zmp_ros2");
  //
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  pub_path_ = nh_->create_publisher<nav_msgs::msg::Path>("path", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*nh_);
  image_pub_ =
      nh_->create_publisher<sensor_msgs::msg::Image>(
          "local_tracking_result_image/image_raw", 20);

          //
  int connect_fd = -1;
  struct sockaddr_in server;
  socklen_t saddrlen = sizeof(server);

  memset(&server, 0, sizeof(server));

  connect_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (connect_fd < 0) {
    printf("socket error!\n");
    return -1;
  }

  server.sin_family = AF_INET;
  server.sin_port = htons(SERVER_PORT);
  server.sin_addr.s_addr = inet_addr(SERVER_IP);

  if (connect(connect_fd, (struct sockaddr*)&server, saddrlen) < 0) {
    printf("connect failed!\n");
    return -1;
  }

  uint8_t buff[recv_Buf_Size];
  std::mutex mutex;
  // std::thread thread([&]() {
  //   while (rclcpp::ok()) {
  //     {
  //       std::lock_guard<std::mutex> lock(mutex);
  //       ProcessData(datas);
  //     }
  //     usleep(10000);
  //     // std::this_thread::sleep_for(std::chrono::microseconds(1));
  //   }
  // });
  while (rclcpp::ok()) {
    int ret = recv(connect_fd, buff, recv_Buf_Size, 0);
    if (ret > 0) {
      // std::lock_guard<std::mutex>lock(mutex);
      for (int i = 0; i < ret; i++) {
        datas.push_back(buff[i]);
        // std::cout<<std::hex<<int(buff[i])<<" ";
      }
      ProcessData(datas);
      // std::cout<<std::endl;
    }
  }
  close(connect_fd);

  return 0;

  //
  //
}