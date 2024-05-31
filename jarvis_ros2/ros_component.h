#ifndef _ROS_COMPONET_H
#define _ROS_COMPONET_H
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>

#include "jarvis/transform/transform.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
//
#include "jarvis/key_frame_data.h"
#include "jarvis/object/object_interface.h"
//
namespace jarvis_ros {
class RosCompont {
 public:
  RosCompont(rclcpp::Node *nh_);

  void CommpressedImagePub(const cv::Mat&image);
  //
  void OnMapPointsCallback(const std::vector<Eigen::Vector3d> &points,
                           const jarvis::transform::Rigid3d &local_to_globle);
  //
  void PosePub(const jarvis::transform::Rigid3d &pose,
               const jarvis::transform::Rigid3d &local_to_globle);

  void PubMapPoints(const std::vector<Eigen::Vector3d> &points);
  //
  void PubPointPlan(const std::vector<Eigen::Vector3f> &points,
                    const Eigen::Vector4f &vector,
                    const Eigen::Vector3f &centriod

  );
  //
  void MarkPub(
      std::map<int, std::vector<jarvis::object::ObjectImageResult>> &t);
  void OnLocalTrackingResultCallback(
      const jarvis::TrackingData &tracking_data,
      std::vector<jarvis::object::ObjectImageResult> *object_result,
      const jarvis::transform::Rigid3d &local_to_globle);
  //

 private:
 void PubPoseWithMark(
                     const std::map<std::string, std::vector<Eigen::Vector3d>>& poses);
  rclcpp::Node *nh_;
  // std::vector<std::shared_ptr<rclcpp::PublisherBase>> publishers_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>
      point_cloud_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>
      map_point_cloud_pub_;
  // ros::Publisher point_cloud_pub_;
  // ros::Publisher map_point_cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  // ros::Publisher pub_mark_points_;
  // ros::Publisher pub_mark_points_arrow_;
  // ros::Publisher compressed_image_pub_;
  // rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
      // compressed_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      image_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      pose_mark_publisher_;
  // ros::Publisher markpub_;
  // ros::Publisher pub_local_tracking_result_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::map<std::string,std::vector<Eigen::Vector3d>> poses_;
  nav_msgs::msg::Path path_;
};
}  // namespace jarvis_ros
#endif