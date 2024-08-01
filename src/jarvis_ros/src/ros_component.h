#ifndef _ROS_COMPONET_H
#define _ROS_COMPONET_H
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "jarvis/transform/transform.h"
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
//
#include "jarvis/object/object_interface.h"
#include "jarvis/key_frame_data.h"
//
namespace jarvis_ros {
class RosCompont {
 public:
  RosCompont(ros::NodeHandle *nh_);
  ~RosCompont();

  void CommpressedImagePub(const sensor_msgs::Image &image);
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
  void MarkPub(std::map<int, std::vector<jarvis::object::ObjectImageResult>> &t);
  void OnLocalTrackingResultCallback(
      const jarvis::TrackingData &tracking_data,
      std::vector<jarvis::object::ObjectImageResult> *object_result,
      const jarvis::transform::Rigid3d &local_to_globle);
  //

 private:
  ros::Publisher point_cloud_pub_;
  ros::Publisher map_point_cloud_pub_;
  ros::Publisher pub_path_;
  ros::Publisher pub_mark_points_;
  ros::Publisher pub_mark_points_arrow_;
  nav_msgs::Path path_;
  ros::Publisher compressed_image_pub_;

  ros::Publisher markpub_;
  ros::Publisher pub_local_tracking_result_;
  tf::TransformBroadcaster *tf_broadcaster_;
  ros::NodeHandle *nh_;
};
}  // namespace jarvis_ros
#endif