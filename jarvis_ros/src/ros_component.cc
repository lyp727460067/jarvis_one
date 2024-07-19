#include "ros_component.h"

#include <compressed_image_transport/compressed_publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/MarkerArray.h>

#include "image_transport/image_transport.h"
#include "jarvis/common/eigen_quaterniond_from_two_vectors.h"
#include "random"
namespace jarvis_ros {
//
namespace {
//
using namespace jarvis;
}  // namespace
cv::Mat GenerateImageWithKeyPoint(
    const cv::Mat &l_img, const std::vector<cv::KeyPoint> &l_key_points,
    const std::vector<cv::KeyPoint> &predict_pts, const cv::Mat &r_img,
    const std::vector<cv::KeyPoint> &r_key_points, const std::string &l_name,
    const std::string &r_name, std::vector<uint64_t> outlier_pointclass_id) {
  int col = l_img.cols;
  int row = l_img.rows;
  // cv::Mat l_img_feat;
  // cv::cvtColor(l_img, l_img_feat, cv::COLOR_GRAY2RGB);
  // cv::Mat r_img_feat;
  // cv::cvtColor(r_img, r_img_feat, cv::COLOR_GRAY2RGB);
  // //
  //   const int gap = 10;
  //   cv::Mat gap_image(row, gap, CV_8UC1, cv::Scalar(255, 255, 255));
  cv::Mat gray_img, loop_match_img;
  //   cv::hconcat(l_img, gap_image, gap_image);
  //   cv::hconcat(gap_image, r_img, gray_img);
  //
  // common::FixedRatioSampler sampler(0.1);
  cvtColor(l_img, loop_match_img, cv::COLOR_GRAY2RGB);
  std::mt19937 rng(42);
  std::uniform_int_distribution r_bound_distribution(1, 255);
  std::uniform_int_distribution b_bound_distribution(1, 255);
  std::uniform_int_distribution g_ound_distribution(1, 255);
  std::set<uint64_t> class_id(outlier_pointclass_id.begin(),
                              outlier_pointclass_id.end());
  //
  for (auto &&keypoint : (l_key_points)) {
    double len = std::min(1.0, 1.0 * keypoint.octave / 20);
    cv::Scalar color = cv::Scalar(255 * (1 - len), 0, 255 * len);
    // if (class_id.count(keypoint.class_id)) {
    //   cv::circle(loop_match_img, keypoint.pt, 2 ,cv::Scalar(0, 0, 255), 2);
    // } else {
    cv::circle(loop_match_img, keypoint.pt, 2, color, 2);
    // }
    // cv::putText(loop_match_img, std::to_string(keypoint.class_id),
    // keypoint.pt,
    //             cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
  }
  for (auto &&keypoint : r_key_points) {
    cv::circle(loop_match_img, keypoint.pt, 1, cv::Scalar(0, 255, 0), 1);
  }
  CHECK_EQ(outlier_pointclass_id.size(), 1)
      << "Outlier_pointclass_id is used display zupt,please assignment it..";
  if (outlier_pointclass_id[0]) {
    cv::putText(loop_match_img, "ZUPT", cv::Point2f(20, 100),
                cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 255, 0), 3);
  }

  //   for (auto &&keypoint : predict_pts) {
  //     if (keypoint.pt.x < 0 || keypoint.pt.y < 0) continue;
  //     cv::circle(loop_match_img,
  //                {(int)keypoint.pt.x + (col + gap), (int)keypoint.pt.y}, 4,
  //                cv::Scalar(255, 0, 0), -1);
  //     cv::putText(loop_match_img, std::to_string(keypoint.class_id),
  //                 {(int)keypoint.pt.x + (col + gap), (int)keypoint.pt.y},
  //                 cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0));
  //   }
  // std::vector<KeyPoint> dispaly_key_points1;
  // std::vector<KeyPoint> dispaly_key_points2;

  // for (auto const& pair : paird_id) {
  //   // info1 << "[ " <<pair.first << " " << pair.second<<" ]";
  //   dispaly_key_points1.push_back(
  //       {descriptor_key_pint1.key_with_dess.key_points[pair.first]});
  //   dispaly_key_points2.push_back(
  //       {r_key_points.key_points[pair.second]});
  // }

  // for (int j = 0; j < paird_id.size(); j++) {
  //   const int &r_index = paird_id[j].first;
  //   const int &l_index = paird_id[j].second;
  //   cv::Point2f old_pt(r_key_points[r_index].point.x(),
  //                      r_key_points[r_index].point.y());
  //   old_pt.x += (col + gap);
  //   // if (!sampler.Pulse()) continue;

  //   cv::line(loop_match_img,
  //            {int(std::round(l_key_points[l_index].point.x())),
  //             int(std::round(l_key_points[l_index].point.y()))},
  //            old_pt,
  //            cv::Scalar(r_bound_distribution(rng), b_bound_distribution(rng),
  //                       g_ound_distribution(rng)),
  //            1, 8, 0);
  // }
  // cv::Mat notation(50, col + gap + col, CV_8UC3, cv::Scalar(255, 255, 255));
  // putText(notation, l_name, cv::Point2f(20, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
  //         cv::Scalar(255), 3);
  // putText(notation, r_name, cv::Point2f(20 + col + gap, 30),
  //         cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 3);
  // cv::vconcat(notation, loop_match_img, loop_match_img);
  return loop_match_img;
}
//
const std::map<std::string, cv::Scalar> kColors{
    {"global", cv::Scalar(0, 0, 255)},
    {"online", cv::Scalar(100, 10, 0)},
    {"fack", cv::Scalar(0, 255, 0)}};
cv::Mat ObjectToCvImage(const Eigen::AlignedBox2d &raw_image_size,
                        const cv::Size &size,
                        const object::ObjectImageResult &object_resut) {
  std::mt19937 rng(42);

  cv::Mat image(size, CV_8UC3, cv::Scalar::all(0));
  // drawing a 3D cubic box
  std::vector<cv::Point> points;
  for (int i = 0; i < object_resut.coners.size(); i++) {
    if (!raw_image_size.contains(object_resut.coners[i])) return image;
    points.push_back(
        cv::Point(object_resut.coners[i].x(), object_resut.coners[i].y()));
  }
  //
  if (!raw_image_size.contains(object_resut.direction[0]) ||
      !raw_image_size.contains(object_resut.direction[1])) {
    return image;
  }
  CHECK(kColors.count(object_resut.type));
  for (int i = 0; i < 3; i++) {
    cv::line(image, points[i], points[i + 1], kColors.at(object_resut.type), 5,
             8, 0);
  }
  cv::line(image, points[3], points[0], kColors.at(object_resut.type), 5, 8, 0);
  //
  if (object_resut.type == "global") {
    // for (int i = 4; i < 7; i++) {
    //   cv::line(image, points[i], points[i + 1], cv::Scalar(255, 0, 0), 5, 8,
    //   0);
    // }
    // cv::line(image, points[7], points[4], cv::Scalar(255, 0, 0), 5, 8, 0);
    // for (int i = 0; i < 4; i++) {
    //   cv::line(image, points[i], points[i + 4], cv::Scalar(255, 0, 0), 5, 8,
    //   0);
    // }
    auto GetClolor = [&]() {
      return kColors.at(object_resut.type);
      //   std::uniform_int_distribution bound_distribution(1, 255);
      //   std::uniform_int_distribution bound_distribution1(1, 255);
      //   return cv::Scalar{bound_distribution(rng), bound_distribution(rng),
      //                     bound_distribution(rng)};
    };

    cv::fillConvexPoly(image,
                       std::vector<cv::Point>{points.begin() + 4, points.end()},
                       GetClolor());

    //
    std::vector<cv::Point> temp(points.begin(), points.begin() + 2);
    temp.insert(temp.end(), points.rbegin() + 2, points.rbegin() + 4);
    cv::fillConvexPoly(image, temp, GetClolor());
    //
    temp.clear();
    temp.insert(temp.end(), points.begin() + 2, points.begin() + 4);
    temp.insert(temp.end(), points.rbegin(), points.rbegin() + 2);
    cv::fillConvexPoly(image, temp, GetClolor());
    //
    temp.clear();
    temp.push_back(points[0]);
    temp.push_back(points[3]);
    temp.push_back(points[7]);
    temp.push_back(points[4]);
    cv::fillConvexPoly(image, temp, GetClolor());

    // //

    // //
    temp.clear();
    temp.push_back(points[1]);
    temp.push_back(points[2]);
    temp.push_back(points[6]);
    temp.push_back(points[5]);
    cv::fillConvexPoly(image, temp, GetClolor());
    //
  }

  cv::fillConvexPoly(image,
                     std::vector<cv::Point>{points.begin(), points.begin() + 4},
                     kColors.at(object_resut.type));

  // cv::arrowedLine(
  //     image,
  //     {int(object_resut.direction[0].x()),
  //     int(object_resut.direction[0].y())},
  //     {int(object_resut.direction[1].x()),
  //     int(object_resut.direction[1].y())}, cv::Scalar(255,0,0), 5, 8, 0,1);
  return image;
}
//

RosCompont::RosCompont(ros::NodeHandle *nh) : nh_(nh) {
  tf_broadcaster_ = new tf::TransformBroadcaster;

  point_cloud_pub_ = nh->advertise<sensor_msgs::PointCloud2>("track_point", 2);
  map_point_cloud_pub_ =
      nh->advertise<sensor_msgs::PointCloud2>("map_point", 2);

  markpub_ = nh->advertise<visualization_msgs::MarkerArray>("object_mark", 1);

  pub_mark_points_ =
      nh->advertise<sensor_msgs::PointCloud2>("plan_mark_points", 10);

  pub_mark_points_arrow_ =
      nh->advertise<geometry_msgs::PoseStamped>("plan_arrow", 10);
  pub_path_ = nh->advertise<nav_msgs::Path>("path", 10);
  // pub_local_tracking_result =
  // nh.advertise<sensor_msgs::Image>("local_tracking_result", 1000);
  compressed_image_pub_ = nh->advertise<sensor_msgs::CompressedImage>(
      "local_tracking_result_image/image_raw/compressed", 20);
}
//
RosCompont::~RosCompont() {
  if (tf_broadcaster_) {
    delete tf_broadcaster_;
    tf_broadcaster_ = NULL;
  }
}

void RosCompont::CommpressedImagePub(const sensor_msgs::Image &image) {
  if (compressed_image_pub_.getNumSubscribers() == 0) return;
  sensor_msgs::CompressedImage image_up;
  image_up.format = "bgr8";
  image_up.format += "; jpeg compressed ";
  image_up.format += "bgr8";
  boost::shared_ptr<compressed_image_transport::CompressedPublisher>
      tracked_object;
  std::string targetFormat = "bgr8";
  cv_bridge::CvImageConstPtr cv_ptr =
      cv_bridge::toCvShare(image, tracked_object, targetFormat);
  std::vector<int> params;
  params.resize(9, 0);
  params[0] = cv::IMWRITE_JPEG_QUALITY;
  params[1] = 0.2;
  params[2] = cv::IMWRITE_JPEG_PROGRESSIVE;
  params[3] = 1;
  params[4] = cv::IMWRITE_JPEG_OPTIMIZE;
  params[5] = 1;
  params[6] = cv::IMWRITE_JPEG_RST_INTERVAL;
  params[7] = 1;

  std::vector<uint8_t> encodeing;
  cv::imencode(".jpg", cv_ptr->image, image_up.data);
  image_up.header = image.header;
  compressed_image_pub_.publish(image_up);
}
//
void RosCompont::OnMapPointsCallback(
    const std::vector<Eigen::Vector3d> &points,
    const transform::Rigid3d &local_to_global) {
  sensor_msgs::PointCloud point_cloud;
  sensor_msgs::PointCloud2 point_cloud2;
  for (auto point : points) {
    point = local_to_global * point;
    geometry_msgs::Point32 geo_point;
    geo_point.x = point.x();
    geo_point.y = point.y();
    geo_point.z = point.z();
    point_cloud.points.push_back(geo_point);
  }
  point_cloud.header.frame_id = "map";
  point_cloud.header.stamp = ros::Time::now();
  sensor_msgs::convertPointCloudToPointCloud2(point_cloud, point_cloud2);
  point_cloud_pub_.publish(point_cloud2);
}
//
void RosCompont::PubPointPlan(const std::vector<Eigen::Vector3f> &mark_points,
                              const Eigen::Vector4f &plan_vector,
                              const Eigen::Vector3f &centriod) {
  sensor_msgs::PointCloud point_cloud;
  sensor_msgs::PointCloud2 point_cloud2;
  for (auto point : mark_points) {
    geometry_msgs::Point32 geo_point;
    geo_point.x = point.x();
    geo_point.y = point.y();
    geo_point.z = point.z();
    point_cloud.points.push_back(geo_point);
  }
  point_cloud.header.frame_id = "map";
  point_cloud.header.stamp = ros::Time::now();
  sensor_msgs::convertPointCloudToPointCloud2(point_cloud, point_cloud2);
  pub_mark_points_.publish(point_cloud2);
  // Eigen::Quaternionf rotation =
  //     transform::AngleAxisVectorToRotationQuaternion<float>(
  //         vector.head<3>().normalized());
  Eigen::Quaternionf rotation ;
  // Eigen::Quaternionf rotation = Eigen::Quaternionf::FromTwoVectors(
  //     Eigen::Vector3f::UnitX(), plan_vector.head<>(3));
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ::ros::Time::now();
  pose.header.frame_id = "map";

  pose.pose.position.x = centriod.x();
  pose.pose.position.y = centriod.y();
  pose.pose.position.z = centriod.z();
  pose.pose.orientation.w = rotation.w();
  pose.pose.orientation.x = rotation.x();
  pose.pose.orientation.y = rotation.y();
  pose.pose.orientation.z = rotation.z();
  pub_mark_points_arrow_.publish(pose);
  // static visualization_msgs::Marker points_mark;
  // points_mark.header.frame_id = "map";
  // points_mark.ns = "plane_points_normal";
  // points_mark.header.stamp = ::ros::Time::now();
  // static int index = 0;
  // points_mark.lifetime = ros::Duration(0);
  // points_mark.id = 0;
  // points_mark.action = visualization_msgs::Marker::ADD;
  // points_mark.type = visualization_msgs::Marker::ARROW;

  // points_mark.pose.orientation.w = rotation.w();
  // points_mark.pose.orientation.x = rotation.x();
  // points_mark.pose.orientation.y = rotation.y();
  // points_mark.pose.orientation.z = rotation.z();
  // points_mark.pose.position.x = centriod.x();
  // points_mark.pose.position.y = centriod.y();
  // points_mark.pose.position.z = centriod.z();
  // points_mark.scale.x = 0.05;
  // points_mark.scale.y = 0.05;
  // points_mark.scale.z = 0.25;
  // points_mark.color.a = 1;
  // points_mark.color.r = 1;
  // points_mark.color.b = 1;
  // points_mark.color.g = 0;
  // pub_mark_points_arrow_.publish(points_mark);
}

//
void RosCompont::MarkPub(
    std::map<int, std::vector<object::ObjectImageResult>> &t) {
  visualization_msgs::MarkerArray marks;
  int index = 0;
  for (const auto &t2 : t) {
    transform::Rigid3d t1 = t2.second[0].global_pose_cam;
    std::stringstream info;
    if (t2.second.size() != 1) {
      auto deta_pose =
          t2.second[0].global_pose_cam.inverse() * t2.second[1].global_pose_cam;
      info << "err x = " << deta_pose.translation().x() * 100 << "\n"
           << "y = " << deta_pose.translation().y() * 100 << "\n"
           << "z = " << deta_pose.translation().z() * 100 << std::endl;
      info << "angle = " << common::RadToDeg(transform::GetAngle(deta_pose));
      for (int i = 0; i < 2; i++) {
        t1 = t2.second[i].global_pose_cam;
        visualization_msgs::Marker mark;
        mark.header.frame_id = "map";
        mark.ns = "object";
        mark.header.stamp = ::ros::Time::now();

        mark.id = ++index;
        mark.action = visualization_msgs::Marker::ADD;
        mark.type = visualization_msgs::Marker::CUBE;
        mark.pose.position.x = t1.translation().x();
        mark.pose.position.y = t1.translation().y();
        mark.pose.position.z = t1.translation().z();
        mark.pose.orientation.w = t1.rotation().w();
        mark.pose.orientation.x = t1.rotation().x();
        mark.pose.orientation.y = t1.rotation().y();
        mark.pose.orientation.z = t1.rotation().z();
        // mark.type = visualization_msgs::Marker::ARROW;
        mark.lifetime = ros::Duration(0);
        mark.scale.x = 0.3;
        mark.scale.y = 0.3;
        mark.scale.z = 0.3;
        mark.color.a = 1;
        mark.color.r = 1;
        mark.color.b = 1;
        mark.color.g = 0;
        marks.markers.push_back(mark);
        if (t2.second[i].type == "global") {
          mark.pose.position.z += 0.4;
          mark.pose.position.y += 0.4;
          mark.scale.x = 0.1;
          mark.scale.y = 0.1;
          mark.scale.z = 0.1;
          mark.text = info.str();
          mark.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
          marks.markers.push_back(mark);
        }
      }
    }

    visualization_msgs::Marker mark;
    mark.header.frame_id = "map";
    mark.ns = "object";
    mark.header.stamp = ::ros::Time::now();
    if (!info.str().empty()) {
      mark.text = info.str();
    }
    mark.id = ++index;
    mark.action = visualization_msgs::Marker::ADD;
    mark.type = visualization_msgs::Marker::CUBE;
    mark.pose.position.x = t1.translation().x();
    mark.pose.position.y = t1.translation().y();
    mark.pose.position.z = t1.translation().z();
    mark.pose.orientation.w = t1.rotation().w();
    mark.pose.orientation.x = t1.rotation().x();
    mark.pose.orientation.y = t1.rotation().y();
    mark.pose.orientation.z = t1.rotation().z();
    // mark.type = visualization_msgs::Marker::ARROW;
    mark.lifetime = ros::Duration(0);
    mark.scale.x = 0.3;
    mark.scale.y = 0.3;
    mark.scale.z = 0.3;
    mark.color.a = 1;
    mark.color.r = 1;
    mark.color.b = 1;
    mark.color.g = 0;
    marks.markers.push_back(mark);
  }

  // mark.pose.position.x = p.x();
  // mark.pose.position.y = p.y();
  // mark.pose.position.z = p.z();
  // for (const auto &p : t) {
  //   geometry_msgs::Point point;
  //   point.x = p.x();
  //   point.y = p.y();
  //   point.z = p.z();
  //   mark.points.push_back(point);
  // }
  markpub_.publish(marks);
}

//
void RosCompont::OnLocalTrackingResultCallback(
    const TrackingData &tracking_data,
    std::vector<object::ObjectImageResult> *object_result,
    const transform::Rigid3d &local_to_global) {
  auto image_result = GenerateImageWithKeyPoint(
      *tracking_data.data->image, tracking_data.data->key_points, {},
      cv::Mat(), {},
      "pre_imag", "curr_imag",{0} );

  std::vector<transform::Rigid3d> mark_pose;
  std::map<int, std::vector<object::ObjectImageResult>> same_marks;
  cv::Mat image_object(image_result.size(), CV_8UC3, cv::Scalar::all(0));
  if (object_result!=nullptr&& !object_result->empty()) {
    for (const auto &result : *object_result) {
      same_marks[result.id].push_back(result);
      if (result.coners.empty()) continue;
      image_object += ObjectToCvImage(
          Eigen::AlignedBox2d(Eigen::Vector2d{0, 0},
                              Eigen::Vector2d{
                                  tracking_data.data->image->cols,
                                  tracking_data.data->image->rows,
                              }),
          image_result.size(), result);
      mark_pose.push_back(result.global_pose_cam);
    }
    MarkPub(same_marks);
  }
  image_result += image_object;
  // for (int i = 0; i < image_object.rows; i++) {
  //   for (int j = 0; j < image_object.cols; j++) {
  //     if (image_object.ptr(i, j)[0]) {
  //       image_result.ptr(i, j)[0] = image_object.ptr(i, j)[0];
  //       image_result.ptr(i, j)[1]= image_object.ptr(i, j)[1];
  //       image_result.ptr(i, j)[2]= image_object.ptr(i, j)[2];
  //     }
  //   }
  // }

  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time::now();
  sensor_msgs::ImagePtr img =
      cv_bridge::CvImage(header, "bgr8", image_result).toImageMsg();
  // pub_local_tracking_result.publish(img);
  CommpressedImagePub(*img);
  OnMapPointsCallback(tracking_data.data->tracking_map_points, local_to_global);
}
//

void RosCompont::PubMapPoints(const std::vector<Eigen::Vector3d> &points) {
  sensor_msgs::PointCloud point_cloud;
  sensor_msgs::PointCloud2 point_cloud2;
  for (auto point : points) {
    geometry_msgs::Point32 geo_point;
    geo_point.x = point.x();
    geo_point.y = point.y();
    geo_point.z = point.z();
    point_cloud.points.push_back(geo_point);
  }
  point_cloud.header.frame_id = "map";
  point_cloud.header.stamp = ros::Time::now();
  sensor_msgs::convertPointCloudToPointCloud2(point_cloud, point_cloud2);
  map_point_cloud_pub_.publish(point_cloud2);
}
//
void RosCompont::PosePub(const transform::Rigid3d &pose,
                         const transform::Rigid3d &local_to_global) {
  ::geometry_msgs::TransformStamped tf_trans;
  ::geometry_msgs::TransformStamped global_tf_trans;
  const Eigen::Quaterniond &q = pose.rotation();
  tf_trans.header.stamp = ros::Time::now();
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

  global_tf_trans.header.stamp = ros::Time::now();
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
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.header.frame_id = "map";
  pose_stamped.pose.position.x = tf_trans.transform.translation.x;
  pose_stamped.pose.position.y = tf_trans.transform.translation.y;
  pose_stamped.pose.position.z = tf_trans.transform.translation.z;

  path_.header.frame_id = "map";
  if (pub_path_.getNumSubscribers() != 0) {
    path_.poses.push_back(pose_stamped);
    pub_path_.publish(path_);
  } else {
    path_.poses.clear();
  }
}

//

//
//

}  // namespace jarvis_ros
   //