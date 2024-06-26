
#include <dirent.h>
#include <sys/types.h>

#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "fstream"
#include "jarvis/sensor/data_process.h"
#include "jarvis/sensor/stereo_sync.h"
#include "jarvis/trajectory_builder.h"
#include "rclcpp/rclcpp.hpp"
#include "ros_component.h"
#include "std_msgs/msg/string.hpp"
#include "unistd.h"
//
#include "jarvis/common/time.h"
#include "glog/logging.h"
#include "jarvis/transform/rigid_transform.h"
#include "pose_optimization.h"
//

namespace {
using namespace jarvis;
Eigen::Vector3d LatLongAltToEcef(const double latitude, const double longitude,
                                 const double altitude) {
  // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates
  constexpr double a = 6378137.;  // semi-major axis, equator to center.
  constexpr double f = 1. / 298.257223563;
  constexpr double b = a * (1. - f);  // semi-minor axis, pole to center.
  constexpr double a_squared = a * a;
  constexpr double b_squared = b * b;
  constexpr double e_squared = (a_squared - b_squared) / a_squared;
  const double sin_phi = std::sin(jarvis::common::DegToRad(latitude));
  const double cos_phi = std::cos(jarvis::common::DegToRad(latitude));
  const double sin_lambda = std::sin(jarvis::common::DegToRad(longitude));
  const double cos_lambda = std::cos(jarvis::common::DegToRad(longitude));
  const double N = a / std::sqrt(1 - e_squared * sin_phi * sin_phi);
  const double x = (N + altitude) * cos_phi * cos_lambda;
  const double y = (N + altitude) * cos_phi * sin_lambda;
  const double z = (b_squared / a_squared * N + altitude) * sin_phi;

  return Eigen::Vector3d(x, y, z);
}

jarvis::transform::Rigid3d ComputeLocalFrameFromLatLong(
    const double latitude, const double longitude) {
  const Eigen::Vector3d translation = LatLongAltToEcef(latitude, longitude, 0.);
  const Eigen::Quaterniond rotation =
      Eigen::AngleAxisd(jarvis::common::DegToRad(latitude - 90.),
                        Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(jarvis::common::DegToRad(-longitude),
                        Eigen::Vector3d::UnitZ());
  return jarvis::transform::Rigid3d(rotation * -translation, rotation);
}
//

using namespace jarvis;
//
// /
struct Pose {
  uint64_t time;
  uint64_t local_time;
  Eigen::Vector3d p;
  Eigen::Quaterniond q;
};
struct RtkData {
  uint64_t time;
  uint64_t local_time;
  double latitude;
  double longitude;
  double altitude;
};

//
double Minute2Degree(double minute){
    return minute / static_cast<double>(60.0);
}

double ToDeg(const std::string& temp) {
      // standard_lla[0] = (std::trunc(lla[0]/100.0)) +
      //       (Minute2Degree(lla[0] - (std::trunc(lla[0]/100.0)*100.0)));
  // return std::stod(temp);
  auto it = temp.find_first_of('.');
  // std::string s = temp.substr(it + 1, temp.size());
  std::string m = temp.substr(it - 2, temp.size());
  std::string d = temp.substr(0, it - 2);
  // LOG(INFO) << d << " " << m << " "  ;
  return std::stod(d) + std::stod(m) / 60. ;
}
std::optional<jarvis::transform::Rigid3d> kEcefToLocalFrame;
//
std::istringstream& operator>>(std::istringstream& ifs, RtkData& rtk_data) {
  std::string name;
  ifs >> name;
  if (name == "rtk") {
    ifs >> rtk_data.time >> rtk_data.local_time;
    int unuse_cout;
    ifs >> unuse_cout;
    std::string lat,log,alt;
    ifs >> lat >> log >> alt;
    rtk_data.altitude = stod(alt);
    rtk_data.longitude = ToDeg(log);
    rtk_data.latitude =ToDeg(lat);
    return ifs;
  }

  throw "not rtk";
  return ifs;
}
//
std::istringstream& operator>>(std::istringstream& ifs, Pose& pose) {
  ifs >> pose.time >> pose.local_time;
  ifs >> pose.p.x() >> pose.p.y() >> pose.p.z() >> pose.q.x() >> pose.q.y() >>
      pose.q.z() >> pose.q.w();
  return ifs;
}

template <typename TypeName>
std::vector<TypeName> ReadFile(const std::string& txt) {
  std::ifstream file;
  LOG(INFO)<<"Open : "<<txt;
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
      result.push_back(data);
    } catch (...) {
    }
  }
  file.close();
  LOG(INFO) << "done";
  return result;
}
//
jarvis::transform::Rigid3d RtkToPose(const RtkData& data) {
  if (!kEcefToLocalFrame.has_value()) {
    kEcefToLocalFrame =
        ComputeLocalFrameFromLatLong(data.latitude, data.longitude);
    LOG(INFO) << "Using NavSatFix. Setting ecef_to_local_frame with lat = "
              << data.latitude << ", long = " << data.longitude << ".";
  }
  return transform::Rigid3d::Translation(
      kEcefToLocalFrame.value() *
      LatLongAltToEcef(data.latitude, data.longitude, data.altitude-43));
}
//
std::vector<Pose> RtkToPose(const std::vector<RtkData>& datas) {
  std::vector<Pose> result;
  for (const auto& d : datas) {

    auto r = RtkToPose(d);
    LOG(INFO)<<r<<d.time;
    result.push_back(Pose{d.time*1000, d.local_time*1000, r.translation(), r.rotation()});
  }
  return result;
}

std::unique_ptr<jarvis::transform::Rigid3d> Alignment(
    const std::vector<Pose>& vio_data, const std::vector<Pose>& rt_data,
    int lenth = 100) {
  jarvis_pic::PoseOptimization pose_alignment(
      jarvis_pic::PoseOptimizationOption{lenth});
  int l = 0;
  for (int i = 0; pose_alignment.PoseSize() < lenth; i++) {
    l++;
    pose_alignment.AddPose(jarvis_pic::PoseData{
        common::FromUniversal(static_cast<int64_t>(vio_data[i].time / 100)),
        transform::Rigid3d(vio_data[i].p, vio_data[i].q)

    });
  }
  for (const auto& d : rt_data) {
    pose_alignment.AddFixedFramePoseData(jarvis::sensor::FixedFramePoseData{
        common::FromUniversal(d.time / 100), transform::Rigid3d(d.p, d.q)});
    if (d.time > vio_data[l].time) break;
  }
  //
  return pose_alignment.AlignmentOptimization();
}

//

void PubPoseWithMark(rclcpp::Node* nh,
                     const std::map<std::string, std::vector<Pose>>& poses) {
  //
  //
  auto pose_mark_publisher =
      nh->create_publisher<visualization_msgs::msg::MarkerArray>("poses", 10);
  //
  visualization_msgs::msg::MarkerArray marks;
  std::default_random_engine e;

  int mark_id = 0;
  for (const auto& pose_with_name : poses) {
    visualization_msgs::msg::Marker mark;
    mark.header.frame_id = "map";
    mark.ns = pose_with_name.first.c_str();
    mark.header.stamp = rclcpp::Time();
    mark.id = mark_id++;
    mark.action = visualization_msgs__msg__Marker__ADD;
    mark.type = visualization_msgs__msg__Marker__POINTS;
    // mark.type = visualization_msgs::Marker::ARROW;
    // mark.lifetime = rclcpp::Duration(0);
    mark.scale.x = 0.1;
    mark.scale.y = 0.1;
    mark.scale.z = 0.1;
    std::uniform_real_distribution<float> ran(0, 1);
    mark.color.r = 1;       // ran(e);//1.0;
    mark.color.a = 1;       // ran(e);
    mark.color.g = ran(e);  //(mark_id / sizeofils);
    mark.color.b = ran(e);  //(sizeofils- mark_id) / sizeofils;
    // LOG(INFO)<<mark.color.g<<mark.color.b;
    int cnt = 0;
    //
    for (const auto& pose : pose_with_name.second) {
      geometry_msgs::msg::Point point;
      point.x = pose.p.x();
      point.y = pose.p.y();
      point.z = pose.p.z();
      mark.points.push_back(point);
    }
    marks.markers.push_back(mark);
  }
  //
  rclcpp::Rate rate(10);
  while (rclcpp::ok) {
    rate.sleep();
    pose_mark_publisher->publish(marks);
  }
}

}  // namespace
//

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  //
  //
  CHECK_EQ(argc, 3) << "Please input vio data,rtk data";
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("jarvis_ros2_pose_alignment");
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  //
  std::string vio_pose_file(argv[1]);
  std::string rtk_pose_file(argv[2]);
  auto vio_data = ReadFile<Pose>(vio_pose_file);
  auto rtk_data = RtkToPose(ReadFile<RtkData>(rtk_pose_file));
  //
  auto local_to_rtk_transform = Alignment(vio_data, rtk_data);
  CHECK(local_to_rtk_transform) << "  Alignment Failed..";

  LOG(INFO) << *local_to_rtk_transform;
  for (auto& p : vio_data) {
  auto correct_pose = (*local_to_rtk_transform).inverse() * transform::Rigid3d(p.p, p.q);
    p.p = correct_pose.translation();
    p.q = correct_pose.rotation();
  }
  //
  std::map<std::string, std::vector<Pose>> poses{
      {"vio_poses", std::move(vio_data)}, {"rtk_poses", std::move(rtk_data)}};
  PubPoseWithMark(node.get(),poses);
  LOG(INFO) << "Done";
  return 0;
}