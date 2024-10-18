#ifndef _JARVIS_OPTION_PARSE_
#define _JARVIS_OPTION_PARSE_
#include <string>
#include "yaml-cpp/yaml.h"
#include "opencv2/opencv.hpp"
#include "transform/transform.h"
namespace jarvis {

struct CheckNode : public YAML::Node {
  using YAML::Node::Node;
  CheckNode(const YAML::Node &node) : Node(node) {}
  CheckNode(const YAML::Node &node, const std::string &key)
      : Node(node), key_(key) {}
  template <typename Key>
  const CheckNode operator[](const Key &key) const {
    key_ = std::string(key);
    return CheckNode(Node::operator[](key), key_);
  }

  template <typename Key>
  CheckNode operator[](const Key &key) {
    key_ = std::string(key);
    return CheckNode(Node::operator[](key), key_);
  }
  template <typename T>
  inline T as() const {
    try {
      return Node::as<T>();
    } catch (...) {
      LOG(FATAL) << "[" << key_ << "]"
                 << " not in the yaml_file or check key whitespace";
    }
    return Node::as<T>();
  }
  mutable std::string key_;
};
struct ImuOption {
  int frequency = 100;
  double gravity_normal = 9.81;
  bool use_imu = true;
  bool do_rectify = false;
  Eigen::Matrix3d gyr_A= Eigen::Matrix3d::Zero();
  Eigen::Vector3d gyr_b{0,0,0};
  Eigen::Matrix3d acc_A= Eigen::Matrix3d::Zero();;
  Eigen::Vector3d acc_b{0,0,0};
  struct Noise {
    double na{0};
    double ng{0};
    double nba{0};
    double nbg{0};
    double na2{0};
    double ng2{0};
    double nba2{0};
    double nbg2{0};
  } imu_noise;
  std::string DebugInfo()const {
    std::stringstream info;
    info << "\n"
         << "imu " << " frequency: " << frequency
         << " constant gravity: " << gravity_normal << "\n"
         << " na: " << imu_noise.na << " ng: " << imu_noise.ng
         << " nba : " << imu_noise.nba << " nbg:" << imu_noise.nbg << "\n"
         << " na2 " << imu_noise.na2 << " ng2" << imu_noise.ng2
         << " nba2 :" << imu_noise.nba2 << " nbg2:" << imu_noise.nbg2
         << " gyr_A :" << gyr_A << "\n"
         << " gyr_b :" << gyr_b << "\n"
         << " acc_A :" << acc_A << "\n"
         << " acc_b :" << acc_b;
    return info.str();
  }
};
//
struct CameraOption {
  std::string name;
  std::string camera_model;
  std::string distortion_model;
  Eigen::Vector2i resolution;
  std::vector<double> intrinsics;
  std::vector<double> distortions;
  static cv::Mat IntrinsicsPinhol(const std::vector<double> &k) {
    return (cv::Mat_<double>(3, 3) << k[0], 0, k[2], 0, k[1], k[3], 0, 0, 1);
  }
  std::string DebugInfo() {
    std::stringstream info;
    info << "\n"
         << name << " resolution: " << resolution.transpose() << "\n"
         << "model:" << camera_model
         << ". distortion_model: " << distortion_model << ". "
         << "fx: " << intrinsics[0] << " fy: " << intrinsics[1]
         << " cx: " << intrinsics[2] << " cy" << intrinsics[3] << std::endl;
    return info.str();
  }
};

struct CalibrateOption {
  std::vector<CameraOption> camera_options;
  std::vector<transform::Rigid3d> extric_camera_to_imu;
  transform::Rigid3d extric_camera_to_robot;
};

template <typename Option>
void ParseYAMLOption(const std::string &file, Option *option);
//
// template <typename Option>
// void ParseYAMLOption(const std::string &file, Option *option,
//                      const CalibrateOption &calib_option);

}  // namespace jarvis
#endif
