
#include <jarvis/estimator/featureTracker/feature_tracker.h>
#include <slip_detection/slip_detect.h>

#include <opencv2/core/eigen.hpp>
#include <slip_detection/simple_vo.h>
#include "jarvis/option_parse.h"
#include "opencv2/opencv.hpp"
#include "yaml.h"
#include "jarvis/option_parse.h"
namespace jarvis {

namespace slip_detect {
constexpr int kCameraNum = 2;

bool CheckFileExist(const std::string &file) {
  FILE *fh = fopen(file.c_str(), "r");
  if (fh == NULL) {
    return false;
  }
  return true;
}

//
//

//
void ParseYAMLOptionSlipDetectOption(const std::string &file,
                                     SlipDetectOption *slip_detection_opiont) {
  

  CHECK(CheckFileExist(file)) << file << "not exist.";
  
  cv::FileStorage vlsam_fsSettings(file, cv::FileStorage::READ);
  if (!vlsam_fsSettings.isOpened()) {
    LOG(FATAL) << "ERROR: Wrong path to settings";
  }
  const std::string slipe_name = vlsam_fsSettings["slip_option"];
  int pn =file.find_last_of('/');
  const std::string configPath =file.substr(0, pn);
  auto slip_detect_option_file = configPath + "/" + slipe_name;
  CHECK(CheckFileExist(slip_detect_option_file))
      << slip_detect_option_file << "not exist.";

  LOG(INFO)<<"open "<<slip_detect_option_file;
  //
  cv::FileStorage fsSettings(slip_detect_option_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    LOG(FATAL) << "ERROR: Wrong path to settings";
  }

  std::string cali_path = vlsam_fsSettings["calibrate_path"];
  jarvis::CalibrateOption calibrate_options;
  jarvis::ParseYAMLOption<jarvis::CalibrateOption>(cali_path,
                                                   &calibrate_options);
  slip_detection_opiont->type = fsSettings["type"];
  slip_detection_opiont->min_disparity_num = fsSettings["min_disparity_num"];
  slip_detection_opiont->max_disparity = fsSettings["max_disparity"];
  slip_detection_opiont->que_time_duration = fsSettings["que_time_duration"];
  slip_detection_opiont->zero_velocity_odom_delte_s_threash_hold =
      fsSettings["zero_velocity_odom_delte_s_threash_hold"];
  slip_detection_opiont->pose_odom_err_s_threash_hold =
      fsSettings["pose_odom_err_s_threash_hold"];
  slip_detection_opiont->pose_odom_err_theta_threash_hold =
      fsSettings["pose_odom_err_theta_threash_hold"];
  slip_detection_opiont->transform_cam_to_odom =
      calibrate_options.extric_camera_to_robot;
}

}  // namespace slip_detect

template <>
void ParseYAMLOption(const std::string &file,
                     slip_detect::SimpleVoOption *option) {
  // auto fsSettings = CheckFile(file);
  // LOG(INFO)<<"ParseYAMLOptionSimpleVoOption ";
  // ParseYAMLOptionSimpleVoOption(&fsSettings, option);
}
//
template <>
void ParseYAMLOption(const std::string &file,
                     slip_detect::SlipDetectOption *option) {
  ParseYAMLOptionSlipDetectOption(file, option);
}
//
template <>
void ParseYAMLOption(const std::string &file,
                     estimator::FeatureTrackerOption *option) {}

}  // namespace jarvis