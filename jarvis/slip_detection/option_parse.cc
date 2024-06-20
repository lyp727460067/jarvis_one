#include "slip_detection/option_parse.h"

#include <jarvis/estimator/featureTracker/feature_tracker.h>

#include <opencv2/core/eigen.hpp>

#include "slip_detection/simple_vo.h"
#include "slip_detection/slip_detect.h"
namespace slip_detect {

//
void ParseYAMLOptionSimpleVoOption(cv::FileStorage *fs,
                                   SimpleVoOption *simple_vo_option) {
  auto &fsSettings = *fs;
  simple_vo_option->min_track_num = fsSettings["min_track_num"];
  simple_vo_option->min_pnp_inlier_num = fsSettings["min_pnp_inlier_num"];
  simple_vo_option->min_track_num = fsSettings["min_track_num"];
  cv::Mat cv_T;
  fsSettings["body_T_cam0"] >> cv_T;
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  cv::cv2eigen(cv_T, T);
  auto cam_to_imu = jarvis::transform::Rigid3d(
      T.block<3, 1>(0, 3), Eigen::Quaterniond(T.block<3, 3>(0, 0)));
  fsSettings["body_T_cam1"] >> cv_T;
  cv::cv2eigen(cv_T, T);

  auto cam1_to_imu = jarvis::transform::Rigid3d(
      T.block<3, 1>(0, 3), Eigen::Quaterniond(T.block<3, 3>(0, 0)));
  simple_vo_option->tracker_option.image_size =
      Eigen::Vector2i(fsSettings["image_width"], fsSettings["image_height"]);
  simple_vo_option->tracker_option.extric = cam_to_imu.inverse() * cam1_to_imu;
  //
}
//
void ParseYAMLOptionSlipDetectOption(cv::FileStorage *fs,
                                     SlipDetectOption *slip_detection_opiont) {
  auto &fsSettings = *fs;
  slip_detection_opiont->type = fsSettings["type"];
  slip_detection_opiont->min_disparity_num = fsSettings["min_disparity_num"];
  slip_detection_opiont->max_disparity = fsSettings["max_disparity"];
  slip_detection_opiont->zero_velocity_odom_delte_s_threash_hold =
      fsSettings["zero_velocity_odom_delte_s_threash_hold"];
  slip_detection_opiont->pose_odom_err_s_threash_hold =
      fsSettings["pose_odom_err_s_threash_hold "];
}
//
void ParseYAMLOptionFetureOption(
    cv::FileStorage *fs,
    jarvis::estimator::FeatureTrackerOption *feature_option, std::string file) {
  auto &fsSettings = *fs;
  std::string cam0Calib;
  fsSettings["cam0_calib"] >> cam0Calib;
  int pn = file.find_last_of('/');
  std::string configPath = file.substr(0, pn);
  std::string cam0Path = configPath + "/" + cam0Calib;
  feature_option->calib_file.push_back(cam0Path);
  std::string cam1Calib;
  fsSettings["cam1_calib"] >> cam1Calib;
  std::string cam1Path = configPath + "/" + cam1Calib;
  feature_option->calib_file.push_back(cam1Path);
  feature_option->pyrmid_option.layer = fsSettings["lk_pre_max_layer"];
  feature_option->pyrmid_option.lk_win_size = fsSettings["lk_win_size"];
  feature_option->pyrmid_option.image_size =
      Eigen::Vector2i(fsSettings["image_width"], fsSettings["image_height"]);
  feature_option->track_back = fsSettings["flow_back"];
}

void ParseYAMLOption(const std::string &file, void *option) {
  FILE *fh = fopen(file.c_str(), "r");
  if (fh == NULL) {
    LOG(FATAL) << "config_file dosen't exist; wrong config_file path" << file;
    return;
  }
  fclose(fh);
  cv::FileStorage fsSettings(file, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    LOG(FATAL) << "ERROR: Wrong path to settings";
  }

  SimpleVoOption *simple_vo_option = static_cast<SimpleVoOption *>(option);
  if (simple_vo_option) {
    ParseYAMLOptionSimpleVoOption(&fsSettings, simple_vo_option);
    return;
  }
  SlipDetectOption *slip_detection_opiont =
      static_cast<SlipDetectOption *>(option);

  if (slip_detection_opiont) {
    ParseYAMLOptionSlipDetectOption(&fsSettings, slip_detection_opiont);
    return;
  }
  jarvis::estimator::FeatureTrackerOption *feate_opiont =
      static_cast<jarvis::estimator::FeatureTrackerOption *>(option);

  if (feate_opiont) {
    ParseYAMLOptionFetureOption(&fsSettings, feate_opiont, file);
    return;
  }
}
}  // namespace slip_detect