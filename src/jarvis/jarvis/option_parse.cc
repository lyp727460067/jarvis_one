
#include <jarvis/estimator/featureTracker/feature_tracker.h>
#include "jarvis/option_parse.h"
#include "opencv2/core/eigen.hpp"
#include "yaml-cpp/yaml.h"
#include "opencv2/opencv.hpp"
#include "jarvis/estimator/estimator.h"
namespace jarvis {

constexpr int kCameraNum =2;

bool CheckFileExist(const std::string &file) {
  FILE *fh = fopen(file.c_str(), "r");
  if (fh == nullptr){
    return false;
  }
  fclose(fh);
  return true;
}
//
cv::FileStorage CheckFile(const std::string &file) {
  if (!CheckFileExist(file)) {
    LOG(FATAL) << file << " not exist.";
  }
  cv::FileStorage fsSettings(file, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    LOG(FATAL) << file << "PERROR: Wrong path to settings";
  }
  return fsSettings;
}
CameraOption ParseYAMLOptionCameraOption(const CheckNode &paras, int i) {
  const CheckNode cam_node = paras["cam" + std::to_string(i)];
  CameraOption camera_option;
  camera_option.name = "cam" + std::to_string(i);
  camera_option.distortions =
      cam_node["distortion_coeffs"].as<std::vector<double>>();
  camera_option.intrinsics = cam_node["intrinsics"].as<std::vector<double>>();
  camera_option.camera_model = cam_node["camera_model"].as<std::string>();
  camera_option.distortion_model =
      cam_node["distortion_model"].as<std::string>();
  camera_option.resolution = {cam_node["resolution"].as<std::vector<int>>()[0],
                              cam_node["resolution"].as<std::vector<int>>()[1]};
  return camera_option;
}
template<>
void ParseYAMLOption(const std::string &file_path,
                     CalibrateOption *calibrate_options) {
  const std::string cam_chain_file = file_path + "/camchain-imucam.yaml";
  const std::string config_file = file_path + "/config.yml";
  CHECK(CheckFileExist(cam_chain_file)) << cam_chain_file << " not exist.";
  CHECK(CheckFileExist(config_file)) << config_file << " not exist.";
  {
    LOG(INFO) << "Start parse " << cam_chain_file;
    CheckNode paras = YAML::LoadFile(cam_chain_file);

    for (int i = 0; i < kCameraNum; i++) {
      calibrate_options->camera_options.push_back(
          ParseYAMLOptionCameraOption(paras, i));
      //
      const CheckNode cam_node = paras["cam" + std::to_string(i)];
      const std::vector<std::vector<double>> camera_to_imu_vector =
          cam_node["T_imu_cam"].as<std::vector<std::vector<double>>>();
      //

      Eigen::Matrix4d camera_to_imu;
      for (int i = 0; i < 4; i++) {
        camera_to_imu.row(i) = Eigen::Vector4d(camera_to_imu_vector[i].data());
      }
      calibrate_options->extric_camera_to_imu.push_back(transform::Rigid3d(
          camera_to_imu.block<3, 1>(0, 3),
          Eigen::Quaterniond(camera_to_imu.block<3, 3>(0, 0))));
      LOG(INFO) << calibrate_options->camera_options.back().DebugInfo();
      LOG(INFO) << "imu_to_cam:"
                << calibrate_options->extric_camera_to_imu.back();
    }
  }
  {
    LOG(INFO) << "Start parse " << cam_chain_file;
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
      LOG(FATAL) << "ERROR: Wrong path to settings";
    }
    cv::Mat cv_T;
    fsSettings["cam2RobotT"] >> cv_T;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    cv::cv2eigen(cv_T, T);
    calibrate_options->extric_camera_to_robot = jarvis::transform::Rigid3d(
        T.block<3, 1>(0, 3), Eigen::Quaterniond(T.block<3, 3>(0, 0)));
  }
}

void ParseYAMLOptionImuOption(cv::FileStorage *fs, jarvis::ImuOption *option,
                              const CalibrateOption &camera_option) {
  //
  auto &fsSettings = *fs;
  option->imu_noise.na = fsSettings["acc_n"];
  option->imu_noise.ng = fsSettings["gyr_n"];
  option->imu_noise.nba = fsSettings["acc_w"];
  option->imu_noise.nbg = fsSettings["gyr_w"];
  option->imu_noise.na2 = option->imu_noise.na * option->imu_noise.na;
  option->imu_noise.ng2 = option->imu_noise.ng * option->imu_noise.ng;
  option->imu_noise.nba2 = option->imu_noise.nba * option->imu_noise.nba;
  option->imu_noise.nbg2 = option->imu_noise.nbg * option->imu_noise.nbg;

  LOG(INFO) << option->DebugInfo();
}
//



//
void ParseYAMLOptionFetureOption(
    cv::FileStorage *fs,
    jarvis::estimator::FeatureTrackerOption *feature_option,
    const CalibrateOption &camera_option, const std::string &file) {
  //
  auto &fsSettings = *fs;
  //
  for (size_t i = 0; i < camera_option.camera_options.size(); i++) {
    camera_models::CameraPtr camera =
        camera_models::CameraFactory::instance()->GenerateCameraFromOption(
            camera_option.camera_options[i]);
    feature_option->cameras.push_back(camera);
  }
  //
  feature_option->pyrmid_option.image_size =
      camera_option.camera_options[0].resolution;

  //
  feature_option->pyrmid_option.layer = fsSettings["lk_pre_max_layer"];
  feature_option->pyrmid_option.lk_win_size = fsSettings["lk_win_size"];

  feature_option->track_back = fsSettings["flow_back"];
  feature_option->max_feat_cnt = fsSettings["max_cnt"];
  feature_option->feature_detect_option.min_distance = fsSettings["min_dist"];
  feature_option->feature_detect_option.mask_min_dist = fsSettings["mask_min_dist"];
  feature_option->feature_detect_option.fast_thresh_hold =
      fsSettings["fast_th"];
  feature_option->feature_detect_option.imag_size =
      camera_option.camera_options[0].resolution;
  LOG(INFO)<<camera_option.camera_options[0].resolution;
  // feature_option->feature_detect_option.grid_size=
  //     camera_option.camera_options[0].resolution;

//   feature_option->calibrate_option = camera_option;
  std::string mask_id;
  fsSettings["mask_id"] >> mask_id;
  int pn = file.find_last_of('/');
  std::string configPath = file.substr(0, pn);
  auto mask_file = configPath + "/" + mask_id;

  LOG(INFO) << "Mask file:  " << mask_file;
  LOG(INFO) << feature_option->pyrmid_option.image_size;
  LOG(INFO) << feature_option->pyrmid_option.layer;
  feature_option->mask = cv::imread(mask_file, cv::IMREAD_GRAYSCALE);
}
//


//

template <>
void ParseYAMLOption(const std::string &file,
                     estimator::EstimatorOption *option) {
  auto opencv_file = CheckFile(file);
  std::string cali_path = opencv_file["calibrate_path"];
  //
  CalibrateOption calib_option;
  ParseYAMLOption(cali_path, &calib_option);
  //
  int pn = file.find_last_of('/');
  std::string configPath = file.substr(0, pn);
  std::string estimator_name = opencv_file["estimator"];
  const std::string estimator_file = configPath + "/" + estimator_name;
  //
  {
    // esitmator yaml
    auto fsSettings = CheckFile(estimator_file);
    ParseYAMLOptionFetureOption(&fsSettings, &option->feature_track_option,
                               calib_option , estimator_file);

    ParseYAMLOptionImuOption(&fsSettings, &option->imu_option, calib_option);
    //
    // option->feature_manager_option.extric_camera_to_imu =
    //     calib_option.extric_camera_to_imu;
    // //

    option->calibrate_option = calib_option;
    option->feature_manager_option.init_pnp_inlier_num =
        fsSettings["init_pnp_inlier_num"];
    option->feature_manager_option.use_stereo =( int(fsSettings["num_of_cam"]) == 2);

    option->use_cam_num = fsSettings["num_of_cam"];
    option->use_imu =fsSettings["imu"];
    option->estimate_td = fsSettings["estimate_td"];
    option->estimate_extrinsic = fsSettings["estimate_extrinsic"];
    option->init_td = fsSettings["td"];
    option->use_odom = fsSettings["use_odom"];
    option->odom_factor_option.optimize_weight =
        fsSettings["odom_optimization_weight"];
    option->odom_factor_option.angle_threas_hold=
        fsSettings["angle_threas_hold"];
    option->init_rotation_th = fsSettings["init_rotation_th"];
    option->init_bas_normal_max = fsSettings["init_bas_normal_max"];

    option->odom_factor_option.transform_imu_to_robot =
        calib_option.extric_camera_to_robot *
        calib_option.extric_camera_to_imu[0].inverse();
    LOG(INFO) << "transform_imu_to_robot: "
              << option->odom_factor_option.transform_imu_to_robot;
    // option->use_stereo_sample_ration =
    // fsSettings["use_stereo_sample_ration"];
    //
   

    option->rejection_points_depth_max_th =
        fsSettings["rejection_points_depth_max_th"];
    option->optimazation_outliers_rejection_th =
        fsSettings["optimazation_outliers_rejection_th"];


    //

    option->fail_detect_option.track_feat_lost_min_num =
        fsSettings["track_feat_lost_min_num"];
    option->fail_detect_option.track_feat_lost_win_size =
        fsSettings["track_feat_lost_win_size"];
    option->fail_detect_option.bas_norm_max = fsSettings["bas_norm_max"];
    option->fail_detect_option.bgs_norm_max = fsSettings["bgs_norm_max"];
    option->fail_detect_option.translation_norm_max =
        fsSettings["translation_norm_max"];
    option->fail_detect_option.translation_z_max =
        fsSettings["translation_z_max"];

    //
    option->fail_detect_option.ratation_max = fsSettings["ratation_max"];
    option->fail_detect_option.zero_translation_norm_max =
        fsSettings["zero_translation_norm_max"];
    option->fail_detect_option.zero_translation_z_max =
        fsSettings["zero_translation_z_max"];
    option->fail_detect_option.zero_ratation_max =
        fsSettings["zero_ratation_max"];
    //
    option->fail_detect_option.enable_odo_zero_lost_detect =
        fsSettings["enable_odo_zero_lost_detect"];
    //

    option->fail_detect_option.zero_odo_win_size =
        fsSettings["zero_odo_win_size"];
    option->fail_detect_option.zero_odo_pose_size =
        fsSettings["zero_odo_pose_size"];

    int t = fsSettings["UpdataZeroVelocityOption"]["enable"];


    
    option->enable_zero_velocity = bool(t);
    option->updata_zerovelocity_option.optimize_weight =
    option->updata_zerovelocity_option.optimize_bias_weight =
        fsSettings["UpdataZeroVelocityOption"]["optimize_bias_weight"];
    option->updata_zerovelocity_option.outlier_max_disparity =
        fsSettings["UpdataZeroVelocityOption"]["outlier_max_disparity"];
    option->updata_zerovelocity_option.zupt_max_velocity =
        fsSettings["UpdataZeroVelocityOption"]["zupt_max_velocity"];
    option->updata_zerovelocity_option.zupt_delay_frames =
        fsSettings["UpdataZeroVelocityOption"]["zupt_delay_frames"];
    option->updata_zerovelocity_option.que_time_duration =
        fsSettings["UpdataZeroVelocityOption"]["que_time_duration"];
    option->updata_zerovelocity_option.imu_velocity_option
        .integrated_accel_constraint =
        fsSettings["UpdataZeroVelocityOption"]["imu_velocity_option"]
                  ["integrated_accel_constraint"];
    option->updata_zerovelocity_option.imu_velocity_option
        .zupt_noise_multiplier =
        fsSettings["UpdataZeroVelocityOption"]["imu_velocity_option"]
                  ["zupt_noise_multiplier"];
    option->updata_zerovelocity_option.imu_velocity_option.zupt_chi2_multipler =
        fsSettings["UpdataZeroVelocityOption"]["imu_velocity_option"]
                  ["zupt_chi2_multipler"];
    option->updata_zerovelocity_option.imag_disparity_option.min_disparity_num =
        fsSettings["UpdataZeroVelocityOption"]["imag_disparity_option"]
                  ["min_disparity_num"];
    option->updata_zerovelocity_option.imag_disparity_option.max_disparity =
        fsSettings["UpdataZeroVelocityOption"]["imag_disparity_option"]
                  ["max_disparity"];
    
  }
}

// void ParseYAMLOptionSimpleVoOption(cv::FileStorage *fs,
//                                    SimpleVoOption *simple_vo_option) {
//   auto &fsSettings = *fs;
//   simple_vo_option->min_track_num = fsSettings["min_track_num"];
//   simple_vo_option->min_pnp_inlier_num = fsSettings["min_pnp_inlier_num"];
//   simple_vo_option->min_track_num = fsSettings["min_track_num"];
//   cv::Mat cv_T;
//   fsSettings["body_T_cam0"] >> cv_T;
//   Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
//   cv::cv2eigen(cv_T, T);
//   auto cam_to_imu = jarvis::transform::Rigid3d(
//       T.block<3, 1>(0, 3), Eigen::Quaterniond(T.block<3, 3>(0, 0)));
//   fsSettings["body_T_cam1"] >> cv_T;
//   cv::cv2eigen(cv_T, T);
//   auto cam1_to_imu = jarvis::transform::Rigid3d(
//       T.block<3, 1>(0, 3), Eigen::Quaterniond(T.block<3, 3>(0, 0)));
//   simple_vo_option->tracker_option.image_size =
//       Eigen::Vector2i(fsSettings["image_width"], fsSettings["image_height"]);
//   simple_vo_option->tracker_option.extric = cam_to_imu.inverse() * cam1_to_imu;
//   //
// }
// //
// void ParseYAMLOptionSlipDetectOption(cv::FileStorage *fs,
//                                      SlipDetectOption *slip_detection_opiont) {
//   auto &fsSettings = *fs;
//   slip_detection_opiont->type = fsSettings["type"];
//   slip_detection_opiont->min_disparity_num = fsSettings["min_disparity_num"];
//   slip_detection_opiont->max_disparity = fsSettings["max_disparity"];
//   slip_detection_opiont->que_time_duration= fsSettings["que_time_duration"];
//   slip_detection_opiont->zero_velocity_odom_delte_s_threash_hold =
//       fsSettings["zero_velocity_odom_delte_s_threash_hold"];
//   slip_detection_opiont->pose_odom_err_s_threash_hold =
//       fsSettings["pose_odom_err_s_threash_hold"];
//    slip_detection_opiont->pose_odom_err_theta_threash_hold =
//       fsSettings["pose_odom_err_theta_threash_hold"]; 
//   cv::Mat cv_T;
//   fsSettings["cam2RobotT"] >> cv_T;
//   Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
//   cv::cv2eigen(cv_T, T);
//   slip_detection_opiont->transform_cam_to_odom = jarvis::transform::Rigid3d(
//       T.block<3, 1>(0, 3), Eigen::Quaterniond(T.block<3, 3>(0, 0)));

//   {
//   cv::Mat cv_T;
//   fsSettings["body_T_cam0"] >> cv_T;
//   Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
//   cv::cv2eigen(cv_T, T);
//   auto cam_to_imu = jarvis::transform::Rigid3d(
//       T.block<3, 1>(0, 3), Eigen::Quaterniond(T.block<3, 3>(0, 0)));
//   LOG(INFO)<<slip_detection_opiont->transform_cam_to_odom*cam_to_imu.inverse();

//   }
// } 
//

//

//
}  // namespace jarvis