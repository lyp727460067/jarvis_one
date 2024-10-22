
#include "jarvis/option_parse.h"

#include <jarvis/estimator/featureTracker/feature_tracker.h>

#include <sstream>

#include "jarvis/estimator/estimator.h"
#include "jarvis/estimator/initial/initialization_stero_imu.h"
#include "opencv2/core/eigen.hpp"
#include "opencv2/opencv.hpp"
#include "yaml-cpp/yaml.h"

namespace jarvis {

double timeshift_cam_imu = 0;

double GetTimeShiftCamImu() { return timeshift_cam_imu; }
std::string defalt_extric = R"(
cam0:
  FOV: [107.93911113317935, 88.90577703348904]
  T_imu_cam :
  - [0.004278094939249882, -0.008781514712983385, -0.9999522903134112, -0.03265168587124466]
  - [0.9999883505210795, 0.002272838395565302, 0.00425828929951011, -0.03861822650981798]
  - [0.0022353357290220144, -0.9999588587561857, 0.00879113583135066, 0.04237282279723379]
  - [0.0, 0.0, 0.0, 1.0]
  camera_model: pinhole
  distortion_coeffs: [-0.11351460368451406, 0.027700300349535557, -0.05182815943506382, 0.033348630966814674]
  distortion_model: equidistant
  intrinsics: [375.1646285252324, 375.1480109372861, 324.34734769582957, 280.8303605144285]
  resolution: [640, 544]
  rostopic: /cam0/image_raw
  timeshift_cam_imu: 0.0036469377829320506
cam1:
  FOV: [108.19716877637063, 88.89444585437855]
  T_imu_cam :
  - [0.00805418108864972, -0.012976385246538658, -0.9998833649946003, -0.03426593099017232]
  - [0.9998453634446182, -0.01552732376971716, 0.008255387020022986, 0.041688009792324746]
  - [-0.015632637822556528, -0.9997952368572698, 0.01284931874555939, 0.04227918368552841]
  - [0.0, 0.0, 0.0, 1.0]
  camera_model: pinhole
  distortion_coeffs: [-0.10006345248147991, -0.0019597758035067043, -0.002504364558405612, -6.999676678582795e-05]
  distortion_model: equidistant
  intrinsics: [373.64381416357105, 373.60969531040723, 328.5177613602167, 274.892312101823]
  resolution: [640, 544]
  rostopic: /cam1/image_raw
  timeshift_cam_imu: 0.003643864894561027
cam2:
  FOV: [108.66961697032585, 130.17812935091388]
  T_imu_cam:
  - [0.941056359814017, -0.3148804124051945, -0.12354454070142512, 0.03455358143724464]
  - [0.0031538214143274346, -0.357062436454436, 0.9340751949836311, 0.19010505184132792]
  - [-0.33823509732740326, -0.8794070402019515, -0.3350228000885397, -0.045030621595181064]
  - [0.0, 0.0, 0.0, 1.0]
  camera_model: pinhole
  distortion_coeffs: [-0.03576599241600331, 0.005311944824974008, -0.008960851453169225, 0.002343472456752729]
  distortion_model: equidistant
  intrinsics: [296.58529220756407, 296.6552793534289, 266.55468172009057, 331.78644925697415]
  resolution: [544, 640]
  rostopic: /cam2/image_raw
  timeshift_cam_imu: 0.0032609176405895054
cam3:
  FOV: [107.39385427619561, 128.33704200392532]
  T_imu_cam:
  - [-0.9423255259020137, -0.31038909667865566, -0.1252246457232003, 0.03345155137170076]
  - [0.01093706854139559, 0.34538628591233134, -0.9383968744808382, -0.19134805802784746]
  - [0.33451903348720485, -0.8856449187833086, -0.3220715666868047, -0.044823913170773166]
  - [0.0, 0.0, 0.0, 1.0]
  camera_model: pinhole
  distortion_coeffs: [-0.03287901867378544, -0.004175992746239715, 0.0037018095011941555, -0.0026293441568893466]
  distortion_model: equidistant
  intrinsics: [299.6732677592133, 299.84763072714117, 258.39072609132694, 324.8680320355175]
  resolution: [544, 640]
  rostopic: /cam3/image_raw
  timeshift_cam_imu: 0.0033212132730980003
)";

constexpr int kCameraNum = 4;
std::stringstream info;
bool CheckFileExist(const std::string &file) {
  FILE *fh = fopen(file.c_str(), "r");
  if (fh == nullptr) {
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
  camera_option.timeshift_cam_imu   =  cam_node["timeshift_cam_imu"].as<double>();
  return camera_option;
}
template <>
void ParseYAMLOption(const std::string &file_path,
                     CalibrateOption *calibrate_options) {
  const std::string cam_chain_file = file_path + "/camchain-imucam.yaml";
  const std::string config_file = file_path + "/config.yml";
  CHECK(CheckFileExist(cam_chain_file)) << cam_chain_file << " not exist.";
  CHECK(CheckFileExist(config_file)) << config_file << " not exist.";
   {
    info << "Start parse " << cam_chain_file << "\n";
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
      LOG(FATAL) << "ERROR: Wrong path to settings";
    }
    {
    cv::Mat cv_T;
    fsSettings["cam2RobotT"] >> cv_T;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    cv::cv2eigen(cv_T, T);
    calibrate_options->extric_camera_to_robot = jarvis::transform::Rigid3d(
        T.block<3, 1>(0, 3), Eigen::Quaterniond(T.block<3, 3>(0, 0)));
    calibrate_options->extric_camera_to_odom.push_back(
        calibrate_options->extric_camera_to_robot);
    //
    }
    {
    cv::Mat cv_T;
    fsSettings["cam2RobotT_side_left"] >> cv_T;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    cv::cv2eigen(cv_T, T);
   auto extric_camera_to_robot = jarvis::transform::Rigid3d(
        T.block<3, 1>(0, 3), Eigen::Quaterniond(T.block<3, 3>(0, 0)).normalized());
    calibrate_options->extric_camera_to_odom.push_back(
        extric_camera_to_robot);
    //
    }
    {
    cv::Mat cv_T;
    fsSettings["cam2RobotT_side_right"] >> cv_T;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    cv::cv2eigen(cv_T, T);
    auto extric_camera_to_robot = jarvis::transform::Rigid3d(
        T.block<3, 1>(0, 3), Eigen::Quaterniond(T.block<3, 3>(0, 0)));
    calibrate_options->extric_camera_to_odom.push_back(
        extric_camera_to_robot);
    //
    }



  }


 {
    LOG(INFO) << "Start parse " << cam_chain_file;
    CheckNode paras = YAML::LoadFile(cam_chain_file);
    CheckNode defalt_paras = YAML::Load(defalt_extric);

    auto GetCameraExt = [](const CheckNode &cam_node) {
      Eigen::Matrix4d camera_to_imu;
      const std::vector<std::vector<double>> camera_to_imu_vector =
          cam_node["T_imu_cam"].as<std::vector<std::vector<double>>>();
      for (int i = 0; i < 4; i++) {
        camera_to_imu.row(i) = Eigen::Vector4d(camera_to_imu_vector[i].data());
      }
      return transform::Rigid3d(
          camera_to_imu.block<3, 1>(0, 3),
          Eigen::Quaterniond(camera_to_imu.block<3, 3>(0, 0)));
    };
    //
    for (int i = 0; i < kCameraNum; i++) {

      const CheckNode cam_node = paras["cam" + std::to_string(i)];
      const transform::Rigid3d ext_para = GetCameraExt(cam_node);

      const CheckNode defalt_cam_node = defalt_paras["cam" + std::to_string(i)];
      const transform::Rigid3d defalt_ext_para = GetCameraExt(defalt_cam_node);
      LOG(INFO)<<i;
      if (abs(defalt_ext_para.translation().norm() -
              ext_para.translation().norm()) > 0.03) {
        LOG(ERROR) << "The calibration result is too far from the reference "
                      "value. defalt:"
                   << defalt_ext_para << "cal " << ext_para
                   << ". distance:"
                   << abs(defalt_ext_para.translation().norm() -
                          ext_para.translation().norm())
                   << ".Load defalt para.";
        calibrate_options->extric_camera_to_imu.push_back(defalt_ext_para);
        // calibrate_options->camera_options.push_back(
        //     ParseYAMLOptionCameraOption(defalt_paras, i));
      } else {
        calibrate_options->extric_camera_to_imu.push_back(ext_para);

      }
      //
        calibrate_options->camera_options.push_back(
            ParseYAMLOptionCameraOption(paras, i));
      LOG(INFO) << calibrate_options->camera_options.back().DebugInfo();
      LOG(INFO) << "imu_to_cam:"
                << calibrate_options->extric_camera_to_imu.back();

      //
      //
    }
    // Eigen::Matrix3d temp;
    // temp << 9.9995443247882831e-01, 4.4208498469253781e-03,
    //     8.4610314132053579e-03, -4.4454512625568387e-03, 9.9998593988640494e-01,
    //     2.8910192969837647e-03, -8.4481316879260245e-03,
    //     -2.9285006631791207e-03, 9.9996002567845144e-01;
    // Eigen::Vector3d temp_t(-7.9909196183143521e+01, 7.4353285997290131e-02,
    //                        4.9535770968658988e-01);

    // temp_t = temp_t * 0.001;
    // transform::Rigid3d temp_ex(temp_t, Eigen::Quaterniond(temp).normalized());
    // LOG(INFO)<<transform::Rot2ypr(temp.transpose());
    // //
    // temp_ex = temp_ex.inverse();
    // LOG(INFO) << "cam0cam1: "
    //           << calibrate_options->extric_camera_to_imu[0].inverse() *
    //                  calibrate_options->extric_camera_to_imu[1];
    // transform::Rigid3d diff_temp_ex = (calibrate_options->extric_camera_to_imu[0].inverse() *
    //                  calibrate_options->extric_camera_to_imu[1]).inverse()* temp_ex;
    // LOG(INFO)<<diff_temp_ex <<" diff_temp_ex "<<transform::Rot2ypr(diff_temp_ex.rotation().toRotationMatrix());
    // LOG(INFO) << "rcam0cam1 " << temp_ex;
    // //

    // LOG(INFO) << transform::Rot2ypr(calibrate_options->extric_camera_to_imu[0]
    //                                     .rotation()
    //                                     .toRotationMatrix()) ;
                    
    // // calibrate_options->extric_camera_to_imu[1] =
    // //     calibrate_options->extric_camera_to_imu[0] * temp_ex;
    // //
    // LOG(INFO)<<calibrate_options->extric_camera_to_imu[0];
    // LOG(INFO)<<calibrate_options->extric_camera_to_imu[1];

    // LOG(INFO) << transform::Rot2ypr(calibrate_options->extric_camera_to_imu[0]
    //                                     .rotation()
    //                                     .toRotationMatrix()) ;
    // LOG(INFO) << transform::Rot2ypr(calibrate_options->extric_camera_to_imu[1]
    //                                     .rotation()
    //                                     .toRotationMatrix()) ;
    
    std::swap(calibrate_options->extric_camera_to_imu[2],
              calibrate_options->extric_camera_to_imu[3]);
    std::swap(calibrate_options->camera_options[2],
              calibrate_options->camera_options[3]);
    //
    transform::Rigid3d cam2tocam0 =
        calibrate_options->extric_camera_to_odom[0].inverse() *
        calibrate_options->extric_camera_to_odom[1];
    LOG(INFO)<<"camcham "<<calibrate_options->extric_camera_to_imu[2];
    calibrate_options->extric_camera_to_imu[2] =
        calibrate_options->extric_camera_to_imu[0] * cam2tocam0;
    //
    LOG(INFO)<<"camodom "<<calibrate_options->extric_camera_to_imu[2];
    transform::Rigid3d cam3tocam0 =
        calibrate_options->extric_camera_to_odom[0].inverse() *
        calibrate_options->extric_camera_to_odom[2];

    LOG(INFO)<<"camcham "<<calibrate_options->extric_camera_to_imu[3];
    calibrate_options->extric_camera_to_imu[3] =
        calibrate_options->extric_camera_to_imu[0] * cam3tocam0;

    LOG(INFO)<<"camodom "<<calibrate_options->extric_camera_to_imu[3];
 }

 
}

void ParseYAMLOptionImuOption(cv::FileStorage *fs, jarvis::ImuOption *option,
                              const CalibrateOption &camera_option) {
  //
  auto &fsSettings = *fs;
  option->imu_noise.na = fsSettings["imu_option"]["acc_n"];
  option->imu_noise.ng = fsSettings["imu_option"]["gyr_n"];
  option->imu_noise.nba = fsSettings["imu_option"]["acc_w"];
  option->imu_noise.nbg = fsSettings["imu_option"]["gyr_w"];
  option->imu_noise.na2 = option->imu_noise.na * option->imu_noise.na;
  option->imu_noise.ng2 = option->imu_noise.ng * option->imu_noise.ng;
  option->imu_noise.nba2 = option->imu_noise.nba * option->imu_noise.nba;
  option->imu_noise.nbg2 = option->imu_noise.nbg * option->imu_noise.nbg;

  info << option->DebugInfo() << "\n";
}
//
jarvis::estimator::OptimizationOption ParseYAMLOptionOptimizationOption(
    const cv::FileNode &fs) {
  //
  auto &fsSettings = fs;
  jarvis::estimator::OptimizationOption op_option;
  op_option.max_solver_time = fsSettings["max_solver_time"];
  op_option.max_num_iterations = fsSettings["max_num_iterations"];
  op_option.use_odom = fsSettings["use_odom"];
  op_option.camera_weight = fsSettings["camera_weight"];
  op_option.estimate_td = fsSettings["estimate_td"];
  op_option.init_td = fsSettings["td"];
  op_option.estimate_extrinsic = fsSettings["estimate_extrinsic"];
  op_option.huber_loss = fsSettings["huber_loss"];

  return op_option;
}
//
jarvis::estimator::FeatureManagerOption ParseYAMLOptionFeatureManagerOption(
    int sw_size, int use_stereo, const cv::FileNode &fs) {
  auto &fsSettings = fs;
  jarvis::estimator::FeatureManagerOption feature_manager_option;
  feature_manager_option.sw_size = sw_size;
  feature_manager_option.use_stereo = use_stereo;
  feature_manager_option.init_depth = fsSettings["init_depth"];
  feature_manager_option.keyframe_parallax = fsSettings["keyframe_parallax"];
  feature_manager_option.init_pnp_inlier_num =
      fsSettings["init_pnp_inlier_num"];
  feature_manager_option.convin_used_num = fsSettings["convin_used_num"];
  feature_manager_option.parallax_option.start_frame =
      fsSettings["start_frame"];
  feature_manager_option.parallax_option.last_track_num =
      fsSettings["last_track_num"];
  feature_manager_option.parallax_option.long_track_num =
      fsSettings["long_track_num"];
  feature_manager_option.parallax_option.new_feature_ration =
      fsSettings["new_feature_ration"];
  feature_manager_option.optimazation_outliers_rejection_th =
      fsSettings["optimazation_outliers_rejection_th"];
  feature_manager_option.optimazation_outliers_rejection_th =
      feature_manager_option.optimazation_outliers_rejection_th / 377.0f;

  feature_manager_option.rejection_points_depth_max_th =
      fsSettings["rejection_points_depth_max_th"];
  return feature_manager_option;
}

//
void ParseYAMLOptionFetureOption(
    cv::FileStorage *fs, int index,
    jarvis::estimator::FeatureTrackerOption *feature_option,
    const CalibrateOption &camera_option, const std::string &file) {
  //
  auto &fsSettings = *fs;
  //
  std::string feat_tack = "feattrack" + std::to_string(index);
  feature_option->pyrmid_option.layer =
      fsSettings[feat_tack]["lk_pre_max_layer"];
  feature_option->pyrmid_option.lk_win_size =
      fsSettings[feat_tack]["lk_win_size"];
  feature_option->ransac_threshold = fsSettings[feat_tack]["F_threshold"];
  feature_option->track_back = fsSettings[feat_tack]["flow_back"];
  feature_option->max_feat_cnt = fsSettings[feat_tack]["max_cnt"];
  feature_option->feature_detect_option.min_distance =
      fsSettings[feat_tack]["min_dist"];
  feature_option->feature_detect_option.mask_min_dist =
      fsSettings[feat_tack]["mask_min_dist"];
  feature_option->feature_detect_option.fast_thresh_hold =
      fsSettings[feat_tack]["fast_th"];
  feature_option->feature_detect_option.grid_size.x() =
      fsSettings[feat_tack]["grid_size_x"];
  feature_option->feature_detect_option.grid_size.y() =
      fsSettings[feat_tack]["grid_size_y"];
  feature_option->klt_type = fsSettings[feat_tack]["klt_type"];
  feature_option->back_flow_min_distance =
      fsSettings[feat_tack]["back_flow_min_distance"];

  //   feature_option->feature_detect_option.imag_size =
  //       camera_option.camera_options[0].resolution;
  //   LOG(INFO)<<camera_option.camera_options[0].resolution;
  //   // feature_option->feature_detect_option.grid_size=
  //   //     camera_option.camera_options[0].resolution;

  //   feature_option->calibrate_option = camera_option;
  std::string mask_id;
  fsSettings[feat_tack]["mask"] >> mask_id;
  int pn = file.find_last_of('/');
  std::string configPath = file.substr(0, pn);
  auto mask_file = configPath + "/" + mask_id;

  info << "Mask file:  " << mask_file << "\n";
  info << feature_option->pyrmid_option.layer << "\n";

  feature_option->mask = cv::imread(mask_file, cv::IMREAD_GRAYSCALE);
  //   CHECK(!feature_option->mask.empty());
}
//

//

template <>
void ParseYAMLOption(const std::string &file,
                     estimator::EstimatorOption *option) {
  info.clear();
  auto opencv_file = CheckFile(file);
  std::string cali_path = opencv_file["calibrate_path"];
  //
  CalibrateOption calib_option;
  ParseYAMLOption(cali_path, &calib_option);
  //
  timeshift_cam_imu   = calib_option.camera_options[0].timeshift_cam_imu;
  int pn = file.find_last_of('/');
  std::string configPath = file.substr(0, pn);
  std::string estimator_name = opencv_file["estimator"];
  const std::string estimator_file = configPath + "/" + estimator_name;

  //
  {
    // esitmator yaml
    //
    auto fsSettings = CheckFile(estimator_file);

    //
    // std::vector<std::vector<int>> trace_sequence =
    // fsSettings["trace_sequence"]; CHECK_EQ(trace_sequence.size(), 3);
    std::string track_sequence_str = fsSettings["track_sequence"];
    std::vector<std::vector<int>> track_sequence;
    for (size_t i = 0; i < track_sequence_str.size(); i++) {
      if (track_sequence_str[i] == '{') {
        track_sequence.push_back(std::vector<int>{});
      } else if (track_sequence_str[i] != ',' && track_sequence_str[i] != '}') {
        track_sequence.back().push_back(track_sequence_str[i] - '0');
      }
    }

    option->track_sequence = track_sequence;
    int track_cam_num =option->track_sequence.size();
    option->win_size = fsSettings["win_size"];
    //
    info << "track_cam_num" << track_cam_num << "\n";
    //
    LOG(INFO) << track_cam_num;
    jarvis::ImuOption imu_option;
    ParseYAMLOptionImuOption(&fsSettings, &imu_option, calib_option);
    jarvis::estimator::SteroImuInitializationOption stero_imu_init_option;
    int j = 0;
    //
    {
      jarvis::estimator::FeatureTrackerOption feature_manager_option;
      for (int i = 0; i < 2; i++, j++) {
        ParseYAMLOptionFetureOption(&fsSettings, 0, &feature_manager_option,
                                    calib_option, estimator_file);
        camera_models::CameraPtr camera =
            camera_models::CameraFactory::instance()->GenerateCameraFromOption(
                calib_option.camera_options[j]);
        feature_manager_option.cameras.push_back(camera);

        stero_imu_init_option.sw_size = option->win_size;
        stero_imu_init_option.imu_option = imu_option;
        stero_imu_init_option.extric_camera_to_imu.push_back(
            calib_option.extric_camera_to_imu[j]);
      }

      stero_imu_init_option.init_v_th = fsSettings["init_velocity_th"];

      //
      feature_manager_option.pyrmid_option.image_size =
          calib_option.camera_options[0].resolution;
      feature_manager_option.feature_detect_option.imag_size =
          feature_manager_option.pyrmid_option.image_size;
      //

      //
      option->feature_track_options.push_back(feature_manager_option);
    }
    //
    for (int i = 1; i < track_cam_num; i++, j++) {
      jarvis::estimator::FeatureTrackerOption feature_manager_option;
      ParseYAMLOptionFetureOption(&fsSettings, i, &feature_manager_option,
                                  calib_option, estimator_file);
      camera_models::CameraPtr camera =
          camera_models::CameraFactory::instance()->GenerateCameraFromOption(
              calib_option.camera_options[j]);
      feature_manager_option.cameras.push_back(camera);

      feature_manager_option.pyrmid_option.image_size =
          calib_option.camera_options[j].resolution;
      feature_manager_option.feature_detect_option.imag_size =
          feature_manager_option.pyrmid_option.image_size;

      option->feature_track_options.push_back(feature_manager_option);
    }
    LOG(INFO) << "1";
    //
    //

    int use_stero = fsSettings["use_stero"];
    option->use_stero = (use_stero == 1);
    // option->use_odom = fsSettings["use_odom"];
    // option->use_stereo_sample_ration =
    //     fsSettings["use_stereo_sample_ration"];
    // //
    option->stero_imu_init_option.feature_manager_option =
        ParseYAMLOptionFeatureManagerOption(option->win_size, option->use_stero,
                                            fsSettings["feature_manager"]);

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
    //
    //
    // option->slide_windows_option.track_cam_num = option->track_cam_num;
    option->slide_windows_option.track_sequence = option->track_sequence;
    option->slide_windows_option.win_size = option->win_size;
    // option->slide_windows_option.optimazation_outliers_rejection_th =
    //     fsSettings["slide_window"]["optimazation_outliers_rejection_th"];

    // option->slide_windows_option.optimazation_outliers_rejection_th =
    //     option->slide_windows_option.optimazation_outliers_rejection_th / 377.0;

    // option->slide_windows_option.rejection_points_depth_max_th =
    //     fsSettings["slide_window"]["rejection_points_depth_max_th"];
    // //
    option->slide_windows_option.use_stereo = option->use_stero;
    option->slide_windows_option.imu_option = imu_option;
    //
    option->slide_windows_option.extric_camera_to_imu =
        calib_option.extric_camera_to_imu;
    //
    option->slide_windows_option.odom_factor_option.angle_threas_hold =
        fsSettings["odom_option"]["angle_threas_hold"];
    option->slide_windows_option.odom_factor_option.optimize_weight =
        fsSettings["odom_option"]["odom_optimization_weight"];
    option->slide_windows_option.odom_factor_option.transform_imu_to_robot =
        calib_option.extric_camera_to_robot *
        calib_option.extric_camera_to_imu[0].inverse();
    //

    option->slide_windows_option.feature_manager_option =
        ParseYAMLOptionFeatureManagerOption(option->win_size, option->use_stero,
                                            fsSettings["feature_manager"]);

    stero_imu_init_option.feature_manager_option =
        option->slide_windows_option.feature_manager_option;
    option->stero_imu_init_option = stero_imu_init_option;
    //
    option->stero_imu_init_option.opti_option =
        ParseYAMLOptionOptimizationOption(fsSettings["init_optimization"]);
    
    //
    option->slide_windows_option.opti_option =
        ParseYAMLOptionOptimizationOption(fsSettings["optimization"]);

    // op_option.max_solver_time =
    //     fsSettings["optimization"]["max_solver_time"];
    // op_option.max_num_iterations =
    //     fsSettings["optimization"]["max_num_iterations"];
    // op_option.use_odom = fsSettings["optimization"]["use_odom"];
    // op_option.camera_weight =
    //     fsSettings["optimization"]["camera_weight"];
    // // op_option.track_cam_num = option->track_cam_num;
    // op_option.estimate_td = fsSettings["optimization"]["estimate_td"];
    // op_option.init_td = fsSettings["optimization"]["td"];
    // op_option.estimate_extrinsic =
    //     fsSettings["optimization"]["estimate_extrinsic"];
    //
    //

    //

    int t = fsSettings["UpdataZeroVelocityOption"]["enable"];
    option->slide_windows_option.enable_zero_velocity = bool(t);
    option->slide_windows_option.updata_zerovelocity_option.optimize_weight =
        option->slide_windows_option.updata_zerovelocity_option
            .optimize_bias_weight =
            fsSettings["UpdataZeroVelocityOption"]["optimize_bias_weight"];
    option->slide_windows_option.updata_zerovelocity_option
        .outlier_max_disparity =
        fsSettings["UpdataZeroVelocityOption"]["outlier_max_disparity"];
    option->slide_windows_option.updata_zerovelocity_option.zupt_max_velocity =
        fsSettings["UpdataZeroVelocityOption"]["zupt_max_velocity"];
    option->slide_windows_option.updata_zerovelocity_option.zupt_delay_frames =
        fsSettings["UpdataZeroVelocityOption"]["zupt_delay_frames"];
    option->slide_windows_option.updata_zerovelocity_option.que_time_duration =
        fsSettings["UpdataZeroVelocityOption"]["que_time_duration"];
    option->slide_windows_option.updata_zerovelocity_option.imu_velocity_option
        .integrated_accel_constraint =
        fsSettings["UpdataZeroVelocityOption"]["imu_velocity_option"]
                  ["integrated_accel_constraint"];
    option->slide_windows_option.updata_zerovelocity_option.imu_velocity_option
        .zupt_noise_multiplier =
        fsSettings["UpdataZeroVelocityOption"]["imu_velocity_option"]
                  ["zupt_noise_multiplier"];
    option->slide_windows_option.updata_zerovelocity_option.imu_velocity_option
        .zupt_chi2_multipler =
        fsSettings["UpdataZeroVelocityOption"]["imu_velocity_option"]
                  ["zupt_chi2_multipler"];
    option->slide_windows_option.updata_zerovelocity_option
        .imag_disparity_option.min_disparity_num =
        fsSettings["UpdataZeroVelocityOption"]["imag_disparity_option"]
                  ["min_disparity_num"];
    option->slide_windows_option.updata_zerovelocity_option
        .imag_disparity_option.max_disparity =
        fsSettings["UpdataZeroVelocityOption"]["imag_disparity_option"]
                  ["max_disparity"];
  }
//   LOG(INFO) << "\n" << info.str() << "\n";
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
//   simple_vo_option->tracker_option.extric = cam_to_imu.inverse() *
//   cam1_to_imu;
//   //
// }
// //
// void ParseYAMLOptionSlipDetectOption(cv::FileStorage *fs,
//                                      SlipDetectOption *slip_detection_opiont)
//                                      {
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