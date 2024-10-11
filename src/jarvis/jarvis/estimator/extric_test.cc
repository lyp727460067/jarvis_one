#include "estimator/updater_zero_velocity.h"

#include <random>
#include "option_parse.h"
#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"
#include "transform/transform.h"
//
namespace jarvis {
namespace estimator {
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
  intrinsics: [373.64381416357105, 373.60969531040723, 326.5177613602167, 280.892312101823]
  resolution: [640, 544]
  rostopic: /cam1/image_raw
  timeshift_cam_imu: 0.003643864894561027
)";

std::string defalt_extric1 = R"(
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
  intrinsics: [375.1646285252324, 375.1480109372861, 330.34734769582957, 286.8303605144285]
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
  intrinsics: [373.64381416357105, 373.60969531040723, 338.5177613602167, 284.892312101823]
  resolution: [640, 544]
  rostopic: /cam1/image_raw
  timeshift_cam_imu: 0.003643864894561027
)";

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

TEST(ZeroVelocityDetectTestOptimazation, ZeroVelocityDetectTestOptimazation1) {
  camera_models::CameraPtr camera0;
  camera_models::CameraPtr camera1;
  {
    CheckNode defalt_paras = YAML::Load(defalt_extric);
    const CheckNode defalt_cam_node = defalt_paras["cam0"];
    auto cameram_option = ParseYAMLOptionCameraOption(defalt_paras, 0);
    camera0 =
        camera_models::CameraFactory::instance()->GenerateCameraFromOption(
            cameram_option);
  }
  {
    CheckNode defalt_paras = YAML::Load(defalt_extric1);
    const CheckNode defalt_cam_node = defalt_paras["cam0"];
    auto cameram_option = ParseYAMLOptionCameraOption(defalt_paras, 0);
    camera1 =
        camera_models::CameraFactory::instance()->GenerateCameraFromOption(
            cameram_option);
  }
  Eigen::Vector3d point{10, 20, 30};   Eigen::Vector2d defalt;
  camera0->spaceToPlane(point, defalt);
  LOG(INFO)<<defalt;

  Eigen::Vector3d world;
  camera1->liftProjective(defalt, world);
  Eigen::Vector2d d_world = defalt * world.z();
  LOG(INFO)<<world;
  //
  Eigen::Vector3d err_point = world*point.z();//{d_world.x(), d_world.y(), world.z()};


  //
  //
  transform::Rigid3d t = transform::Rigid3d(Eigen::Vector3d{0, 0, 1},
                                            transform::RollPitchYaw(0, 1, 1));

  //
  {
    Eigen::Vector2d result;
    Eigen::Vector3d piont1 = t * err_point;
    Eigen::Vector2d result1;
    camera1->spaceToPlane(piont1, result1);
    LOG(INFO) << result1.transpose();
  }
  {
    Eigen::Vector2d result;
    Eigen::Vector3d piont1 = t * point;
    camera0->spaceToPlane(piont1, result);
    LOG(INFO) << result.transpose();

  }
}

}  // namespace estimator
//
}  // namespace jarvis