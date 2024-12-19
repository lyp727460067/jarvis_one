#include "glog/logging.h"
#include "gtest/gtest.h"
#include "jarvis/mapping/local_map_track.h"
namespace jarvis {
namespace mapping {

TEST(LocalMapTrackTest, LocalMapTrackOptimization) {
  LocalMapTrackOption option;
  option.track_sequence.push_back({0});
  option.huber_loss =0.01;
  LocalMapTrack local_map_track_(option);

  std::vector<transform::Rigid3d> extric_camera_to_imu;
  for (int i = 0; i < 4; i++) {
    extric_camera_to_imu.push_back(transform::Rigid3d::Identity());
  }

  int lower = -65536, upper = 65536;
  //
   std::map<int, std::vector<LocalMapTrack::MatchData>> constraints;
  //
   transform::Rigid3d pose(Eigen::Vector3d(1, 0, 0),
                           transform::RollPitchYaw(0, 0, common::DegToRad(30)));
   float denot = 0.001;
   for (int i = 0; i < 100; i++) {
     int randomx = lower + rand() % (upper - lower + 1);
     int randomy = lower + rand() % (upper - lower + 1);
     int randomz = rand() % (upper + 1);

     Eigen::Vector3d xyz(float(randomx) * denot, float(randomx) * denot,
                         float(randomy) * denot);

     constraints[0].push_back({(xyz / xyz.z()).head<2>(), pose * xyz});
   }
   transform::Rigid3d op_pose = local_map_track_.Optimize(
       transform::Rigid3d::Identity(), extric_camera_to_imu, constraints,
       std::array<float, 2>{100, 100});
   transform::Rigid3d op_pose1 = local_map_track_.FourOptimize(
       transform::Rigid3d::Identity(), extric_camera_to_imu, constraints,
       std::array<float, 2>{100, 100});
    LOG(INFO)<<op_pose.translation();
    CHECK_NEAR(common::RadToDeg(transform::GetYaw(op_pose)),30,1e-4);
  
}

}  // namespace mapping
}  // namespace jarvis