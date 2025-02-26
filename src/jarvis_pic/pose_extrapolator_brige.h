#ifndef _POSE_EXTRAPOLATRO_BRIGE_H
#define  _POSE_EXTRAPOLATRO_BRIGE_H
#include "jarvis/common/time.h"
#include "jarvis/pose_extrapolator.h"
namespace jarvis_pic {

class PoseExtrapolatorBrige {
 public:
  explicit PoseExtrapolatorBrige(jarvis::common::Duration pose_queue_duration,
                                 double imu_gravity_time_constant,
                                 jarvis::common::Time start_time);
  void AddPose(jarvis::common::Time time,
               const jarvis::transform::Rigid3d& pose, bool is_v = false);
  void AddImuData(const jarvis::sensor::ImuData& imu_data);
  void AddOdometryData(const jarvis::sensor::OdometryData& odometry_data);
  jarvis::transform::Rigid3d LastPose() { return catch_last_pose_; }

 private:
  //
  std::unique_ptr<jarvis::PoseExtrapolator> extrapolator_;
  std::unique_ptr<jarvis::PoseExtrapolator> last_extrapolator_;
  //
  jarvis::common::Time last_pose_time_;
  jarvis::transform::Rigid3d catch_last_pose_;
  jarvis::transform::Rigid3d vio_to_odom_transform_;
  bool pose_state_ =true;
};

}  // namespace jarvis_pic

#endif
