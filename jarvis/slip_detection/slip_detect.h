#ifndef JAIRVSI_PIC_SLIP_DETECT_
#define JAIRVSI_PIC_SLIP_DETECT_
//
#include <jarvis/sensor/image_data.h>
#include <jarvis/sensor/imu_data.h>
#include <jarvis/sensor/odometry_data.h>
#include <jarvis/transform/rigid_transform.h>

#include <deque>
#include <vector>

#include "jarvis/common/time.h"

namespace jarvis {
namespace estimator {
class FeatureTracker;
}
}  // namespace jarvis

namespace slip_detect {
class SimpleVo;
struct TimePose {
  jarvis::common::Time time;
  jarvis::transform::Rigid3d pose;
};

struct SlipDetectOption {
  int type = 0;
  int min_disparity_num = 20;
  float max_disparity = 1;
  float que_time_duration = 1;
  double zero_velocity_odom_delte_s_threash_hold = 0.1;
  double pose_odom_err_s_threash_hold = 0.1;
  void* feat_tracker_option;
};

//
class SlipDetect {
 public:
  explicit SlipDetect(const SlipDetectOption& option);
  ~SlipDetect();
  void AddImu(const jarvis::sensor::ImuData& imu_data);
  void AddOdometry(const jarvis::sensor::OdometryData& odom);
  void AddPose(const TimePose& pose);
  void AddImage(const jarvis::sensor::ImageData& image_data);
  bool Detect(const jarvis::common::Time&time);

 private:
  using KeyPointData =
      std::unordered_map<uint64_t, std::map<jarvis::common::Time, Eigen::Vector2d>>;
  template <typename T>
  void DropData(const jarvis::common::Time& time, std::deque<T>* deque);
  void DropData(const jarvis::common::Time& time, KeyPointData* deque);

  template <typename T>
  double ComputePosesS(std::deque<T>* deque);
  bool ZeroVelocityDetect(const jarvis::common::Time&time);
  bool SimpleDetect(const jarvis::common::Time&time);

  bool IsZeroVelocity();
  const SlipDetectOption options_;
  std::unique_ptr<jarvis::estimator::FeatureTracker> feature_tracker_;
  KeyPointData key_point_datas_;
  std::deque<jarvis::sensor::OdometryData> odometry_datas_;
  std::deque<TimePose> pose_datas_;
  std::unique_ptr<SimpleVo> simple_vo_;
  jarvis::common::Time latest_time_;
};
std::unique_ptr<SlipDetect> FactorSlipDetect(
    const std::string& file);
}  // namespace jarvis_pic

#endif
