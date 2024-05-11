#include "motion_filter.h"

#include "glog/logging.h"

namespace jarvis {
MotionFilter::MotionFilter(const MotionFilterOptions& options)
    : options_(options) {}

bool MotionFilter::IsSimilar(const jarvis::common::Time time,
                             const jarvis::transform::Rigid3d& pose) {
  LOG_IF_EVERY_N(INFO, num_total_ >= 500, 500)
      << "Motion filter reduced the number of nodes to "
      << 100. * num_different_ / num_total_ << "%.";
  ++num_total_;
  if (num_total_ > 1 &&
      time - last_time_ <= common::FromSeconds(options_.max_time_seconds) &&
      (pose.translation() - last_pose_.translation()).norm() <=
          options_.max_distance_meters &&
      transform::GetAngle(pose.inverse() * last_pose_) <=
          options_.max_angle_radians) {
    return true;
  }
  //   LOG(INFO) << "Update this insertion x, y: " ;
  last_time_ = time;
  last_pose_ = pose;
  ++num_different_;
  return false;
}

}  // namespace jarvis