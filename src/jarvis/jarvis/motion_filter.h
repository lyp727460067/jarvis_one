#ifndef JARVIS_MOTION_FILTER_H_
#define JARVIS_MOTION_FILTER_H_

#include <limits>

#include "common/time.h"
#include "transform/rigid_transform.h"
#include "transform/transform.h"

namespace jarvis {
struct MotionFilterOptions {
  double max_time_seconds=1000;
  double max_distance_meters=0.1;
  double max_angle_radians=0.3;
};

class MotionFilter {
 public:
  explicit MotionFilter(const MotionFilterOptions& options);

  bool IsSimilar(jarvis::common::Time time,
                 const jarvis::transform::Rigid3d& pose);

 private:
  const MotionFilterOptions options_;
  int num_total_ = 0;
  int num_different_ = 0;
  jarvis::common::Time last_time_;
  jarvis::transform::Rigid3d last_pose_;
};

}  // namespace jarvis

#endif