#ifndef __JARVIS_MAPPING_KEYFRAMEFILTER_HPP__
#define __JARVIS_MAPPING_KEYFRAMEFILTER_HPP__
#include <set>

#include "jarvis/common/time.h"
#include "jarvis/key_frame_data.h"
#include "jarvis/transform/rigid_transform.h"
//
namespace jarvis {
namespace mapping {

//
struct KeyFrameFilterOption {
  double max_distance = 0.1;
  double max_angle = 20;
  double max_time = 10.0;
  double min_intersection_ration = 0.25;
};
//
class KeyFrameFilter {
 public:
  KeyFrameFilter(const KeyFrameFilterOption &option);
  bool IsKeyFrame(const TrackingData &tracking_data);

 private:
  std::set<FeatureId> ExtracttrackingDataId(const TrackingData &tracking_data);
  bool IsCoviLasttrackingFrame(const std::set<FeatureId> &tracking_ids);

  std::set<FeatureId> last_tracking_ids_;
  KeyFrameFilterOption options_;
  common::Time last_time_;
  transform::Rigid3d last_pose_;
};
}  // namespace mapping
}  // namespace jarvis
#endif