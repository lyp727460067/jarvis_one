#include "jarvis/mapping/key_frame_filter.h"

#include "jarvis/transform/transform.h"
namespace jarvis {
namespace mapping {

KeyFrameFilter::KeyFrameFilter(const KeyFrameFilterOption &option)
    : options_(option) {}
bool KeyFrameFilter::IsKeyFrame(const TrackingData &tracking_data) {
  auto const delta_pose =
      last_pose_.inverse() * tracking_data.data->imu_state.Pose();
  //
  const auto tracking_ids = ExtracttrackingDataId(tracking_data);
  if (delta_pose.translation().norm() <= options_.max_distance &&
      common::RadToDeg(transform::GetAngle(delta_pose)) <= options_.max_angle &&
      common::ToSeconds(tracking_data.data->time - last_time_) <=
          options_.max_time &
      IsCoviLasttrackingFrame(tracking_ids)) {
    return false;
  }
  last_tracking_ids_ = std::move(tracking_ids);
  last_pose_ = tracking_data.data->imu_state.Pose();
  last_time_ = tracking_data.data->time;

  return true;
}

//

std::set<FeatureId> KeyFrameFilter::ExtracttrackingDataId(
    const TrackingData &tracking_data) {
  std::set<FeatureId> tracking_ids;
  for (auto f : tracking_data.data->features_datas) {
    for (const auto &key_point : f.second.key_points) {
      tracking_ids.emplace(f.first, key_point.first);
    }
  }
  return tracking_ids;
}
bool KeyFrameFilter::IsCoviLasttrackingFrame(
    const std::set<FeatureId> &tracking_ids) {
  std::vector<FeatureId> result;
  std::set_intersection(tracking_ids.begin(), tracking_ids.end(),
                        last_tracking_ids_.begin(), last_tracking_ids_.end(),
                        std::back_inserter(result));
  if (result.size() > static_cast<int>(tracking_ids.size() *
                                       options_.min_intersection_ration)) {
    return true;
  }
  return false;
}
}  // namespace mapping
}  // namespace jarvis