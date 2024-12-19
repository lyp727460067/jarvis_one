#include "jarvis/mapping/local_map_track_map.h"
namespace jarvis {
namespace mapping {

//
void LocalMapTrackMap::AddKeyFrameData(
    const KeyFrameData &key_frame_data,
    const FrontMapPointData &map_points_data) {
  auto key_frame_id =
      key_frames_datas_.Append(trajector_default, key_frame_data);
  StructureMapPoints(key_frame_id, map_points_data);

  if (key_frames_datas_.size() >size_t(options_.kf_num)) {
    auto it = key_frames_datas_.begin();
    key_frames_datas_.Trim(it->id);
    auto rm_map_points = covisibility_->TrimKeyFrame(it->id);
    for (auto mp : rm_map_points) {
      CHECK(map_points_.Contains(mp))<<mp;
      if (std::prev(map_points_.EndOfTrajectory(mp.trajectory_id))->id == mp) {
        continue;
      }
      map_points_.Trim(mp);
      covisibility_->TrimMapPoint(mp);
    }
  }
}
//
//
void LocalMapTrackMap::StructureMapPoints(
    const KeyFrameId &id,
    const FrontMapPointData &front_map_points) {
  //
  std::map<MapPointId, FeatureId> key_point_map_point_index;
  for (const auto &map_point_with_s : front_map_points) {
    for (const auto &map_point : map_point_with_s.second) {
      const MapPointId mp_id(map_point_with_s.first, map_point.first);
      const FeatureId feat_id = std::get<2>(map_point.second);
      key_point_map_point_index.emplace(mp_id, feat_id);
      if (!map_points_.Contains(mp_id)) {

        map_points_.Insert(mp_id, MapPointData{std::make_unique<MapPoint>(
                                      id, std::get<0>(map_point.second))});
      } else {
        // map_points_.at(mp_id).data->UpdatePos(id,
        //                                       std::get<0>(map_point.second));
      }
    }
  }
  covisibility_->UpdateWithFrameData(id, std::move(key_point_map_point_index));
};
//
std::map<MapPointId, MapPointData> LocalMapTrackMap::GetKeyFrameMapPoints(
    const KeyFrameId &kf_id) {
  auto map_poin_ids = covisibility_->GetKeyFrameMapPointId(kf_id).first;
  std::map<MapPointId, MapPointData> result;

  for (const auto &id : map_poin_ids) {
    result.emplace(id, map_points_.at(id));
  }
  return result;
}
}  // namespace mapping
}  // namespace jarvis