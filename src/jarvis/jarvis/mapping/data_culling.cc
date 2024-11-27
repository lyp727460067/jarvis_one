
#include "mapping/data_culling.h"

#include <set>
#include <vector>

#include "jarvis/mapping/covisibility.h"
#include "jarvis/mapping/map_manger.h"
namespace jarvis {
//
namespace mapping {
DataCulling::DataCulling(const DataCullingOption& option, DataFuse* data_fuse)
    : options_(option), data_fuse_(data_fuse) {
}

//
//
std::map<MapPointId, std::map<KeyFrameId, FeatureId>>
DataCulling::SearchMatchesKeyFrames(
    const MapById<KeyFrameId, KeyFrameData>& key_frame_datas,
    const MapById<MapPointId, MapPointData>& map_points, const KeyFrameId& id) {
  std::map<MapPointId, std::map<KeyFrameId, FeatureId>> result;
  //

  //

  //
  for (const auto& key_frame : key_frame_datas) {
    auto sequence_feautes = key_frame.data.data->features.trajectory_ids();
    //
    //
    std::map<int, std::unique_ptr<match::AreaSearch>> area_searchs =
        match::AreaSearch::CreateAreaSearchFromeKeyFrameData(
            options_.image_bboxs, options_.grid_lenth, key_frame.data);
    //
    match::ProjectionOption project_option{
        options_.viewing_angle, options_.area_search_radius,
        options_.project_pix_err, options_.best_map_fuse_des_dis,
        [=](const Eigen::Vector3d& point, int s) {
          return data_fuse_->PorjectPoint(point, s);
        }};

    for (const auto& map_point : map_points) {
      if (key_frame.id == id) continue;
      if (data_fuse_->GetMapObservations(map_point.id).count(key_frame.id))
        continue;
      FeatureId index = match::SearchMatchesByProjection(
          project_option, key_frame.data, area_searchs, map_point.data);
      //
      if (index == FeatureId{-1, 0}) {
        continue;
      }
      result[map_point.id].emplace(key_frame.id, index);
    }
    // if(one_feature.id)
  }

  return result;
}

bool DataCulling::DataCulling::IsRedundant(
    const MapById<MapPointId, mapping::MapPointData>& map_points) {
  int redundant_observations = 0;
  for (const auto& map_point : map_points) {
    if (data_fuse_->GetMapObservations(map_point.id).size() >
        options_.map_culling_obs) {
      ++redundant_observations;
    }
  }
  if (redundant_observations >
      options_.redundant_observations_ration * map_points.size()) {
    return true;
  }
  return false;
}
//
void DataCulling::KeyFrameCulling(const KeyFrameId& id) {
  //
  auto const covisibility_frame_ids =
      data_fuse_->GetKeyLevelConnectedKeyFrames(id, {10});
  if (covisibility_frame_ids.empty()) {
    // 如果跟丢了后的第一帧的话，进来的话是跟前面没有共识;
    return;
  }
  const auto& all_key_frame_datas = data_fuse_->GetAllKeyFramesData();
  MapById<KeyFrameId, KeyFrameData> covisibility_frame;

  //
  // for (const auto& covisi_id : covisibility_frame_ids) {
  for (const auto& covisi_id : covisibility_frame_ids) {
    if (!covisibility_frame.Contains(covisi_id.first)) {
      covisibility_frame.Insert(covisi_id.first,
                                all_key_frame_datas.at(covisi_id.first));
    }
  }
  std::set<KeyFrameId> result;
  for (auto it = std::prev(covisibility_frame.end());
       it != covisibility_frame.begin(); --it) {
    // for (const auto& covisi_id : covisibility_frame) {
    auto const map_points = data_fuse_->GetMapPoints(it->id);
    if (IsRedundant(map_points)) {
      data_fuse_->CullKeyFrame(std::set<KeyFrameId>{it->id});
      // result.insert(covisi_id.first);
      // return;
    }
  }
  // if (result.empty()) return;
  // data_fuse_->CullKeyFrame(result);
}

void DataCulling::CullingMapSimilarMap(const KeyFrameId& id) {
  auto const covisibility_frame_ids = data_fuse_->GetKeyLevelConnectedKeyFrames(
      id, {options_.fisrt_covisible_num, options_.second_covisible_num});
  //
  if (covisibility_frame_ids.empty()) {
    // 如果跟丢了后的第一帧的话，进来的话是跟前面没有共识;
    return;
  }
  const auto& all_key_frame_datas = data_fuse_->GetAllKeyFramesData();
  MapById<KeyFrameId, KeyFrameData> covisibility_frame;
  for (const auto& covisi_id : covisibility_frame_ids) {
    if (!covisibility_frame.Contains(covisi_id.first)) {
      CHECK(all_key_frame_datas.Contains(covisi_id.first) != 0)
          << covisi_id.first;
      covisibility_frame.Insert(covisi_id.first,
                                all_key_frame_datas.at(covisi_id.first));
    }
  }

  //
  MapById<MapPointId, mapping::MapPointData> cur_map_points;
  if (covisibility_frame.empty()) return;
  CHECK(!covisibility_frame.empty());
  std::map<MapPointId, std::map<KeyFrameId, FeatureId>> result;

  auto fuse_result = SearchMatchesKeyFrames(covisibility_frame,
                                            data_fuse_->GetMapPoints(id), id);
  result.insert(fuse_result.begin(), fuse_result.end());
  MapById<MapPointId, mapping::MapPointData> covisibility_map_points;
  for (const auto& covisi_id : covisibility_frame_ids) {
    auto const map_points = data_fuse_->GetMapPoints(covisi_id.first);
    for (auto map_point_id : map_points) {
      if (covisibility_map_points.Contains(map_point_id.id)) continue;
      covisibility_map_points.Insert(map_point_id.id, map_point_id.data);
    }
  }

  MapById<KeyFrameId, KeyFrameData> curr_frame_data;
  curr_frame_data.Insert(id, all_key_frame_datas.at(id));
  fuse_result =
      SearchMatchesKeyFrames(curr_frame_data, data_fuse_->GetMapPoints(id), id);

  result.insert(fuse_result.begin(), fuse_result.end());
  if (result.empty()) return;
  data_fuse_->FuseMapPoint(id, result);
  //

  //
}
}  // namespace mapping
}  // namespace jarvis