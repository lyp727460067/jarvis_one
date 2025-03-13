#include "jarvis/mapping/covisibility.h"
namespace jarvis {
namespace mapping {
//
constexpr int kMinCoviNumm = 4;
//

//
//
//
void Covisibility::UpdateWithFrameData(
    const KeyFrameId& key_frame_id,
    std::map<MapPointId, FeatureId>&& frame_map_feature_data_id) {
  CHECK(!frame_map_feature_data_id.empty());
  for (auto const& feature_id : frame_map_feature_data_id) {
    //
    if (map_point_observe_frames_.count(feature_id.first) != 0) {
      const auto& map_point_observe_frame =
          map_point_observe_frames_[feature_id.first];
      //

      for (const auto& observe_key_frame_id_pair : map_point_observe_frame) {
       const  KeyFrameId  &observe_key_frame_id   = observe_key_frame_id_pair.first; 
        if (observe_key_frame_id == key_frame_id) continue;  // 已经添加过了
        if (covisible_frames_[observe_key_frame_id].count(key_frame_id)) {
          covisible_frames_[observe_key_frame_id][key_frame_id]++;
          covisible_frames_[key_frame_id][observe_key_frame_id]++;
        } else {
          covisible_frames_[observe_key_frame_id].emplace(key_frame_id, 1);
          covisible_frames_[key_frame_id].emplace(observe_key_frame_id, 1);
        }
      }
    }
    //
    CHECK(map_point_observe_frames_[feature_id.first].count(key_frame_id) == 0)
        << "Duplicate map points" << "[m:" << feature_id.first << ",k"
        << key_frame_id << "]";
    //
    map_point_observe_frames_[feature_id.first].emplace(key_frame_id,
                                                             feature_id.second);
  }
  //

  if (covisible_frames_.count(key_frame_id) == 0) {//第一帧
    covisible_frames_[key_frame_id];
  }
  key_frame_feature_data_[key_frame_id].merge(
      frame_map_feature_data_id);
}
//
std::set<KeyFrameId> Covisibility::GetMapObservations(
    const MapPointId& map_point_id) {
  std::set<KeyFrameId> r;
  const auto& keyframezs = map_point_observe_frames_.at(map_point_id);
  for (const auto& key_frame_id : keyframezs) {
    r.insert(key_frame_id.first);
  }
  return r;
}

//

FeatureId Covisibility::GetMapPointFeatureIndex(const KeyFrameId& key_frame_id,
                                                const MapPointId& mp)const {
  CHECK(key_frame_feature_data_.count(key_frame_id))<< key_frame_id;
  CHECK(key_frame_feature_data_.at(key_frame_id).count(mp)) << mp;
  return key_frame_feature_data_.at(key_frame_id).at(mp);
}
//
void Covisibility::ReplaceFrameIndex(const MapPointId& sou,
                                     const MapPointId& tar) {
  const auto& sour_mp_abserve_frames = map_point_observe_frames_[sou];
  CHECK_NE(sou, tar);
  for (const auto& frame_id_pair : sour_mp_abserve_frames) {
    const KeyFrameId& frame_id = frame_id_pair.first;
    if (key_frame_feature_data_.count(frame_id) == 0) continue;
    //
    auto& frames_indexs = key_frame_feature_data_[frame_id];
    auto sou_node = frames_indexs.extract(sou);
    if (!sou_node.empty()) {
      sou_node.key() = tar;
    }
    frames_indexs.insert(std::move(sou_node));
  }
}
//
void Covisibility::UpdateWithFuseMapPoint(const MapPointId& target,
                                          const MapPointId& sou) {
  //
  //
  CHECK(map_point_observe_frames_.count(sou)) << sou;
  CHECK(map_point_observe_frames_.count(target)) << target;
  //
  const auto& sour_mp_abserve_frames = map_point_observe_frames_[sou];
  const auto& target_mp_abserve_frames = map_point_observe_frames_[target];
  for (const auto& frame_id_pair : sour_mp_abserve_frames) {
    const KeyFrameId & frame_id  = frame_id_pair.first  ;
    for (const auto& target_frame_pair : target_mp_abserve_frames) {
      const KeyFrameId & target_frame  = target_frame_pair.first  ;
      if (target_frame == frame_id) continue;
      if (covisible_frames_[frame_id].count(target_frame) != 0) {
        covisible_frames_[target_frame][frame_id]++;
        covisible_frames_[frame_id][target_frame]++;
      } else {
        covisible_frames_[target_frame].emplace(frame_id, 1);
        covisible_frames_[frame_id].emplace(target_frame, 1);
      }
    }
  }
  //
  ReplaceFrameIndex(sou, target);
  map_point_observe_frames_[target].insert(sour_mp_abserve_frames.begin(),
                                           sour_mp_abserve_frames.end());
  map_point_observe_frames_.erase(sou);
  //
}
//
//
void Covisibility::TrimMapPoint(const MapPointId& map_point_id) {
  auto& map_in_key_frams = map_point_observe_frames_.at(map_point_id);
  //
  for (auto it = map_in_key_frams.begin(); it != map_in_key_frams.end(); ++it) {
    if (std::next(it) == map_in_key_frams.end()) {
      break;
    }
    for (auto next_it = std::next(it); next_it != map_in_key_frams.end();
         ++next_it) {
      CHECK(covisible_frames_[it->first].count(next_it->first));
      covisible_frames_[it->first][next_it->first]--;
      covisible_frames_[next_it->first][it->first]--;
    }
  }

  for (auto it = map_in_key_frams.begin(); it != map_in_key_frams.end(); ++it) {
    key_frame_feature_data_.at(it->first).erase(map_point_id);
  }
  map_point_observe_frames_.erase(map_point_id);
}
//
std::set<MapPointId> Covisibility::TrimLessMapPoint(const KeyFrameId& id) {
  std::set<MapPointId> result;
  auto const& map_points_for_frame = key_frame_feature_data_[id];
  for (auto const& map_point_id : map_points_for_frame) {
    CHECK_NE(map_point_observe_frames_.count(map_point_id.first), size_t(0))
        << map_point_id.first << " Not exist";
    if (map_point_observe_frames_[map_point_id.first].size() <= size_t(2)) {
      result.insert(map_point_id.first);
    }
  }
  return result;
}
//
std::set<MapPointId> Covisibility::TrimKeyFrame(const KeyFrameId& id) {
  if (covisible_frames_.count(id) == 0 ||
      key_frame_feature_data_.count(id) == 0) {
    LOG(INFO) << "connect not has key_frame id: " << id;
    return {};
  }

  auto& covisible_this_id_frames = covisible_frames_.at(id);
  for (const auto& frame : covisible_this_id_frames) {
    LOG_IF(ERROR, covisible_frames_[frame.first].count(id) == 0)
        << id << " Not Exist."
        << "Maby  Not Each other update Conect ,ex c[12] = c[11], c[11] "
           "!=c[12]";
    covisible_frames_[frame.first].erase(id);
  }
  covisible_frames_.erase(id);
  // 删除mappoint 观察到的这个id的frame
  auto const& map_points_for_frame = key_frame_feature_data_[id];
  for (auto const& map_point_id : map_points_for_frame) {
    CHECK_NE(map_point_observe_frames_.count(map_point_id.first), size_t(0));
    CHECK_NE(map_point_observe_frames_[map_point_id.first].count(id), size_t(0));
    map_point_observe_frames_[map_point_id.first].erase(id);
  }

  // 判断mappoint共视如果小于2的话返回除去ID
  std::set<MapPointId> result;
  for (auto const& map_point_id : map_points_for_frame) {
    CHECK_NE(map_point_observe_frames_.count(map_point_id.first), size_t(0))
        << map_point_id.first << " Not exist";
    if (map_point_observe_frames_[map_point_id.first].size() <= size_t(2)) {
      result.insert(map_point_id.first);
    }
  }
  key_frame_feature_data_.erase(id);
  return result;
}
//
//
std::vector<KeyFrameId> Covisibility::GetConnectedKeyFrames(
    const KeyFrameId& frame_id, int n) const {
  std::vector<KeyFrameId> result;
  
  CHECK(covisible_frames_.count(frame_id)) << frame_id;
  for (auto frame : covisible_frames_.at(frame_id)) {
    if (frame.second > kMinCoviNumm) {
      result.push_back(frame.first);
    }
  }
  if (n == -1 || int(result.size()) < n) {
    return result;
  }
  return {result.begin(), result.begin() + n};
}
//
std::vector<std::pair<KeyFrameId,int>>
Covisibility::GetOrderConnectedKeyFrames(const KeyFrameId& frame_id,
                                         int n) const {
  std::vector<std::pair<KeyFrameId, int>> result;
  CHECK(covisible_frames_.count(frame_id)) << frame_id;
  for (auto frame : covisible_frames_.at(frame_id)) {
    if (frame.second > kMinCoviNumm) {
      result.push_back(frame);
    }
  }
  std::sort(result.begin(), result.end(),
            [](const std::pair<KeyFrameId, int>& lhs,
               const std::pair<KeyFrameId, int>& rhs) {
              return lhs.second > rhs.second;
            });
  if (n == -1 || int(result.size()) < n) return result;
  return {result.begin(), result.begin() + n};
}
//
bool Covisibility::IsMapPointConnectKeyFrame(const MapPointId& mp_id,
                                             const KeyFrameId& kf_id) const {
  return map_point_observe_frames_.at(mp_id).count(kf_id) != 0;
}

int Covisibility::GetConnectedWeigt(const KeyFrameId& id_i,
                                    const KeyFrameId& id_j) {
  CHECK(covisible_frames_.count(id_i));
  if (covisible_frames_[id_i].count(id_j) == 0) return 0;
  return covisible_frames_[id_i][id_j];
}
//
std::vector<KeyFrameId> Covisibility::GetKeyLevelConnectedKeyFrames(
    const KeyFrameId& frame_id, const std::vector<int>& levels)const {
  const auto connect_frames_ids = GetConnectedKeyFrames(frame_id);
  if (levels.size() == 1) {
    return connect_frames_ids;
  }

  std::vector<KeyFrameId> result;
  result.insert(result.begin(), connect_frames_ids.begin(),
                connect_frames_ids.end());
  for (auto connect_frames_id : connect_frames_ids) {
    auto scond_connet = GetKeyLevelConnectedKeyFrames(
        connect_frames_id, {levels.begin() + 1, levels.end()});
    result.insert(result.begin(), scond_connet.begin(), scond_connet.end());
  }
  //
  return result;
}

//
std::pair<std::vector<MapPointId>, std::vector<FeatureId>>
Covisibility::GetKeyFrameMapPointId(const KeyFrameId& frame_id) const {
  CHECK(key_frame_feature_data_.count(frame_id));

  auto& key_frame_data_id = key_frame_feature_data_.at(frame_id);
  std::vector<MapPointId> map_point_ids;
  std::vector<FeatureId> feat_ids;
  for (const auto& ids : key_frame_data_id) {
    map_point_ids.push_back(ids.first);
    feat_ids.push_back(ids.second);
  }
  return {map_point_ids, feat_ids};
}


}  // namespace mapping
}  // namespace jarvis