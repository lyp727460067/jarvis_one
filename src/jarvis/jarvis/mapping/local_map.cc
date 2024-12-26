#include "jarvis/mapping/local_map.h"
namespace jarvis {
namespace mapping {
//
//
LocalMap::LocalMap(const LocalMapOption &option,
                   const transform::Rigid3d &local_pose)
    : options_(option),
      local_pose_(local_pose),
      key_frame_data_base_(
          std::make_unique<KeyFrameDataBase>(option.key_frame_data_option)),
      covisibility_(std::make_unique<mapping::Covisibility>()),
      culling_sampler_(new common::FixedRatioSampler(option.culling_sampler)) {
  //
  CHECK(!options_.cameras.empty());
  CHECK(!options_.image_boxs.empty());
  data_fuse_ = std::make_unique<LocalDataFuse>(this);
  DataCullingOption data_culling_option = option.data_culling_option;
  data_culling_option.image_bboxs = option.image_boxs;
  data_culling_ =
      std::make_unique<DataCulling>(data_culling_option, data_fuse_.get());
  local_opimization_ = std::make_unique<LocalMapOptimization>(
      option.local_map_optimization_option);
}
//
void LocalMap::AddKeyFrameData(const KeyFrameId &kf_id,
                               const KeyFrameData &key_frame_data) {
  //
  //
  std::map<MapPointId, FeatureId> key_point_map_point_index;
  for (const auto &map_data : key_frame_data.data->map_points) {
    const MapPointId mp_id = key_frame_data.data->map_point_ids.at(map_data.id);
    const FeatureId feat_id = map_data.id;
    key_point_map_point_index.emplace(mp_id, feat_id);
    const Eigen::Vector3d xyz = local_pose_.inverse() * map_data.data;
    //
    Descriptor des;
    if (!key_frame_data.data->descriptors.empty()) {
      const auto des = key_frame_data.data->descriptors.at(feat_id);
    }
    if (!map_points_.Contains(mp_id)) {
      map_points_.Insert(mp_id, MapPointData{std::make_unique<MapPoint>(
                                    MapPoint{xyz, des, mp_id, kf_id})});
    }
  }
  covisibility_->UpdateWithFrameData(kf_id,
                                     std::move(key_point_map_point_index));
  key_frames_datas_.Insert(kf_id, key_frame_data);
  ref_poses_.emplace(kf_id,
                          local_pose_.inverse() * key_frame_data.data->pose);
  trim_befor_key_frame_id_.insert(kf_id);
}
//
bool LocalMap::operator=(LocalMap &&rhs) {
  local_pose_ = rhs.local_pose_;
  local_opimization_ = std::move(rhs.local_opimization_);
  culling_sampler_ = std::move(rhs.culling_sampler_);
  key_frames_datas_ = std::move(rhs.key_frames_datas_);
  key_frames_ref_pose = std::move(rhs.key_frames_ref_pose);
  key_frame_data_base_ = std::move(rhs.key_frame_data_base_);
  covisibility_ = std::move(rhs.covisibility_);
  map_points_ = std::move(rhs.map_points_);
  key_frames_id_with_pose_ = std::move(rhs.key_frames_id_with_pose_);
  trim_befor_key_frame_id_ = std::move(rhs.trim_befor_key_frame_id_);
  //
  data_fuse_ = std::make_unique<LocalDataFuse>(this);
  DataCullingOption data_culling_option = options_.data_culling_option;
  data_culling_option.image_bboxs = options_.image_boxs;
  data_culling_ =
      std::make_unique<DataCulling>(data_culling_option, data_fuse_.get());
  //
  finish_ = rhs.finish_;
  is_optimization = rhs.is_optimization;
  return true;
  // data_culling_ = std::move(rhs.data_culling_);
}
//
void LocalMap::UpdadataExtendFinishData() {
  for (const auto &data : key_frames_datas_) {
    key_frame_data_base_->AddData(data.id, data.data.data);
  }
  TrimRedundancy();
  is_optimization  =true;
  // Opimization();
}
//

//
bool LocalMap::TrimMapPoint(const MapPointId &id) {
  if (std::prev(map_points_.EndOfTrajectory(id.trajectory_id))->id == id) {
    return false;
  }
  map_points_.Trim(id);
  return true;
}
//
void LocalMap::FuseMapPoint(
    const KeyFrameId &kf_id,
    const std::map<MapPointId, std::map<KeyFrameId, FeatureId>> &matches) {
  std::map<MapPointId, MapPointId> fuse_map_points;

  for (const auto &matche : matches) {
    for (auto const &key_frame_id : matche.second) {
      //
      const auto map_points_with_idex =
          GetKeyFrameMapPointsData(key_frame_id.first).first;
      if (map_points_with_idex.count(key_frame_id.second) == 0) {
        // LOG(WARNING) << "index not exist";
        continue;
      }
      fuse_map_points.emplace(matche.first,
                              map_points_with_idex.at(key_frame_id.second));
    }
  }
  if (fuse_map_points.empty()) return;
  std::stringstream info;
  bool merg_point = false;
  info << "Merge map point:";
  //
  // auto const key_frame_map_points = GetKeyFrameMapPointsData(kf_id).second;
  for (const auto &matched_id : fuse_map_points) {
    if (matched_id.first == matched_id.second) {
      LOG(WARNING) << "merg same map point. continue";
      continue;
    }
    if (map_points_.Contains(matched_id.second) &&
        map_points_.Contains(matched_id.first)) {
      // if (key_frame_map_points.Contains(matched_id.second)) continue;

      std::pair<MapPointId, MapPointId> match_temp = matched_id;
      if (covisibility_->GetMapObservations(match_temp.first).size() <
          covisibility_->GetMapObservations(match_temp.second).size()) {
        match_temp.first = match_temp.second;
        match_temp.second = matched_id.first;
      }
      if (!TrimMapPoint(match_temp.second)) {
        continue;
      }
      covisibility_->UpdateWithFuseMapPoint(match_temp.first,
                                            match_temp.second);
      info << " " << match_temp.first << "<-" << match_temp.second << " ";
      merg_point = true;
      ComputeMapPointDistinctiveDescriptors(match_temp.first);
    }
  }
  LOG(INFO) << log_info::MAGENTA << info.str() << log_info::RESET;
}

void LocalMap::TrimKeyFrame(const KeyFrameId &id) {
  if (!key_frames_datas_.Contains(id)) return;
  // 不能删除第一个和最后一个
  if (key_frames_datas_.BeginOfTrajectory(id.trajectory_id)->id == id) return;
  if (std::prev(key_frames_datas_.EndOfTrajectory(id.trajectory_id))->id == id)
    return;

  std::stringstream info;
  key_frames_datas_.Trim(id);
  key_frame_data_base_->Erase(id);
  ref_poses_.erase(id);
  auto trim_map_points = covisibility_->TrimKeyFrame(id);

  info << "trim kf :" << id
       << "with map points size : " << trim_map_points.size();
  for (auto const &map_point_id : trim_map_points) {
    if (TrimMapPoint(map_point_id)) {
      info << " " << map_point_id << " ";
      covisibility_->TrimMapPoint(map_point_id);
    }
  }
  LOG(INFO) << log_info::RED << info.str() << log_info::RESET;
}
//

void LocalMap::Opimization(const std::vector<LocalMapConstraint>& constrants) {
  CHECK(false) << "Not Implement";
}
//

//
void LocalMap::ComputeMapPointDistinctiveDescriptors(const MapPointId &id) {
  auto obs = covisibility_->GetMapObservations(id);
  if (obs.empty()) {
    LOG(WARNING) << " obs empty";
    return;
  }
  std::vector<BrifBitset> descriptors;
  int obs_size = int(obs.size());
  descriptors.reserve(obs_size);
  for (auto &ob : obs) {
    auto const &key_frame_data = key_frames_datas_.at(ob);
    auto feat_id = covisibility_->GetMapPointFeatureIndex(ob, id);
    if (key_frame_data.data->descriptors.Contains(feat_id)) {
      descriptors.push_back(key_frame_data.data->descriptors.at(feat_id));
    }
  }
  // Compute distances between them
  //
  obs_size = int(descriptors.size());
  if (obs_size < options_.compute_map_point_min_des_num) return;
  std::vector<std::vector<int>> distances(obs_size, std::vector<int>(obs_size));
  for (int i = 0; i < obs_size; i++) {
    distances[i][i] = 0;
    for (int j = i + 1; j < obs_size; j++) {
      int distij = HammingDis(descriptors[i], descriptors[j]);
      distances[i][j] = distij;
      distances[j][i] = distij;
    }
  }
  // Take the descriptor with least median distance to the rest
  int best_median = INT_MAX;
  int best_idx = 0;
  for (int i = 0; i < obs_size; i++) {
    auto median = distances[i].begin() + obs_size / 2;
    std::nth_element(distances[i].begin(), median, distances[i].end());
    if (*median < best_median) {
      best_median = *median;
      best_idx = i;
    }
  }
  //
  map_points_.at(id).data->des = descriptors[best_idx];
}
//
std::pair<std::map<FeatureId, MapPointId>, MapById<MapPointId, MapPointData>>
LocalMap::GetKeyFrameMapPointsData(const KeyFrameId &frame_id) const {
  MapById<MapPointId, MapPointData> datas;
  CHECK(covisibility_);
  auto ids = covisibility_->GetKeyFrameMapPointId(frame_id).first;
  std::map<FeatureId, MapPointId> index_to_id;
  for (const auto &id : ids) {
    CHECK(map_points_.Contains(id)) << id;
    datas.Insert(id, map_points_.at(id));
    index_to_id.emplace(covisibility_->GetMapPointFeatureIndex(frame_id, id),
                        id);
  }
  return {std::move(index_to_id), datas};
}
//
void LocalMap::TrimRedundancy() {
  //
  std::set<KeyFrameId> culling_key_frame_ids;
  for (const auto &kf_data : key_frames_datas_) {
    if (culling_sampler_->Pulse()) {
      culling_key_frame_ids.insert(kf_data.id);
    }
  }

  for (const auto c_id : culling_key_frame_ids) {
    if (!key_frames_datas_.Contains(c_id)) continue;
    data_culling_->CullingMapSimilarMap(c_id);
    data_culling_->KeyFrameCulling(c_id);
  }
}
//
//
void LocalMap::Finish() { finish_ = true; }

//
std::map<MapPointId, MapPointData> LocalMap::GetKeyFrameMapPoints(
    const KeyFrameId &id) {
  auto map_poin_ids = covisibility_->GetKeyFrameMapPointId(id).first;
  std::map<MapPointId, MapPointData> result;

  for (const auto &id : map_poin_ids) {
    result.emplace(id, map_points_.at(id));
  }
  return result;
}
//

//
//
LocalMap::LocalDataFuse::LocalDataFuse(LocalMap *local_map)
    : local_map_(local_map) {}
//
//

void LocalMap::LocalDataFuse::FuseMapPoint(
    const KeyFrameId &key_frame_id,
    const std::map<MapPointId, std::map<KeyFrameId, FeatureId>> &matches) {
  std::lock_guard<std::mutex> lock(local_map_->mutex_);
  local_map_->FuseMapPoint(key_frame_id, matches);
}
//
//
void LocalMap::LocalDataFuse::CullKeyFrame(const std::set<KeyFrameId> &target) {
  std::lock_guard<std::mutex> lock(local_map_->mutex_);
  for (const auto id : target) {
    local_map_->TrimKeyFrame(id);
  }
}
//
//
const MapById<MapPointId, mapping::MapPointData>
LocalMap::LocalDataFuse::GetMapPoints(const KeyFrameId &id) {
  return local_map_->GetKeyFrameMapPointsData(id).second;
}
//
//
//
const std::set<KeyFrameId> LocalMap::LocalDataFuse::GetMapObservations(
    const MapPointId &map_point_id) {
  return local_map_->covisibility_->GetMapObservations(map_point_id);
}
//
//
bool LocalMap::LocalDataFuse::PorjectPoint(const transform::Rigid3d &cam_pose,
                                           const Eigen::Vector3d &point, int s,
                                           Eigen::Vector2d *p) {
  const Eigen::Vector3d p_point =
      cam_pose.inverse() * local_map_->local_pose_ * point;
  if (point.z() < 0.1) return false;
  Eigen::Vector2d b;
  local_map_->options_.cameras.at(s)->spaceToPlane(p_point, b);
  *p = b;
  return true;
}
//

const MapById<KeyFrameId,const KeyFrameData> &
LocalMap::LocalDataFuse::GetAllKeyFramesData() const {
  return local_map_->AllKeyFrameDatas();
}

//
std::vector<std::pair<KeyFrameId, int>>
LocalMap::LocalDataFuse::GetKeyLevelConnectedKeyFrames(
    const KeyFrameId &frame_id, const std::vector<int> &levels) {
  //
  // std::vector<std::pair<KeyFrameId, int>> result;
  const auto connect_frames_ids =
      local_map_->covisibility_->GetOrderConnectedKeyFrames(frame_id,
                                                            *levels.begin());
  if (levels.size() == 1) {
    return connect_frames_ids;
  }

  std::vector<std::pair<KeyFrameId, int>> result;
  for (auto connect_frames_id : connect_frames_ids) {
    auto scond_connet = GetKeyLevelConnectedKeyFrames(
        connect_frames_id.first, {levels.begin() + 1, levels.end()});
    result.insert(result.begin(), scond_connet.begin(), scond_connet.end());
  }

  return result;
}

//
void ActiveLocalMap::AddKeyFrameData(const KeyFrameId &id,
                                     const KeyFrameData &data) {
  //
  CHECK(data.data)<<"Keyframe data empty";
  if (localmaps_.empty() ||
      localmaps_.back()->Size() == local_map_option_.max_kf_num) {
    AddLocalMap(local_map_option_, data.data->pose);
  }
  //
  for (auto &local_map : localmaps_) {
    local_map->AddKeyFrameData(id, data);
  }
  //
  if (localmaps_.front()->Size() == 2 * local_map_option_.max_kf_num) {
    localmaps_.front()->Finish();
    LOG(INFO)<<"Finish data";
  }
}
//
void ActiveLocalMap::AddLocalMap(const LocalMapOption &local_option,
                                 const transform::Rigid3d &local_pose) {
  if (localmaps_.size() >= 2) {
    front_finsh_ = *localmaps_.begin();
    localmaps_.erase(localmaps_.begin());
  }
  //
  localmaps_.push_back(std::make_shared<LocalMap>(local_option, local_pose));
  //
}
//
}  // namespace mapping
}  // namespace jarvis
