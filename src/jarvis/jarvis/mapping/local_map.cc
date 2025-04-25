#include "jarvis/mapping/local_map.h"
namespace jarvis {
namespace mapping {
//
//
LocalMap::LocalMap(const LocalMapOption &option,
                   const transform::Rigid3d &local_pose)
    : options_(option),
      culling_sampler_(new common::FixedRatioSampler(option.culling_sampler)) {
  //
  CHECK(!options_.cameras.empty());
  CHECK(!options_.image_boxs.empty());
  data_.local_pose = transform::Rigid3d(local_pose.translation(),
                                        Eigen::Quaterniond::Identity());
  data_fuse_ = std::make_unique<LocalDataFuse>(this);
  DataCullingOption data_culling_option = option.data_culling_option;
  data_culling_option.image_bboxs = option.image_boxs;
  data_culling_ =
      std::make_unique<DataCulling>(data_culling_option, data_fuse_.get());
}

// 添加地图点,更新共视关系
void LocalMap::AddKeyFrameData(const KeyFrameId &kf_id,
                               const KeyFrameData &key_frame_data) {
  //
  //
  std::map<MapPointId, FeatureId> key_point_map_point_index;
  for (const auto &map_data : key_frame_data.data->map_points) {
    const MapPointId mp_id = key_frame_data.data->map_point_ids.at(map_data.id);
    if(out_outliers_map_points_catch_.count(mp_id))continue;
    const FeatureId feat_id = map_data.id;
    key_point_map_point_index.emplace(mp_id, feat_id);
    const Eigen::Vector3d ref_xyz =
        key_frame_data.data->pose.inverse() * map_data.data;
    //

    const Eigen::Vector3d xyz =
        data_.local_pose.inverse() * key_frame_data.data->pose * ref_xyz;
    //
    Descriptor des;
    if (!key_frame_data.data->descriptors.empty()) {
      const auto des = key_frame_data.data->descriptors.at(feat_id);
    }
    if (!data_.map_points.Contains(mp_id)) {
      data_.map_points.Insert(mp_id, MapPointData{std::make_unique<MapPoint>(
                                         MapPoint{local_to_ref_*xyz, des, mp_id, kf_id})});
    } else {
      // map_points_.at(mp_id).data->pos = xyz;
      // map_points_.at(mp_id).data->reference_frame_id = kf_id;
    }
  }
  data_.covisibility.UpdateWithFrameData(kf_id,
                                         std::move(key_point_map_point_index));
  data_.key_frames_datas.Insert(kf_id, key_frame_data);
  data_.key_frames_ref_pose.emplace(
      kf_id,
      local_to_ref_ * data_.local_pose.inverse() * key_frame_data.data->pose);
  data_.removed_keyframes_ids_before_trim.insert(kf_id);
}
//
//
void LocalMap::InsertOutOutliers(
    const std::set<MapPointId> &new_out_outliers_map_points) {
  out_outliers_map_points_catch_.insert(new_out_outliers_map_points.begin(),
                                        new_out_outliers_map_points.end());
  for (const auto &mp_id : new_out_outliers_map_points) {
    data_.covisibility.TrimMapPoint(mp_id);
    data_.map_points.Trim(mp_id);
  }
  LOG(INFO)<<data_.map_points.size();
}
//
//
void LocalMap::UpdateExistData(const LocalMap &rhs) {
  //
  transform::Rigid3d rhs_to_this = data_.local_pose.inverse() * rhs.LocalPose();
  //
  //
  if (data_.key_frames_datas.begin()->data.data->time ==
      rhs.data_.key_frames_datas.begin()->data.data->time) {
    rhs_to_this = transform::Rigid3d::Identity();
    data_.local_pose = rhs.LocalPose();
  }
  //

  bool have_updated =false;
  transform::Rigid3d local_front_to_ref;
  for (auto &kf_ref_pose : data_.key_frames_ref_pose) {
    if (rhs.data_.key_frames_ref_pose.count(kf_ref_pose.first) != 0) {
      //
      have_updated = true;
      const transform::Rigid3d op_ref_pose =
          rhs_to_this * rhs.data_.key_frames_ref_pose.at(kf_ref_pose.first);
      local_to_ref_ = op_ref_pose *
                      (data_.local_pose.inverse() *
                       data_.key_frames_datas.at(kf_ref_pose.first).data->pose)
                          .inverse();
      //
      local_front_to_ref = op_ref_pose * kf_ref_pose.second.inverse();
      //
      kf_ref_pose.second =
          rhs_to_this * rhs.data_.key_frames_ref_pose.at(kf_ref_pose.first);
    } else {
      if (have_updated) {
        kf_ref_pose.second = local_front_to_ref * kf_ref_pose.second;
      }
    }
  }
  if (!have_updated) return;
  for (const auto &mp : data_.map_points) {
    if(!data_.map_points.Contains(mp.id))continue;
    if (rhs.data_.map_points.Contains(mp.id)) {
      data_.map_points.at(mp.id).data->pos =
          rhs_to_this * rhs.data_.map_points.at(mp.id).data->pos;
    } else {
      data_.map_points.at(mp.id).data->pos =
          local_front_to_ref * data_.map_points.at(mp.id).data->pos;
    }
  }

  // local_to_ref_ =transform::Rigid3d::Identity();
}
  //
bool LocalMap::operator=(const LocalMap &rhs) {
  data_ = rhs.data_;
  // local_opimization_ = std::move(rhs.local_opimization_);
  culling_sampler_ =
      std::make_unique<common::FixedRatioSampler>(options_.culling_sampler);

  data_fuse_ = std::make_unique<LocalDataFuse>(this);
  DataCullingOption data_culling_option = options_.data_culling_option;
  data_culling_option.image_bboxs = options_.image_boxs;
  data_culling_ =
      std::make_unique<DataCulling>(data_culling_option, data_fuse_.get());
  //
  finish_ = rhs.finish_;
  is_optimization = rhs.is_optimization;
  local_to_ref_ = rhs.local_to_ref_;
  out_outliers_map_points_catch_.clear();
  //
  for (const auto &mp_id : rhs.out_outliers_map_points_catch_) {
    if (data_.map_points.Contains(mp_id)) {
      data_.covisibility.TrimMapPoint(mp_id);
      data_.map_points.Trim(mp_id);
      out_outliers_map_points_catch_.insert(mp_id);
    }
  }

  return true;
}

//
bool LocalMap::operator=(LocalMap &&rhs) {
  data_ = std::move(rhs.data_);
  // local_opimization_ = std::move(rhs.local_opimization_);
  culling_sampler_ = std::move(rhs.culling_sampler_);
  //
  data_fuse_ = std::make_unique<LocalDataFuse>(this);
  DataCullingOption data_culling_option = options_.data_culling_option;
  data_culling_option.image_bboxs = options_.image_boxs;
  data_culling_ =
      std::make_unique<DataCulling>(data_culling_option, data_fuse_.get());
  //
  finish_ = rhs.finish_;
  is_optimization = rhs.is_optimization;
  local_to_ref_ = rhs.local_to_ref_;
  out_outliers_map_points_catch_.clear();
  //
  for (const auto &mp_id : rhs.out_outliers_map_points_catch_) {
    if (data_.map_points.Contains(mp_id)) {
      data_.covisibility.TrimMapPoint(mp_id);
      data_.map_points.Trim(mp_id);
      out_outliers_map_points_catch_.insert(mp_id);
    }
  }

  //
  return true;
}
//
void LocalMap::UpdadataExtendFinishData(bool f) {
  if (f) {
    TrimRedundancy();
  }
  is_optimization = true;
}
//

//
bool LocalMap::TrimMapPoint(const MapPointId &id) {
  if (std::prev(data_.map_points.EndOfTrajectory(id.trajectory_id))->id == id) {
    return false;
  }
  data_.map_points.Trim(id);
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
    if (data_.map_points.Contains(matched_id.second) &&
        data_.map_points.Contains(matched_id.first)) {
      // if (key_frame_map_points.Contains(matched_id.second)) continue;

      std::pair<MapPointId, MapPointId> match_temp = matched_id;
      if (data_.covisibility.GetMapObservations(match_temp.first).size() <
          data_.covisibility.GetMapObservations(match_temp.second).size()) {
        match_temp.first = match_temp.second;
        match_temp.second = matched_id.first;
      }
      if (!TrimMapPoint(match_temp.second)) {
        continue;
      }
      data_.covisibility.UpdateWithFuseMapPoint(match_temp.first,
                                            match_temp.second);
      info << " " << match_temp.first << "<-" << match_temp.second << " ";
      merg_point = true;
      ComputeMapPointDistinctiveDescriptors(match_temp.first);
    }
  }
  LOG(INFO) << log_info::MAGENTA << info.str() << log_info::RESET;
}

void LocalMap::TrimKeyFrame(const KeyFrameId &id) {
  if (!data_.key_frames_datas.Contains(id)) return;
  // 不能删除第一个和最后一个
  if (data_.key_frames_datas.BeginOfTrajectory(id.trajectory_id)->id == id) return;
  if (std::prev(data_.key_frames_datas.EndOfTrajectory(id.trajectory_id))->id == id)
    return;

  std::stringstream info;
  data_.key_frames_datas.Trim(id);
  data_.key_frames_ref_pose.erase(id);
  auto trim_map_points = data_.covisibility.TrimKeyFrame(id);

  info << "trim kf :" << id
       << "with map points size : " << trim_map_points.size();
  for (auto const &map_point_id : trim_map_points) {
    if (TrimMapPoint(map_point_id)) {
      info << " " << map_point_id << " ";
      data_.covisibility.TrimMapPoint(map_point_id);
    }
  }
  LOG(INFO) << log_info::RED << info.str() << log_info::RESET;
}
//

void LocalMap::Opimization(const std::vector<LocalMapConstraint>& constrants) {
  CHECK(false) << "Not Implement";
}
//

// 取所有共视特征之间的中间值作为地图点的特征
void LocalMap::ComputeMapPointDistinctiveDescriptors(const MapPointId &id) {
  auto obs = data_.covisibility.GetMapObservations(id);
  if (obs.empty()) {
    LOG(WARNING) << " obs empty";
    return;
  }
  std::vector<BrifBitset> descriptors;
  int obs_size = int(obs.size());
  descriptors.reserve(obs_size);
  for (auto &ob : obs) {
    auto const &key_frame_data = data_.key_frames_datas.at(ob);
    auto feat_id = data_.covisibility.GetMapPointFeatureIndex(ob, id);
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
  data_.map_points.at(id).data->des = descriptors[best_idx];
}
//
std::pair<std::map<FeatureId, MapPointId>, MapById<MapPointId, MapPointData>>
LocalMap::GetKeyFrameMapPointsData(const KeyFrameId &frame_id) const {
  MapById<MapPointId, MapPointData> datas;
  auto ids = data_.covisibility.GetKeyFrameMapPointId(frame_id).first;
  std::map<FeatureId, MapPointId> index_to_id;
  for (const auto &id : ids) {
    CHECK(data_.map_points.Contains(id)) << id;
    datas.Insert(id, data_.map_points.at(id));
    index_to_id.emplace(data_.covisibility.GetMapPointFeatureIndex(frame_id, id),
                        id);
  }
  return {std::move(index_to_id), datas};
}
//
void LocalMap::TrimRedundancy() {
  //
  std::set<KeyFrameId> culling_key_frame_ids;
  for (const auto &kf_data : data_.key_frames_datas) {
    if (culling_sampler_->Pulse()) {
      culling_key_frame_ids.insert(kf_data.id);
    }
  }

  for (const auto c_id : culling_key_frame_ids) {
    if (!data_.key_frames_datas.Contains(c_id)) continue;
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
  auto map_poin_ids = data_.covisibility.GetKeyFrameMapPointId(id).first;
  std::map<MapPointId, MapPointData> result;

  for (const auto &id : map_poin_ids) {
    CHECK(data_.map_points.Contains(id));
    result.emplace(id, data_.map_points.at(id));
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
  return local_map_->data_.covisibility.GetMapObservations(map_point_id);
}
//
//
bool LocalMap::LocalDataFuse::PorjectPoint(const transform::Rigid3d &cam_pose,
                                           const Eigen::Vector3d &point, int s,
                                           Eigen::Vector2d *p) {
  const Eigen::Vector3d p_point =
      cam_pose.inverse() * local_map_->LocalPose() * point;
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
      local_map_->GetCovisibility()->GetOrderConnectedKeyFrames(frame_id,
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
LocalMap::~LocalMap(){

 }
//
void ActiveLocalMap::AddKeyFrameData(const KeyFrameId &id,
                                     const KeyFrameData &data) {
  //
  CHECK(data.data)<<"Keyframe data empty";
  // 最新的局部地图窗口大小足够达到设置上限的一半,建立新的局部地图窗口
  if (localmaps_.empty() ||
      localmaps_.back()->Size() == local_map_option_.max_kf_num) {
    AddLocalMap(local_map_option_, data.data->pose);
  }
  // localmaps_最多只会存在两个
  for (auto &local_map : localmaps_) {
    local_map->AddKeyFrameData(id, data);
  }
  // 旧的局部地图窗口达到上限,边缘化得到固定约束
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
