#include "jarvis/mapping/map_manger.h"

#include <mutex>
// #include <opencv2/core/eigen.hpp>
#include <set>
#include <vector>

#include "Eigen/Core"
#include "glog/logging.h"
#include "jarvis/common/math.h"
#include "jarvis/mapping/covisibility.h"
#include "jarvis/mapping/key_frame_database.h"
#include "jarvis/mapping/map_manger.h"
#include "jarvis/mapping/mapping_data.h"
#include "jarvis/mapping/match/des_matcher.h"

// /
namespace jarvis {
namespace mapping {
namespace {
inline Eigen::Matrix<double, 3, 3> Skew(const Eigen::Matrix<double, 3, 1> &w) {
  Eigen::Matrix<double, 3, 3> w_x;
  w_x << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
  return w_x;
}
Eigen::Vector3d TriangulatePoint(
    const std::vector<transform::Rigid3d> &poses,
    const std::vector<Eigen::Vector3d> &key_point_normal) {
  Eigen::MatrixXd H(poses.size() * 2, 4);
  // CHECK_EQ(poses.size(), 2) << "Function Just adoptor 2 size pose";
  // Eigen::MatrixXd H;
  for (int i = 0; i < poses.size(); i++) {
    Eigen::Matrix<double, 3, 4> pose_matrix;
    pose_matrix.block<3, 3>(0, 0) = poses[i].rotation().toRotationMatrix();
    pose_matrix.block<3, 1>(0, 3) = poses[i].translation();
    const Eigen::Vector2d point = key_point_normal[i].head<2>();
    H.row(i * 2 + 0) = point[0] * pose_matrix.row(2) - pose_matrix.row(0);
    H.row(i * 2 + 1) = point[1] * pose_matrix.row(2) - pose_matrix.row(1);
  }
  //
  Eigen::Vector4d triangulated_point;
  triangulated_point =
      H.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
  return (triangulated_point / triangulated_point(3)).head<3>();
}

}  // namespace
//
MapManager::MapManager(const MapManagerOption &option,
                       std::map<int, camera_models::CameraPtr> cameras,
                       std::unique_ptr<dbow::Vocabulary> voc)
    : options_(option),
      cameras_(cameras),
      des_extractor_(
          std::make_unique<DescriptorExtract>(option.descriptor_option)),
      key_frame_data_base_(std::make_unique<KeyFrameDataBase>(
          option.key_frame_data_option, std::move(voc))),
      covisibility_(std::make_unique<Covisibility>()),
      key_points_extractor_(KeyPointExtract::Create({})) {}
//
//
cv::Mat GenerateMask(const std::vector<cv::KeyPoint> &exit_point) {
  return cv::Mat();
}
//
void MapManager::GenerateForExtendKeyPoint(const KeyFrameId &id) {
  auto sequence_feautes =
      key_frames_datas_.at(id).data->features.trajectory_ids();
  for (const auto &sequence_id : sequence_feautes) {
    auto one_sequence_feautes =
        key_frames_datas_.at(id).data->features.trajectory(sequence_id);
    std::vector<cv::KeyPoint> exist_key_points;
    for (const auto &feat : one_sequence_feautes) {
      exist_key_points.push_back(feat.data.key_point);
    }
    const cv::Mat image =
        key_frames_datas_.at(id).data->pyramid[sequence_id][0];
    std::vector<cv::KeyPoint> key_points =
        key_points_extractor_->Extract(image, GenerateMask(exist_key_points));
    exist_key_points.insert(exist_key_points.begin(), key_points.begin(),
                            key_points.end());
    Descriptors descriptors = des_extractor_->Extract(image, exist_key_points);
    for (int i = 0; i < key_points.size(); i++) {
      const FeatureId feat_id(sequence_id, i);
      if (!key_frames_datas_.at(id).data->features.Contains(feat_id)) {
        Eigen::Vector2d a(key_points[i].pt.x, key_points[i].pt.y);
        Eigen::Vector3d b;
        cameras_.at(sequence_id)->liftProjective(a, b);  // 注意这里找对应的相机
        key_frames_datas_.at(id).data->features.Insert(
            feat_id, FeatureData{key_points[i], b});
      }
      //
      key_frames_datas_.at(id).data->descriptors.Insert(feat_id,
                                                        descriptors[i]);
    }
  }
  //
  key_frames_datas_.at(id).data->dbow_data =
      key_frame_data_base_->Vocabulary()->Transform(
          key_frames_datas_.at(id).data->descriptors,
          options_.dbow_trasform_level);
}
//

//
//
// 前端只是建立工视关系就可以了
KeyFrameData MapManager::ExtractKeyFrameData(
    const TrackingData &data,
    std::map<int,
             std::map<uint64_t, std::tuple<Eigen::Vector3d, mapping::Descriptor,
                                           FeatureId>>> *front_map_points) {
  //
  //
  KeyFrameData result{std::make_shared<KeyFrameData::Data>(
      KeyFrameData::Data{data.data->time, data.data->imu_state.Pose(),
                         data.data->extric_camera_to_imu})};
  //

  std::map<int, std::map<int, uint64>> track_ids;
  for (auto &senqu_features : data.data->features_datas) {
    std::vector<FeatureData> feat_datas;
    for (auto &feature : senqu_features.second.key_points) {
      Eigen::Vector2d a(feature.second.pt.x, feature.second.pt.y);
      Eigen::Vector3d b;
      cameras_.at(senqu_features.first)
          ->liftProjective(a, b);  // 注意这里找对应的相机
      track_ids[feature.first][feat_datas.size()] = feature.first;
      feat_datas.emplace_back(FeatureData{feature.second, b});
      //
    }
    for (int j = 0; j < feat_datas.size(); j++) {
      FeatureId feat_id{senqu_features.first, j};
      result.data->features.Insert(feat_id, feat_datas[j]);
      //

      const uint64_t track_id = track_ids.at(senqu_features.first).at(j);
      (*front_map_points)[senqu_features.first].emplace(
          track_id,
          std::make_tuple(senqu_features.second.map_points.at(track_id),
                          Descriptor{}, feat_id));
      //
    }
  }
  //
  return result;
}

void MapManager::ExtendKeyFrameData(const KeyFrameId &id) {
  if (!options_.extend_point) return;
  GenerateForExtendKeyPoint(id);
  // 优先把以前地图的点和当前做匹配
  UpdateConnectMapPointProjectMatchSearch(id);
  ConStructExtendMapPoints(id);
  auto map_point_ids = covisibility_->GetKeyFrameMapPointId(id);
  for (const auto &map_point_id : map_point_ids.first) {
    if (map_points_.at(map_point_id).data->Fix()) {
      ComputeMapPointDistinctiveDescriptors(map_point_id);
    }
  }
}
//
//
void MapManager::StructureMapPoints(
    const KeyFrameId &id,
    const std::map<
        int, std::map<uint64_t, std::tuple<Eigen::Vector3d, mapping::Descriptor,
                                           FeatureId>>> &front_map_points) {
  //
  std::map<MapPointId, FeatureId> key_point_map_point_index;
  std::map<int, std::set<uint64_t>> key_points_class_ids;
  //
  for (const auto &map_point_with_s : front_map_points) {
    for (const auto &map_point : map_point_with_s.second) {
      key_points_class_ids[map_point_with_s.first].insert(map_point.first);
      if (!IsExist(map_point_with_s.first, map_point.first)) {
        const auto map_point_id = AddMapPoint(
            id.trajectory_id,
            std::pair<int, uint64_t>(map_point_with_s.first, map_point.first),
            MapPointData{
                std::make_shared<MapPoint>(id, std::get<0>(map_point.second),
                                           std::get<1>(map_point.second))});
      }
      CHECK(key_point_map_point_index
                .emplace(GetWithTrackingId(
                             {map_point_with_s.first, map_point.first}),
                         std::get<2>(map_point.second))
                .second)
          << "key point id is same.";
    }
  }

  std::map<int, std::vector<uint64_t>> move_out_tracking_ids;
  for (auto &last_key_points_class_ids_s : last_key_points_class_ids_) {
    std::vector<uint64> move_out_tracking_id;
    auto &last_key_points_class_ids = last_key_points_class_ids_s.second;
    if (!last_key_points_class_ids.empty()) {
      std::set_difference(
          last_key_points_class_ids.begin(), last_key_points_class_ids.end(),
          key_points_class_ids[last_key_points_class_ids_s.first].begin(),
          key_points_class_ids[last_key_points_class_ids_s.first].lower_bound(
              *last_key_points_class_ids.rbegin()),
          std::back_insert_iterator(move_out_tracking_id));
      //
      move_out_tracking_ids.emplace(last_key_points_class_ids_s.first,
                                    std::move(move_out_tracking_id));
    }
  }
  //
  //
  covisibility_->UpdateWithFrameData(id, std::move(key_point_map_point_index));
  for (const auto &id_s : move_out_tracking_ids) {
    TriagulateMapUpdata(id_s.second);
    for (const auto &id : id_s.second) {
      const auto &map_point_id = GetWithTrackingId({id_s.first, id});
      // ComputeMapPointDistinctiveDescriptors(map_point_id);
      map_points_.at(map_point_id).data->SetFix();
    }
  }
  // 注意这里如果要删除地图点的话，需要是已经滑动除去的点，需要在mappoint里面加一个标志
  last_key_points_class_ids_ = key_points_class_ids;
};
//
Eigen::Matrix3d ComputeF12(const transform::Rigid3d &pose0,
                           const transform::Rigid3d &pose1,
                           const Eigen::Matrix3d &k) {
  const transform::Rigid3d relative_pose = pose0.inverse() * pose1;

  return k.transpose().inverse() * Skew(relative_pose.translation()) *
         relative_pose.rotation().toRotationMatrix() * k.inverse();
}
//
bool CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
                           const Eigen::Matrix3d &F12) {
  // Epipolar line in second image l = x1'F12 = [a b c]
  const Eigen::Vector3d l =
      Eigen::Vector3d(kp1.pt.x, kp1.pt.y, 1).transpose() * F12;
  const double param_dist = l.head<2>().norm();
  //
  if (param_dist == 0) return false;
  //
  const Eigen::Vector3d kp2_point(kp2.pt.x, kp2.pt.y, 1);
  //
  const double kp2_dist_l = (kp2_point.transpose().dot(l)) / param_dist;
  //
  return common::Pow2(kp2_dist_l) < 3.84;
}
//

//
bool MapManager::CheckDistEpipolarLine(
    const FeatureData &kp1, const FeatureData &kp2,
    const transform::Rigid3d &relative_pose,
    const std::vector<camera_models::Camera *> &camera,
    Eigen::Vector3d *triang_map_point) {
  auto kp2_in_pose1 = relative_pose.rotation() * kp2.r_normal;
  const float cos_parallax = kp2_in_pose1.dot(kp1.r_normal) /
                             (kp1.r_normal.norm() * kp2_in_pose1.norm());
  //
  if (cos_parallax > options_.check_dist_epipolar_line_cos_parallax)
    return false;
  std::vector<Eigen::Vector3d> normal_kp{kp1.r_normal, kp2.r_normal};
  auto map_point_pos = TriangulatePoint(
      std::vector<transform::Rigid3d>{transform::Rigid3d::Identity(),
                                      relative_pose.inverse()},
      normal_kp);
  //
  if (map_point_pos.z() <= 0.1) return false;
  //
  Eigen::Vector2d project_p1;
  camera[0]->spaceToPlane(map_point_pos, project_p1);

  const auto err = Eigen::Vector2d(
      project_p1 - Eigen::Vector2d{kp1.key_point.pt.x, kp1.key_point.pt.y});
  if (err.squaredNorm() > 5.991) return false;
  //
  auto pose_in_2 = relative_pose.inverse() * map_point_pos;
  if (pose_in_2.z() <= 0.1 || pose_in_2.z() > 6) return false;
  Eigen::Vector2d project_p2;
  camera[1]->spaceToPlane(pose_in_2, project_p2);
  auto err1 = Eigen::Vector2d(
      project_p2 - Eigen::Vector2d(kp2.key_point.pt.x, kp2.key_point.pt.x));
  if (err1.squaredNorm() > 5.991) return false;
  *triang_map_point = map_point_pos;

  return true;
}
//
//
FeatureId MapManager::UpdateConnectMapPointProjectMatchSearch(
    const KeyFrameId &id) {
  //
  const auto &current_id_data = key_frames_datas_.at(id).data;
  const auto connect_frames_temp_1 =
      covisibility_->GetOrderConnectedKeyFrames(id, 20);
  //
  std::set<KeyFrameId> connect_key_frames_ids;
  for (const auto &id : connect_frames_temp_1) {
    connect_key_frames_ids.insert(id.first);
  }
  //
  std::set<MapPointId> connect_map_point_ids;
  for (const auto &frame_id : connect_frames_temp_1) {
    auto map_points = covisibility_->GetKeyFrameMapPointId(frame_id.first);
    for (const auto map_points_id : map_points.first) {
      connect_map_point_ids.insert(map_points_id);
    }
  }
  //

  std::map<int, std::unique_ptr<match::AreaSearch>> area_searchs =
      match::AreaSearch::CreateAreaSearchFromeKeyFrameData(
          options_.image_boxs, 10, key_frames_datas_.at(id));

  match::ProjectionOption project_option{
      0.5, 8, 0.0, 80, [this](const Eigen::Vector3d &point, int s) {
        Eigen::Vector2d b;
        cameras_.at(s)->spaceToPlane(point, b);
        return b;
      }};
  //
  std::map<KeyFrameId, std::map<MapPointId, FeatureId>> index_map_point_ids;
  for (const auto &map_point_id : connect_map_point_ids) {
    //
    auto index =
        SearchMatchesByProjection(project_option, key_frames_datas_.at(id),
                                  area_searchs, map_points_.at(map_point_id));

    if (index != FeatureId{-1, 0}) {
      index_map_point_ids[id].emplace(map_point_id, index);
    }
  }
  //
  if (!index_map_point_ids.empty()) {
    for (auto &&frame_index_map_point_ids : index_map_point_ids) {
      covisibility_->UpdateWithFrameData(
          frame_index_map_point_ids.first,
          std::move(frame_index_map_point_ids.second));
    }
  }
}

//

//
//

//
//
void MapManager::TriagulateMapUpdata(
    const std::vector<uint64_t> &move_out_tracking_id) {}
//

//
KeyFrameId MapManager::AddTrackingData(int t, const TrackingData &data) {
  //
  std::map<int, std::map<uint64_t, std::tuple<Eigen::Vector3d,
                                              mapping::Descriptor, FeatureId>>>
      front_map_points_data;
  //
  auto key_frame_id = key_frames_datas_.Append(
      t, ExtractKeyFrameData(data, &front_map_points_data));
  key_frame_data_base_->AddData(key_frame_id,
                                key_frames_datas_.at(key_frame_id).data);
  //
  StructureMapPoints(key_frame_id, front_map_points_data);

  return key_frame_id;
}

//
MapPointId MapManager::AddMapPoint(const int &t,
                                   const std::pair<int, uint64_t> &tracking_id,
                                   const MapPointData &map_point) {
  if (tracking_id_corresponding_to_map_point_id_.count(tracking_id.first)) {
    CHECK(!tracking_id_corresponding_to_map_point_id_[tracking_id.first].count(
        tracking_id.second))
        << "insert mappoint with same id.";
  }
  //
  auto map_point_id = map_points_.Append(t, map_point);
  //

  tracking_id_corresponding_to_map_point_id_[tracking_id.first].emplace(
      tracking_id.second, map_point_id);
  //
  map_point_id_corresponding_to_tracking_id_.emplace(map_point_id, tracking_id);
  //
  //
  return map_point_id;
}
//

//
void MapManager::ConStructExtendMapPoints(const KeyFrameId &id) {
  const auto connect_frames_temp_1 =
      covisibility_->GetOrderConnectedKeyFrames(id, 20);
  //
  std::set<KeyFrameId> connect_key_frames_ids;
  for (const auto &id : connect_frames_temp_1) {
    connect_key_frames_ids.insert(id.first);
  }
  std::vector<std::pair<KeyFrameId, int>> connect_frames_temp;
  const auto current_id_data = key_frames_datas_.at(id).data;
  //
  //
  for (int i = -options_.construct_map_point_near_keframd_num; i < -1; i++) {
    const KeyFrameId near_id(id.trajectory_id, id.keyframe_index + i);
    if (!key_frames_datas_.Contains(near_id) ||
        (connect_key_frames_ids.count(near_id) == 0))
      continue;
    auto delta_pose =
        (key_frames_datas_.at(near_id).data->pose * current_id_data->pose)
            .translation()
            .norm();
    if (delta_pose < 0.1) continue;
    connect_frames_temp.emplace_back(near_id, 0);
    // connected_key_frame_ids.insert(near_id);
  }
  double min_angle = -10;
  double sencode_min_angle = -10;
  std::vector<std::pair<KeyFrameId, int>> connect_frames;
  // //
  KeyFrameId min_key_frame_id(-1, 0);
  KeyFrameId senco_min_key_frame_id(-1, 0);
  for (const auto id : connect_frames_temp) {
    auto delta_pose =
        (key_frames_datas_.at(id.first).data->pose * current_id_data->pose)
            .translation()
            .norm();
    auto const delta_angle = delta_pose;
    if (min_angle < delta_angle) {
      min_angle = delta_angle;
      min_key_frame_id = id.first;

    } else if (sencode_min_angle < delta_angle) {
      sencode_min_angle = delta_angle;
      senco_min_key_frame_id = id.first;
    }
  }
  if (min_key_frame_id != KeyFrameId(-1, 0)) {
    connect_frames.push_back({min_key_frame_id, 0});
  }
  if (senco_min_key_frame_id != KeyFrameId(-1, 0)) {
    connect_frames.push_back({senco_min_key_frame_id, 0});
  }
  //
  std::map<KeyFrameId, std::map<MapPointId, FeatureId>> index_map_point_ids;
  //
  //

  for (const auto &frame_id : connect_frames) {
    const auto connect_id_data = key_frames_datas_.at(frame_id.first).data;
    auto paired_idex = match::DbowFindMathed(
        connect_id_data->descriptors, current_id_data->descriptors,
        connect_id_data->dbow_data, current_id_data->dbow_data,
        options_.dbow_match_min_distance);
    if (paired_idex.size() < 3) continue;
    // test_data::WriteImageWithKeyPoint(
    //     *connect_id_data->image, *current_id_data->image,
    //     connect_id_data->key_points, current_id_data->key_points,
    //     kTestResultDir +
    //         std::to_string(common::ToUniversal(current_id_data->time)) +
    //         ".png",
    //     " ", paired_idex, connect_id_data->tracking_index_size,
    //     current_id_data->tracking_index_size);

    const auto current_id_map_data = GetKeyFrameMapPointsData(id);
    const auto connect_id_map_data = GetKeyFrameMapPointsData(frame_id.first);

    std::stringstream info;
    int new_construct_map_point_size = 0;
    int tracking_construct_map_point_size = 0;
    // LOG(INFO)<<paired_idex.size();
    for (const auto &index : paired_idex) {
      //
      if (current_id_map_data.first.count(index.first)) continue;
      //
      Eigen::Vector3d triangulate_point_in_pose1;
      //
      const transform::Rigid3d connect_to_cur_pose =
          connect_id_data->CameraPose(index.first.sequence_id).inverse() *
          current_id_data->CameraPose(index.second.sequence_id);
      //
      std::vector<camera_models::Camera *> cameras;
      cameras.push_back(cameras_[index.second.sequence_id].get());
      cameras.push_back(cameras_[index.first.sequence_id].get());
      // /
      if (!CheckDistEpipolarLine(connect_id_data->features.at(index.first),
                                 current_id_data->features.at(index.second),
                                 connect_to_cur_pose, cameras,
                                 &triangulate_point_in_pose1))
        continue;
      //
      if (connect_id_map_data.first.count(index.first)) {
        index_map_point_ids[id].emplace(
            connect_id_map_data.first.at(index.first), index.second);
        //
        ComputeMapPointDistinctiveDescriptors(
            connect_id_map_data.first.at(index.first));
        tracking_construct_map_point_size++;
      } else {
        const auto map_point_pos =
            connect_id_data->pose * triangulate_point_in_pose1;
        //
        MapPoint new_map_point(frame_id.first, map_point_pos,
                               connect_id_data->descriptors.at(index.first),
                               true);

        auto map_point_id = map_points_.Append(
            id.trajectory_id,
            MapPointData{std::make_shared<MapPoint>(new_map_point),
                         globle_to_local_transform_ * map_point_pos});
        //
        index_map_point_ids[id].emplace(map_point_id, index.second);
        index_map_point_ids[frame_id.first].emplace(map_point_id, index.first);
        new_construct_map_point_size++;
        LOG(INFO) << "New Construct Map point With : " << map_point_id;
        //
      }
    }
    info << "New Construct Map point  : " << new_construct_map_point_size
         << ",Tracking Construct Map point  : "
         << tracking_construct_map_point_size << "Total :"
         << new_construct_map_point_size + tracking_construct_map_point_size;
    LOG(INFO) << info.str();
  }

  for (auto &&frame_index_map_point_ids : index_map_point_ids) {
    covisibility_->UpdateWithFrameData(
        frame_index_map_point_ids.first,
        std::move(frame_index_map_point_ids.second));
  }
}

//
KeyFrameId MapManager::AddKeyFrame(int t, const KeyFrameData &data) {
  return key_frames_datas_.Append(t, data);
}
//
//
const MapById<KeyFrameId, KeyFrameData> &MapManager::KeyAllFrameDatas() const {
  return key_frames_datas_;
}
//
//
//
MapPointId MapManager::GetWithTrackingId(
    const std::pair<int, uint64_t> &tracking_id) {
  return tracking_id_corresponding_to_map_point_id_[tracking_id.first].at(
      tracking_id.second);
}
//
//
bool MapManager::IsExist(const int s, const uint64_t &tracking_id) {
  if (tracking_id_corresponding_to_map_point_id_.count(s)) {
    if (tracking_id_corresponding_to_map_point_id_[s].count(tracking_id)) {
      return true;
    }
  }
  return false;
}
//
//
std::unique_ptr<Eigen::Vector2d> MapManager::ProjectMapPointToKeyFrame(
    const MapPointId &mp_id, const KeyFrameId &kf_id) const {
  return nullptr;
}

//

//
//
void MapManager::ComputeMapPointDistinctiveDescriptors(const MapPointId &id) {
  auto obs = covisibility_->GetMapObservations(id);
  if (obs.empty()) {
    LOG(WARNING) << " obs empty";
    return;
  }
  std::vector<BrifBitset> descriptors;
  const int obs_size = obs.size();
  descriptors.reserve(obs_size);
  for (auto &ob : obs) {
    auto const &key_frame_data = key_frames_datas_.at(ob);
    descriptors.push_back(key_frame_data.data->descriptors.at(
        covisibility_->GetMapPointFeatureIndex(ob, id)));
  }
  // Compute distances between them
  std::vector<std::vector<int>> distances(obs_size, std::vector<int>(obs_size));
  for (size_t i = 0; i < obs_size; i++) {
    distances[i][i] = 0;
    for (size_t j = i + 1; j < obs_size; j++) {
      int distij = HammingDis(descriptors[i], descriptors[j]);
      distances[i][j] = distij;
      distances[j][i] = distij;
    }
  }
  // Take the descriptor with least median distance to the rest
  int best_median = INT_MAX;
  int best_idx = 0;
  for (size_t i = 0; i < obs_size; i++) {
    auto median = distances[i].begin() + obs_size / 2;
    std::nth_element(distances[i].begin(), median, distances[i].end());
    if (*median < best_median) {
      best_median = *median;
      best_idx = i;
    }
  }
  //
  *map_points_.at(id).data->MutableDescriptor() = descriptors[best_idx];
}

//

//
//
std::vector<KeyFrameId> MapManager::GetConnectedKeyFrames(
    const KeyFrameId &frame_id, int num) const {
  return covisibility_->GetConnectedKeyFrames(frame_id, num);
}
//
//
//
std::pair<std::map<FeatureId, MapPointId>, MapById<MapPointId, MapPointData>>
MapManager::GetKeyFrameMapPointsData(const KeyFrameId &frame_id) const {
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

//
bool MapManager::TrimMapPoint(const MapPointId &id) {
  // 不要删除最后一个地图点
  if (std::prev(map_points_.EndOfTrajectory(id.trajectory_id))->id == id) {
    return false;
  }
  if (map_point_id_corresponding_to_tracking_id_.count(id) == 0) {
    map_points_.Trim(id);
    return true;
  }
  if (!map_points_.at(id).data->Fix()) return false;
  const auto &tracking_id = map_point_id_corresponding_to_tracking_id_[id];
  tracking_id_corresponding_to_map_point_id_[tracking_id.first].erase(
      tracking_id.second);
  map_point_id_corresponding_to_tracking_id_.erase(id);
  map_points_.Trim(id);
  return true;
}
//

//
void MapManager::TrimKeyFrame(const KeyFrameId &id) {
  if (!key_frames_datas_.Contains(id)) return;
  // 不能删除第一个和最后一个
  if (key_frames_datas_.BeginOfTrajectory(id.trajectory_id)->id == id) return;
  if (std::prev(key_frames_datas_.EndOfTrajectory(id.trajectory_id))->id == id)
    return;

  std::stringstream info;
  key_frames_datas_.Trim(id);
  key_frame_data_base_->Erase(id);
  auto trim_map_points = covisibility_->TrimKeyFrame(id);
  info << "trim kf :" << id
       << "with map points size : " << trim_map_points.size();
  for (auto const &map_point_id : trim_map_points) {
    if (TrimMapPoint(map_point_id)) {
      info << " " << map_point_id << " ";
      covisibility_->TrimMapPoint(map_point_id);
    }
  }
  LOG(INFO) << info.str();
}
//
//
void MapManager::FuseMapPoint(
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

      if (!TrimMapPoint(match_temp.second)) continue;
      covisibility_->UpdateWithFuseMapPoint(match_temp.first,
                                            match_temp.second);
      info << " " << match_temp.first << "<-" << match_temp.second << " ";
      merg_point = true;
      ComputeMapPointDistinctiveDescriptors(match_temp.first);
    }
  }
  LOG(INFO) << info.str();
  //
}
//
//
//

}  // namespace mapping
}  // namespace jarvis