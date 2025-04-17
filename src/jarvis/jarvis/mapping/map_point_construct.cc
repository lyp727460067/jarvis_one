#include "jarvis/mapping/map_point_construct.h"

#include <map>
#include <memory>
#include <set>

#include "Eigen/Core"
#include "Eigen/Geometry"
//
#include "jarvis/camera_models/camera_models/camera.h"
//
#include "jarvis/common/id.h"
#include "jarvis/key_frame_data.h"
#include "jarvis/mapping/covisibility.h"
#include "jarvis/mapping/key_frame_database.h"
#include "jarvis/mapping/key_point_exract.h"
#include "jarvis/mapping/mapping_data.h"
#include "jarvis/mapping/match/des_matcher.h"
#include "jarvis/mapping/match/pic_writer.h"
#include "jarvis/transform/transform.h"
namespace jarvis {
namespace mapping {

//
namespace {

Eigen::Vector3d TriangulatePoint(
    const std::vector<transform::Rigid3d> &poses,
    const std::vector<Eigen::Vector3d> &key_point_normal) {
  Eigen::MatrixXd H(poses.size() * 2, 4);
  // CHECK_EQ(poses.size(), 2) << "Function Just adoptor 2 size pose";
  // Eigen::MatrixXd H;
  for (size_t i = 0; i < poses.size(); i++) {
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
}  // namespace
//

//
KeyFrameData MapPointConstruct::TrackDataToKeyFrameData(
    const TrackingData &data, std::shared_ptr<LocalMapMatchResult> track_data) {
  KeyFrameData result{std::make_shared<KeyFrameData::Data>(
      KeyFrameData::Data{data.data->time, data.data->imu_state.Pose(),
                         data.data->extric_camera_to_imu,
                         data.data->images.Pyramid(), &options_.image_boxs})};
  // 右目的图像删除
  result.data->pyramid[1].clear();
  //
  std::map<int, std::map<size_t, uint64>> track_ids;
  for (auto &senqu_features : data.data->features_datas) {
    std::vector<FeatureData> feat_datas;
    for (auto &feature_point : senqu_features.second.features.data->features) {
      const auto &feature = feature_point.second.camera_features[0].uv;
      const auto &normal =
          feature_point.second.camera_features[0].normal_points;
      track_ids[senqu_features.first][feat_datas.size()] = feature_point.first;
      feat_datas.emplace_back(
          FeatureData{cv::KeyPoint(feature.x(), feature.y(), 2), normal});
      //
    }
    for (size_t j = 0; j < feat_datas.size(); j++) {
      //  {senqu_features.first, j};
      // result.data->features.Insert(feat_id, feat_datas[j]);
      FeatureId feat_id =
          result.data->features.Append(senqu_features.first, feat_datas[j]);
      const uint64_t track_id = track_ids.at(senqu_features.first).at(j);

      const Eigen::Vector3d frame_re_map_point =
          senqu_features.second.map_points.at(track_id);
      result.data->map_points.Insert(feat_id, frame_re_map_point);
      MapPointId map_point_local_id(0, 0);
      if (!IsExist(senqu_features.first, track_id, &map_point_local_id)) {
        const std::pair<int, uint64_t> tracking_id(senqu_features.first,
                                                   track_id);
        map_point_local_id = AppendMapPointId(&tracking_id);

      } else {
      }
      result.data->map_point_ids.emplace(feat_id, map_point_local_id);
    }
  }
  if (track_data && options_.use_local_track_match) {
    for (auto &match : track_data->matchs) {
      auto feat_id = result.data->features.Append(
          match.s,
          FeatureData{cv::KeyPoint(match.key_point.x(), match.key_point.y(), 2),
                      Eigen::Vector3d(match.normal.x(), match.normal.y(), 1)});
      result.data->map_points.Insert(feat_id, match.map_point);
      //
      result.data->map_point_ids.emplace(feat_id, match.mp_point_id);
    }
  }
  return result;
}
//
bool MapPointConstruct::IsExist(const int s, const uint64_t &tracking_id,
                                MapPointId *local_id) {
  if (tracking_id_corresponding_to_map_point_id_.count(s)) {
    if (tracking_id_corresponding_to_map_point_id_[s].count(tracking_id)) {
      *local_id = tracking_id_corresponding_to_map_point_id_[s].at(tracking_id);
      return true;
    }
  }
  return false;
}
//
MapPointId MapPointConstruct::AppendMapPointId(
    const std::pair<int, uint64_t> *tracking_id) {
  if (tracking_id) {
    if (tracking_id_corresponding_to_map_point_id_.count(tracking_id->first)) {
      CHECK(
          !tracking_id_corresponding_to_map_point_id_[tracking_id->first].count(
              tracking_id->second))
          << "insert mappoint with same id.";
    }
    //
    const MapPointId mp_id(trajctory, map_points_local_ids_.size());
    tracking_id_corresponding_to_map_point_id_[tracking_id->first].emplace(
        tracking_id->second, mp_id);
    map_point_id_corresponding_to_tracking_id_[mp_id] = *tracking_id;
    //
  }
  std::lock_guard<std::mutex> lock(mutex_);
  map_points_local_ids_.insert(
      MapPointId(trajctory, map_points_local_ids_.size()));
  return *map_points_local_ids_.rbegin();
}

//
bool MapPointConstruct::CheckDistEpipolarLine(
    const FeatureData &kp1, const FeatureData &kp2,
    const transform::Rigid3d &relative_pose,
    const std::vector<camera_models::Camera *> &camera,
    Eigen::Vector3d *triang_map_point) {
  const auto &option = options_.point_check_dist_epipolar_option;
  auto kp2_in_pose1 = relative_pose.rotation() * kp2.f;
  const float cos_parallax =
      kp2_in_pose1.dot(kp1.f) / (kp1.f.norm() * kp2_in_pose1.norm());
  //
  // LOG(INFO)<<cos_parallax ;
  if (cos_parallax > option.check_dist_epipolar_line_cos_parallax) return false;
  std::vector<Eigen::Vector3d> normal_kp{kp1.f, kp2.f};
  auto map_point_pos = TriangulatePoint(
      std::vector<transform::Rigid3d>{transform::Rigid3d::Identity(),
                                      relative_pose.inverse()},
      normal_kp);
  //

  if (map_point_pos.z() <= option.first_cam_min_z_distance) return false;
  //
  Eigen::Vector2d project_p1;
  camera[0]->spaceToPlane(map_point_pos, project_p1);
  // LOG(INFO)<<project_p1.transpose();
  const auto err = Eigen::Vector2d(project_p1 - kp1.Point());
  //
  if (err.squaredNorm() > option.first_cam_chi_squared) return false;
  //
  auto pose_in_2 = relative_pose.inverse() * map_point_pos;
  if (pose_in_2.z() <= option.second_cam_min_z_distance) return false;
  Eigen::Vector2d project_p2;
  camera[1]->spaceToPlane(pose_in_2, project_p2);
  auto err1 = Eigen::Vector2d(project_p2 - kp2.Point());
  //
  if (err1.squaredNorm() > option.second_cam_chi_squared) return false;
  *triang_map_point = map_point_pos;
  return true;
}
//

//
MapPointConstruct::MapPointConstruct(
    const MapPointConstructOption &option,
    std::map<int, camera_models::CameraPtr> camera, dbow::Vocabulary *voc)
    : options_(option), cameras_(camera), voc_(voc) {
  key_points_extractor_ =
      std::make_unique<KeyPointExtract>(option.key_points_extract_option);
  des_extractor_ =
      std::make_unique<DescriptorExtract>(option.descriptor_option);
}
//

//
cv::Mat MapPointConstruct::GenerateMask(
    const cv::Size &size, const std::vector<cv::KeyPoint> &exit_point) {
  cv::Mat mask(size, CV_8UC1, cv::Scalar(255));
  for (auto const &p : exit_point) {
    cv::circle(mask, p.pt, 10, 0, -1);
  }
  return mask;
}
//
//
void MapPointConstruct::GenerateForExtendKeyPoint(KeyFrameData::Data &data) {
  // 新提取的特征点和描述子都会保存在data中

  auto sequence_feautes = data.features.trajectory_ids();
  for (const auto &sequence_id : sequence_feautes) {
    auto one_sequence_feautes = data.features.trajectory(sequence_id);

    std::vector<cv::KeyPoint> exist_key_points;
    for (const auto &feat : one_sequence_feautes) {
      exist_key_points.push_back(feat.data.key_point);
    }
    CHECK(!data.pyramid.empty());
    const cv::Mat image = data.Pyramid(sequence_id)[0];
    // cv::imshow("image", image);
    // cv::waitKey(0);
    // 在已跟踪特征点的基础上再提取新的特征点
    std::vector<cv::KeyPoint> key_points = key_points_extractor_->Extract(
        image,
        options_.masks[sequence_id] &
            GenerateMask(cv::Size(options_.image_boxs[sequence_id].sizes().x(),
                                  options_.image_boxs[sequence_id].sizes().y()),
                         exist_key_points));
    //
    LOG_EVERY_N(INFO, 1) << log_info::BLUE << "s-" << sequence_id
                         << " Extend keypoint nun: " << key_points.size()
                         << log_info::RESET;
    //
    exist_key_points.insert(exist_key_points.end(), key_points.begin(),
                            key_points.end());
    // 提取特征
    Descriptors descriptors = des_extractor_->Extract(image, exist_key_points);
    for (size_t i = 0; i < exist_key_points.size(); i++) {
      const FeatureId feat_id(sequence_id, i);

      if (!data.features.Contains(feat_id)) {
        Eigen::Vector2d a(exist_key_points[i].pt.x, exist_key_points[i].pt.y);
        Eigen::Vector3d b;
        cameras_.at(sequence_id)->liftProjective(a, b);  // 注意这里找对应的相机
        data.features.Insert(feat_id,
                                   FeatureData{exist_key_points[i], b / b.z()});
      }
      //
      data.descriptors.Insert(feat_id, descriptors[i]);
    }
  }

  //
  data.dbow_data =
      voc_->Transform(data.descriptors, options_.dbow_trasform_level);
}
//

bool MapPointConstruct::ExtractExtendData(const LocalMap &local_map,
                                          KeyFrameData::Data* data) {
  if (voc_ == nullptr) return false;
  if (!data->dbow_data.bow_vector.empty()) return false;
  GenerateForExtendKeyPoint(*data);
  return true;
}
//
bool MapPointConstruct::ConstructExtend(const LocalMap &local_map,
                                        KeyFrameData *data) {
  // 优先把以前地图的点和当前做匹配
  if (local_map.AllKeyFrameDatas().size() <= 1) return false;
  UpdateConnectMapPointProjectMatchSearch(local_map, *data);
  ConStructExtendMapPoints(local_map, *data);
  //
  return true;
}

//
void MapPointConstruct::UpdateConnectMapPointProjectMatchSearch(
    const LocalMap &local_map, KeyFrameData &data) {
  //

  auto &key_frames_datas = local_map.AllKeyFrameDatas();
  //
  auto const &map_points = local_map.AllMapPoints();
  //

  const KeyFrameId pre_id = std::prev(key_frames_datas.end())->id;
  const auto connect_frames =
      local_map.GetCovisibility()->GetOrderConnectedKeyFrames(pre_id, 20);
  //
  const auto &current_id_data = data.data;
  //
  std::set<MapPointId> cur_exsist_map_point_ids;
  for (const auto &id : data.data->map_point_ids) {
    cur_exsist_map_point_ids.insert(id.second);
  }
  //
  std::set<MapPointId> connect_map_point_ids;
  int track_point_size = 0;
  for (const auto &frame_id : connect_frames) {
    auto map_points =
        local_map.GetCovisibility()->GetKeyFrameMapPointId(frame_id.first);
    for (const auto map_points_id : map_points.first) {
      if (cur_exsist_map_point_ids.count(map_points_id)) continue;
      connect_map_point_ids.insert(map_points_id);
    }
  }
  //
  LOG(INFO) << "Project covisi map candidata size :"
            << connect_map_point_ids.size();
  std::map<int, std::unique_ptr<match::AreaSearch>> area_searchs =
      match::AreaSearch::CreateAreaSearchFromeKeyFrameData(
          options_.image_boxs, options_.area_search_grid_lenth, data);

  match::ProjectionOption project_option = options_.track_project_search_option;
  project_option.PorjectPoint = [this, local_map](
                                    const transform::Rigid3d &cam_pose,
                                    const Eigen::Vector3d &point, int s,
                                    Eigen::Vector2d *p) {
    const Eigen::Vector3d p_point =
        cam_pose.inverse() * local_map.LocalPose() * point;
    if (point.z() < 0.1) return false;
    Eigen::Vector2d b;
    cameras_.at(s)->spaceToPlane(p_point, b);
    *p = b;
    return true;
  };
  //
  std::map<MapPointId, FeatureId> index_map_point_ids;

  for (const auto &map_point_id : connect_map_point_ids) {
    // 投影后在一定半径范围內找特征最相似的点(汉明距离)作为匹配点
    auto index = SearchMatchesByProjection(project_option, data, area_searchs,
                                           map_points.at(map_point_id));
    if (index != FeatureId{-1, 0}) {
      index_map_point_ids.emplace(map_point_id, index);
      if (!options_.test_match_pic_write_path.empty()) {
        auto feture_id = local_map.GetCovisibility()->GetMapPointFeatureIndex(
            map_points.at(map_point_id).data->reference_frame_id, map_point_id);
        auto &connect_id_data =
            key_frames_datas
                .at(map_points.at(map_point_id).data->reference_frame_id)
                .data;
        //
        std::vector<std::pair<FeatureId, FeatureId>> pair_index{
            std::make_pair(index, feture_id)};
        match::WriteImageWithKeyPoint(options_.test_match_pic_write_path,
                                      *data.data, *connect_id_data, pair_index);
      }
    }
  }
  std::stringstream info;

  if (!index_map_point_ids.empty()) {
    for (auto &mp_id : index_map_point_ids) {
      // 以前的地图点匹配到当前有地图点的特征上了
      if (data.data->map_points.Contains(mp_id.second)) continue;
      //
      track_point_size++;
      info << mp_id.first;
      data.data->map_point_ids.emplace(
          mp_id.second, map_points.at(mp_id.first).data->local_id);
      data.data->map_points.Insert(mp_id.second,
                                   local_map.GetMapPointPosw(mp_id.first));
      data.data->map_point_ids.emplace(mp_id.second, mp_id.first);
    }
  }

  //
  LOG(INFO) << log_info::GREEN
            << "Track near map point size: " << track_point_size << "-->"
            << info.str() << log_info::RESET;
}
//
void MapPointConstruct::ConStructExtendMapPoints(const LocalMap &local_map,
                                                 KeyFrameData &data) {
  const auto &key_frames_datas = local_map.AllKeyFrameDatas();
  //
  //
  const KeyFrameId pre_id = std::prev(key_frames_datas.end())->id;
  const auto connect_frames_temp_1 =
      local_map.GetCovisibility()->GetOrderConnectedKeyFrames(pre_id, 20);
  std::set<KeyFrameId> connect_key_frames_ids;
  for (const auto &id : connect_frames_temp_1) {
    connect_key_frames_ids.insert(id.first);
  }
  std::vector<std::pair<KeyFrameId, int>> connect_frames_temp;
  const auto &current_id_data = data.data;
  //
  //
  for (int i = -options_.construct_map_point_near_keframd_num; i < -1; i++) {
    const KeyFrameId near_id(pre_id.trajectory_id, pre_id.keyframe_index + i);
    if (!key_frames_datas.Contains(near_id) ||
        (connect_key_frames_ids.count(near_id) == 0))
      continue;
    auto delta_pose = (key_frames_datas.at(near_id).data->pose.inverse() *
                       current_id_data->pose)
                          .translation()
                          .norm();
    if (delta_pose < options_.con_struct_map_point_frame_min_distance) continue;
    connect_frames_temp.emplace_back(near_id, 0);
    // connected_key_frame_ids.insert(near_id);
  }
  if (connect_frames_temp.empty()) return;
  double min_angle = -10;
  double sencode_min_angle = -10;
  std::vector<std::pair<KeyFrameId, int>> connect_frames;
  // //
  KeyFrameId min_key_frame_id(-1, 0);
  KeyFrameId senco_min_key_frame_id(-1, 0);
  for (const auto &id : connect_frames_temp) {
    auto delta_pose = (key_frames_datas.at(id.first).data->pose.inverse() *
                       current_id_data->pose)
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

  std::map<MapPointId, FeatureId> index_map_point_ids;
  //
  //

  for (const auto &frame_id : connect_frames) {
    std::stringstream point_id_info;
    std::stringstream track_point_id_info;

    const auto connect_id_data = key_frames_datas.at(frame_id.first).data;
    auto paired_idex = match::DbowFindMathed(
        current_id_data->descriptors, connect_id_data->descriptors,
        current_id_data->dbow_data, connect_id_data->dbow_data,
        options_.dbow_match_min_distance);
    if (paired_idex.size() < 4) continue;
    //

    const auto &current_id_map_data = data.data->map_point_ids;
    //
    const auto connect_id_map_data =
        local_map.GetKeyFrameMapPointsData(frame_id.first);

    std::stringstream info;
    int new_construct_map_point_size = 0;
    int tracking_construct_map_point_size = 0;
    // LOG(INFO)<<paired_idex.size();
    std::vector<std::pair<FeatureId, FeatureId>> check_paired_idex;
    for (const auto &index : paired_idex) {
      //
      const auto &cur_feat_id = index.first;
      const auto &connect_feat_id = index.second;
      if (current_id_map_data.count(cur_feat_id)) continue;
      //
      Eigen::Vector3d triangulate_point_in_pose1;
      //
      const transform::Rigid3d connect_to_cur_pose =
          connect_id_data->CameraPose(connect_feat_id.sequence_id).inverse() *
          current_id_data->CameraPose(cur_feat_id.sequence_id);
      //
      std::vector<camera_models::Camera *> cameras;
      cameras.push_back(cameras_[connect_feat_id.sequence_id].get());
      cameras.push_back(cameras_[cur_feat_id.sequence_id].get());
      // /
      if (!CheckDistEpipolarLine(connect_id_data->features.at(connect_feat_id),
                                 current_id_data->features.at(cur_feat_id),
                                 connect_to_cur_pose, cameras,
                                 &triangulate_point_in_pose1))
        continue;
      //
      check_paired_idex.push_back(index);
      if (connect_id_map_data.first.count(connect_feat_id)) {
        // //
        // // 有一种情况当前的地图点是前端跟踪过来的话就不需要添加了
        // if (!current_id_map_data.second.Contains(
        //         connect_id_map_data.first.at(connect_feat_id))) {
        //   index_map_point_ids.emplace(
        //       connect_id_map_data.first.at(connect_feat_id), cur_feat_id);
        //   // ComputeMapPointDistinctiveDescriptors(
        //   // connect_id_map_data.first.at(connect_feat_id));

        tracking_construct_map_point_size++;
        track_point_id_info << connect_id_map_data.first.at(connect_feat_id);
        // }
        //

        //
      } else {
        const auto map_point_pos =
            connect_id_data->CameraPose(connect_feat_id.sequence_id) *
            triangulate_point_in_pose1;
        //
        auto map_point_local_id = AppendMapPointId(nullptr);
        data.data->map_point_ids.emplace(cur_feat_id, map_point_local_id);
        data.data->map_points.Insert(cur_feat_id, map_point_pos);
        //
        // index_map_point_ids.emplace(map_point_id, cur_feat_id);
        new_construct_map_point_size++;
        point_id_info << map_point_local_id;
        // LOG(INFO) << "New Construct Map point With : " << map_point_local_id;
        //
      }
    }

    if (!options_.test_match_pic_write_path.empty()) {
      match::WriteImageWithKeyPoint(options_.test_match_pic_write_path,
                                    *current_id_data, *connect_id_data,
                                    check_paired_idex);
    }

    info << "conect_kf_" << frame_id.first << "New Construct Map point  "
         << new_construct_map_point_size << ",Tracking Construct Map point: "
         << tracking_construct_map_point_size << ",Total: "
         << new_construct_map_point_size + tracking_construct_map_point_size
         << " [" << point_id_info.str() << "]" << "-->[t "
         << track_point_id_info.str() << "]";
    LOG(INFO) << log_info::YELLOW << info.str() << log_info::RESET;
  }
  auto const &map_points = local_map.AllMapPoints();
  if (!index_map_point_ids.empty()) {
    for (auto &mp_id : index_map_point_ids) {
      // 以前的地图点匹配到当前有地图点的特征上了
      if (data.data->map_points.Contains(mp_id.second)) continue;
      data.data->map_point_ids.emplace(
          mp_id.second, map_points.at(mp_id.first).data->local_id);
      CHECK(!data.data->map_points.Contains(mp_id.second));
      data.data->map_points.Insert(mp_id.second,
                                   local_map.GetMapPointPosw(mp_id.first));
      data.data->map_point_ids.emplace(mp_id.second, mp_id.first);
    }
  }
}
}  // namespace mapping
}  // namespace jarvis
