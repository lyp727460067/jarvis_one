#include "jarvis/mapping/local_map_track.h"

#include "jarvis/mapping/covisibility.h"
#include "jarvis/mapping/map_manger.h"
//
namespace jarvis {
namespace mapping {
namespace {

struct ReProjectionErr {
 public:
  ReProjectionErr(const Eigen::Vector2d& nor_poit,
                  const Eigen::Vector3d& map_point, const double& factor)
      : nor_point_(nor_poit), map_point_(map_point), factor_(factor) {}

  template <typename T>
  bool operator()(const T* t1_, const T* q1_, const T* te_, const T* qe_,
                  T* residul) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t1(t1_);
    Eigen::Map<const Eigen::Quaternion<T>> q1(q1_);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> te(te_);
    Eigen::Map<const Eigen::Quaternion<T>> qe(qe_);
    Eigen::Matrix<T, 3, 1> project_p =
        q1 * qe * map_point_.template cast<T>() + q1 * te + t1;
    T x_normal = project_p[0] / project_p[2];
    T y_normal = project_p[1] / project_p[2];
    residul[0] = T(factor_) * (x_normal - T(nor_point_.x()));
    residul[1] = T(factor_) * (y_normal - T(nor_point_.y()));
    return true;
  }
  static ceres::CostFunction* Creat(const Eigen::Vector2d& nor_poit,
                                    const Eigen::Vector3d& map_point,
                                    double factor) {
    return new ceres::AutoDiffCostFunction<ReProjectionErr, 2, 3, 4, 3, 4>(
        new ReProjectionErr(nor_poit.head<2>(), map_point, factor));
  }

 private:
  const double factor_;
  const Eigen::Vector2d nor_point_;
  const Eigen::Vector3d map_point_;
};

//

}  // namespace
//
void LocalMapTrack::ToFrame(const KeyFrameData& key_frame_data,
                            match::Frame& fram, int index) {
  fram.cam = cameras_.at(index);
  fram.image_size = key_frame_data.data->image_sizes->at(index).sizes();
  fram.pose = key_frame_data.data->CameraPose(index);
  fram.f_pose = key_frame_data.data->pose;
  fram.img_pyr = key_frame_data.data->Pyramid(index);
  fram.f_top_left = &px_top_lefts_[index];
}
int LocalMapTrack::IsInFrame(const MapPoint& map_point,
                             const KeyFrameData& track_data) {
  const Eigen::Vector3d xyz_w = map_point.Pos();
  //
  //
  for (int sequence_id = 0; sequence_id < options_.track_sequence.size();
       sequence_id++) {
    const auto& pose = track_data.data->CameraPose(sequence_id);

    Eigen::Vector3d xyz_f = pose * xyz_w;
    //
    if(xyz_f.z()<0)return -1;
    // Eigen::Vector2d px_top_left(0.0, 0.0);
    Eigen::Vector3d& f_top_left = px_top_lefts_[sequence_id];
    // cameras_.at(sequence_id)
        // ->liftProjective(px_top_left, f_top_left);  // 注意这里找对应的相机
    const Eigen::Vector3d z(0.0, 0.0, 1.0);
    const double min_cos = f_top_left.dot(z);
    const double cur_cos = xyz_f.normalized().dot(z);
    if (cur_cos < min_cos) {
      return sequence_id;
    }
  }
  return -1;
}

LocalMapTrack::LocalMapTrack(const LocalMapTrackOption& option)
    : options_(option), cameras_(options_.cameras) {
  direct_match_ =
      std::make_unique<match::DirectMatch>(options_.derect_match_option);
  local_map_ = std::make_unique<LocalMapTrackMap>(options_.map_option);
  for (int i = 0; i < options_.track_sequence.size(); i++) {
    Eigen::Vector3d f_top_left;
    Eigen::Vector2d px_top_left(0.0, 0.0);
    cameras_.at(i)
        ->liftProjective(px_top_left, f_top_left);  // 注意这里找对应的相机

    px_top_lefts_.push_back( f_top_left/ f_top_left.z());
    LOG(INFO)<<px_top_lefts_.back();
  }
}
//
void LocalMapTrack::AddTracingData(const KeyFrameData& key_frame_data,
                                   const FrontMapPointData& map_points_data) {
  local_map_->AddKeyFrameData(key_frame_data, map_points_data);
}

//
std::unique_ptr<transform::Rigid3d> LocalMapTrack::Track(
    const KeyFrameData& track_data) {
  const auto& all_kf_frames = local_map_->AllKeyFrameDatas();
  if (all_kf_frames.size() < options_.min_track_frame_num) return nullptr;
  //

  const auto sequence_feautes = track_data.data->features.trajectory_ids();
  std::map<int, std::vector<KeyFrameId>> overlap_kfs;
  std::map<int, std::shared_ptr<match::Frame>> cur_frames;
  //
  for (int i = 0; i < options_.track_sequence.size(); i++) {
    cur_frames[i] = std::make_shared<match::Frame>();
    ToFrame(track_data, *cur_frames[i],i);
  }
  //

  for (const auto& kf : all_kf_frames) {
    const auto map_points = local_map_->GetKeyFrameMapPoints(kf.id);
    // 这里选择领域和公视的关键帧，还有只能投影一个点的3D点的
    for (const auto& map_point : map_points) {
      int index = IsInFrame(*map_point.second.data, track_data);
      if (index >= 0) {
        overlap_kfs[index].push_back(kf.id);
        break;
      }
    }
  }
  std::map<int, std::vector<LocalMapTrack::MatchData>> matchs;
  //

  int match_sum_num = 0;
  std::stringstream info;
  for (int i = 0; i < cur_frames.size(); i++) {
    auto match_result = MatchCandidates(
        PickCandidates(overlap_kfs[i], cur_frames[i]), cur_frames[i]);
    match_sum_num += match_result.size();
    info << "s:n=" << match_result.size() << " ";
    matchs[i] = std::move(match_result);
  }
  LOG(INFO) << log_info::YELLOW << "Total match num :" << match_sum_num
            << " Seperate: " << info.str() << log_info::RESET;

  //
  if (match_sum_num == 0) return nullptr;
  WriteCheckMatchResult(matchs);
  //
  transform::Rigid3d pose = Optimize(
      track_data.data->pose, track_data.data->extric_camera_to_imu, matchs,
      std::array<float, 2>{options_.op_weight, options_.op_weight});
  //
  return std::make_unique<transform::Rigid3d>(pose);
}

//
std::vector<LocalMapTrack::Candidate> LocalMapTrack::PickCandidates(
    std::vector<KeyFrameId> overlap_kfs,
    const std::shared_ptr<match::Frame>& frame) {
  std::vector<LocalMapTrack::Candidate> candidates;
  for (const auto& ref_frame_id : overlap_kfs) {
    const auto& map_point_feature_ids =
        local_map_->GetCovisibility()->GetKeyFrameMapPointId(ref_frame_id);
    for (int i = 0; i < map_point_feature_ids.first.size(); i++) {
      const auto& point =
          local_map_->AllMapPoints().at(map_point_feature_ids.first[i]);
      int map_ob_kf_num =
          local_map_->GetCovisibility()
              ->GetMapPointObserv(map_point_feature_ids.first[i])
              .size();
      //
      if (map_ob_kf_num < 2 && options_.remove_unconstrained_points) {
        continue;
      }
      //
      Eigen::Vector3d point_world = point.data->Pos();
      Eigen::Vector2d px;
      if (!frame->IsVisible(point_world, &px)) continue;
      constexpr int kPatchSize = 8;
      if (!frame->IsKeypointVisibleWithMargin(px, kPatchSize)) continue;
      candidates.push_back(LocalMapTrack::Candidate{
          ref_frame_id, map_point_feature_ids.second[i], px, 0, 0,
          map_ob_kf_num, map_point_feature_ids.first[i]});
    }
  }
  LOG(INFO) << log_info::MAGENTA << "Candidate size: " << candidates.size()
            << log_info::RESET;
  return candidates;
}
//

//
std::vector<LocalMapTrack::MatchData> LocalMapTrack::MatchCandidates(
    const std::vector<Candidate>& candidates,
    const std::shared_ptr<match::Frame>& cur_frame) {
  if (!grid_) {
    grid_.reset(new match::svo::OccupandyGrid2D(
        options_.cell_size,
        match::svo::OccupandyGrid2D::getNCell(cur_frame->image_size.x(),
                                              options_.cell_size),
        match::svo::OccupandyGrid2D::getNCell(cur_frame->image_size.y(),
                                              options_.cell_size)));
  }
  std::vector<LocalMapTrack::MatchData> result;
  const auto& all_map_points = local_map_->AllMapPoints();
  for (auto& candidate : candidates) {
    size_t grid_index =
        grid_->getCellIndex(candidate.cur_px.x(), candidate.cur_px.y(), 1);
    if (options_.max_n_features_per_frame > 0 &&
        grid_->isOccupied(grid_index)) {
      continue;
    }
    auto math_result = MatchCandidate(candidate, cur_frame);
    if (math_result.state == match::MatchResultState::kSuccess) {
      result.push_back({math_result.norm.head<2>(),
                        all_map_points.at(candidate.mp_id).data->Pos(),
                        candidate});
    }
    //
  }
  grid_->reset();
  return result;
}
//
match::MatchResult LocalMapTrack::MatchCandidate(
    const Candidate& candidate, const std::shared_ptr<match::Frame>& frame) {
  match::GradientVector grad_ref;
  //
  const auto& all_kf_frames = local_map_->AllKeyFrameDatas();
  const auto& all_map_points = local_map_->AllMapPoints();

  auto& ref_frame_data = all_kf_frames.at(candidate.frame_id);
  //
  // const auto& map_point_feature_ids =
  //     local_map_->GetCovisibility()->GetKeyFrameMapPointId(candidate.frame_id);

  // if (ref_frams_catch_.count(candidate.frame_id) == 0 ||
  //     ref_frams_catch_[candidate.frame_id].count(
  //         candidate.feature_id.sequence_id) == 0) {
    // auto& ref_frame =
    //     ref_frams_catch_[candidate.frame_id][candidate.feature_id.sequence_id];
    auto ref_frame = std::make_shared<match::Frame>();
    ToFrame(all_kf_frames.at(candidate.frame_id), *ref_frame,
            candidate.feature_id.sequence_id);
  // }
  LOG(INFO)<<candidate.feature_id;
  int track_id = -1;
  auto ref_feature = all_kf_frames.at(candidate.frame_id)
                         .data->features.at(candidate.feature_id);
  match::FeatureWrapper feat_wrap{{match::FeatureType::kCorner},
                                  ref_feature.Point(),
                                  ref_feature.f,
                                  int(0),
                                  *all_map_points.at(candidate.mp_id).data};
  
  // auto ref_frame =
  //     ref_frams_catch_[candidate.frame_id][candidate.feature_id.sequence_id];

  double ref_depth = (all_kf_frames.at(candidate.frame_id).data->pose.inverse() *
                      all_map_points.at(candidate.mp_id).data->Pos())
                         .z();
  //
  return direct_match_->FindMatch(*ref_frame, *frame, feat_wrap, ref_depth,
                                  candidate.cur_px);
  //
  //
}
//
transform::Rigid3d LocalMapTrack::Optimize(
    const transform::Rigid3d& init_pose,
    const std::vector<transform::Rigid3d>& extric_camera_to_imu,
    const std::map<int, std::vector<LocalMapTrack::MatchData>>& constraints,
    const std::array<float, 2>& weight) {
  ceres::Problem problem;
  ceres::LocalParameterization* quaternion_local =
      new ceres::EigenQuaternionParameterization;
  Eigen::Quaterniond rotation = init_pose.rotation();
  Eigen::Vector3d traslation = init_pose.translation();

  //
  std::vector<Eigen::Quaterniond> ex_rotation;
  std::vector<Eigen::Vector3d> ex_traslation;
  //
  //
  for (int i = 0; i < extric_camera_to_imu_.size(); i++) {
    ex_rotation.push_back(
        extric_camera_to_imu[track_sequence[i][0]].rotation());
    ex_traslation.push_back(
        extric_camera_to_imu[track_sequence[i][0]].translation());
  }
  //
  for (const auto& constraist_seq : constraints) {
    for (int j = 0; j < constraist_seq.second.size(); j++) {
      //
      problem.AddResidualBlock(
          ReProjectionErr::Creat(constraist_seq.second[j].cur_normal_px,
                                 constraist_seq.second[j].map_point, weight[0]),
          new ceres::HuberLoss(0.5), traslation.data(),
          rotation.coeffs().data(),
          ex_rotation[options_.track_sequence[constraist_seq.first][0]]
              .coeffs()
              .data(),
          ex_traslation[options_.track_sequence[constraist_seq.first][0]]
              .data());
    }
    problem.SetParameterization(rotation.coeffs().data(), quaternion_local);
    //
    problem.SetParameterization(
        ex_rotation[options_.track_sequence[constraist_seq.first][0]]
            .coeffs()
            .data(),
        quaternion_local);
    //
    problem.SetParameterBlockConstant(
        ex_rotation[options_.track_sequence[constraist_seq.first][0]]
            .coeffs()
            .data());
    problem.SetParameterBlockConstant(
        ex_traslation[options_.track_sequence[constraist_seq.first][0]].data());
  }

  // problem.SetManifold(rotation.coeffs().data(), quaternion_manifold);
  // problem.SetManifold (rotation, quaternion_manifold);
  //
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 2;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // std::cout << summary.FullReport() << '\n';

  return {traslation, rotation};
}
//

void LocalMapTrack::WriteCheckMatchResult(
    const std::map<int, std::vector<LocalMapTrack::MatchData>>& matchs) {}

}  // namespace mapping
}  // namespace jarvis