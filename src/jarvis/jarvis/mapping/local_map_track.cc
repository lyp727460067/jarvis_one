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
  bool operator()(const T* t1_, const T* q1_, T* residul) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t1(t1_);
    Eigen::Map<const Eigen::Quaternion<T>> q1(q1_);
    Eigen::Matrix<T, 3, 1> project_p = q1 * map_point_.template cast<T>() + t1;
    T x_normal = project_p[0] / project_p[2];
    T y_normal = project_p[1] / project_p[2];
    residul[0] = T(factor_) * (x_normal - T(nor_point_.x()));
    residul[1] = T(factor_) * (y_normal - T(nor_point_.y()));
    // LOG(INFO)<<residul[0];
    // LOG(INFO)<<residul[1];
    return true;
  }
  static ceres::CostFunction* Creat(const Eigen::Vector2d& nor_poit,
                                    const Eigen::Vector3d& map_point,
                                    double factor) {
    return new ceres::AutoDiffCostFunction<ReProjectionErr, 2, 3, 4>(
        new ReProjectionErr(nor_poit.head<2>(), map_point, factor));
  }

 private:
  const double factor_;
  const Eigen::Vector2d nor_point_;
  const Eigen::Vector3d map_point_;
};

void ToFrame(const TrackingData& track_data, match::Frame& fram, int index) {}
//
void ToFrame(const KeyFrameData& key_frame_data, match::Frame& fram,
             int index) {}
}  // namespace
//
transform::Rigid3d LocalMapTrack::Track(const TrackingData& track_data) {
  const auto& all_kf_frames = map_manager_->AllKeyFrameDatas();
  const auto& all_map_points = map_manager_->AllMapPoints();

  std::vector<std::vector<KeyFrameId>> overlap_kfs;
  overlap_kfs.resize(track_data.data->features_datas.size());
  //
  //
  //
  std::vector<std::shared_ptr<match::Frame>> cur_frames(
      track_data.data->features_datas.size());
  //

  for (int i = 0; i < track_data.data->features_datas.size(); i++) {
    ToFrame(track_data, *cur_frames[i], i);
  }
  //
  for (const auto& kf : all_kf_frames) {
    const auto& map_point_feature_ids =
        map_manager_->GetCovisibility()->GetKeyFrameMapPointId(kf.id);
    // 这里选择领域和公视的关键帧，还有只能投影一个点的3D点的
    for (auto& id : map_point_feature_ids.first) {
      auto& map_point = all_map_points.at(id);
      int index = IsInFrame(*map_point.data, track_data);
      if (index >= 0) {
        overlap_kfs.at(index).push_back(kf.id);
        break;
      }
    }
  }

  for (int i = 0; i < cur_frames.size(); i++) {
    auto match_result = MatchCandidates(
        PickCandidates(overlap_kfs[i], cur_frames[i]), cur_frames[i]);
  }

  //

  //
}

//
std::vector<LocalMapTrack::Candidate> LocalMapTrack::PickCandidates(
    std::vector<KeyFrameId> overlap_kfs,
    const std::shared_ptr<match::Frame>& frame) {
  std::vector<LocalMapTrack::Candidate> candidates;
  // const auto& all_kf_frames = map_manager_->AllKeyFrameDatas();
  const auto& all_map_points = map_manager_->AllMapPoints();
  for (const auto& ref_frame_id : overlap_kfs) {
    const auto& map_point_feature_ids =
        map_manager_->GetCovisibility()->GetKeyFrameMapPointId(ref_frame_id);
    for (int i = 0; i < map_point_feature_ids.first.size(); i++) {
      const auto& point = all_map_points.at(map_point_feature_ids.first[i]);
      if (point.data->ObNum() < 2 && options_.remove_unconstrained_points) {
        continue;
      }
      Eigen::Vector3d point_world = point.data->Pos();
      Eigen::Vector2d px;
      if (!frame->IsVisible(point_world, &px)) continue;
      constexpr int kPatchSize = 8;
      if (!frame->IsKeypointVisibleWithMargin(px, kPatchSize)) continue;
      candidates.push_back(LocalMapTrack::Candidate{
          ref_frame_id, map_point_feature_ids.second[i], px, 0, 0,
          point.data->ObNum(), map_point_feature_ids.first[i]});
    }
  }
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
  grid_->reset();
  std::vector<LocalMapTrack::MatchData> result;
  const auto& all_map_points = map_manager_->AllMapPoints();
  for (auto& candidate : candidates) {
    size_t grid_index =
        grid_->getCellIndex(candidate.cur_px.x(), candidate.cur_px.y(), 1);
    if (options_.max_n_features_per_frame > 0 &&
        grid_->isOccupied(grid_index)) {
      continue;
    }
    auto math_result = MatchCandidate(candidate, cur_frame);
    if (math_result.state == match::MatchResultState::kSuccess) {
      result.push_back(
          {math_result.norm.head<2>(), all_map_points.at(candidate.mp_id).data->Pos()});
    }
    //
  }
  return result;
}
//
match::MatchResult LocalMapTrack::MatchCandidate(
    const Candidate& candidate, const std::shared_ptr<match::Frame>& frame) {
  match::GradientVector grad_ref;
  const auto& all_kf_frames = map_manager_->AllKeyFrameDatas();
  const auto& all_map_points = map_manager_->AllMapPoints();

  auto& ref_frame_data = all_kf_frames.at(candidate.frame_id);
  //
  const auto& map_point_feature_ids =
      map_manager_->GetCovisibility()->GetKeyFrameMapPointId(
          candidate.frame_id);

  if (ref_frams_catch_.count(candidate.frame_id) == 0 ||
      ref_frams_catch_[candidate.frame_id].count(
          candidate.feature_id.sequence_id) == 0) {
    auto& ref_frame =
        ref_frams_catch_[candidate.frame_id][candidate.feature_id.sequence_id];
    ref_frame = std::make_shared<match::Frame>();
    ToFrame(all_kf_frames.at(candidate.frame_id), *ref_frame,
            candidate.feature_id.sequence_id);
  }

  int track_id = -1;
  auto ref_feature = all_kf_frames.at(candidate.frame_id)
                         .data->features.at(candidate.feature_id);
  match::FeatureWrapper feat_wrap{{match::FeatureType::kCorner},
                                  ref_feature.Point(),
                                  ref_feature.f,
                                  int(0),
                                  *all_map_points.at(candidate.mp_id).data};

  auto ref_frame =
      ref_frams_catch_[candidate.frame_id][candidate.feature_id.sequence_id];

  double ref_depth = (all_kf_frames.at(candidate.frame_id).data->pose *
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
    std::map<int, std::vector<LocalMapTrack::MatchData>> constraints,
    const std::array<double, 2>& weight) {
  ceres::Problem problem;
  // ceres::LocalParameterization* quaternion_manifold = new
  // ceres::EigenQuaternionManifold;
  ceres::LocalParameterization* quaternion_local =
      new ceres::EigenQuaternionParameterization;
  Eigen::Quaterniond rotation = init_pose.rotation();
  Eigen::Vector3d traslation = init_pose.translation();

  for (int i = 0; i < constraints.size(); i++) {
    for (int j = 0; j < constraints[i].size(); j++) {
      // problem.AddResidualBlock(
      //     ReProjectionErr::Creat(normal_2d[i], map_points[i], weight[0]),
      //     new ceres::HuberLoss(0.5), traslation.data(),
      //     rotation.coeffs().data());

      problem.SetParameterization(rotation.coeffs().data(), quaternion_local);
    }
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

}  // namespace mapping
}  // namespace jarvis