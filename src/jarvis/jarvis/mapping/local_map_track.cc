#include "jarvis/mapping/local_map_track.h"

#include "jarvis/mapping/covisibility.h"
#include "jarvis/mapping/map_manger.h"
#include "jarvis/mapping/match/pic_writer.h"
//
#include "jarvis/utility/tic_toc.h"
namespace jarvis {
namespace mapping {
namespace {
template <typename T>
inline T NormalizeAngle(const T& angle_radians) {
  // Use ceres::floor because it is specialized for double and Jet types.
  T two_pi(2.0 * M_PI);
  return angle_radians -
         two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}
// template <typename T>
// void YawPitchRollToRotationMatrix(const T yaw, const T pitch, const T roll,
//                                   T R[9]) {
//   T y = yaw / T(180.0) * T(M_PI);
//   T p = pitch / T(180.0) * T(M_PI);
//   T r = roll / T(180.0) * T(M_PI);

//   R[0] = cos(y) * cos(p);
//   R[1] = -sin(y) * cos(r) + cos(y) * sin(p) * sin(r);
//   R[2] = sin(y) * sin(r) + cos(y) * sin(p) * cos(r);
//   R[3] = sin(y) * cos(p);
//   R[4] = cos(y) * cos(r) + sin(y) * sin(p) * sin(r);
//   R[5] = -cos(y) * sin(r) + sin(y) * sin(p) * cos(r);
//   R[6] = -sin(p);
//   R[7] = cos(p) * sin(r);
//   R[8] = cos(p) * cos(r);
// };

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
        qe * q1 * map_point_.template cast<T>() + qe * t1 + te;
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
  const Eigen::Vector2d nor_point_;
  const Eigen::Vector3d map_point_;
  const double factor_;
};

struct FourReProjectionErr {
 public:
  FourReProjectionErr(const Eigen::Vector2d& nor_poit,
                      const Eigen::Vector3d& map_point, const double& roll,
                      const double& pitch, const double& factor)
      : nor_point_(nor_poit),
        map_point_(map_point),
        factor_(factor),
        pith_roll_rotation_(transform::RollPitchYaw(roll, pitch, 0.0)) {}

  template <typename T>
  bool operator()(const T* t1_, const T* q1_, const T* te_, const T* qe_,
                  T* residul) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t1(t1_);
    // Eigen::Map<const Eigen::Quaternion<T>> q1(q1_);
    //
    const Eigen::Quaternion<T> q1 =
        Eigen::AngleAxis<T>(q1_[0], Eigen::Matrix<T, 3, 1>::UnitZ()) *
        pith_roll_rotation_.cast<T>();
    //
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> te(te_);
    Eigen::Map<const Eigen::Quaternion<T>> qe(qe_);
    Eigen::Matrix<T, 3, 1> project_p =
        qe * q1 * map_point_.template cast<T>() + qe * t1 + te;
    T x_normal = project_p[0] / project_p[2];
    T y_normal = project_p[1] / project_p[2];
    residul[0] = T(factor_) * (x_normal - T(nor_point_.x()));
    residul[1] = T(factor_) * (y_normal - T(nor_point_.y()));
    return true;
  }
  static ceres::CostFunction* Creat(const Eigen::Vector2d& nor_poit,
                                    const Eigen::Vector3d& map_point,
                                    const double& roll, const double& pitch,
                                    double factor) {
    return new ceres::AutoDiffCostFunction<FourReProjectionErr, 2, 3, 1, 3, 4>(
        new FourReProjectionErr(nor_poit.head<2>(), map_point, roll, pitch,
                                factor));
  }

 private:
  const Eigen::Vector2d nor_point_;
  const Eigen::Vector3d map_point_;
  const double factor_;
  const Eigen::Quaterniond pith_roll_rotation_;
};

class TranslationCostFunctor {
 public:
  static ceres::CostFunction* Create(const Eigen::Vector3d& translation,
                                     const double& factor) {
    return new ceres::AutoDiffCostFunction<TranslationCostFunctor, 3, 3>(
        new TranslationCostFunctor(translation, factor));
  }

  template <typename T>
  bool operator()(const T* const translation, T* residual) const {
    residual[0] = factor_ * (x_ - translation[0]);
    residual[1] = factor_ * (y_ - translation[1]);
    residual[2] = factor_ * (z_ - translation[2]);
    return true;
  }

 private:
  explicit TranslationCostFunctor(const Eigen::Vector3d& translation,
                                  const double& factor)
      : factor_(factor),
        x_(translation.x()),
        y_(translation.y()),
        z_(translation.z()) {}

  const double factor_;
  const double x_;
  const double y_;
  const double z_;
};

class RotationDeltaCostFunctor {
 public:
  static ceres::CostFunction* Create(const Eigen::Quaterniond& rotation,
                                     const double factor) {
    return new ceres::AutoDiffCostFunction<RotationDeltaCostFunctor, 3, 4>(
        new RotationDeltaCostFunctor(rotation, factor));
  }

  template <typename T>
  bool operator()(const T* const q1_, T* residual) const {
    Eigen::Map<const Eigen::Quaternion<T>> q1(q1_);
    Eigen::Matrix<T, 3, 1> delta =
        T(2.0) * (q1.inverse() * rotaion_.template cast<T>()).vec();
    residual[0] = factor_ * delta.x();
    residual[1] = factor_ * delta.y();
    residual[2] = factor_ * delta.z();
    return true;
  }

 private:
  explicit RotationDeltaCostFunctor(const Eigen::Quaterniond& rotation,
                                    const double& factor)
      : factor_(factor), rotaion_(rotation) {}

  const double factor_;
  const Eigen::Quaterniond rotaion_;
};
//
class YawRotationDeltaCostFunctor {
 public:
  static ceres::CostFunction* Create(const double& rotation,
                                     const double factor) {
    return new ceres::AutoDiffCostFunction<YawRotationDeltaCostFunctor, 1, 1>(
        new YawRotationDeltaCostFunctor(rotation, factor));
  }

  template <typename T>
  bool operator()(const T* const q1_, T* residual) const {
    residual[0] = factor_ * NormalizeAngle(T(yaw_) - q1_[0]);
    return true;
  }

 private:
  explicit YawRotationDeltaCostFunctor(const double& yaw, const double& factor)
      : factor_(factor), yaw_(yaw) {}

  const double factor_;
  const double yaw_;
};

//

}  // namespace
//
void LocalMapTrack::ToFrame(const KeyFrameData& key_frame_data,
                            match::Frame& fram, int index,
                            const transform::Rigid3d& ref_key_frame_pos) {
  CHECK( cameras_.count(index));
  fram.cam = cameras_.at(index);

  fram.image_size = key_frame_data.data->image_sizes->at(index).sizes();
  //
  fram.pose = key_frame_data.data->CameraPose(ref_key_frame_pos, index);
  fram.f_pose = ref_key_frame_pos;
  fram.img_pyr = key_frame_data.data->Pyramid(index);
  fram.f_top_left = &px_top_lefts_[index];

}
//
int LocalMapTrack::IsInFrame(const MapPoint& map_point,
                             const KeyFrameData& track_data,
                             const transform::Rigid3d& ref_key_frame_pos) {
  const Eigen::Vector3d xyz_w = map_point.pos;
  //
  //
  for (size_t sequence_id = 0; sequence_id < options_.track_sequence.size();
       sequence_id++) {
    const auto pose =
        track_data.data->CameraPose(ref_key_frame_pos, sequence_id);
    Eigen::Vector3d xyz_f = pose.inverse() * xyz_w;
    //
    if (xyz_f.z() < 0) continue;
    return sequence_id;
    // Eigen::Vector2d px_top_left(0.0, 0.0);
    Eigen::Vector3d f_top_left = px_top_lefts_[sequence_id];
    // cameras_.at(sequence_id)
    // ->liftProjective(px_top_left, f_top_left);  // 注意这里找对应的相机
    const Eigen::Vector3d z(0.0, 0.0, 1.0);
    const double min_cos = f_top_left.dot(z);
    const double cur_cos = xyz_f.normalized().dot(z);
    if (cur_cos > min_cos) {
      return sequence_id;
    }
  }
  return -1;
}

LocalMapTrack::LocalMapTrack(const LocalMapTrackOption& option)
    : options_(option), cameras_(option.cameras) {
  direct_match_ =
      std::make_unique<match::DirectMatch>(options_.derect_match_option);
  CHECK(!options_.track_sequence.empty());
  LOG(INFO) << options_.track_sequence.size();
  for (size_t i = 0; i < options_.track_sequence.size(); i++) {
    Eigen::Vector3d f_top_left;
    Eigen::Vector2d px_top_left(0.0, 0.0);
    cameras_.at(i)->liftProjective(px_top_left,
                                   f_top_left);  // 注意这里找对应的相机

    px_top_lefts_.push_back((f_top_left / f_top_left.z()).normalized());
    LOG(INFO) << px_top_lefts_.back().transpose();
  }

  thread_pool_ = std::make_unique<common::ThreadPool>(1);
  when_done_task_ = std::make_unique<common::Task>();
}

//
void LocalMapTrack::RunWorks() {}

//
std::unique_ptr<LocalMapMatchResult> LocalMapTrack::Track(
    const std::shared_ptr<LocalMap>& local_map,
    const KeyFrameData& track_data) {
  local_map_ = local_map;
  const auto& all_kf_frames = local_map_->AllKeyFrameDatas();
  const auto& all_kf_re_poses = local_map_->AllKeyFrameRefPose();
  LOG_EVERY_N(INFO, 100) << "Local map size: " << all_kf_frames.size();
  if (all_kf_frames.size() < size_t(options_.min_track_frame_num))
    return nullptr;
  //

  const auto sequence_feautes = track_data.data->features.trajectory_ids();
  std::map<int, std::vector<KeyFrameId>> overlap_kfs;
  std::map<int, std::shared_ptr<match::Frame>> cur_frames;
  //
  const transform::Rigid3d& cur_ref_kf_pose =
      local_map->LocalPose().inverse() * track_data.data->pose;
  //
  for (size_t i = 0; i < options_.track_sequence.size(); i++) {
    cur_frames[i] = std::make_shared<match::Frame>();
    ToFrame(track_data, *cur_frames[i], i, cur_ref_kf_pose);
  }
  //

  auto const time_it = all_kf_frames.lower_bound(
      all_kf_frames.begin()->id.trajectory_id,
      track_data.data->time - common::FromSeconds(options_.out_time));
  //
  for (auto it = all_kf_frames.begin(); it != time_it; ++it) {
    // for (const auto& kf : all_kf_frames) {
    // 这里选择领域和公视的关键帧，还有只能投影一个点的3D点的
    const auto& kf = *it;
    const float distance =
        (track_data.data->pose.inverse() * kf.data.data->pose)
            .translation()
            .norm();
    if (distance > options_.kf_max_distance) continue;
    const auto map_points = local_map_->GetKeyFrameMapPoints(kf.id);
    const transform::Rigid3d& ref_kf_pose = all_kf_re_poses.at(kf.id);
    for (const auto& map_point : map_points) {
      int index = IsInFrame(*map_point.second.data, track_data, ref_kf_pose);
      if (options_.sequence_match.count(index) == 0) continue;
      if (index >= 0) {
        overlap_kfs[index].push_back(kf.id);
        break;
      }
    }
  }
  std::map<int, std::vector<LocalMapTrack::MatchData>> matchs;
  //
  int match_sum_num = 0;
  auto start = std::chrono::high_resolution_clock::now();
  std::map<int, std::vector<LocalMapTrack::Candidate>> pick_cadidates;
  std::stringstream cost_time_info;
  for (size_t i = 0; i < cur_frames.size(); i++) {
    if (options_.sequence_match.count(i) == 0) continue;
    //
    estimator::TicToc pick_candidata_tic;
    auto candidates = PickCandidates(overlap_kfs[i], cur_frames[i], i);
    cost_time_info << "s" << i << " " << pick_candidata_tic.toc();

    pick_cadidates[i] = std::move(candidates);
    auto& grid = grids_[i];
    if (!grid) {
      grid.reset(new match::svo::OccupandyGrid2D(
          options_.cell_sizes.at(i),
          match::svo::OccupandyGrid2D::getNCell(cur_frames[i]->image_size.x(),
                                                options_.cell_sizes.at(i)),
          match::svo::OccupandyGrid2D::getNCell(cur_frames[i]->image_size.y(),
                                                options_.cell_sizes.at(i))));
    }
  }

  for (size_t i = 0; i < cur_frames.size(); i++) {
    //
    if (options_.sequence_match.count(i) == 0) continue;

    auto sequ_match_task = std::make_unique<common::Task>();
    sequ_match_task->SetWorkItem([&, i]() {
      estimator::TicToc match_candidata_tic;
      auto& grid = grids_[i];
      auto& candidates = pick_cadidates[i];
      if (candidates.size() <
          size_t(options_.one_frame_pick_candidates_min_num))
        return;

      auto match_result = MatchCandidates(candidates, cur_frames[i], grid);
      grid->reset();
      if (match_result.size() <
          size_t(options_.one_frame_match_candidates_min_num)) {
        return;
      }
      matchs[i] = std::move(match_result);
      LOG(INFO) << " mach:" << match_candidata_tic.toc();
    });
    auto sequ_match_task_handle =
        thread_pool_->Schedule(std::move(sequ_match_task));
    //
    when_done_task_->AddDependency(sequ_match_task_handle);
  }

  std::mutex mutex;
  std::condition_variable condtion;
  bool match_finish = false;
  when_done_task_->SetWorkItem([&] {
    std::lock_guard<std::mutex> lock(mutex);
    match_finish = true;
    condtion.notify_all();
  });
  //
  //

  thread_pool_->Schedule(std::move(when_done_task_));
  when_done_task_ = std::make_unique<common::Task>();
  //
  {
    std::unique_lock<std::mutex> locker(mutex);
    condtion.wait(locker, [&]() { return match_finish; });
  }
  std::stringstream info;
  for (size_t i = 0; i < cur_frames.size(); i++) {
    size_t cadidates_size = 0;
    size_t match_size = 0;
    if (pick_cadidates.count(i)) {
      cadidates_size = pick_cadidates[i].size();
    }
    if (matchs.count(i)) {
      match_size = matchs[i].size();
      match_sum_num += match_size;
    }
    info << "s(" << i << ")" << "pick canditate size:" << cadidates_size
         << ",match candianti size:" << match_size << " ";
  }

  // std::stringstream info;
  // std::stringstream cost_time_info;
  // // auto start = std::chrono::high_resolution_clock::now();
  // for (size_t i = 0; i < cur_frames.size(); i++) {
  //   //
  //   if (options_.sequence_match.count(i) == 0) continue;
  //   //
  //   estimator::TicToc pick_candidata_tic;
  //   auto const candidates = PickCandidates(overlap_kfs[i], cur_frames[i],i);
  //   cost_time_info << "s" << i << " " << pick_candidata_tic.toc();
  //   if (candidates.size() <
  //   size_t(options_.one_frame_pick_candidates_min_num))
  //     continue;
  //   auto& grid = grids_[i];
  //   if (!grid) {
  //     grid.reset(new match::svo::OccupandyGrid2D(
  //         options_.cell_sizes.at(i),
  //         match::svo::OccupandyGrid2D::getNCell(cur_frames[i]->image_size.x(),
  //                                               options_.cell_sizes.at(i)),
  //         match::svo::OccupandyGrid2D::getNCell(cur_frames[i]->image_size.y(),
  //                                               options_.cell_sizes.at(i))));
  //   }
  //   estimator::TicToc match_candidata_tic;
  //   auto match_result = MatchCandidates(candidates, cur_frames[i], grid);
  //   //
  //   cost_time_info <<" mach:" <<  match_candidata_tic.toc();
  //   grid->reset();
  //   if (match_result.size() <
  //   size_t(options_.one_frame_match_candidates_min_num))
  //     continue;

  //   info << "s(" << i << ")" << "pick canditate size:" << candidates.size()
  //        << ",match candianti size:" << match_result.size() << " ";
  //   //
  //   match_sum_num += match_result.size();
  //   matchs[i] = std::move(match_result);
  // }
  transform::Rigid3d pose =
      local_map_->LocalPose().inverse() * track_data.data->pose;

  int inliner =
      RemoveOutliersRejection(matchs, track_data.data->extric_camera_to_imu,
                              pose, options_.first_outlier_err);
  info << " init_inliner match cnt :" << inliner;
  LOG(INFO) << "total match cost : "
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::high_resolution_clock::now() - start)
                   .count()
            << " " << cost_time_info.str();
  LOG(INFO) << log_info::GREEN << "Total match num :" << match_sum_num
            << ",Seperate: " << info.str() << log_info::RESET;

  //
  if (match_sum_num < options_.min_match_size ||
      inliner < options_.min_op_inlier)
    return nullptr;
  WriteCheckMatchResult(track_data, matchs);
  //
  // CHECK(false);
  info.clear();

  //
  
  //
  if (options_.op_type == 1) {
    transform::Rigid3d pose =
        Optimize(local_map_->LocalPose().inverse() * track_data.data->pose,
                 track_data.data->extric_camera_to_imu, matchs,
                 std::array<float, 2>{options_.op_weight, options_.op_weight});
    //
    return std::make_unique<LocalMapMatchResult>(
        LocalMapMatchResult{local_map->LocalPose() * pose, {}});
  } else if (options_.op_type == 0) {
    transform::Rigid3d init_pose = pose;
    for (int i = 0; i < options_.max_num_iterations; i++) {
      pose = FourOptimize(
          pose, track_data.data->extric_camera_to_imu, matchs,
          std::array<float, 2>{options_.op_weight, options_.op_weight});

      inliner =
          RemoveOutliersRejection(matchs, track_data.data->extric_camera_to_imu,
                                  pose, options_.outlier_err);
      info << " inter" << i << " inliner " << inliner;

      //
    }
    // WriteCheckMatchResult(track_data, matchs);
    LOG(INFO) << log_info::RED << "yaw "
              << common::RadToDeg(transform::GetYaw(init_pose)) << " -> "
              << common::RadToDeg(transform::GetYaw(pose))
              << " t:" << init_pose.translation().transpose() << "-> "
              << pose.translation().transpose() << info.str();

    return std::make_unique<LocalMapMatchResult>(
        LocalMapMatchResult{local_map->LocalPose() * pose, {}});

  } else if (options_.op_type == 2) {
    std::vector<LocalMapMatchResult::Match> matchs_result;
    for (auto& constraist_matchs : matchs) {
      for (auto& match : constraist_matchs.second) {
        matchs_result.push_back(LocalMapMatchResult::Match{
            constraist_matchs.first, local_map->LocalPose() * match.map_point,
            match.cur_normal_px});
      }
    }

    return std::make_unique<LocalMapMatchResult>(
        LocalMapMatchResult{local_map->LocalPose() * pose, matchs_result});
  }

//

return nullptr;
}
//
int LocalMapTrack::RemoveOutliersRejection(
    std::map<int, std::vector<LocalMapTrack::MatchData>>& matchs,
    const std::vector<transform::Rigid3d>& extric_camera_to_imu,
    const transform::Rigid3d& pose, float outlier) {
  auto ReprojectionError = [](const Eigen::Vector3d world_point_i,
                              const transform::Rigid3d& pose_j,
                              const Eigen::Vector2d& uvj) {
    //
    const Eigen::Vector3d pts_cj = pose_j.inverse() * world_point_i;
    Eigen::Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj;
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);
  };
  int inliner = 0;
  for (auto& constraist_matchs : matchs) {
    transform::Rigid3d exti =
        extric_camera_to_imu[options_
                                 .track_sequence[constraist_matchs.first][0]];

    for (auto it = constraist_matchs.second.begin();
         it != constraist_matchs.second.end();) {
      float err =
          ReprojectionError(it->map_point, pose * exti, it->cur_normal_px);
      if (err > outlier) {
        it = constraist_matchs.second.erase(it);
        continue;
      }
      inliner++;
      ++it;
    }
  }
  return inliner;
}
//
//
std::vector<LocalMapTrack::Candidate> LocalMapTrack::PickCandidates(
    std::vector<KeyFrameId> overlap_kfs,
    const std::shared_ptr<match::Frame>& frame, int cur_s) {
  std::vector<LocalMapTrack::Candidate> candidates;
  std::set<MapPointId> eixst_map_point_ids;
  if (overlap_kfs.empty()) return {};
  for (const auto& ref_frame_id : overlap_kfs) {
    std::vector<LocalMapTrack::Candidate> candidates_temp;
    //
    const float distance = (frame->f_pose.inverse() *
                            local_map_->AllKeyFrameRefPose().at(ref_frame_id))
                               .translation()
                               .norm();
    //
    const auto& map_point_feature_ids =
        local_map_->GetCovisibility()->GetKeyFrameMapPointId(ref_frame_id);
    for (size_t i = 0; i < map_point_feature_ids.first.size(); i++) {
      //
      if (eixst_map_point_ids.count(map_point_feature_ids.first[i])) continue;
      //
      const auto& point =
          local_map_->AllMapPoints().at(map_point_feature_ids.first[i]);
      int map_ob_kf_num =
          local_map_->GetCovisibility()
              ->GetMapPointObserv(map_point_feature_ids.first[i])
              .size();
      //
      if (map_ob_kf_num < options_.min_convi_num &&
          options_.remove_unconstrained_points) {
        continue;
      }
      //
      Eigen::Vector3d point_world = point.data->pos;
      Eigen::Vector2d px;
      if (!frame->IsVisible(point_world, &px)) continue;
      constexpr int kPatchSize = 8;
      if (!frame->IsKeypointVisibleWithMargin(px, kPatchSize)) continue;
      //
      if (options_.sequence_match.count(
              map_point_feature_ids.second[i].sequence_id) == 0)
        continue;
      //
      if (options_.match_senquence0_alone) {
        if ((map_point_feature_ids.second[i].sequence_id != 0 && cur_s == 0) ||
            (cur_s != 0 && map_point_feature_ids.second[i].sequence_id == 0)) {
          continue;
        }
      }
      eixst_map_point_ids.insert(map_point_feature_ids.first[i]);
      candidates_temp.push_back(LocalMapTrack::Candidate{
          ref_frame_id, map_point_feature_ids.second[i], px,0,distance*10,
          map_ob_kf_num, map_point_feature_ids.first[i]});
    }
    if (candidates_temp.size() >
        size_t(options_.one_kf_match_candidates_min_num)) {
      candidates.insert(candidates.end(), candidates_temp.begin(),
                        candidates_temp.end());
    }
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const LocalMapTrack::Candidate& c,
               const LocalMapTrack::Candidate& c1) {
              if (c.score > c1.score) {
                return true;
              } else if ((c.score == c1.score) && c.n_obs > c1.n_obs) {
                return true;
              } 
                return false;
            });
  return candidates;
}
//
#define _THREAD_POOL_
//
std::vector<LocalMapTrack::MatchData> LocalMapTrack::MatchCandidates(
    const std::vector<Candidate>& candidates,
    const std::shared_ptr<match::Frame>& cur_frame,
    std::shared_ptr<match::svo::OccupandyGrid2D> grid) {
  std::vector<LocalMapTrack::MatchData> result;
  // std::vector<std::shared_ptr<LocalMapTrack::MatchData>> result;
  const auto& all_map_points = local_map_->AllMapPoints();
  for (auto& candidate : candidates) {
    size_t grid_index =
        grid->getCellIndex(candidate.cur_px.x(), candidate.cur_px.y(), 1);
    if (options_.max_n_features_per_frame > 0 && grid->isOccupied(grid_index)) {
      continue;
    }
    // #ifdef  _THREAD_POOL_

    // #else
    // #endif
    // auto match_task = std::make_unique<common::Task>();
    // result.emplace_back(nullptr);
    // //
    // match_task->SetWorkItem([&]() {
    //   auto math_result = MatchCandidate(candidate, cur_frame);
    //   if (math_result.state == match::MatchResultState::kSuccess) {
    //     result.back()=
    //         std::make_shared<LocalMapTrack::MatchData>(LocalMapTrack::MatchData{
    //             math_result.norm.head<2>(),
    //             all_map_points.at(candidate.mp_id).data->pos, candidate});
    //     result.back()->candidate.value().cur_px = math_result.pt;
    //   }
    // });
    // auto match_task_handle = thread_pool_->Schedule(std::move(match_task));
    // when_done_task_->AddDependency(match_task_handle);
    // //

    //
    auto math_result = MatchCandidate(candidate, cur_frame);

    if (math_result.state == match::MatchResultState::kSuccess) {
      result.push_back({math_result.norm.head<2>(),
                        all_map_points.at(candidate.mp_id).data->pos,
                        candidate});
      result.back().candidate.value().cur_px = math_result.pt;
    }
    //
  }
  return result;
  // std::mutex mutex;
  // std::condition_variable condtion;
  // bool match_finish = false;
  // when_done_task_->SetWorkItem([&] {
  //   std::lock_guard<std::mutex> lock(mutex);
  //   match_finish = true;
  //   condtion.notify_all();
  // });
  // //
  // thread_pool_->Schedule(std::move(when_done_task_));
  // when_done_task_ = std::make_unique<common::Task>();
  // //
  // {
  //   std::unique_lock<std::mutex> locker(mutex);
  //   condtion.wait(locker, [&]() { return match_finish; });
  // }
  // std::vector<LocalMapTrack::MatchData> result1;
  // for (size_t i = 0; i < result.size(); i++) {
  //   if (result[i]) {
  //     result1.emplace_back(std::move(*result[i]));
  //   }
  // }

  // grid_->reset();
  // return result1;
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
  //

  ToFrame(all_kf_frames.at(candidate.frame_id), *ref_frame,
          candidate.feature_id.sequence_id,
          local_map_->LocalPose().inverse() * ref_frame_data.data->pose);
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

  const auto& ref_pose =
      local_map_->AllKeyFrameRefPose().at(candidate.frame_id);

  double ref_depth =
      (all_kf_frames.at(candidate.frame_id)
           .data->CameraPose(ref_pose, candidate.feature_id.sequence_id)
           .inverse() *
       all_map_points.at(candidate.mp_id).data->pos)
          .z();
  //

  return direct_match_->FindMatch(*ref_frame, *frame, feat_wrap, ref_depth,
                                  candidate.cur_px);
  //
  //
}

transform::Rigid3d LocalMapTrack::PnpSolver(
    const transform::Rigid3d& init_pose,
    const std::vector<transform::Rigid3d>& extric_camera_to_imu,
    const std::map<int, std::vector<MatchData>>& constraints,
    const std::array<float, 2>& weight) {
  //

  std::vector<cv::Point3f> map_result;
  std::vector<cv::Point2f> normal_result;
  return {};
}

//
transform::Rigid3d LocalMapTrack::Optimize(
    const transform::Rigid3d& init_pose,
    const std::vector<transform::Rigid3d>& extric_camera_to_imu,
    const std::map<int, std::vector<LocalMapTrack::MatchData>>&
        constraints,
    const std::array<float, 2>& weight) {
  //
  ceres::Problem problem;
  ceres::LocalParameterization* quaternion_local =
      new ceres::EigenQuaternionParameterization;

  Eigen::Quaterniond rotation = init_pose.inverse().rotation();
  Eigen::Vector3d traslation = init_pose.inverse().translation();

  //
  Eigen::Quaterniond ex_rotation[options_.track_sequence.size()];
  Eigen::Vector3d ex_traslation[options_.track_sequence.size()];
  //
  //
  problem.AddParameterBlock(traslation.data(), 3);
  problem.AddParameterBlock(rotation.coeffs().data(), 4);
  problem.SetParameterization(rotation.coeffs().data(), quaternion_local);
  //

  for (size_t i = 0; i < options_.track_sequence.size(); i++) {
    // LOG(INFO)<<extric_camera_to_imu[options_.track_sequence[i][0]];
    transform::Rigid3d extir_iverse =
        extric_camera_to_imu[options_.track_sequence[i][0]].inverse();
    ex_rotation[i] = extir_iverse.rotation();
    ex_traslation[i] = extir_iverse.translation();
    problem.AddParameterBlock(ex_rotation[i].coeffs().data(), 4);
    problem.AddParameterBlock(ex_traslation[i].data(), 3);
    problem.SetParameterBlockConstant(ex_rotation[i].coeffs().data());
    problem.SetParameterBlockConstant(ex_traslation[i].data());

    //
    problem.SetParameterization(ex_rotation[i].coeffs().data(),
                                quaternion_local);
  }

  for (const auto& constraist_seq : constraints) {
    for (size_t j = 0; j < constraist_seq.second.size(); j++) {
      //
      problem.AddResidualBlock(
          ReProjectionErr::Creat(constraist_seq.second[j].cur_normal_px,
                                 constraist_seq.second[j].map_point, weight[0]),
          new ceres::HuberLoss(options_.huber_loss), traslation.data(),
          rotation.coeffs().data(), ex_traslation[constraist_seq.first].data(),
          ex_rotation[constraist_seq.first].coeffs().data());
    }
  }

  problem.AddResidualBlock(
      RotationDeltaCostFunctor::Create(init_pose.inverse().rotation(),
                                       options_.op_init_r_weight),
      nullptr, rotation.coeffs().data());

  problem.AddResidualBlock(
      TranslationCostFunctor::Create(init_pose.inverse().translation(),
                                     options_.op_init_t_weight),
      nullptr, traslation.data());
  // problem.SetManifold(rotation.coeffs().data(), quaternion_manifold);
  // problem.SetManifold (rotation, quaternion_manifold);
  //
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = options_.max_num_iterations;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  return transform::Rigid3d(traslation, rotation).inverse();
}
//
transform::Rigid3d LocalMapTrack::FourOptimize(
    const transform::Rigid3d& init_pose,
    const std::vector<transform::Rigid3d>& extric_camera_to_imu,
    const std::map<int, std::vector<LocalMapTrack::MatchData>>&
        constraints,
    const std::array<float, 2>& weight) {
  ceres::Problem problem;
  ceres::LocalParameterization* quaternion_local =
      new ceres::EigenQuaternionParameterization;
  //
  //

  Eigen::Quaterniond rotation = init_pose.inverse().rotation();
  Eigen::Vector3d traslation = init_pose.inverse().translation();

  Eigen::Vector3d ypr = transform::Rot2ypr(rotation.toRotationMatrix());
  //
  double yaw = common::DegToRad(ypr[0]);
  const double pitch = common::DegToRad(ypr[1]);
  const double roll = common::DegToRad(ypr[2]);
  Eigen::Quaterniond ex_rotation[options_.track_sequence.size()];
  Eigen::Vector3d ex_traslation[options_.track_sequence.size()];
  for (size_t i = 0; i < options_.track_sequence.size(); i++) {
    // LOG(INFO)<<extric_camera_to_imu[options_.track_sequence[i][0]];
    transform::Rigid3d extir_iverse =
        extric_camera_to_imu[options_.track_sequence[i][0]].inverse();
    ex_rotation[i] = extir_iverse.rotation();
    ex_traslation[i] = extir_iverse.translation();
    problem.AddParameterBlock(ex_rotation[i].coeffs().data(), 4);
    problem.AddParameterBlock(ex_traslation[i].data(), 3);
    problem.SetParameterBlockConstant(ex_rotation[i].coeffs().data());
    problem.SetParameterBlockConstant(ex_traslation[i].data());

    //
    problem.SetParameterization(ex_rotation[i].coeffs().data(),
                                quaternion_local);
  }

  for (const auto& constraist_seq : constraints) {
    for (size_t j = 0; j < constraist_seq.second.size(); j++) {
      //
      problem.AddResidualBlock(
          FourReProjectionErr::Creat(constraist_seq.second[j].cur_normal_px,
                                     constraist_seq.second[j].map_point, roll,
                                     pitch, weight[0]),
          new ceres::HuberLoss(options_.huber_loss), traslation.data(), &yaw,
          ex_traslation[constraist_seq.first].data(),
          ex_rotation[constraist_seq.first].coeffs().data());
    }
    //
  }

  problem.AddResidualBlock(
      TranslationCostFunctor::Create(init_pose.inverse().translation(),
                                     options_.op_init_t_weight),
      nullptr, traslation.data());

  problem.AddResidualBlock(
      YawRotationDeltaCostFunctor::Create(common::DegToRad(ypr[0]),
                                          options_.op_init_r_weight),
      nullptr, &yaw);

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 1;  // options_.max_num_iterations;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  LOG(INFO) << log_info::RED << summary.BriefReport() << log_info::RESET;
  // LOG(INFO) << summary.FullReport();
  const auto pose =
      transform::Rigid3d(traslation,
                         transform::RollPitchYaw(roll, pitch, yaw).normalized())
          .inverse();

  return pose;
}
//
void LocalMapTrack::WriteCheckMatchResult(
    const KeyFrameData& key_frame_data,
    const std::map<int, std::vector<LocalMapTrack::MatchData>>& matchs) {
  if (options_.test_match_pic_write_path.empty()) return;
  std::map<KeyFrameId, std::vector<std::pair<FeatureId, FeatureId>>>
      paire_indexs;
  //
  KeyFrameData key_frame_data_tem = key_frame_data;

  for (auto& match : matchs) {
    for (size_t i = 0; i < match.second.size(); i++) {
      auto& candidata = match.second[i].candidate.value();
      auto cur_feature_id = key_frame_data_tem.data->features.Append(
          match.first, FeatureData({cv::KeyPoint(candidata.cur_px.x(),
                                                 candidata.cur_px.y(), 2)}));

      paire_indexs[candidata.frame_id].push_back(
          {cur_feature_id, candidata.feature_id});
    }
  }
  const auto& all_kf_frames = local_map_->AllKeyFrameDatas();
  for (auto& tar_frame : paire_indexs) {
    match::WriteImageWithKeyPoint(
        options_.test_match_pic_write_path, *key_frame_data_tem.data,
        *all_kf_frames.at(tar_frame.first).data, tar_frame.second);
  }
}
//

}  // namespace mapping
}  // namespace jarvis