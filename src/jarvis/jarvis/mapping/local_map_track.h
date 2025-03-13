#ifndef _JARVIS_LAOCAL_MAP_TRACK_H
#define _JARVIS_LAOCAL_MAP_TRACK_H
#include "jarvis/key_frame_data.h"
#include "jarvis/mapping/match/direct_match.h"
#include "jarvis/transform/rigid_transform.h"
#include "jarvis/transform/transform.h"
//
#include "jarvis/common/thread_pool.h"
#include "jarvis/mapping/match/occupancy_grid_2d.h"
#include "jarvis/mapping/local_map.h"
#include <condition_variable>
namespace jarvis {
namespace mapping {

struct LocalMapTrackOption {
  match::DirectMatchOption derect_match_option;
  std::map<int, camera_models::CameraPtr> cameras; 
  int min_track_frame_num =5;
  bool remove_unconstrained_points=true;
  int min_convi_num =2;
  int max_n_features_per_frame = 420;
  std::vector<std::vector<int>> track_sequence;
  std::vector<transform::Rigid3d> extric_camera_to_imu;
  float op_weight = 377./2;
  float outlier_err=3./377;
  float first_outlier_err=10./377;
  float op_init_t_weight = 1;
  float op_init_r_weight = 5;
  int min_match_size =7;
  int min_op_inlier =8;
  int one_frame_pick_candidates_min_num =5;
  int one_frame_match_candidates_min_num =2;
  int one_kf_match_candidates_min_num =5;
  float same_came_senquece_max_angle  =50;
  std::set<int> sequence_match = {0,1,2};
  bool match_senquence0_alone=true;
  double huber_loss =0.1;
  double kf_max_distance =5;
  int max_num_pick_num=150;
  int min_out_time_kf_num =2;
  std::map<int,int> cell_sizes{{0, 100}, {1, 20}, {2, 20}};
  std::map<int,int> max_cell_sizes{{0, 100}, {1, 20}, {2, 20}};
  std::vector<Eigen::AlignedBox2i> image_boxs;
  double out_time=20;
  std::string test_match_pic_write_path = "";
  int max_num_iterations =3;
  int op_type  =0;//o  four
  common::ThreadPool* thread_pool;
};

class LocalMapTrack {
 public:
  LocalMapTrack(const LocalMapTrackOption& option);
  std::shared_ptr<LocalMapMatchResult> Track(
      const std::shared_ptr<LocalMap>& local_map,
      const KeyFrameData& track_data,
      const std::set<MapPointId>& exist_map_id = std::set<MapPointId>{});
  //
  struct Candidate {
    KeyFrameId frame_id;
    FeatureId feature_id;
    Eigen::Vector2d cur_px;  //!< Projected 2D pixel location in current frame.
    int n_reproj = 0;
    int score;  //!< Feature Detection Score
    int n_obs;
    MapPointId mp_id;
  };
  struct MatchData {
    Eigen::Vector2d cur_normal_px{0,0};
    Eigen::Vector3d map_point{0,0,0};
    std::optional<Candidate> candidate;  // for check
  };

  transform::Rigid3d Optimize(
      const transform::Rigid3d& init_pose,
      const std::vector<transform::Rigid3d>& extric_camera_to_imu,
      const std::map<int, std::vector<MatchData>>& constraints,
      const std::array<float, 2>& weight);
  //
  transform::Rigid3d FourOptimize(
      const transform::Rigid3d& init_pose,
      const std::vector<transform::Rigid3d>& extric_camera_to_imu,
      const std::map<int, std::vector<MatchData>>& constraints,
      const std::array<float, 2>& weight);

 private:
  void WriteCheckMatchResult(
      const KeyFrameData& key_frame_data,
      const std::map<int, std::vector<LocalMapTrack::MatchData>>& match_ids);
  //
  MapById<KeyFrameId, match::Frame> frame_warps_;
  //
  //
  int RemoveOutliersRejection(
      std::map<int, std::vector<LocalMapTrack::MatchData>>& matchs,
      const std::vector<transform::Rigid3d>& extric_camera_to_imu,
      const transform::Rigid3d& pose,float outlier);
  //
  std::vector<Candidate> PickCandidates(
      std::vector<KeyFrameId> overlap_kfs,
      const std::shared_ptr<match::Frame>& frame, int cur_s,
      const std::set<MapPointId>& exist_map_id);
  //
  //
  //

  //
  std::vector<MatchData> MatchCandidates(
      const std::vector<Candidate>& candidates,
      const std::shared_ptr<match::Frame>& frame,
      std::shared_ptr<match::svo::OccupandyGrid2D> grid);
  match::MatchResult MatchCandidate(const Candidate& candidates,
                                    const std::shared_ptr<match::Frame>& frame);

  //
  void ToFrame(const KeyFrameData& key_frame_data, match::Frame& fram,
               int index, const transform::Rigid3d& ref_key_frame_pos);

  int IsInFrame(const MapPoint& map_point, const KeyFrameData& track_data,
                const transform::Rigid3d& ref_key_frame_pos,int s);
  bool MatchCandidate(const Candidate& candidate,
                      estimator::FeatureData& feature);
  //
  //

  //
  std::vector<std::thread> threads_pools_;
  void RunWorks();
  // std::conditional thread_conditional;
  transform::Rigid3d PnpSolver(
      const transform::Rigid3d& init_pose,
      const std::vector<transform::Rigid3d>& extric_camera_to_imu,
      const std::map<int, std::vector<MatchData>>& constraints,
      const std::array<float, 2>& weight);

  // std::shared_ptr<match::svo::OccupandyGrid2D> grid_;
  std::map<int, std::shared_ptr<match::svo::OccupandyGrid2D>> grids_;
  std::map<int, std::shared_ptr<match::svo::OccupandyGrid2D>> max_grids_;
  std::map<int, std::shared_ptr<match::svo::OccupandyGrid2D>> temp_grids_;
  std::unique_ptr<match::DirectMatch> direct_match_;
  const LocalMapTrackOption options_;
  std::map<KeyFrameId, std::map<int, std::shared_ptr<match::Frame>>>
      ref_frams_catch_;
  std::vector<transform::Rigid3d> extric_camera_to_imu_;

  common::ThreadPool* thread_pool_;
  std::map<int, camera_models::CameraPtr> cameras_;
  std::vector<Eigen::Vector3d> px_top_lefts_;
  // for debug
  //
  std::shared_ptr<LocalMap> local_map_ = nullptr;
  //
  std::map<int,int> max_cell_sizes_{{0, 30}, {1, 30}, {2, 20}};

  std::unique_ptr<common::Task> when_done_task_ ;
  MapById<KeyFrameId, KeyFrameData> key_frames_datas_;

};

}  // namespace mapping
}  // namespace jarvis
#endif