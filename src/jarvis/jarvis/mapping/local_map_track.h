#ifndef _JARVIS_LAOCAL_MAP_TRACK_H
#define _JARVIS_LAOCAL_MAP_TRACK_H
#include "jarvis/mapping/map_manger.h"
#include "jarvis/mapping/match/direct_match.h"
#include "jarvis/transform/rigid_transform.h"
#include "jarvis/transform/transform.h"
//
#include "jarvis/mapping/match/occupancy_grid_2d.h"
#include "jarvis/mapping/local_map.h"
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
  float op_weight = 377/2;
  float op_init_t_weight = 1;
  float op_init_r_weight = 5;
  int min_match_size =7;
  int one_frame_pick_candidates_min_num =5;
  int one_frame_match_candidates_min_num =2;
  int one_kf_match_candidates_min_num =5;
  std::set<int> sequence_match = {0,1,2};
  bool match_senquence0_alone=true;
  double huber_loss =0.1;
  double kf_max_distance =5;
  std::map<int,int> cell_sizes{{0, 100}, {1, 20}, {2, 20}};
  double out_time=20;
  std::string test_match_pic_write_path = "";
  int max_num_iterations =3;
  int op_type  =0;//o  four
};

class LocalMapTrack {
 public:
  explicit LocalMapTrack(const LocalMapTrackOption& option);
  std::unique_ptr<transform::Rigid3d> Track(
      const std::shared_ptr<LocalMap>& local_map,
      const KeyFrameData& track_data);
  //
  //for debug
  std::vector<Eigen::Vector3d> GetMapPoints()const;
  std::vector<transform::Rigid3d> GetKfPose()const ;
  struct Candidate {
    KeyFrameId frame_id;
    FeatureId feature_id;
    Eigen::Vector2d cur_px;  //!< Projected 2D pixel location in current frame.
    int n_reproj =
        0;        //!< Number of previously successful projections for quality.
    float score;  //!< Feature Detection Score
    int n_obs;
    MapPointId mp_id;
  };
  struct MatchData {
    Eigen::Vector2d cur_normal_px;
    Eigen::Vector3d map_point;
    std::optional<Candidate> candidate;  // for check
  };

    transform::Rigid3d Optimize(
      const transform::Rigid3d& init_pose,
      const std::vector<transform::Rigid3d>& extric_camera_to_imu,
      const std::map<int, std::vector<MatchData>>& constraints,
      const std::array<float, 2>& weight);


    transform::Rigid3d FourOptimize(
      const transform::Rigid3d& init_pose,
      const std::vector<transform::Rigid3d>& extric_camera_to_imu,
      const std::map<int, std::vector<MatchData>>& constraints,
      const std::array<float, 2>& weight);
 private:


  void WriteCheckMatchResult(const KeyFrameData& key_frame_data,
      const std::map<int, std::vector<LocalMapTrack::MatchData>>& matchs);
  //
  MapById<KeyFrameId, match::Frame> frame_warps_;
  //
  //
  std::vector<Candidate> PickCandidates(
      std::vector<KeyFrameId> overlap_kfs,
      const std::shared_ptr<match::Frame>& frame,int cur_s);
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
                const transform::Rigid3d& ref_key_frame_pos);
  bool MatchCandidate(const Candidate& candidate,
                      estimator::FeatureData& feature);
  //
  //

  //

  transform::Rigid3d PnpSolver(
      const transform::Rigid3d& init_pose,
      const std::vector<transform::Rigid3d>& extric_camera_to_imu,
      const std::map<int, std::vector<MatchData>>& constraints,
      const std::array<float, 2>& weight);

  // std::shared_ptr<match::svo::OccupandyGrid2D> grid_;
  std::map<int, std::shared_ptr<match::svo::OccupandyGrid2D>> grids_;
  std::unique_ptr<match::DirectMatch> direct_match_;
  const LocalMapTrackOption options_;
  std::map<KeyFrameId, std::map<int, std::shared_ptr<match::Frame>>>
      ref_frams_catch_;
  std::vector<transform::Rigid3d> extric_camera_to_imu_;
  std::map<int, camera_models::CameraPtr> cameras_;
  std::vector<Eigen::Vector3d> px_top_lefts_;
  // for debug
  mutable std::mutex mutex_;
  //
  std::shared_ptr<LocalMap> local_map_ = nullptr;
  //
  MapById<KeyFrameId, KeyFrameData> key_frames_datas_;

};

}  // namespace mapping
}  // namespace jarvis
#endif