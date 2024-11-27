#ifndef _JARVIS_LAOCAL_MAP_TRACK_H
#define _JARVIS_LAOCAL_MAP_TRACK_H
#include "jarvis/mapping/map_manger.h"
#include "jarvis/mapping/match/direct_match.h"
#include "jarvis/transform/rigid_transform.h"
#include "jarvis/transform/transform.h"
//
#include "jarvis/mapping/match/occupancy_grid_2d.h"
namespace jarvis {
namespace mapping {

struct LocalMapTrackOption {
  match::DirectMatchOption derect_match_option;
  bool remove_unconstrained_points;
  int cell_size;
  int max_n_features_per_frame = 120;
  std::vector<std::vector<int>> track_sequence;
};

class LocalMapTrack {
 public:
  transform::Rigid3d Track(const TrackingData& track_data);
  //
  struct MatchData {
    Eigen::Vector2d cur_normal_px;
    Eigen::Vector3d map_point;
  };

 private:
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

  //
  int MapPointIsInFrame(const Eigen::Vector3d pws,
                        const transform::Rigid3d& frame_pose);
  MapById<KeyFrameId, match::Frame> frame_warps_;
  //
  //
  std::vector<Candidate> PickCandidates(
      std::vector<KeyFrameId> overlap_kfs,
      const std::shared_ptr<match::Frame>& frame);
  //
  //
  //

  //
  std::vector<MatchData> MatchCandidates(
      const std::vector<Candidate>& candidates,
      const std::shared_ptr<match::Frame>& frame);
  match::MatchResult MatchCandidate(const Candidate& candidates,
                                    const std::shared_ptr<match::Frame>& frame);

  //

  int IsInFrame(const MapPoint& map_point, const TrackingData& track_data);
  bool MatchCandidate(const Candidate& candidate,
                      estimator::FeatureData& feature);
  //
  transform::Rigid3d Optimize(const transform::Rigid3d& init_pose,
                              std::map<int, std::vector<MatchData>> constraints,
                              const std::array<double, 2>& weight);
  //
  std::vector<camera_models::Camera> cameras_;
  std::shared_ptr<match::svo::OccupandyGrid2D> grid_;
  std::unique_ptr<match::DirectMatch> direct_match_;
  const MapManager* map_manager_;
  const LocalMapTrackOption options_;
  std::map<KeyFrameId, std::map<int, std::shared_ptr<match::Frame>>>
      ref_frams_catch_;
  std::vector<transform::Rigid3d> extric_camera_to_imu_;
};

}  // namespace mapping
}  // namespace jarvis
#endif