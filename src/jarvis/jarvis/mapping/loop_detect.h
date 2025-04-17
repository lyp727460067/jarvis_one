#ifndef JARVIS_MAPPING_LOOP_DETECT_H
#define JARVIS_MAPPING_LOOP_DETECT_H
#include <functional>
#include <mutex>
#include "jarvis/alg/pnp_solver.h"
#include "jarvis/common/id.h"
#include "jarvis/mapping/constraint_consistent_filter.h"
#include "jarvis/mapping/local_map.h"
#include "jarvis/transform/rigid_transform.h"
//
#include "jarvis/common/thread_pool.h"
namespace jarvis {
namespace mapping {

struct LoopDetectOption {
  std::vector<Eigen::AlignedBox2i> image_boxs;
  std::vector<std::vector<int>> track_sequence; 
  //
  KeyFrameDataBaseOption key_frame_data_option;
  match::ProjectionOption project_option;
  alg::PnpSolverOption pnp_solver_option;
  std::vector<int> convisi_level_search_num{5, 3};
  int pnp_solve_typ =0;
  double huber_loss = 1.0;
  double outlier_min_err = 8.0;
  int max_num_iterations = 4;
  double op_weight = 500;
  double op_init_t_weight = 10;
  double op_init_r_weight = 10;
  double min_filter_dbow_covisi_score = 0.75;
  int candidata_reproject_min_num = 20;
  int area_search_grid_lenth = 10;
  int end_non_adjacent_id_count = 5;
  int constraint_consistent_filter_num = 5;
  int dbow_match_describe_distance_threashold = 80;
  int dbow_search_match_num = 20;
  int min_pnp_need_features_num = 10;
  int min_pnp_inliers_num = 8;
  std::string test_match_pic_write_path = "";
  double constraint_max_yaw = 50;
  double constraint_max_distance = 10;
};
//
enum TrajectorStates { Normal, frozen, Finish };
//
struct LoopDetctResult {
  KeyFrameId kf_id;
  LocalMapId local_map_id;
  transform::Rigid3d relative_pose;  // in local_pose;
  double relative_yaw;
  std::map<FeatureId, MapPointId> match_ids;
};
//
class LoopDetect {
 public:
  LoopDetect(const LoopDetectOption& option, common::ThreadPool* thread_pool,
            const std::map<int, camera_models::CameraPtr> &cameras);
  //
  //
  void Detect(const std::pair<LocalMapId, std::shared_ptr<LocalMap>>& local_map,
              const std::map<KeyFrameId, KeyFrameData>& kf_datas,
              const double min_score);
  void WhenDone(
      std::function<void(std::vector<std::unique_ptr<LoopDetctResult>>)>
          call_back);
  //
  std::unique_ptr<LoopDetctResult> DetectForOne(
      std::shared_ptr<LocalMap> local_map, KeyFrameId kf_id,
      const KeyFrameData& kf_data, const KeyFrameDataBase* data_base,
      std::unique_ptr<ConstraintConsistentFilter>* consistent_filte,
      const double min_score);
  //

  void ContinueAndDistanceCheck(std::shared_ptr<LocalMap> local_map,
                                LoopDetctResult* data);

 private:
  //

  using KeyFrameMapPointsDataWithFeatIds =
      std::pair<std::map<FeatureId, MapPointId>,
                MapById<MapPointId, mapping::MapPointData>>;

  std::vector<std::pair<FeatureId, MapPointId>> CheckValidityByProjections(
      const KeyFrameMapPointsDataWithFeatIds& target_map_points,
      const transform::Rigid3d& correct_candidate_pose,
      const KeyFrameData& candidate_kf_data,
      const std::set<MapPointId>& already_matched);
  //
  //
  std::vector<std::pair<FeatureId, MapPointId>> SearchForAdditionalMapPoints(
      std::shared_ptr<LocalMap> local_map, const KeyFrameId& candidate_id,
      const transform::Rigid3d& pose, const KeyFrameData& target_kf_data);
  //
  //
  transform::Rigid3d FourOptimize(
      const transform::Rigid3d& init_pose,
      const std::vector<transform::Rigid3d>& extric_camera_to_imu,
      const std::map<MapPointId, Eigen::Vector3d> map_points,
      const std::vector<std::pair<FeatureId, MapPointId>> matched_ids,
      const KeyFrameData& target_kf_data, const std::array<double, 2>& weight);
  //
  int RemoveOutliersRejection(
      const std::vector<transform::Rigid3d>& extric_camera_to_imu,
      const std::map<MapPointId, Eigen::Vector3d> map_points,
      const MapById<FeatureId, FeatureData> target_features,
      const transform::Rigid3d& pose, const float outlier,
      std::vector<std::pair<FeatureId, MapPointId>>& matched_ids);

  std::unique_ptr<LoopDetctResult> ComputeConstraint(
      std::shared_ptr<LocalMap> local_map, const KeyFrameId& canditate_id,
      const KeyFrameId& target_id, const KeyFrameData& target_kf_data);
  //
  bool CheckValidityByBidirectionalReprojection();

  void CalculatedSingleResultFinish(LoopDetctResult* data);
  double ComputeCovisibleMinScore(const KeyFrameId& id);
  //
  //
  std::pair<transform::Rigid3d, std::vector<std::pair<FeatureId, FeatureId>>>
  ComputePnpPose(std::shared_ptr<LocalMap> local_map,
                 const KeyFrameId& canditate_id, const KeyFrameId& target_id,
                 const KeyFrameData& target_kf_data);
  //
  std::map<LocalMapId, std::weak_ptr<common::Task>>
      data_base_insert_task_hanlde;
  //
  bool IsMapPointsValid(const std::map<FeatureId, Eigen::Vector3d>& map_points);
  void WriteCheckMatchResult(const KeyFrameData& first_data,
                             const KeyFrameData& sencond_data,
                             std::vector<std::pair<FeatureId, FeatureId>>);
  std::set<KeyFrameId> NotNeedToDetectKf(
      const std::shared_ptr<LocalMap>& local_map);
  //
  std::vector<std::pair<KeyFrameId, double>>
  FilterBestDbowResultWithCovisibility(
      std::shared_ptr<LocalMap> local_map,
      const std::unordered_map<KeyFrameId, double>& similar_with_score_ids);
  //

  std::unique_ptr<common::Task> finish_task_;
  std::unique_ptr<common::Task> when_done_task_;
  //
  std::vector<std::unique_ptr<LoopDetctResult>> loop_result_catchs_;
  //
  std::map<LocalMapId, std::unique_ptr<KeyFrameDataBase>> key_frame_data_base_;
  std::mutex mutex_;
  common::ThreadPool* thread_pool_;
  LoopDetectOption options_;
  std::map<int, camera_models::CameraPtr> cameras_;
  std::map<KeyFrameId, std::map<int, std::unique_ptr<match::AreaSearch>>>
      KeyFrameAreaDataBases_;
};
}  // namespace mapping
}  // namespace jarvis

#endif