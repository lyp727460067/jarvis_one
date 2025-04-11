#ifndef JARVIS_MAPPING_LOOP_DETECT_H
#define JARVIS_MAPPING_LOOP_DETECT_H
#include <functional>

#include "jarvis/mapping/constraint_consistent_filter.h"
#include "jarvis/mapping/local_map.h"
//
namespace jarvis {
namespace mapping {

struct LoopDetectOption {
  KeyFrameDataBaseOption key_frame_data_option;
  int end_non_adjacent_id_count = 5;
};

struct LoopDetctResult {
  struct Data {
    KeyFrameId kf_id;
    LocalMapId local_map_id;
    transform::Rigid3d relative_pose;  // in local_pose;
    std::map<FeatureId, MapPointId> match_ids;
  };
  std::vector<Data> datas;
};
//
class LoopDetect {
 public:
  LoopDetect(const LoopDetectOption& option, common::ThreadPool* thread_pool);
  //
  void Detect(const std::pair<LocalMapId, std::shared_ptr<LocalMap>>& local_map,
              const std::map<KeyFrameId, KeyFrameData>& kf_datas,
              double min_score);
  void NotifyNodeAdditionFinished();
  void WhenDone(
      std::function<void(std::vector<std::shared_ptr<LoopDetctResult>>)>&&
          result);
  //
  void DetectForOne(
      std::shared_ptr<LocalMap> local_map, KeyFrameId kf_id;
      const KeyFrameData& kf_data, const KeyFrameDataBase* data_base,
      LoopResultData* result,
      std::unique_ptr<ConstraintConsistentFilter>* consistent_filte,
      double min_score);
  //
  struct LoopResultData {
    KeyFrameId target_kf_id;
    LocalMapId target_local_id;
    std::unique_ptr<LoopResultData> pose_result;
  };
  void ContinueAndDistanceCheck(std::shared_ptr<LocalMap> local_map,
                                LoopResultData* data);

 private:
  //
  std::unique_ptr<LoopDetctResult> ComputeConstraint(
      std::shared_ptr<LocalMap> local_map, KeyFrameId kf_id;
      const KeyFrameData& kf_data, LoopResultData* result);
  //

  void CalculatedSingleResultFinish(LoopResultData* data);
  double ComputeCovisibleMinScore(const KeyFrameId& id);
  //
  //
  std::map<KeyFrameId, std::weak_ptr<common::Task>>
      data_base_insert_task_hanlde;
  //
  std::set<KeyFrameId> NotNeedToDetectKf(
      const std::shared_ptr<LocalMap>& local_map);
  //
  std::vector<std::pair<KeyFrameId, double>>
  FilterBestDbowResultWithCovisibility(
      std::shared_ptr<LocalMap> local_map,
      const std::unordered_map<KeyFrameId, double>& similar_with_score_ids);
  //
  std::set<KeyFrameId> FilterCandidata(
      const std::unordered_map<KeyFrameId, double>& candidate_kfs,
      std::unique_ptr<ConstraintConsistentFilter>* consistent_filter);
  std::unique_ptr<common::Task> finish_task_;
  std::vector<LoopResultData> loop_result_catchs_;
  std::map<LocalMapId, std::unique_ptr<KeyFrameDataBase>> key_frame_data_base_;
  common::ThreadPool* thread_pool_;
  LoopDetectOption options_;
};
}  // namespace mapping
}  // namespace jarvis

#endif