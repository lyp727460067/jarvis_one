#ifndef JARVIS_MAPPING_LOOP_DETECT_H
#define JARVIS_MAPPING_LOOP_DETECT_H
#include "jarvis/mapping/local_map.h"
#include <functional>
//
namespace jarvis {
namespace mapping {

struct LoopDetectOption {};

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
              const std::map<KeyFrameId, KeyFrameData>& kf_datas);
  void NotifyNodeAdditionFinished();
  void WhenDone(
      std::function<void(std::vector<std::shared_ptr<LoopDetctResult>>)>&&
          result);
  //
 private:
  LoopDetectOption options_;
};
}  // namespace mapping
}  // namespace jarvis

#endif