#ifndef JARVIS_MAPPING_LOOP_CLOSURE_H
#define JARVIS_MAPPING_LOOP_CLOSURE_H

#include "jarvis/mapping/bundle_adjustment.h"
#include "jarvis/mapping/local_map.h"
#include "jarvis/mapping/loop_detect.h"
#include "jarvis/mapping/pose_graph_op.h"
//
namespace jarvis {
namespace mapping {
struct LoopClosureOption {
  PoseGraphOption pose_graph_option;
  BundleAdjustmentOption full_ba_option;
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

class LoopClosure {
 public:
  //
  void Detect(const std::map<LocalMapId, std::shared_ptr<LocalMap>>& local_maps,
              const std::map<KeyFrameId, KeyFrameData>& kf_datas,
              std::function<void(std::vector<std::unique_ptr<LoopDetctResult>>)>
                  call_back);
  //
  //
 private:
  std::unique_ptr<LoopDetect> loop_detect_;
};
}  // namespace mapping
}  // namespace jarvis

#endif