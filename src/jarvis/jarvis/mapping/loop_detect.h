#ifndef JARVIS_MAPPING_LOOP_DETECT_H
#define JARVIS_MAPPING_LOOP_DETECT_H
#include "jarvis/mapping/local_map.h"

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

class LoopDetect {
 public:
  void Detect(const std::pair<LocalMapId, std::shared_ptr<LocalMap>>& local_map,
              const std::map<KeyFrameId, KeyFrameData>& kf_datas);
              
};
}  // namespace mapping
}  // namespace jarvis

#endif