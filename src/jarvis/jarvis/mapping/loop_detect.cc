#include "jarvis/mapping/loop_detect.h"

namespace jarvis {
namespace mapping {
void LoopDetect::Detect(
    const std::pair<LocalMapId, std::shared_ptr<LocalMap>>& local_map,
    const std::map<KeyFrameId, KeyFrameData>& kf_datas) {
  if (!key_frame_data_base_.count(local_map.first)) {
    key_frame_data_base_[local_map.first];
    auto insert_data_base_task = std::make_unique<common::Task>();
    sequ_match_task->SetWorkItem([]() {});
  }
}
}  // namespace mapping
}  // namespace jarvis
