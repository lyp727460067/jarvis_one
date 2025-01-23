#include "jarvis/mapping/loop_closure.h"

namespace jarvis {
namespace mapping {
void LoopClosure::Detect(
    const std::map<LocalMapId, std::shared_ptr<LocalMap>>& local_maps,
    const std::map<KeyFrameId, KeyFrameData>& kf_datas,
    std::function<void(std::vector<std::unique_ptr<LoopDetctResult>>)>
        call_back) {}
//

}  // namespace mapping
}  // namespace jarvis