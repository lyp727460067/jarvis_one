#ifndef _CONSTRAINT_CONSISTENT_FILTER_
#define _CONSTRAINT_CONSISTENT_FILTER_
#include <set>
#include <vector>

#include "jarvis/common/id.h"
//
namespace jarvis {
namespace mapping {
class ConstraintConsistentFilter {
 public:
  explicit ConstraintConsistentFilter(const int& consistent_num);
  void Update(const KeyFrameId& id,
              const std::vector<KeyFrameId>& candidate_group,
              const KeyFrameId& kf_id);
  std::map<KeyFrameId, std::map<KeyFrameId, std::set<KeyFrameId>>> Result();

 private:
  struct Group {
    int group_id;
    KeyFrameId key_frame_id;
    std::set<KeyFrameId> candidatas;
    std::map<KeyFrameId, std::set<KeyFrameId>> continue_candidatas;
  };
  std::vector<Group> consistent_groups_;
  std::vector<Group> current_consistent_groups_;
  std::set<int> consistent_group_flag_;
  const int consistent_num_;
};
}  // namespace mapping
}  // namespace jarvis
#endif