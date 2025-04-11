#include "jarvis/mapping/constraint_consistent_filter.h"

namespace jarvis {
namespace mapping {

//
ConstraintConsistentFilter::ConstraintConsistentFilter(
    const int& consistent_num)
    : consistent_num_(consistent_num) {
  // CHECK(consistent_num_ != 0);
}
//
std::set<KeyFrameId> ConstraintConsistentFilter::Result() {
  std::set<KeyFrameId> result;
  for (const auto& group : current_consistent_groups_) {
    if (group.group_id >= consistent_num_) {
      result.insert(group.key_frame_id);
    }
  }
  consistent_group_flag_.clear();
  consistent_groups_.swap(current_consistent_groups_);
  current_consistent_groups_.clear();
  return result;
}
//
//
void ConstraintConsistentFilter::Update(
    const KeyFrameId& id, const std::vector<KeyFrameId>& candidate_group) {
  std::set<KeyFrameId> set_candidate_group(candidate_group.begin(),
                                           candidate_group.end());
  bool consistent_for_some_group = false;
  for (int i = 0; i < consistent_groups_.size(); i++) {
    const auto& consistent_group = consistent_groups_[i];
    std::vector<KeyFrameId> intersection;
    std::set_intersection(
        consistent_group.candidatas.begin(), consistent_group.candidatas.end(),
        set_candidate_group.begin(), set_candidate_group.end(),
        std::back_insert_iterator(intersection));
    if (intersection.empty()) {
      continue;
    }
    int current_consistency = consistent_group.group_id + 1;
    if (consistent_group_flag_.count(i) == 0 ||
        current_consistency >= consistent_num_) {
      current_consistent_groups_.emplace_back(
          Group{current_consistency, id, std::move(set_candidate_group)});
      consistent_group_flag_.insert(i);
    }
    consistent_for_some_group = true;
  }
  if (!consistent_for_some_group) {
    current_consistent_groups_.emplace_back(
        Group{0, id, std::move(set_candidate_group)});
  }
}
}
}  // namespace jarvis