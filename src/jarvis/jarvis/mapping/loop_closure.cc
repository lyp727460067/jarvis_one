#ifndef JARVIS_MAPPING_LOOP_DETECT_H
#define JARVIS_MAPPING_LOOP_DETECT_H

#include "jarvis/mapping/bundle_adjustment.h"
#include "jarvis/mapping/pose_graph.h"
//
namespace jarvis {
namespace mapping {
struct LoopDetectOption {
  PoseGraphOption pose_graph_option;
  BundleAdjustmentOption full_ba_option;
};
class LoopDetect {
 public:
};
}  // namespace mapping
}  // namespace jarvis

#endif