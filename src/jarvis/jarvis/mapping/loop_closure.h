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


}  // namespace mapping
}  // namespace jarvis

#endif