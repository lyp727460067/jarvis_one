#ifndef _JARVIS_ESTIMATOR_SLIDE_WINDOW_H
#define _JARVIS_ESTIMATOR_SLIDE_WINDOW_H
#include <queue>
#include "transform/rigid_transform.h"
#include "jarvis/common/time.h"
#include "jarvis/estimator/featureTracker/feature_tracker.h"

namespace jarvis {
namespace estimator {
struct SlideWindowOption {
  int win_size;
};

//
struct NodeData {
  struct State {
    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;
    Eigen::Vector3d Velocity;
    Eigen::Vector3d bas;
    Eigen::Vector3d bgs;
  };
  common::Time time;
  State state;
  ImageFeatureTrackerData feature_datas;
};

class SlideWindow {
 public:
  SlideWindow(const SlideWindowOption& option);

 private:
  // std::deque<FrameData> frame_datas_;
};
}  // namespace estimator
}
#endif
