#ifndef _JARVIS_ESTIMATOR_SLIDE_WINDOW_H
#define _JARVIS_ESTIMATOR_SLIDE_WINDOW_H
#include <queue>
#include "transform/rigid_transform.h"
#include ""
namespace jarvis {
namespace estimator {

struct FrameData {
  struct Data {
    struct State {
      transform::Rigid3d pose;
      Eigen::Vector3d velocity;
      Eigen::Vector3d linear_acceleration_bias;
      Eigen::Vector3d angular_velocity_bias;
    };
    State state;
    // std::map<int,CameraFeature>
  };
};

struct SlideWindowOption
{


};

//
class SlideWindow {
 public:
  SlideWindow(const SlideWindowOption& option);

 private:
  std::deque<FrameData> frame_datas_;
};
}  // namespace estimator
}
#endif
