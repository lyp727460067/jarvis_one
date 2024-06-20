#ifndef JARVIS_PIC_SIMPLE_VO_H
#define JARVIS_PIC_SIMPLE_VO_H
//
#include <deque>
#include <memory>
#include "jarvis/common/time.h"
#include "Eigen/Core"
#include "ceres/ceres.h"
#include <optional>
#include <map>
#include "jarvis/transform/rigid_transform.h"
#include "jarvis/trajectory_builder.h"
//
#include "jarvis/key_frame_data.h"

namespace jarvis {
namespace estimator {
class FeatureTracker;
}
}  // namespace jarvis

namespace slip_detect {
//
struct TrackerOption {
  jarvis::transform::Rigid3d extric;
  Eigen::Vector2i image_size;
};

struct SimpleVoOption {
  std::string config_file;
  int min_track_num = 20;
  int min_pnp_inlier_num = 10;
  int min_convisible_count = 60;
  TrackerOption tracker_option;
  //
};

class SimpleVo : public jarvis::TrajectorBuilder {
 public:
  SimpleVo(const SimpleVoOption& option, jarvis::CallBack call_back);
  ~SimpleVo();
  void AddImageData(const jarvis::sensor::ImageData& images) override;

 private:
  class TrakcerImpl;
  std::unique_ptr<TrakcerImpl> tracker_;
  std::unique_ptr<jarvis::estimator::FeatureTracker> feature_tracker_;
};
std::unique_ptr<jarvis::TrajectorBuilder> FactorSimipleVo(
    const std::string& file,jarvis::CallBack call_back);
}  // namespace jarvis_pic

#endif