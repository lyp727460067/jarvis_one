#ifndef __JARVIS_MAPPING_MATCH_DIRECT_H__
#define __JARVIS_MAPPING_MATCH_DIRECT_H__
#include "jarvis/common/id.h"
#include "jarvis/estimator/featureTracker/feature_tracker.h"
#include "jarvis/mapping/mapping_data.h"
#include "jarvis/mapping/match/occupancy_grid_2d.h"
//
#include "jarvis/mapping/match/data_type.h"
#include <map>
#include <memory>
#include <set>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "jarvis/camera_models/camera_models/camera.h"
#include "jarvis/mapping/match/patch_score.h"
#include "jarvis/mapping/match/patch_warp.h"

namespace jarvis {
namespace mapping {
namespace match {
//


struct FrameWarp {
  KeyFrameData data;
  std::map<FeatureId, std::pair<MapPointId, MapPoint>> map_points;
};
//


struct DirectMatchOption {
  bool use_affine_warp = true;
  bool affine_est_offset=true;
  bool affine_est_gain=true;
  
  int align_max_iter=10;

  bool no_simd=true;
  double max_patch_diff_ratio=0.9;
  bool subpix_refinement=true;
  bool align_1d = false;
  bool scan_on_unit_sphere=false;
  int max_epi_search_steps=1;
  double min_update_squared=0.03*0.03;
};

enum class MatchResultState {
  kSuccess,
  kFailScore,
  kFailTriangulation,
  kFailVisibility,
  kFailWarp,
  kFailAlignment,
  kFailRange,
  kFailAngle,
  kFailCloseView,
  kFailLock,
  kFailTooFar
};

//
struct MatchResult {
  MatchResultState state;
  Eigen::Vector2d pt;
  Eigen::Vector3d norm;
  int level;
};

//
class DirectMatch {
 public:
  typedef svo::patch_score::ZMSSD<kHalfPatchSize> PatchScore;
  //
  DirectMatch(const DirectMatchOption&option):options_(option){
  }
  MatchResult FindMatch(const Frame& ref_frame, const Frame& cur_frame,
                        const FeatureWrapper& ref_ftr, const double& ref_depth,
                        const Keypoint& pr);
  //
  MatchResult FindEpipolarMatchDirect(const Frame& ref_frame,
                                      const Frame& cur_frame,
                                      const FeatureWrapper& ref_ftr,
                                      const double d_estimate_inv,
                                      const double d_min_inv,
                                      const double d_max_inv, double& depth);
  MatchResult FindEpipolarMatchDirect(const Frame& ref_frame,
                                      const Frame& cur_frame,
                                      const transform::Rigid3d& T_cur_ref,
                                      const FeatureWrapper& ref_ftr,
                                      const double d_estimate_inv,
                                      const double d_min_inv,
                                      const double d_max_inv, double& depth);
  static std::string ToDebugString(const MatchResultState& result);
  MatchResultState FindLocalMatch(const Frame& frame,
                                  const GradientVector& direction,
                                  const int patch_level, Keypoint& px_cur);
  bool UpdateZMSSD(const Frame& frame, const Eigen::Vector2i& pxi,
                   const int patch_level, const PatchScore& patch_score,
                   int* zmssd_best);
  bool IsPatchWithinImage(const Frame& frame, const Eigen::Vector2i& pxi,
                          const int patch_level);
  void ScanEpipolarLine(const Frame& frame, const Eigen::Vector3d& A,
                        const Eigen::Vector3d& B, const Eigen::Vector3d& C,
                        const PatchScore& patch_score, const int patch_level,
                        Keypoint* image_best, int* zmssd_best);
  void ScanEpipolarUnitPlane(const Frame& frame, const Eigen::Vector3d& A,
                             const Eigen::Vector3d& B, const Eigen::Vector3d& C,
                             const PatchScore& patch_score,
                             const int patch_level, Keypoint* image_best,
                             int* zmssd_best);
  void ScanEpipolarUnitSphere(const Frame& frame, const Eigen::Vector3d& A,
                              const Eigen::Vector3d& B,
                              const Eigen::Vector3d& C,
                              const PatchScore& patch_score,
                              const int patch_level, Keypoint* image_best,
                              int* zmssd_best);

 private:
  MatchResultState DepthFromTriangulation(
      const transform::Rigid3d& T_search_ref, const Eigen::Vector3d& f_ref,
      const Eigen::Vector3d& f_cur, double* depth);

  std::set<MapPointId> last_project_kf_id_;
  // static constexpr int kHalfPatchSize = 4;
  static constexpr int kPatchSize =kHalfPatchSize*2;

  uint8_t patch_[kPatchSize * kPatchSize] __attribute__((aligned(16)));
  uint8_t patch_with_border_[(kPatchSize + 2) * (kPatchSize + 2)]
      __attribute__((aligned(16)));
  const DirectMatchOption options_;
};
}  // namespace match
}  // namespace mapping
}  // namespace jarvis
#endif
