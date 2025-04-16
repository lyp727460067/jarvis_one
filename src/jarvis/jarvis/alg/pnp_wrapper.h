#ifndef JARVIS_ALG_INTERNAL_PNP_WARAPPER_H
#define JARVIS_ALG_INTERNAL_PNP_WARAPPER_H

#include <opencv2/core/core.hpp>
#include "jarvis/common/id.h"
#include "jarvis/mapping/mapping_data.h"
#include "jarvis/transform/transform.h"
#include "jarvis/alg/pnp_solver.h"
//
namespace jarvis {
namespace alg {
//
enum SolveType { use_opencv_one_cam = 0, use_orb_slam_one_cam, use_muty_cam };
//
std::pair<transform::Rigid3d, std::set<FeatureId>> CalculatePoseUsingPnP(
    SolveType solve_type, const PnpSolverOption& option,
    const std::map<FeatureId, Eigen::Vector3d>& map_points,
    const std::map<FeatureId, mapping::FeatureData>& features,
    const transform::Rigid3d& init_pos);
//
//
}  // namespace alg
}  // namespace jarvis
#endif