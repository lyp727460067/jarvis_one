#ifndef __JARVIS_MAPPING_DES_MATCHER__
#define __JARVIS_MAPPING_DES_MATCHER__
#include <vector>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "jarvis/mapping/dbow/vocabulary.h"
#include "jarvis/mapping/mapping_data.h"
#include "opencv2/opencv.hpp"
//
#include "jarvis/mapping/match/area_search.h"
namespace jarvis {
namespace mapping {

namespace match {
//

//
struct ProjectionOption {
  double viewing_angle_threash_hold = 0.5;
  double area_search_radius = 8 ;
  double project_pix_err = 0.0;
  double project_best_des_dis = 80;
  //
  std::function<Eigen::Vector2d(const Eigen::Vector3d& point, int s)>
      PorjectPoint=nullptr;
};

//
FeatureId SearchMatchesByProjection(
    const ProjectionOption& option, const KeyFrameData& key_frame_data,
    const std::map<int, std::unique_ptr<AreaSearch>> &raius_search,
    const MapPointData& target_map_point);
//

//
std::vector<std::pair<FeatureId, FeatureId>> DbowFindMathed(
    const MapById<FeatureId, Descriptor>& des1,
    const MapById<FeatureId, Descriptor>& des2, const dbow::DbowData& feat_vec1,
    const dbow::DbowData& feat_vec2, double describe_distance_threashold);
//
//
}  // namespace match
}  // namespace mapping
}  // namespace jarvis
#endif