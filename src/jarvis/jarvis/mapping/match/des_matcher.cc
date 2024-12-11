#include "jarvis/mapping/match/des_matcher.h"

#include "common/math.h"
#include "jarvis/mapping/mapping_data.h"
//
namespace jarvis {
namespace mapping {
namespace match {
//
//

std::vector<std::pair<FeatureId, FeatureId>> DbowFindMathed(
    const MapById<FeatureId, Descriptor>& des1,
    const MapById<FeatureId, Descriptor>& des2,
    const dbow::DbowData& feat_vec1,
    const dbow::DbowData& feat_vec2,
    double describe_distance_threashold){
  auto f1it = feat_vec1.index_to_local_features.begin();
  auto f2it = feat_vec2.index_to_local_features.begin();
  auto f1end = feat_vec1.index_to_local_features.end();
  auto f2end = feat_vec2.index_to_local_features.end();
  std::set<FeatureId> matched_index;
  std::vector<std::pair<FeatureId, FeatureId>> result;
  while (f1it != f1end && f2it != f2end) {
    if (f1it->first == f2it->first) {
      
      for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++) {
        const FeatureId idx1 = f1it->second[i1];
        const auto d1 = des1.at(idx1);
        int bestDist1 = 256;
        FeatureId bestIdx2{-1, 0};
        int bestDist2 = 256;
        for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; i2++) {
          const FeatureId idx2 = f2it->second[i2];
          if (matched_index.count(idx2)) continue;
          const auto d2 = des2.at(idx2);
          int dist = HammingDis(d1, d2);
          if (dist < bestDist1) {
            bestDist2 = bestDist1;
            bestDist1 = dist;
            bestIdx2 = idx2;
          } else if (dist < bestDist2) {
            bestDist2 = dist;
          }
        }
        if (bestDist1 < describe_distance_threashold) {
          if (static_cast<float>(bestDist1) <
              0.8 * static_cast<float>(bestDist2)) {
            matched_index.insert(bestIdx2);
            result.push_back({idx1, bestIdx2});
          }
        }
      }
      f1it++;
      f2it++;
    } else if (f1it->first < f2it->first) {
      f1it = feat_vec1.index_to_local_features.lower_bound(f2it->first);
    } else {
      f2it = feat_vec2.index_to_local_features.lower_bound(f1it->first);
    }
  }
  return result;
}

bool MayMatchesByProjection(const KeyFrameData& key_frame,
                            const mapping::MapPointData& map_point,
                            double viewing_angle_threash_hold) {
  //
  return true;
}

FeatureId SearchMatchesByProjection(
    const ProjectionOption& option, const KeyFrameData& key_frame_data,
    const std::map<int, std::unique_ptr<AreaSearch>>& raius_search,
    const MapPointData& target_map_point) {
  //
  FeatureId r(-1,0);
  auto sequence_feautes = key_frame_data.data->features.trajectory_ids();
  for(auto const &i:sequence_feautes){
    const transform::Rigid3d cam_pose = key_frame_data.data->CameraPose(i);
    //
    const Eigen::Vector3d project_pose =
        cam_pose.inverse() * target_map_point.data->Pos();
    //
    //
    if (project_pose.z() < 0.1) {
      continue;
    }
    const Eigen::Vector2d project_map_point =
        option.PorjectPoint(project_pose, i);
    //
    Eigen::AlignedBox2i image_box = key_frame_data.data->image_sizes->at(i);
    //
    CHECK(key_frame_data.data->image_sizes);
    if (!image_box.contains(
            Eigen::Vector2i(std::ceil(project_map_point.x()),
                            std::ceil(project_map_point.y())))) {
      continue;
    }

    auto near_key_points_id = raius_search.at(i)->GetRadiusIndex(
        project_map_point, option.area_search_radius);
    int best_dist = 256;
    FeatureId best_idx{-1, 0};
    for (const FeatureId& index : near_key_points_id) {
      CHECK(key_frame_data.data->features.Contains(index));
      const auto& kp = key_frame_data.data->features.at(index).key_point;
      if (option.project_pix_err != 0.0) {
        const float& kpx = kp.pt.x;
        const float& kpy = kp.pt.y;
        const float ex = project_map_point.x() - kpx;
        const float ey = project_map_point.y() - kpy;
        const float e2 = ex * ex + ey * ey;
        if (e2 > option.project_pix_err) continue;
      }
      //
      CHECK(key_frame_data.data->descriptors.Contains(index))<<index;
      //这里多线程可能导致出错
      if(!target_map_point.data->HasDescriptor())continue;
      auto const dist = HammingDis(target_map_point.data->Descriptor(),
                                   key_frame_data.data->descriptors.at(index));
      if (dist < best_dist) {
        best_dist = dist;
        best_idx = index;
      }

    }
    if (best_dist < option.project_best_des_dis) {
      return best_idx;
    }
  }
  return r;
}

}  // namespace match
//
}  // namespace mapping
}  // namespace jarvis
