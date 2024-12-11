
#ifndef __JARVIS_MAPPING_DATA_CULLING_DATA_H
#define __JARVIS_MAPPING_DATA_CULLING_DATA_H
#include <set>

#include "Eigen/Geometry"
#include "jarvis/common/id.h"
#include "jarvis/mapping/mapping_data.h"
#include "jarvis/mapping/match/des_matcher.h"
//
namespace jarvis {
namespace mapping {
class MapManager;
class Covisibility;
//
struct DataCullingOption {
  std::vector<Eigen::AlignedBox2i> image_bboxs;
  double viewing_angle = 0.5;
  double area_search_radius = 2;
  double project_pix_err = 8.99;
  double best_map_fuse_des_dis = 80;
  int fisrt_covisible_num = 10;
  int second_covisible_num = 20;
  int map_culling_obs = 5;
  int grid_lenth = 10;
  float redundant_observations_ration = 0.8;
};

class DataFuse {
 public:
  virtual void FuseMapPoint(
      const KeyFrameId& key_frame_id,
      const std::map<MapPointId, std::map<KeyFrameId, FeatureId>>& matches) = 0;

  virtual void CullKeyFrame(const std::set<KeyFrameId>& target) = 0;
  virtual const MapById<MapPointId, MapPointData> GetMapPoints(
      const KeyFrameId& id) = 0;
  //
  virtual const std::set<KeyFrameId> GetMapObservations(
      const MapPointId& map_point_id) = 0;
  virtual Eigen::Vector2d PorjectPoint(const Eigen::Vector3d& point, int s) = 0;
  //
  virtual const MapById<KeyFrameId, KeyFrameData>& GetAllKeyFramesData() = 0;
  virtual std::vector<std::pair<KeyFrameId, int>> GetKeyLevelConnectedKeyFrames(
      const KeyFrameId& frame_id, const std::vector<int>& levels) = 0;
  virtual ~DataFuse() {};
};

class DataCulling {
 public:
  DataCulling(const DataCullingOption& option, DataFuse* data_fuse);
  //
  void CullingMapSimilarMap(const KeyFrameId& id);

  std::map<MapPointId, std::map<KeyFrameId, FeatureId>> SearchMatchesKeyFrames(
      const MapById<KeyFrameId, KeyFrameData>& key_frame_datas,
      const MapById<MapPointId, MapPointData>& map_points,
      const KeyFrameId& id);
  //

  bool IsRedundant(const MapById<MapPointId, MapPointData>& map_points);
  //

  void KeyFrameCulling(const KeyFrameId& id);
  //

 private:
  std::vector<std::pair<MapPointId, MapPointId>> Fuse(
      const KeyFrameId& id,
      const std::vector<std::pair<MapPointId, int>>& map_points);

  const DataCullingOption options_;
  DataFuse* data_fuse_;
};
}  // namespace mapping
}  // namespace jarvis
#endif
