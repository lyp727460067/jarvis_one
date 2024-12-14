#ifndef _JARVIS_LAOCAL_MAP_TRACK_MAP_H
#define _JARVIS_LAOCAL_MAP_TRACK_MAP_H
#include "jarvis/mapping/map_manger.h"
#include "jarvis/mapping/match/direct_match.h"
#include "jarvis/transform/rigid_transform.h"
#include "jarvis/transform/transform.h"
//
#include "jarvis/mapping/match/occupancy_grid_2d.h"
namespace jarvis {
namespace mapping {
using FrontMapPointData = std::map<
    int, std::map<uint64_t,
                  std::tuple<Eigen::Vector3d, mapping::Descriptor, FeatureId>>>;
struct LocalMapTrackMapOption {
  int kf_num = 100;
};

class LocalMapTrackMap {
 public:
  LocalMapTrackMap(const LocalMapTrackMapOption& option)
      : options_(option), covisibility_(new Covisibility()) {}
  void AddKeyFrameData(const KeyFrameData& key_frame_data,
                       const FrontMapPointData& map_points_data);

  std::map<MapPointId, MapPointData> GetKeyFrameMapPoints(const KeyFrameId& id);
  Covisibility* GetCovisibility() { return covisibility_.get(); }
  const MapById<MapPointId, MapPointData>& AllMapPoints() {
    return map_points_;
  }
  const MapById<KeyFrameId, KeyFrameData>& AllKeyFrameDatas() {
    return key_frames_datas_;
  }

 private:
  void StructureMapPoints(const KeyFrameId& id,
                          const FrontMapPointData& front_map_points);
  //
  int trajector_default = 0;
  MapById<KeyFrameId, KeyFrameData> key_frames_datas_;
  std::unique_ptr<mapping::Covisibility> covisibility_;
  MapById<MapPointId, MapPointData> map_points_;
  LocalMapTrackMapOption options_;
};

}  // namespace mapping
}  // namespace jarvis
#endif