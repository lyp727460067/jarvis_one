
#include "mapping/data_culling.h"

#include <set>
#include <vector>

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "jarvis/mapping/covisibility.h"
#include "jarvis/mapping/map_manger.h"
#include "random"
namespace jarvis {
//
namespace mapping {

class DataCullingTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
};

class DataFuseTest : public DataFuse {
 public:
  DataFuseTest(
      const MapById<MapPointId, MapPointData>& map_points,
      const MapById<KeyFrameId, KeyFrameData>& key_frames_data,
      //
      std::map<KeyFrameId, std::vector<std::pair<KeyFrameId, int>>> covisibity,
      //
      std::map<MapPointId, std::set<KeyFrameId>> observation,
      std::map<KeyFrameId, std::vector<MapPointId>> key_frame_map_points)
      : map_points_(map_points),
        key_frames_data_(key_frames_data),
        observations_(observation),
        covisibity_(covisibity),
        key_frame_map_points_(key_frame_map_points) {
    //
  }
  void FuseMapPoint(
      const KeyFrameId& key_frame_id,
      const std::map<MapPointId, std::map<KeyFrameId, FeatureId>>& matches) {
    CHECK(!matches.empty());
    for (auto const& match : matches) {
      for (auto key : match.second) {
        LOG(INFO) << match.first << " key: " << key.first << " index"
                  << key.second;
      }
    }
  }
  std::set<KeyFrameId> target_result;
  void CullKeyFrame(const std::set<KeyFrameId>& target) {
    for (auto const tart : target) {
      LOG(INFO) << tart;
    }
    target_result = target;
  }

  const std::set<KeyFrameId> GetCullingKeframes() { return target_result; }
  const MapById<MapPointId, MapPointData> GetMapPoints(const KeyFrameId& id) {
    MapById<MapPointId, MapPointData> result;
    CHECK(key_frame_map_points_.count(id));
    auto map_point_key = key_frame_map_points_[id];
    for (auto map_id : map_point_key) {
      result.Insert(map_id, map_points_.at(map_id));
    }
    return result;
  }
  //
  const std::set<KeyFrameId> GetMapObservations(
      const MapPointId& map_point_id) {
    return observations_[map_point_id];
  }
  //
  Eigen::Vector2d PorjectPoint(const Eigen::Vector3d& point, int s) {
    return Eigen::Vector3d(point / point.z()).head<2>() +
           Eigen::Vector2d{32, 20};
  }
  //
  const MapById<KeyFrameId, KeyFrameData>& GetAllKeyFramesData() {
    return key_frames_data_;
  }
  std::vector<std::pair<KeyFrameId, int>> GetKeyLevelConnectedKeyFrames(
      const KeyFrameId& frame_id, const std::vector<int>& levels) {
    return covisibity_[frame_id];
  }

 private:
  MapById<MapPointId, MapPointData> map_points_;
  MapById<KeyFrameId, KeyFrameData> key_frames_data_;
  std::map<KeyFrameId, std::vector<std::pair<KeyFrameId, int>>> covisibity_;
  std::map<MapPointId, std::set<KeyFrameId>> observations_;
  std::map<KeyFrameId, std::vector<MapPointId>> key_frame_map_points_;
};

//

}  // namespace mapping
}  // namespace jarvis