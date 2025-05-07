#ifndef __JARVIS_MAPPING_COVISIBILITY_H
#define __JARVIS_MAPPING_COVISIBILITY_H
#include <mutex>
#include <map>
#include <set>
#include <unordered_map>

#include "jarvis/common/id.h"
namespace jarvis {
namespace mapping {
class Covisibility {
 public:
  Covisibility() = default;
  // bool operator=(const Covisibility&rhs){

  // }
  //
  void UpdateWithFrameData(
      const KeyFrameId& key_frame_id,
      std::map<MapPointId, FeatureId>&& frame_map_feature_data_id);
  //
  std::vector<KeyFrameId> GetConnectedKeyFrames(const KeyFrameId& frame_id,
                                                int num = -1) const;
  std::set<MapPointId> TrimKeyFrame(const KeyFrameId& id);
  void TrimMapPoint(const MapPointId& id);
  //
 std::set<KeyFrameId> GetMapObservations(
      const MapPointId& map_point_id);
  //
  const std::map<KeyFrameId, FeatureId>& GetMapPointObserv(
      const MapPointId& mp) const {
    CHECK(map_point_observe_frames_.count(mp));
    return map_point_observe_frames_.at(mp);
  }
  bool IsMapPointConnectKeyFrame(const MapPointId& mp_id,
                                             const KeyFrameId& kf_id) const;

  FeatureId GetMapPointFeatureIndex(const KeyFrameId& map_point_id,
                                    const MapPointId& mp) const;

  // MapPointId GetMapPointId(const KeyFrameId& map_point_id,
  //                          const FeatureId& ft_id) const;
  //
  void UpdateWithFuseMapPoint(const MapPointId& target, const MapPointId& sou);

  std::pair<std::vector<MapPointId>, std::vector<FeatureId>>
  GetKeyFrameMapPointId(const KeyFrameId& frame_id) const;
  //
  void ReplaceFrameIndex(const MapPointId& sou, const MapPointId& tar);

  std::vector<std::pair<KeyFrameId, int>> GetOrderConnectedKeyFrames(
      const KeyFrameId& frame_id, int num = -1) const;
  int GetConnectedWeigt(const KeyFrameId& id_i,
                                      const KeyFrameId& id_j);
  //
  void RemoveMapPoint(const MapPointId& id);
  std::set<MapPointId> RemoveKeyFrame(const KeyFrameId& id);
  //
  std::vector<KeyFrameId> GetKeyLevelConnectedKeyFrames(
      const KeyFrameId& frame_id, const std::vector<int>& levels)const;
  std::set<MapPointId> TrimLessMapPoint(const KeyFrameId& id);
 private:
  //
  std::map<KeyFrameId, std::map<KeyFrameId, int>>
      covisible_frames_;

  std::map<MapPointId, std::map<KeyFrameId, FeatureId>>
      map_point_observe_frames_;
  //
  std::map<KeyFrameId,std::map<MapPointId, FeatureId>> key_frame_feature_data_;
//   /
  //
};
}  // namespace mapping
}  // namespace jarvis
#endif
