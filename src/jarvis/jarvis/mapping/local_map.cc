#include "jarvis/mapping/local_map.h"
namespace jarvis {
namespace mapping {
//
LocalMap::LocalMap(const LocalMapOption &option) {}
void LocalMap::AddKeyFrameData(const KeyFrameId &kf_id,
                               const KeyFrameData &key_frame_data) {}
//
void LocalMap::FuseMapPoint(
    const KeyFrameId &kf_id,
    const std::map<MapPointId, std::map<KeyFrameId, FeatureId>> &matches) {
  CHECK(false) << "Not Implement";
}
bool LocalMap::TrimMapPoint(const MapPointId &id) {
  CHECK(false) << "Not Implement";
  return false;
}
void LocalMap::TrimKeyFrame(const KeyFrameId &id) {
  CHECK(false) << "Not Implement";
}
//
std::unique_ptr<Eigen::Vector2d> LocalMap::ProjectMapPointToKeyFrame(
    const MapPointId &mp_id, const KeyFrameId &kf_id) const {
  CHECK(false) << "Not Implement";
  return nullptr;
}

void LocalMap::Opimization(std::vector<LocalMapConstraint> constrants) {
  CHECK(false) << "Not Implement";
}
//
std::map<MapPointId, MapPointData> LocalMap::GetKeyFrameMapPoints(
    const KeyFrameId &id) {

  CHECK(false) << "Not Implement";
  return {};
}
//
//
const MapById<MapPointId, MapPointData> &LocalMap::AllMapPoints() {
    return map_points_;
}
const MapById<KeyFrameId, KeyFrameData> &LocalMap::AllKeyFrameDatas() {
  CHECK(false) << "Not Implement";
  return key_frames_datas_;
}
//
void LocalMap::StructureMapPoints(const KeyFrameId &id,
                                  FrontMapPointData &front_map_points) {
  CHECK(false) << "Not Implement";
}

void LocalMap::Finish() { finish_ = true; }
}  // namespace mapping

}  // namespace jarvis
