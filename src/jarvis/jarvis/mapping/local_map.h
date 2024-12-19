#ifndef JARVIS_MAPPING_LOCAL_MAP_
#define JARVIS_MAPPING_LOCAL_MAP_
#include <map>
#include <memory>
#include <set>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "jarvis/common/id.h"
#include "jarvis/mapping/covisibility.h"
#include "jarvis/mapping/key_frame_database.h"
#include "jarvis/mapping/key_point_exract.h"
#include "jarvis/mapping/mapping_data.h"
#include "jarvis/mapping/match/des_matcher.h"
namespace jarvis {
namespace mapping {
//
struct LocalMapOption {
  int max_kf_num = 100;
  KeyFrameDataBaseOption key_frame_data_option;
  KeyPointExtractOption key_points_extract_option;
  DescriptorExtractOption descriptor_option;
  match::ProjectionOption local_track_project_search_option;
  std::vector<Eigen::AlignedBox2i> image_boxs;
};

using FrontMapPointData = std::map<
    int, std::map<uint64_t,
                  std::tuple<Eigen::Vector3d, mapping::Descriptor, FeatureId>>>;

//
struct KeyFrameIdWithPose {
  KeyFrameId id;
  transform::Rigid3d local_pose;
};
//

// 维护局部的地图数据
//
struct LocalMapConstraint {};

class LocalMap {
 public:
  //
  LocalMap(const LocalMapOption &option);
  void AddKeyFrameData(const KeyFrameId &kf_id,
                       const KeyFrameData &key_frame_data);
  //
  //
  //
  void Finish();
  bool IsFinish() { return finish_; }
  //
  //
  void Opimization(std::vector<LocalMapConstraint> constrants);
  //
  std::map<MapPointId, MapPointData> GetKeyFrameMapPoints(const KeyFrameId &id);
  //
  Covisibility *GetCovisibility() { return covisibility_.get(); }
  //
  const MapById<MapPointId, MapPointData> &AllMapPoints();
  const MapById<KeyFrameId, KeyFrameData> &AllKeyFrameDatas();
  //
  void StructureMapPoints(const KeyFrameId &id,
                          FrontMapPointData &front_map_points);
  //
 public:
  //
  void FuseMapPoint(
      const KeyFrameId &kf_id,
      const std::map<MapPointId, std::map<KeyFrameId, FeatureId>> &matches);
  bool TrimMapPoint(const MapPointId &id);
  void TrimKeyFrame(const KeyFrameId &id);
  //
  std::unique_ptr<Eigen::Vector2d> ProjectMapPointToKeyFrame(
      const MapPointId &mp_id, const KeyFrameId &kf_id) const;

  std::map<int, camera_models::CameraPtr> cameras_;
  transform::Rigid3d local_pose_;
  MapById<KeyFrameId,  KeyFrameData> key_frames_datas_;
  std::unique_ptr<KeyFrameDataBase> key_frame_data_base_;
  std::unique_ptr<mapping::Covisibility> covisibility_;
  MapById<MapPointId, MapPointData> map_points_;
  std::vector<KeyFrameIdWithPose> key_frames_id_with_pose_;
  //
  bool finish_ = false;
};
//

class ActiveLocalMap {
 public:
  std::vector<std::shared_ptr<LocalMap>> localmaps_;
};
}  // namespace mapping
}  // namespace jarvis

#endif