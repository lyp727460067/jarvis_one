#ifndef __JARVIS_MAPPING_MAPMANAGER_H__
#define __JARVIS_MAPPING_MAPMANAGER_H__


#include <map>
#include <memory>
#include <set>

#include "Eigen/Core"
#include "Eigen/Geometry"
//
#include "jarvis/camera_models/camera_models/camera.h"
//
#include "jarvis/key_frame_data.h"
#include "jarvis/mapping/key_frame_database.h"
#include "jarvis/mapping/key_point_exract.h"
#include "jarvis/mapping/mapping_data.h"
#include "jarvis/transform/transform.h"
#include "jarvis/common/id.h"
#include "jarvis/mapping/covisibility.h"
#include "jarvis/mapping/match/des_matcher.h"
namespace jarvis {
namespace mapping {
//
class DataCulling;
//
struct SubMap {
  struct Data {
    std::vector<KeyFrameId> key_frame_id;
    std::shared_ptr<KeyFrameDataBase> key_frame_data_base_;
  };
  std::shared_ptr<Data> data;
};

struct LocalTrackData {
  MapById<KeyFrameId, KeyFrameData> key_frame_datas_;
  MapById<MapPointId, MapPointData> map_points_;
};
struct MapManagerOption {
  bool extend_point = true;
  KeyFrameDataBaseOption key_frame_data_option;
  KeyPointExtractOption key_points_extract_option;
  DescriptorExtractOption descriptor_option;
  match::ProjectionOption local_track_project_search_option;
  int dbow_trasform_level = 4;
  bool print_trim_info = true;
  std::vector<Eigen::AlignedBox2i> image_boxs;
  float dbow_match_min_distance = 100;
  int construct_map_point_near_keframd_num = 10;
  int compute_map_point_min_des_num = 5;
  struct DistEpipolarLineOption {
    float check_dist_epipolar_line_cos_parallax = 0.9998;
    float first_cam_min_z_distance = 0.05;
    float first_cam_chi_squared = 5.991;
    float second_cam_min_z_distance = 0.05;
    float second_cam_chi_squared = 5.991;
  } point_check_dist_epipolar_option;
  int area_search_grid_lenth =10;
  float con_struct_map_point_frame_min_distance =0.1;
  std::vector<cv::Mat> masks;
};
//
class MapManager {
 public:
  MapManager(const MapManagerOption &option,
             const std::map<int, camera_models::CameraPtr> &cameras,
             std::unique_ptr<dbow::Vocabulary> voc);
  //
  mapping::MapPointData *MapPointMutable(const MapPointId &id) {
    return &map_points_.at(id);
  }
  // /
  //必须先要快，只是简单的构建共视关系供localmaptrack使用
  KeyFrameId AddTrackingData(int t, const TrackingData &data);
  //
  //

  //
  //计算的慢可能比ExtractKeyFrameData慢好多

  void ExtendKeyFrameData(const KeyFrameId&id);
  //

  //
  KeyFrameId AddKeyFrame(int t, const KeyFrameData &data);

  void FuseMapPoint(
      const KeyFrameId &kf_id,
      const std::map<MapPointId, std::map<KeyFrameId, FeatureId>> &matches);
  bool TrimMapPoint(const MapPointId &id);
  void TrimKeyFrame(const KeyFrameId &id);
  //
  // 优化的时候直接修改值
  camera_models::Camera *GetCamereBase(int s) {
    return nullptr;
  }
  //
  const MapById<KeyFrameId, KeyFrameData> &AllKeyFrameDatas()const {
    return key_frames_datas_;
  }
  //
  const MapById<MapPointId, MapPointData> &AllMapPoints()const {
    return map_points_;
  }
  const MapById<KeyFrameId, KeyFrameData> &KeyAllFrameDatas() const;

  const KeyFrameData &GetKeyFrameData(const KeyFrameId &id) const {
    return key_frames_datas_.at(id);
  }
  std::unique_ptr<Eigen::Vector2d> ProjectMapPointToKeyFrame(
      const MapPointId &mp_id, const KeyFrameId &kf_id) const;

  //
  std::vector<KeyFrameId> GetConnectedKeyFrames(const KeyFrameId &frame_id,
                                                int num = -1) const;
  //
  std::pair<std::map<FeatureId, MapPointId>,
            MapById<MapPointId, mapping::MapPointData>>
  GetKeyFrameMapPointsData(const KeyFrameId &frame_id) const;
  //
  //

  //
  //
  const KeyFrameDataBase *GetKeyFrameDataBase() const {
    return key_frame_data_base_.get();
  }
  const MapById<MapPointId, mapping::MapPointData> &GetAllMapPoints() const {
    return map_points_;
  }

  transform::Rigid3d GetLocalToGlobleTransfrom() {
    return globle_to_local_transform_;
  }
  mapping::Covisibility *Covisibility()const { return covisibility_.get(); }
  
 private:

  void GenerateForExtendKeyPoint(const KeyFrameId&id );
  //
  KeyFrameData ExtractKeyFrameData(
      const TrackingData &data,
      std::map<int,
               std::map<uint64_t, std::tuple<Eigen::Vector3d,
                                             mapping::Descriptor, FeatureId>>>
          *front_map_points);
  //
  //
  void StructureMapPoints(
      const KeyFrameId &id,
      const std::map<
          int, std::map<uint64_t, std::tuple<Eigen::Vector3d,
                                             mapping::Descriptor, FeatureId>>>
          &front_map_points);

  MapPointId AddMapPoint(const int &s,
                         const std::pair<int, uint64_t> &tracking_id,
                         const mapping::MapPointData &map_point);

  //
  bool CheckDistEpipolarLine(const FeatureData &kp1, const FeatureData &kp2,
                             const transform::Rigid3d &relative_pose,
                             const std::vector<camera_models::Camera *>& camera,
                             Eigen::Vector3d *triang_map_point);
  //
  void TriagulateMapUpdata(const std::vector<uint64_t> &move_out_tracking_id);
  //

  void ComputeMapPointDistinctiveDescriptors(const MapPointId &id);
  void UpadateExtendMapPointDes(const KeyFrameId &id);
  //
  //
  void UpdateConnectMapPointProjectMatchSearch(
      const KeyFrameId &id);
  //
  void ConStructExtendMapPoints(const KeyFrameId &id);
  //

  bool IsExist(const int, const uint64_t &tracking_id);
  //

  transform::Rigid3d globle_to_local_transform_;
  //
  MapPointId GetWithTrackingId(const std::pair<int,uint64_t>& tracking_id);
  //
  //
  //
  MapManagerOption options_;
  std::unique_ptr<mapping::Covisibility> covisibility_;
  std::unique_ptr<KeyFrameDataBase> key_frame_data_base_;
  //
  std::unique_ptr<DescriptorExtract> des_extractor_;
  std::unique_ptr<KeyPointExtract> key_points_extractor_;
  //
  std::map<int, camera_models::CameraPtr> cameras_;
  MapById<KeyFrameId, KeyFrameData> key_frames_datas_;
  //

  std::set<int> move_out_tracking_id_;
  //
  std::map<int, std::map<uint64_t, MapPointId>>
      tracking_id_corresponding_to_map_point_id_;
  std::map<MapPointId, std::pair<int, uint64_t>>
      map_point_id_corresponding_to_tracking_id_;
  //
  //
  std::mutex mutex_;
  std::map<int, std::set<uint64_t>> last_key_points_class_ids_;

  MapById<MapPointId, MapPointData> map_points_;
  //
};
}  // namespace mapping
}  // namespace jarvis
#endif
