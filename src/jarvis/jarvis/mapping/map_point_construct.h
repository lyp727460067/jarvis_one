#ifndef __JARVIS_MAPPING_MAP_POINT_CONSTRUCT_MAPMANAGER_H__
#define __JARVIS_MAPPING_MAP_POINT_CONSTRUCT_MAPMANAGER_H__

#include <map>
#include <memory>
#include <set>

#include "Eigen/Core"
#include "Eigen/Geometry"
//
#include "jarvis/camera_models/camera_models/camera.h"
//
#include "jarvis/common/id.h"
#include "jarvis/key_frame_data.h"
#include "jarvis/mapping/covisibility.h"
#include "jarvis/mapping/key_frame_database.h"
#include "jarvis/mapping/key_point_exract.h"
#include "jarvis/mapping/local_map.h"
#include "jarvis/mapping/mapping_data.h"
#include "jarvis/mapping/match/des_matcher.h"
#include "jarvis/transform/transform.h"
namespace jarvis {
namespace mapping {
//
using FrontMapPointData = std::map<
    int, std::map<uint64_t,
                  std::tuple<Eigen::Vector3d, mapping::Descriptor, FeatureId>>>;

// 维护一定规模大小的图，然后重建出当前的一部分的地图点
struct MapPointConstructOption {

  float con_struct_map_point_frame_min_distance = 0.4;
  int dbow_trasform_level = 4;
  int area_search_grid_lenth = 10;
  float construct_map_point_near_keframd_num = 10;
  int dbow_match_min_distance = 80;
  std::vector<cv::Mat> masks;
  KeyPointExtractOption key_points_extract_option;
  DescriptorExtractOption descriptor_option;
  match::ProjectionOption track_project_search_option;
  bool use_local_track_match =false;
  struct DistEpipolarLineOption {
    float check_dist_epipolar_line_cos_parallax = 0.9998;
    float first_cam_min_z_distance = 0.05;
    float first_cam_chi_squared = 5.991;
    float second_cam_min_z_distance = 0.05;
    float second_cam_chi_squared = 5.991;
  } point_check_dist_epipolar_option;

  std::vector<Eigen::AlignedBox2i> image_boxs;
  std::string test_match_pic_write_path =
      "";  //= "/home/lyp/project/vslam/jarvis/test/image/";
};
class MapPointConstruct {
  //
 public:
  MapPointConstruct(const MapPointConstructOption& option,std::map<int, camera_models::CameraPtr> camera,
                    dbow::Vocabulary* voc);
  //
  //
  KeyFrameData TrackDataToKeyFrameData(
      const TrackingData& data,
      std::shared_ptr<LocalMapMatchResult> track_data = nullptr);

  //
  bool ConstructExtend(const LocalMap& local_map,
                       KeyFrameData* data);
  //
  MapPointId AppendMapPointId(const std::pair<int, uint64_t>* tracking_id);
  void GenerateForExtendKeyPoint(KeyFrameData& data);

 private:
  //
  bool IsExist(const int s, const uint64_t& tracking_id, MapPointId* local_id);
  bool CheckDistEpipolarLine(const FeatureData& kp1, const FeatureData& kp2,
                             const transform::Rigid3d& relative_pose,
                             const std::vector<camera_models::Camera*>& camera,
                             Eigen::Vector3d* triang_map_point);

  cv::Mat GenerateMask(const cv::Size& size,
                       const std::vector<cv::KeyPoint>& exit_point);
  //
  //
  void UpdateConnectMapPointProjectMatchSearch(
      const LocalMap& local_map, KeyFrameData& kf_data);
  //
  void ConStructExtendMapPoints(const LocalMap& local_map,
                                KeyFrameData& kf_data);
  //
  MapPointConstructOption options_;
  std::map<int, camera_models::CameraPtr> cameras_;
  dbow::Vocabulary* voc_;


  std::map<int, std::map<uint64_t, MapPointId>>
      tracking_id_corresponding_to_map_point_id_;
  std::map<MapPointId, std::pair<int, uint64_t>>
      map_point_id_corresponding_to_tracking_id_;
  //
  std::set<MapPointId> map_points_local_ids_;
  int trajctory =0;
  std::mutex mutex_;

  std::unique_ptr<KeyPointExtract> key_points_extractor_;
  std::unique_ptr<DescriptorExtract> des_extractor_;
  std::set<uint64_t> map_points_local_ids;
};

}  // namespace mapping
}  // namespace jarvis
#endif
