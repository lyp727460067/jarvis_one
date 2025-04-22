#ifndef JARVIS_MAPPING_MAPPING_DATA_
#define JARVIS_MAPPING_MAPPING_DATA_
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "jarvis/common/id.h"
#include "jarvis/estimator/featureTracker/feature_tracker.h"
#include "jarvis/mapping/dbow/vocabulary.h"
#include "jarvis/mapping/des/des_data_type.h"
#include "jarvis/sensor/imu_data.h"
#include "jarvis/transform/rigid_transform.h"
//
namespace jarvis {
namespace mapping {
namespace log_info {

constexpr char RESET[] = "\033[0m";
constexpr char BLACK[] = "\033[30m";   /* Black */
constexpr char RED[] = "\033[31m";     /* Red */
constexpr char GREEN[] = "\033[32m";   /* Green */
constexpr char YELLOW[] = "\033[33m";  /* Yellow */
constexpr char BLUE[] = "\033[34m";    /* Blue */
constexpr char MAGENTA[] = "\033[35m"; /* Magenta */
constexpr char CYAN[] = "\033[36m";    /* Cyan */
constexpr char WHITE[] = "\033[37m";   /* White */
//
//
}  // namespace log_info
//
struct MapPoint {
  Eigen::Vector3d pos;//相对于
  mapping::Descriptor des;
  MapPointId local_id;
  KeyFrameId reference_frame_id;
  bool extend = false;
  // Eigen::Vector3d global_pos;
};

//
//应该把地图点绑定在一个局部坐标下面，后面在改把
struct MapPointData {
  std::shared_ptr<MapPoint> data;
};

//
//
//
struct FeatureData {
  cv::KeyPoint key_point;
  const Eigen::Vector2d Point() const {
    return Eigen::Vector2d{key_point.pt.x, key_point.pt.y};
  }
  Eigen::Vector3d f;
  cv::KeyPoint r_key_point_normal;
  Eigen::Vector3d r_normal{0,0,0};
};
//
enum TrajectorStates { Normal, Frozen, Finish };
extern const std::vector<std::vector<int>> track_sequence ;//= {{0, 1}, {2}, {3}};
struct KeyFrameData {
  struct Data {
    common::Time time;
    transform::Rigid3d pose;  // imu_pose
    std::vector<transform::Rigid3d> extric_camera_to_imu;
    //
    std::vector<std::vector<cv::Mat>> pyramid;
    std::vector<Eigen::AlignedBox2i> *image_sizes;

    MapById<FeatureId, Eigen::Vector3d> map_points;  // esitimap points
    std::map<FeatureId, MapPointId> map_point_ids;
    //
    MapById<FeatureId, Descriptor> descriptors;
    MapById<FeatureId, FeatureData> features;
    dbow::DbowData dbow_data;
    //
    std::vector<sensor::ImuData> imu_datas;
    const std::vector<cv::Mat> &Pyramid(int s) const {
      return pyramid.at(track_sequence[s][0]);
    }
    transform::Rigid3d CameraPose(int s)const {
      return pose * extric_camera_to_imu[track_sequence[s][0]];
    }
    //
    transform::Rigid3d CameraPose(const transform::Rigid3d &pos, int s) {
      return pos * extric_camera_to_imu[track_sequence[s][0]];
    }
    bool extend_data_compute = false;
  };
  std::shared_ptr<Data> data;
  transform::Rigid3d global_pose;
};

}  // namespace mapping
}  // namespace jarvis

#endif
