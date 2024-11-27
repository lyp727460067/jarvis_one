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


//
//
struct MapPoint {
 public:
  MapPoint(const KeyFrameId &ref_frame_id, const Eigen::Vector3d &local_pos,
           const mapping::Descriptor &des, bool fix = false)
      : reference_frame_id_(ref_frame_id), local_pos_(local_pos), fix_(fix) {}
  void UpdateReferencePose(const KeyFrameId &frame, const Eigen::Vector3d &) {}
  const Eigen::Vector3d &Pos() const { return local_pos_; }
  int ObNum() { return obs_num_; }
  void SetFix() { fix_ = true; }
  const mapping::Descriptor &Descriptor() { return *descriptor_; }
  mapping::Descriptor *MutableDescriptor() { return descriptor_.get(); }
  bool Fix() { return fix_; }
  void ComputeMapPointDistinctiveDescriptors(
      const std::vector<mapping::Descriptor> &descriptors);

 private:
  bool fix_;
  //
  KeyFrameId reference_frame_id_;
  Eigen::Vector3d local_pos_;
  //
  std::shared_ptr<mapping::Descriptor> descriptor_;
  int obs_num_=0;
};
//
//应该把地图点绑定在一个局部坐标下面，后面在改把
struct MapPointData {
  std::shared_ptr<MapPoint> data;
  Eigen::Vector3d globla_pos;
};

//
//
//
struct FeatureData {
  cv::KeyPoint key_point;
  Eigen::Vector2d Point() {
    return Eigen::Vector2d{key_point.pt.x, key_point.pt.y};
  }
  Eigen::Vector3d f;
  cv::KeyPoint r_key_point_normal;
  Eigen::Vector3d r_normal;
};
//

extern const std::vector<std::vector<int>> track_sequence ;//= {{1, 2}, {3}, {4}};
struct KeyFrameData {
  struct Data {
    common::Time time;
    transform::Rigid3d pose;  // imu_pose
    std::vector<transform::Rigid3d> extric_camera_to_imu;
    MapById<FeatureId, Descriptor> descriptors;
    MapById<FeatureId, FeatureData> features;
    dbow::DbowData dbow_data;
    std::map<int, Eigen::Vector2i> image_sizes;
    MapById<FeatureId, Eigen::Vector3d> map_points;  // esitimap points
    std::vector<sensor::ImuData> imu_datas;
    std::vector<std::vector<cv::Mat>> pyramid;
    transform::Rigid3d CameraPose(int s) {
      return pose * extric_camera_to_imu[track_sequence[s][0]];
    }
  };
  std::shared_ptr<Data> data;
};

}  // namespace mapping
}  // namespace jarvis

#endif
