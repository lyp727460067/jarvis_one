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
struct MapPoint {
 public:
  MapPoint(const KeyFrameId &ref_frame_id, const Eigen::Vector3d &local_pos,
            bool fix = false,bool extend=false)
      : reference_frame_id_(ref_frame_id),
        local_pos_(local_pos),
         fix_(fix),extend_( extend) {}
  MapPoint(const KeyFrameId &ref_frame_id, const Eigen::Vector3d &local_pos,
           const mapping::Descriptor des, bool fix = false,bool extend=false)
      : reference_frame_id_(ref_frame_id),
        local_pos_(local_pos),
        descriptor_(std::make_shared<mapping::Descriptor>(des)),
        fix_(fix) ,extend_( extend){}
  void UpdateReferencePose(const KeyFrameId &frame, const Eigen::Vector3d &) {}
  const Eigen::Vector3d &Pos() const { return local_pos_; }
  int ObNum() { return obs_num_; }
  void SetFix() { fix_ = true; }
  const mapping::Descriptor &Descriptor() {
    CHECK(descriptor_);
    return *descriptor_;
  }
  bool HasDescriptor() { return descriptor_ != nullptr; }
  void SetDes(const mapping::Descriptor &des) {
    if (!descriptor_) {
      descriptor_ = std::make_unique<mapping::Descriptor>(des);
    } else {
      *descriptor_ = des;
    }
  }
  //
  bool Fix() { return fix_; }
  void ComputeMapPointDistinctiveDescriptors(
      const std::vector<mapping::Descriptor> &descriptors);
  bool  Extend()const {
    return extend_;
  }

  KeyFrameId reference_frame_id_;
 private:
  bool fix_;
  bool extend_=false;
  //
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
  const Eigen::Vector2d Point() const {
    return Eigen::Vector2d{key_point.pt.x, key_point.pt.y};
  }
  Eigen::Vector3d f;
  cv::KeyPoint r_key_point_normal;
  Eigen::Vector3d r_normal;
};
//

extern const std::vector<std::vector<int>> track_sequence ;//= {{0, 1}, {2}, {3}};
struct KeyFrameData {
  struct Data {
    common::Time time;
    transform::Rigid3d pose;  // imu_pose
    std::vector<transform::Rigid3d> extric_camera_to_imu;
    //
    std::vector<std::vector<cv::Mat>> pyramid;
    std::vector<Eigen::AlignedBox2i> *image_sizes;
    // std::map<int, Eigen::Vector2i> image_sizes;
    //
    MapById<FeatureId, Descriptor> descriptors;
    MapById<FeatureId, FeatureData> features;
    dbow::DbowData dbow_data;
    MapById<FeatureId, Eigen::Vector3d> map_points;  // esitimap points
    std::vector<sensor::ImuData> imu_datas;
    const std::vector<cv::Mat> &Pyramid(int s) const {
      return pyramid.at(track_sequence[s][0]);
    }
    transform::Rigid3d CameraPose(int s) {
      return pose * extric_camera_to_imu[track_sequence[s][0]];
    }
  };
  std::shared_ptr<Data> data;
};

}  // namespace mapping
}  // namespace jarvis

#endif
