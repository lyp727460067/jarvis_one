#ifndef _JARVIS_MAPPING_MATH_DATA_TYPE_H
#define _JARVIS_MAPPING_MATH_DATA_TYPE_H
#include "Eigen/Eigen"
#include "jarvis/camera_models/camera_models/camera.h"
#include "jarvis/transform/transform.h"
#include "jarvis/mapping/mapping_data.h"
namespace jarvis {
namespace mapping {
namespace match {

//
using GradientVector =   Eigen::Vector2d;
using AffineTransformation2 = Eigen::Matrix2d;
using Keypoint = Eigen::Vector2d;
//
constexpr int kHalfPatchSize = 7;
struct Frame {
  Eigen::Vector2i image_size;
  transform::Rigid3d pose;    // cam pose
  transform::Rigid3d f_pose;  // imu pose
  std::shared_ptr<camera_models::Camera> cam;
  std::vector<cv::Mat> img_pyr;
  
  int num_features;
  bool IsVisible(const Eigen::Vector3d& xyz_w, Eigen::Vector2d* pt);
  bool IsKeypointVisibleWithMargin(const Eigen::Vector2d& pt,int margin);
  Eigen::Vector3d* f_top_left;
};
//
enum class FeatureType : uint8_t {
  kEdgeletSeed = 0,
  kCornerSeed = 1,
  kMapPointSeed = 2,
  kEdgeletSeedConverged = 3,
  kCornerSeedConverged = 4,
  kMapPointSeedConverged = 5,
  kEdgelet = 6,
  kCorner = 7,
  kMapPoint = 8,
  kFixedLandmark = 9,
  kOutlier = 10
};
//

struct FeatureWrapper {
  FeatureType type;    //!< Type can be corner or edgelet.
  Eigen::Vector2d px;  //!< Coordinates in pixels on pyramid level 0.
  Eigen::Vector3d f;   //!< Unit-bearing vector of the feature.
  int level;  //!< Image pyramid level where feature was extracted.
  MapPoint landmark;
  int track_id;
  double score;
  Eigen::Vector2d
      grad;  //!< Dominant gradient direction for edglets, normalized.
};

}  // namespace match
}  // namespace mapping
}  // namespace jarvis
#endif