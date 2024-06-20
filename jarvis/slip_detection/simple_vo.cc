#include "simple_vo.h"

#include <map>
#include <opencv2/core/eigen.hpp>

#include "glog/logging.h"
#include "jarvis/estimator/featureTracker/feature_tracker.h"
#include "opencv2/opencv.hpp"
#include "transform/transform.h"
#include "jarvis/estimator/parameters.h"
#include "option_parse.h"
//
//
using namespace jarvis;
namespace slip_detect {

namespace {
//
Eigen::Vector3d TriangulatePoint(
    const std::vector<transform::Rigid3d>& poses,
    const std::vector<Eigen::Vector3d>& key_point_normal) {
  Eigen::MatrixXd H(poses.size() * 2, 4);
  // CHECK_EQ(poses.size(), 2) << "Function Just adoptor 2 size pose";
  // Eigen::MatrixXd H;
  for (int i = 0; i < poses.size(); i++) {
    Eigen::Matrix<double, 3, 4> pose_matrix;
    pose_matrix.block<3, 3>(0, 0) = poses[i].rotation().toRotationMatrix();
    pose_matrix.block<3, 1>(0, 3) = poses[i].translation();
    const Eigen::Vector2d point = key_point_normal[i].head<2>();
    H.row(i * 2 + 0) = point[0] * pose_matrix.row(2) - pose_matrix.row(0);
    H.row(i * 2 + 1) = point[1] * pose_matrix.row(2) - pose_matrix.row(1);
  }
  //
  Eigen::Vector4d triangulated_point;
  triangulated_point =
      H.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
  return (triangulated_point / triangulated_point(3)).head<3>();
}
//
std::vector<cv::Point2f> EigenVectorToCvVector2f(
    const std::vector<Eigen::Vector2d>& points) {
  std::vector<cv::Point2f> result;
  for (int i = 0; i < points.size(); i++) {
    // LOG(INFO) << points[i].x() << " " << points[i].y() << " " <<
    // points[i].z();
    result.push_back(
        {static_cast<float>(points[i].x()), static_cast<float>(points[i].y())});
  }
  return result;
}
std::vector<cv::Point3f> EigenVectorToCvVector(
    const std::vector<Eigen::Vector3d>& points) {
  std::vector<cv::Point3f> result;
  for (int i = 0; i < points.size(); i++) {
    // LOG(INFO) << points[i].x() << " " << points[i].y() << " " <<
    // points[i].z();
    result.push_back({static_cast<float>(points[i].x()),
                      static_cast<float>(points[i].y()),
                      static_cast<float>(points[i].z())});
  }
  return result;
}
//
std::pair<transform::Rigid3d, std::vector<bool>> ComputePoseWithPnp(
    const std::vector<Eigen::Vector3d>& map_points,
    const std::vector<Eigen::Vector2d>& normal_2d,
    const transform::Rigid3d& init_pose = transform::Rigid3d::Identity()) {
  CHECK(map_points.size());
  CHECK(normal_2d.size());
  cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
  cv::Mat r, rvec, t;
  Eigen::Vector3d rot_ve;
  cv::Mat D = cv::Mat::zeros(1, 5, CV_32F);
  rot_ve = transform::RotationQuaternionToAngleAxisVector(init_pose.rotation());
  cv::eigen2cv(rot_ve, rvec);
  cv::eigen2cv(init_pose.translation(), t);
  // cv::Mat tem_r;
  // cv::eigen2cv(init_pose.rotation().matrix(), tem_r);
  // cv::Rodrigues(tem_r, rvec);

  // LOG(INFO)<<t;
  // LOG(INFO)<<rvec;
  cv::Mat inliers;
  auto map_temp = EigenVectorToCvVector(map_points);
  auto normal_temp = EigenVectorToCvVector2f(normal_2d);
  solvePnPRansac(map_temp, normal_temp, K, D, rvec, t, true, 200, 10.0 / 460.0,
                 0.99, inliers);
  std::vector<bool> status(map_points.size(), false);
  for (int i = 0; i < inliers.rows; i++) {
    int n = inliers.at<int>(i);
    status[n] = 1;
  }
  // LOG(INFO)<<rvec;
  // LOG(INFO)<<t;
  //
  cv::Rodrigues(rvec, r);
  Eigen::Matrix3d R_pnp;
  cv::cv2eigen(r, R_pnp);
  Eigen::Vector3d T_pnp;
  cv::cv2eigen(t, T_pnp);
  return {transform::Rigid3d(T_pnp, Eigen::Quaterniond(R_pnp)), status};
}
cv::KeyPoint EigenToCv(const Eigen::Vector2d& p) {
  return cv::KeyPoint(p.x(), p.y(), 2);
}
}  // namespace

class SimpleVo::TrakcerImpl {
 public:
  explicit TrakcerImpl();
  struct Frame {
    jarvis::common::Time time;
    jarvis::transform::Rigid3d pose;
    jarvis::estimator::ImageFeatureTrackerData features;
    std::map<uint64_t, double> depths;
  };
  jarvis::TrackingData AddImageData(
      const jarvis::estimator::ImageFeatureTrackerData&);
  jarvis::TrackingData ComputePose(
      const jarvis::estimator::ImageFeatureTrackerData&);

  bool IsKeyFrame(const jarvis::estimator::ImageFeatureTrackerData&);

 private:
  std::unique_ptr<jarvis::estimator::FeatureTracker> feature_tracker_;
  const jarvis::transform::Rigid3d cam0_cam1_extrix_;
  uint8_t tracking_state_=0;
  std::unique_ptr<Frame> reference_frame_;
  SimpleVoOption options_;
};


SimpleVo::TrakcerImpl::TrakcerImpl() {}
//
TrackingData ExtractKeyFrameMapPoints(
    const estimator::ImageFeatureTrackerData& feature_result) {
  TrackingData result;
  result.data = std::make_shared<TrackingData::Data>();
  for (const auto& p : feature_result.data->features) {
    result.data->key_points.push_back(
        EigenToCv(p.second.camera_features[0].uv));
    result.data->key_points.back().class_id = p.first;
    result.data->key_points.back().octave =
        feature_result.data->tracker_features_num[p.first];
  }
  result.status = 0;
  result.data->imu_state.data->pose = transform::Rigid3d::Identity();
  result.data->image =
      std::make_shared<cv::Mat>(feature_result.data->images[0].clone());
  return result;
}
//
TrackingData SimpleVo::TrakcerImpl::AddImageData(
    const jarvis::estimator::ImageFeatureTrackerData& frame) {
  //
  if (reference_frame_) {
    return ComputePose(frame);
  }
  //
  std::map<uint64_t, double> new_depths;
  for (const auto& p : frame.data->features) {
    if (p.second.camera_features.size() != 2) continue;
    auto const points =
        TriangulatePoint({transform::Rigid3d::Identity(), cam0_cam1_extrix_},
                         {p.second.camera_features[0].normal_points,
                          p.second.camera_features[1].normal_points});
    auto const points_in_r = cam0_cam1_extrix_.inverse() * points;
    if (points.z() > 0 && points_in_r.z() > 0) {
      new_depths.emplace(p.first, points.z());
    }
  }
  reference_frame_ = std::make_unique<Frame>(
      Frame{common::Time(common::FromSeconds(frame.data->time)),
            transform::Rigid3d::Identity(), frame, std::move(new_depths)});

  return ExtractKeyFrameMapPoints(frame);
  //
};
//
bool SimpleVo::TrakcerImpl::IsKeyFrame(
    const jarvis::estimator::ImageFeatureTrackerData& frame) {
  int same_count = 0;
  for (const auto& p : frame.data->features) {
    if (reference_frame_->features.data->features.count(p.first)) {
      same_count++;
    }
  }
  return same_count < options_.min_convisible_count;
}
//
TrackingData SimpleVo::TrakcerImpl::ComputePose(
    const jarvis::estimator::ImageFeatureTrackerData& frame) {
  CHECK(reference_frame_);
  TrackingData result = ExtractKeyFrameMapPoints(frame);
  //
  std::map<uint64_t, double> new_depths;
  for (const auto& p : frame.data->features) {
    if (p.second.camera_features.size() != 2) continue;
    // if (reference_frame_->depths.count(p.first) == 0) {
    auto const points =
        TriangulatePoint({transform::Rigid3d::Identity(), cam0_cam1_extrix_},
                         {p.second.camera_features[0].normal_points,
                          p.second.camera_features[1].normal_points});
    auto const points_in_r = cam0_cam1_extrix_.inverse() * points;
    if (points.z() > 0 && points_in_r.z() > 0) {
      new_depths.emplace(p.first, points.z());
    }
    // }
    //  else {
    //   new_depths[p.first] = reference_frame_->depths[p.first];
    // }
  }
  std::vector<Eigen::Vector3d> map_points;
  std::vector<Eigen::Vector2d> normal_2d;
  for (const auto& p : frame.data->features) {
    if (reference_frame_->depths.count(p.first) == 1) {
      const Eigen::Vector3d points =
          reference_frame_->depths[p.first] *
          reference_frame_->features.data->features[p.first]
              .camera_features[0]
              .normal_points;
      map_points.push_back(points);
      normal_2d.push_back(p.second.camera_features[0].normal_points.head<2>());
    }
  }
  //
  if (map_points.size() < options_.min_track_num) {
    reference_frame_ = std::make_unique<Frame>(
        Frame{common::Time(common::FromSeconds(frame.data->time)),
              transform::Rigid3d::Identity(), frame, std::move(new_depths)});
    return result;
  }
  auto relative_pose = ComputePoseWithPnp(map_points, normal_2d);
  auto inli_coutn =
      std::count(relative_pose.second.begin(), relative_pose.second.end(), 1);
  //
  if (inli_coutn < options_.min_pnp_inlier_num) {
    return result;
  }
  //
  const auto new_pose = reference_frame_->pose * relative_pose.first.inverse();
  //
  result.data->imu_state.data->pose = new_pose;

  if (IsKeyFrame(frame)) {
    reference_frame_ = std::make_unique<Frame>(
        Frame{common::Time(common::FromSeconds(frame.data->time)), new_pose,
              frame, std::move(new_depths)});
  }
  result.status = 1;
  return result;
}
SimpleVo::SimpleVo(const SimpleVoOption& option, jarvis::CallBack call_back) {
  jarvis::estimator::FeatureTrackerOption feat_option;
  ParseYAMLOption(option.config_file, static_cast<void*>(&feat_option));
  feature_tracker_ =
      std::make_unique<jarvis::estimator::FeatureTracker>(feat_option);
}

void SimpleVo::AddImageData(const jarvis::sensor::ImageData& images) {

}
SimpleVo::~SimpleVo() {}
std::unique_ptr<jarvis::TrajectorBuilder> FactorSimipleVo(
    const std::string& file,jarvis::CallBack call_back) {
  SimpleVoOption simple_vo_option{file};
  ParseYAMLOption(file, static_cast<void*>(&simple_vo_option));
  return std::make_unique<SimpleVo>(simple_vo_option,
                                                    std::move(call_back));
}
}  // namespace slip_detect