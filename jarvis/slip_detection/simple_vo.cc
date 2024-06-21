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
transform::Rigid3d ToRigid3d(const cv::Mat& pose) {
  Eigen::Matrix3d r;
  cv::cv2eigen(pose.rowRange(0, 3).colRange(0, 3), r);
  Eigen::Vector3d t;
  cv::cv2eigen(pose.rowRange(0, 3).col(3), t);
  return transform::Rigid3d(t, Eigen::Quaterniond(r));
}


template <typename T>
inline T NormalizeAngle(const T& angle_radians) {
  // Use ceres::floor because it is specialized for double and Jet types.
  T two_pi(2.0 * M_PI);
  return angle_radians -
         two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}
template <typename T>
T NormalizeAngleDeg(const T& angle_degrees) {
  if (angle_degrees > T(180.0))
    return angle_degrees - T(360.0);
  else if (angle_degrees < T(-180.0))
    return angle_degrees + T(360.0);
  else
    return angle_degrees;
};
struct ReProjectionErr {
 public:
  ReProjectionErr(const Eigen::Vector2d& nor_poit,
                  const Eigen::Vector3d& map_point, const double& factor)
      : nor_point_(nor_poit), map_point_(map_point), factor_(factor) {}

  template <typename T>
  bool operator()(const T* t1_, const T* q1_, T* residul) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t1(t1_);
    Eigen::Map<const Eigen::Quaternion<T>> q1(q1_);
    Eigen::Matrix<T, 3, 1> project_p = q1 * map_point_.template cast<T>() + t1;
    T x_normal = project_p[0] / project_p[2];
    T y_normal = project_p[1] / project_p[2];
    residul[0] = T(factor_) * (x_normal - T(nor_point_.x()));
    residul[1] = T(factor_) * (y_normal - T(nor_point_.y()));
    // LOG(INFO)<<residul[0];
    // LOG(INFO)<<residul[1];
    return true;
  }
  static ceres::CostFunction* Creat(const Eigen::Vector2d& nor_poit,
                                    const Eigen::Vector3d& map_point,
                                    double factor) {
    return new ceres::AutoDiffCostFunction<ReProjectionErr, 2, 3, 4>(
        new ReProjectionErr(nor_poit.head<2>(), map_point, factor));
  }

 private:
  const double factor_;
  const Eigen::Vector2d nor_point_;
  const Eigen::Vector3d map_point_;
};

struct RReProjectionErr {
 public:
  RReProjectionErr(const Eigen::Vector2d& nor_poit,
                   const Eigen::Vector3d& map_point,
                   const transform::Rigid3d& extric, const double& factor)
      : nor_point_(nor_poit),
        map_point_(map_point),
        extric_(extric),
        factor_(factor) {}

  template <typename T>
  bool operator()(const T* t1_, const T* q1_, T* residul) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t1(t1_);
    Eigen::Map<const Eigen::Quaternion<T>> q1(q1_);
    Eigen::Matrix<T, 3, 1> project_p =
        q1 * (extric_.rotation().cast<T>()) * map_point_.template cast<T>() +
        q1 * (extric_.translation().cast<T>()) +t1;
    T x_normal = project_p[0] / project_p[2];
    T y_normal = project_p[1] / project_p[2];
    residul[0] = T(factor_) * (x_normal - T(nor_point_.x()));
    residul[1] = T(factor_) * (y_normal - T(nor_point_.y()));
    // LOG(INFO)<<residul[0];
    // LOG(INFO)<<residul[1];
    return true;
  }
  static ceres::CostFunction* Creat(const Eigen::Vector2d& nor_poit,
                                    const Eigen::Vector3d& map_point,
                                    const transform::Rigid3d& extric,
                                    double factor) {
    return new ceres::AutoDiffCostFunction<RReProjectionErr, 2, 3, 4>(
        new RReProjectionErr(nor_poit.head<2>(), map_point, extric,factor));
  }

 private:
  const double factor_;
  const Eigen::Vector2d nor_point_;
  const transform::Rigid3d extric_;
  const Eigen::Vector3d map_point_;
};

class TranslationCostFunctor {
 public:
  static ceres::CostFunction* Create(const Eigen::Vector3d& translation,
                                     const double& factor) {
    return new ceres::AutoDiffCostFunction<TranslationCostFunctor, 3, 3>(
        new TranslationCostFunctor(translation, factor));
  }

  template <typename T>
  bool operator()(const T* const translation, T* residual) const {
    residual[0] = factor_ * (x_ - translation[0]);
    residual[1] = factor_ * (y_ - translation[1]);
    residual[2] = factor_ * (z_ - translation[2]);
    return true;
  }

 private:
  explicit TranslationCostFunctor(const Eigen::Vector3d& translation,
                                  const double& factor)
      : factor_(factor),
        x_(translation.x()),
        y_(translation.y()),
        z_(translation.z()) {}

  const double factor_;
  const double x_;
  const double y_;
  const double z_;
};

class RotationDeltaCostFunctor {
 public:
  static ceres::CostFunction *Create(const Eigen::Quaterniond &rotation,
                                     const double factor) {
    return new ceres::AutoDiffCostFunction<RotationDeltaCostFunctor, 3, 4>(
        new RotationDeltaCostFunctor(rotation, factor));
  }

  template <typename T>
  bool operator()(const T *const q1_, T *residual) const {
    Eigen::Map<const Eigen::Quaternion<T>> q1(q1_);
    Eigen::Matrix<T, 3, 1> delta =
        T(2.0) * (q1.inverse() * rotaion_.template cast<T>()).vec();
    residual[0] = factor_ * delta.x();
    residual[1] = factor_ * delta.y();
    residual[2] = factor_ * delta.z();
    return true;
  }

 private:
  explicit RotationDeltaCostFunctor(const Eigen::Quaterniond &rotation,
                                    const double &factor)
      : factor_(factor), rotaion_(rotation) {}

  const double factor_;
  const Eigen::Quaterniond rotaion_;
};


transform::Rigid3d OptimizationPose(
    const std::vector<Eigen::Vector3d>& map_points,
    const std::vector<Eigen::Vector2d>& normal_2d,
    const transform::Rigid3d& init_pose, const std::array<double, 2>& weight) {
  ceres::Problem problem;
  // ceres::LocalParameterization* quaternion_manifold = new
  // ceres::EigenQuaternionManifold;
  ceres::LocalParameterization* quaternion_local =
      new ceres::EigenQuaternionParameterization;
  Eigen::Quaterniond rotation = init_pose.rotation();
  Eigen::Vector3d traslation = init_pose.translation();

  // double rotation[4] = {init_pose.rotation().x(), init_pose.rotation().y(),
  //                       init_pose.rotation().z(), init_pose.rotation().w()};
  // double traslation[3] = {init_pose.translation().x(),
  //                         init_pose.translation().y(),
  //                         init_pose.translation().z()};
  // //

  for (int i = 0; i < normal_2d.size(); i++) {
    CHECK(!isnan(normal_2d[i].norm()) && !isnan(map_points[i].norm()))
        << normal_2d[i].transpose()
        << " map_point:" << map_points[i].transpose();
    // LOG(INFO)<<normal_2d[i].transpose()<<" "<<map_points[i].transpose();
    problem.AddResidualBlock(
        ReProjectionErr::Creat(normal_2d[i], map_points[i], weight[0]),
     new ceres::HuberLoss(0.5), traslation.data(), rotation.coeffs().data());

    problem.SetParameterization(rotation.coeffs().data(), quaternion_local);
  }
  //
  problem.AddResidualBlock(
      RotationDeltaCostFunctor::Create(init_pose.rotation(), 1), nullptr,
      rotation.coeffs().data());
  problem.AddResidualBlock(
      TranslationCostFunctor::Create(init_pose.translation(), 1), nullptr,
      traslation.data());

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 2;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // std::cout << summary.FullReport() << '\n';

  return {traslation, rotation};
  // return transform::Rigid3d(
  //     Eigen::Vector3d{traslation[0], traslation[1], traslation[2]},
  //     Eigen::Quaterniond{rotation[3], rotation[0], rotation[1],
  //     rotation[2]});
}


transform::Rigid3d StereoOptimizationPose(
    const std::vector<Eigen::Vector3d>& map_points,
    const std::vector<Eigen::Vector2d>& normal_2d,
    const std::map<int,Eigen::Vector2d>& rnormal_2d,
    const transform::Rigid3d& init_pose, 
    const transform::Rigid3d& extric, 
    const std::array<double, 2>& weight) {
  ceres::Problem problem;
  // ceres::LocalParameterization* quaternion_manifold = new
  // ceres::EigenQuaternionManifold;
  ceres::LocalParameterization* quaternion_local =
      new ceres::EigenQuaternionParameterization;
  Eigen::Quaterniond rotation = init_pose.rotation();
  Eigen::Vector3d traslation = init_pose.translation();
  //
  for (int i = 0; i < normal_2d.size(); i++) {
    CHECK(!isnan(normal_2d[i].norm()) && !isnan(map_points[i].norm()))
        << normal_2d[i].transpose()
        << " map_point:" << map_points[i].transpose();
    // LOG(INFO)<<normal_2d[i].transpose()<<" "<<map_points[i].transpose();
    problem.AddResidualBlock(
        ReProjectionErr::Creat(normal_2d[i], map_points[i], weight[0]),
        new ceres::HuberLoss(0.5), traslation.data(), rotation.coeffs().data());
    problem.SetParameterization(rotation.coeffs().data(), quaternion_local);

    // if (rnormal_2d.count(i)) {
    //   problem.AddResidualBlock(
    //       RReProjectionErr::Creat(rnormal_2d.at(i), map_points[i], extric,
    //                              weight[0]),
    //       new ceres::HuberLoss(0.5), traslation.data(),
    //       rotation.coeffs().data());
    // }
  }
  //
  // problem.AddResidualBlock(
  //     RotationDeltaCostFunctor::Create(init_pose.rotation(), 1), nullptr,
  //     rotation.coeffs().data());
  // problem.AddResidualBlock(
  //     TranslationCostFunctor::Create(init_pose.translation(), 1), nullptr,
  //     traslation.data());

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 2;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // std::cout << summary.FullReport() << '\n';

  return {traslation, rotation};
  // return transform::Rigid3d(
  //     Eigen::Vector3d{traslation[0], traslation[1], traslation[2]},
  //     Eigen::Quaterniond{rotation[3], rotation[0], rotation[1],
  //     rotation[2]});
}

constexpr uint8_t kGlogLevel = 6;
Eigen::Vector3d TriangulatePoint(
    const std::vector<transform::Rigid3d>& poses,
    const std::vector<Eigen::Vector3d>& key_point_normal) {
  Eigen::MatrixXd H(poses.size() * 2, 4);
  // CHECK_EQ(poses.size(), 2) << "Function Just adoptor 2 size pose";
  // Eigen::MatrixXd H;
  for (size_t i = 0; i < poses.size(); i++) {
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
  for (size_t i = 0; i < points.size(); i++) {
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
  for (size_t i = 0; i < points.size(); i++) {
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
  solvePnPRansac(map_temp, normal_temp, K, D, rvec, t, true, 200, 1.0 / 377.0,
                 0.99, inliers,cv::SOLVEPNP_EPNP);
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
  explicit TrakcerImpl(const SimpleVoOption&option ):options_(option),
  cam0_cam1_extrix_(option.tracker_option.extric)
  {
    LOG(INFO)<<cam0_cam1_extrix_;
  }
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
  auto imu_state_data = std::make_shared<estimator::ImuState::Data>(
      estimator::ImuState::Data{transform::Rigid3d::Identity()});
  result.data->imu_state = estimator::ImuState{imu_state_data};
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
  LOG(INFO)<<"Initlize depth...";
  std::map<uint64_t, double> new_depths;
  for (const auto& p : frame.data->features) {
    if (p.second.camera_features.size() != 2) continue;
    auto const points =
        TriangulatePoint({transform::Rigid3d::Identity(), cam0_cam1_extrix_.inverse()},
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
        TriangulatePoint({transform::Rigid3d::Identity(), cam0_cam1_extrix_.inverse()},
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
  std::map<int, Eigen::Vector2d> rnormal_2d;
  for (const auto& p : frame.data->features) {
    if (reference_frame_->depths.count(p.first) == 1) {
      const Eigen::Vector3d points =
          reference_frame_->depths[p.first] *
          reference_frame_->features.data->features[p.first]
              .camera_features[0]
              .normal_points;
      map_points.push_back(points);
      normal_2d.push_back(p.second.camera_features[0].normal_points.head<2>());
      if (p.second.camera_features.size() == 2) {
        rnormal_2d.emplace(normal_2d.size() - 1,
                           p.second.camera_features[1].normal_points.head<2>());
      }
    }
  }
  //
  if (map_points.size() < (size_t)( options_.min_track_num)) {
    LOG(WARNING) << "map_points.size:  " << map_points.size()
                 << " < options_.min_track_num " << options_.min_track_num;
    reference_frame_ = std::make_unique<Frame>(
        Frame{common::Time(common::FromSeconds(frame.data->time)),
              reference_frame_->pose, frame, std::move(new_depths)});
    result.data->imu_state.data->pose = reference_frame_->pose;
    return result;
  }
  auto relative_pose =
      ComputePoseWithPnp(map_points, normal_2d, reference_frame_->pose);

  // auto relative_pose = StereoOptimizationPose(map_points, normal_2d, rnormal_2d,
  //                                             options_.tracker_option.extric,
  //                                             reference_frame_->pose,{100,100});
  // int inli_coutn = 100;
  auto inli_coutn =
      std::count(relative_pose.second.begin(), relative_pose.second.end(), 1);
  LOG(INFO)<<inli_coutn; 
  
  const auto new_pose = reference_frame_->pose * relative_pose.first.inverse();
  // const auto new_pose = reference_frame_->pose * relative_pose.inverse();
  //
  LOG(INFO)<<relative_pose.first.inverse();
  result.data->imu_state.data->pose = new_pose;
  if (IsKeyFrame(frame) || inli_coutn < options_.min_pnp_inlier_num) {
    if(inli_coutn < options_.min_pnp_inlier_num){
        LOG(WARNING) << "Pnp Ilie " << inli_coutn
                 << " < options_.min_pnp_inlier_num "
                 << options_.min_pnp_inlier_num;
    }
    LOG(INFO)<<"New Key frame.";
    reference_frame_ = std::make_unique<Frame>(
        Frame{common::Time(common::FromSeconds(frame.data->time)), new_pose,
              frame, std::move(new_depths)});
  }
  result.status = 1;
  return result;
}
SimpleVo::SimpleVo(const SimpleVoOption& option, jarvis::CallBack call_back) {
  jarvis::estimator::FeatureTrackerOption feat_option;
  ParseYAMLOption(option.config_file, &feat_option);
  feature_tracker_ =
      std::make_unique<jarvis::estimator::FeatureTracker>(feat_option);
  tracker_=  std::make_unique<TrakcerImpl>(option);
  call_back_ = std::move(call_back);
}

void SimpleVo::AddImageData(const jarvis::sensor::ImageData& images) {
  double d_time = common::ToSeconds(images.time - common::FromUniversal(0));
  std::map<int, int> track_num;
  auto feature_frame = feature_tracker_->trackImage(
      d_time, *images.image[0], *images.image[1], &track_num, 0);
  auto data = tracker_->AddImageData(feature_frame);
  if (call_back_) {
    call_back_(data);
  }
}
//
SimpleVo::~SimpleVo() {}
std::unique_ptr<jarvis::TrajectorBuilder> FactorSimipleVo(
    const std::string& file, jarvis::CallBack call_back) {
  SimpleVoOption simple_vo_option{file};
  ParseYAMLOption(file, &simple_vo_option);
  return std::make_unique<SimpleVo>(simple_vo_option, std::move(call_back));
}
}  // namespace slip_detect