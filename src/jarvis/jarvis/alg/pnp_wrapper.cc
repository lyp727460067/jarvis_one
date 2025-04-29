#include "jarvis/alg/pnp_wrapper.h"
#include "opencv2/core/eigen.hpp"
#include "jarvis/alg/pnp_solver.h"
//
//

namespace jarvis {
namespace alg {
//

std::pair<transform::Rigid3d, std::set<FeatureId>> CalculatePoseUsingOpencvPnP(
    const PnpSolverOption& option,
    const std::map<FeatureId, Eigen::Vector3d>& map_points,
    const std::map<FeatureId, mapping::FeatureData>& features,
    const transform::Rigid3d& init_pose) {
  CHECK_GT(map_points.size(), 4);
  CHECK_GT(features.size(), 4);
  std::vector<cv::Point3f> map_points_temp;
  std::vector<cv::Point2f> key_points_temp;
  std::map<int, FeatureId> index_to_feat_ids;
  for (const auto& map_point : map_points) {
    Eigen::Vector3f eigen_3f = map_point.second.cast<float>();
    index_to_feat_ids.emplace(index_to_feat_ids.size(), map_point.first);
    map_points_temp.push_back(
        cv::Point3f(eigen_3f.x(), eigen_3f.y(), eigen_3f.z()));
    Eigen::Vector2f eigen_f =
        features.at(map_point.first).f.head<2>().cast<float>();
    key_points_temp.push_back(cv::Point2f(eigen_f.x(), eigen_f.y()));
  }

  cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
  cv::Mat r, rvec, t;
  Eigen::Vector3d rot_ve;
  cv::Mat D = cv::Mat::zeros(1, 5, CV_32F);
  rot_ve = transform::RotationQuaternionToAngleAxisVector(init_pose.rotation());
  cv::eigen2cv(rot_ve, rvec);
  cv::eigen2cv(init_pose.translation(), t);
  cv::Mat inliers;

  solvePnPRansac(map_points_temp, key_points_temp, K, D, rvec, t, false, 200,
                 1.0 / 460.0, 0.99, inliers);
  std::pair<transform::Rigid3d, std::set<FeatureId>> return_result;
  for (int i = 0; i < inliers.rows; i++) {
    return_result.second.insert(index_to_feat_ids.at(inliers.at<int>(i)));
  }
  cv::Rodrigues(rvec, r);
  Eigen::Matrix3d R_pnp;
  cv::cv2eigen(r, R_pnp);
  Eigen::Vector3d T_pnp;
  cv::cv2eigen(t, T_pnp);
  return_result.first =
      transform::Rigid3d(T_pnp, Eigen::Quaterniond(R_pnp)).inverse();
  return return_result;
}
//
std::pair<transform::Rigid3d, std::set<FeatureId>> CalculatePoseUsingOrbslamPnP(
    const PnpSolverOption& option,
    const std::map<FeatureId, Eigen::Vector3d>& map_points,
    const std::map<FeatureId, mapping::FeatureData>& features,
    const transform::Rigid3d& init_pose) {
  CHECK_GT(map_points.size(), 4);
  CHECK_GT(features.size(), 4);
  PnpSolver pnp_solver(option);
  //
  std::vector<Eigen::Vector3d> map_points_temp;
  std::vector<Eigen::Vector2d> key_points_temp;
  std::map<int, FeatureId> index_to_feat_ids;
  for (const auto& map_point : map_points) {
    index_to_feat_ids.emplace(index_to_feat_ids.size(), map_point.first);
    map_points_temp.push_back(map_point.second);
    key_points_temp.push_back(features.at(map_point.first).f.head<2>());
  }
  auto result = pnp_solver.Solve(map_points_temp, key_points_temp, init_pose);
  std::pair<transform::Rigid3d, std::vector<bool>> pnp_pose;
  std::pair<transform::Rigid3d, std::set<FeatureId>> return_result;
  if (result && !result->extend) {
    pnp_pose = std::make_pair<transform::Rigid3d, std::vector<bool>>(
        std::move(result->pose), std::move(result->inliers));
  } else {
    return {};
  }
  return_result.first = pnp_pose.first.inverse();
  for (size_t i = 0; i < pnp_pose.second.size(); i++) {
    if (pnp_pose.second[i]) {
      return_result.second.insert(index_to_feat_ids.at(i));
    }
  }
  return return_result;
}
//
//
namespace {

std::map<
    SolveType,
    std::function<std::pair<transform::Rigid3d, std::set<FeatureId>>(
        const PnpSolverOption&, const std::map<FeatureId, Eigen::Vector3d>&,
        const std::map<FeatureId, mapping::FeatureData>&,
        const transform::Rigid3d&)>>
    kFunctionMap{
        {SolveType::use_opencv_one_cam, CalculatePoseUsingOpencvPnP},
        {SolveType::use_orb_slam_one_cam, CalculatePoseUsingOrbslamPnP}};
}

//
std::pair<transform::Rigid3d, std::set<FeatureId>> CalculatePoseUsingPnP(
    SolveType solve_type, const PnpSolverOption& option,
    const std::map<FeatureId, Eigen::Vector3d>& map_points,
    const std::map<FeatureId, mapping::FeatureData>& features,
    const transform::Rigid3d& init_pose) {
  CHECK(kFunctionMap.count(solve_type));
  return kFunctionMap.at(solve_type)(option, map_points, features, init_pose);
}

}  // namespace alg
}  // namespace jarvis