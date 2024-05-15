#include "jarvis/object/object_detect.h"

#include <opencv2/core/eigen.hpp>
#include <vector>

// #include "ippe.h"
#include "transform/transform.h"
namespace jarvis {
namespace object {

namespace {
std::vector<cv::Point2f> Normalize(const std::vector<cv::Point2f>& points,
                                   const camera_models::CameraBase* came_base) {
  std::vector<cv::KeyPoint> temp;
  for (const auto point : points) {
    temp.emplace_back(point, 2);
  }
  auto normals = came_base->UndistortPointsNormal(temp);
  std::vector<cv::Point2f> result;
  for (const auto nor : normals) {
    result.emplace_back(nor.x(), nor.y());
  }
  return result;
}

constexpr uint8_t kGlogLevel = 1;
const std::map<std::string, int> kDict{
    {"DICT_4X4_50", 0},   {"DICT_4X4_100", 1},  {"DICT_4X4_250", 2},
    {"DICT_4X4_1000", 3}, {"DICT_5X5_50", 4},   {"DICT_5X5_100", 5},
    {"DICT_5X5_250", 6},  {"DICT_5X5_1000", 7}, {"DICT_6X6_50", 8},
    {"DICT_6X6_100", 9},  {"DICT_6X6_250", 10}, {"DICT_6X6_1000", 11},
    {"DICT_7X7_50", 12},  {"DICT_7X7_100", 13}, {"DICT_7X7_250", 14},
    {"DICT_7X7_1000", 15}};
//
// //

}  // namespace
// //
CvDetect::CvDetect(const std::string& dict) {
  CHECK(kDict.count(dict));
  dictionary_ = cv::aruco::getPredefinedDictionary(kDict.at(dict));
}

std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>>
CvDetect::Detect(const cv::Mat& image) {
  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<int> marker_ids;

  std::map<int, ObejectData> result;
  cv::aruco::detectMarkers(image, dictionary_, marker_corners,
                                      marker_ids);

  VLOG(kGlogLevel) << "Detect mark size: " << marker_ids.size();

  return {std::move(marker_ids), std::move(marker_corners)};
}

// ArucoDetect::ArucoDetect(const int& dict) {
//   aruco::DetectionMode dmode = aruco::DM_FAST;
//   detector_ = std::make_unique<aruco::MarkerDetector>();
//   detector_->setDictionary("ARUCO");
//   detector_->setDetectionMode(dmode, 0.0);
// }
// //
// std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>>
// ArucoDetect::Detect(const cv::Mat& image) {
//   std::vector<aruco::Marker> markers = detector_->detect(image);
//   LOG(INFO)<<markers.size();
//   std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>> result;
//   for (auto mark : markers) {
//     result.first.push_back(mark.id);
//     std::vector<cv::Point2f> temp;
//     for (auto& m : mark) {
//       temp.emplace_back(m.x, m.y);
//     }
//     result.second.push_back(std::move(temp));
//   }
//   return result;
// }

//
//
ObjectDetect::ObjectDetect(const ObjectDetectOption& option,
                           const camera_models::CameraBase* came_base)
    : option_(option), came_base_(came_base) {
  cv_aruce_detect_ = std::make_unique<CvDetect>(option_.opencv_aruco_dict);
  // cv_aruce_detect_ = std::make_unique<ArucoDetect>(0);
}
//

std::map<uint64_t, ObejectData> ObjectDetect::Detect(const cv::Mat& image) {
  //
  auto [marker_ids, marker_corners] = cv_aruce_detect_->Detect(image);
  if (marker_ids.empty()) return {};

  auto marker_poses = EstimatePose(marker_corners);
  std::map<uint64_t, ObejectData> result;
  for (int i = 0; i < marker_ids.size(); i++) {
    if (marker_poses[i].translation().norm() > 3) continue;
    if (common::RadToDeg(transform::GetYaw(marker_poses[i])) > 40) continue;
    result.emplace(static_cast<uint64_t>(marker_ids[i]),
                   ObejectData{marker_poses[i],
                               std::make_shared<ObejectData::Appended>(
                                   ObejectData::Appended{
                                       "aruco_" + option_.opencv_aruco_dict +
                                           std::to_string(marker_ids[i]),
                                       std::move(marker_corners[i])})});
  }
  return std::move(result);
}
//
//
void estimatePoseSingleMarkers(
    const std::vector<std::vector<cv::Point2f>>& points, const double& lenth,
    const cv::Mat& K, const cv::Mat& D, std::vector<cv::Vec3d>& rvecs,
    std::vector<cv::Vec3d>& tvecs) {
  cv::aruco::estimatePoseSingleMarkers(points, lenth, K, D, rvecs,
                                                  tvecs);
  // for (const auto& point : points) {
  //   auto solutions = aruco::solvePnP_(lenth, aruco::Marker(point), K, D);
  //   tvecs.push_back(solutions[0].first.rowRange(0, 3).colRange(3, 4));
  //   cv::Vec3d r;
  //   cv::Rodrigues(solutions[0].first.rowRange(0, 3).colRange(0, 3), r);
  //   rvecs.push_back(r);
  // }
}

std::vector<transform::Rigid3d> ObjectDetect::EstimatePose(
    const std::vector<std::vector<cv::Point2f>>& coners) {
  CHECK(came_base_);
  std::vector<std::vector<cv::Point2f>> marker_corners_normal;
  for (auto const& mark_corner : coners) {
    marker_corners_normal.push_back(Normalize(mark_corner, came_base_));
  }
  cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
  cv::Mat D = cv::Mat::zeros(1, 5, CV_32F);
  std::vector<cv::Point2f> charucoCorners;
  std::vector<int> charucoIds;
  std::vector<cv::Vec3d> rvecs;
  std::vector<cv::Vec3d> tvecs;
  //

  estimatePoseSingleMarkers(marker_corners_normal, option_.mark_lenth, K, D,
                            rvecs, tvecs);

  //
  std::vector<transform::Rigid3d> result;
  for (int i = 0; i < coners.size(); i++) {
    //
    //
    cv::Mat r;
    cv::Rodrigues(rvecs[i], r);
    Eigen::Matrix3d R_pnp;
    cv::cv2eigen(r, R_pnp);
    Eigen::Vector3d T_pnp;
    cv::cv2eigen(tvecs[i], T_pnp);

    result.push_back(transform::Rigid3d(T_pnp, Eigen::Quaterniond(R_pnp)));
    VLOG(kGlogLevel) << "Mark Pose :" << result.back();
    //
  }
  return result;
}
}  // namespace object
}  // namespace jarvis