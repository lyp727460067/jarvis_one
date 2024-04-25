#ifndef __JARVIS_OBJECT_DETECT_H
#define __JARVIS_OBJECT_DETECT_H
#include <map>
#include <opencv2/highgui.hpp>

#include "common/time.h"
#include "jarvis/camera_models/camera_base_interface.h"
#include "opencv2/aruco.hpp"
#include "opencv2/opencv.hpp"
#include "transform/rigid_transform.h"
//
namespace jarvis {
namespace object {

//
struct ObejectData {
  struct Appended {
    std::string name;
    std::vector<cv::Point2f> corners;
  };
  transform::Rigid3d pose;
  std::shared_ptr<Appended> append;
};

struct ObjectDetectOption {
  std::string opencv_aruco_dict = "DICT_6X6_1000";
  // float mark_lenth = 0.172;
  float mark_lenth = 0.250;
};

class CvDetect {
 public:
  explicit CvDetect(const std::string& dict);
  std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>> Detect(
      const cv::Mat& image);

 private:
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
};
// class ArucoDetect {
//  public:
//   explicit ArucoDetect(const int& dict);
//   std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>> Detect(
//       const cv::Mat& image);

//  private:
//   std::unique_ptr<aruco::MarkerDetector> detector_;
// };

class ObjectDetect {
 public:
  ObjectDetect(const ObjectDetectOption& option,
               const camera_models::CameraBase* came_base);
  std::map<uint64_t, ObejectData> Detect(const cv::Mat& image);

 private:
  //
  std::vector<transform::Rigid3d> EstimatePose(
      const std::vector<std::vector<cv::Point2f>>& coners);
  //
  std::unique_ptr<CvDetect> cv_aruce_detect_;
  // std::unique_ptr<ArucoDetect> cv_aruce_detect_;
  const ObjectDetectOption option_;
  const camera_models::CameraBase* came_base_;
};
}  // namespace object

}  // namespace jarvis
#endif