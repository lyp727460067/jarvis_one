#ifndef JARVIS_ESTIMATOR_FEATURE_DETECT_
#define JARVIS_ESTIMATOR_FEATURE_DETECT_

#include <vector>
#include "opencv2/opencv.hpp"

namespace jarvis {
namespace estimator {

struct FeatureDetectOption {};

class FeatureDetect {
 public:
  FeatureDetect(const FeatureDetectOption& option,
                const std::vector<cv::Point2f>& have_corners);
  std::vector<cv::Point2f> operator()(const cv::Mat& image);
};
}  // namespace estimator
}  // namespace jarvis
#endif
