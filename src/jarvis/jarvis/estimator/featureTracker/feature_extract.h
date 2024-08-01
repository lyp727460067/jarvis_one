#ifndef _JARVIS_ESTIMATIOR_FEATURE_EXTRACT
#define _JARVIS_ESTIMATIOR_FEATURE_EXTRACT
#include <opencv2/opencv.hpp>
#include "Eigen/Core"
namespace jarvis {
namespace estimator {
void GoodFeaturesToTrack_neon(const cv::Mat& image0,
                              std::vector<cv::KeyPoint>& corners,  // NOLINT
                              int maxCorners, double qualityLevel,
                              double minDistance);
}
}  // namespace jarvis

#endif