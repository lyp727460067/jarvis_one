#ifndef _JARVIS_ESTIMATIOR_FEATURE_DETECT_
#define _JARVIS_ESTIMATIOR_FEATURE_DETECT_

#include <vector>
#include <opencv2/opencv.hpp>
namespace jarvis {
namespace estimator {
struct FeatureDetectOption {
  int fast_thresh_hold = 10;
  int min_distance = 30;
};
class FeatureDetect {
 public:
  FeatureDetect(const FeatureDetectOption& options);
  std::vector<cv::Point2f> Detect(const cv::Mat& image,
                                  const std::vector<cv::Point2f>& have_corners,
                                  int max_corners, const cv::Mat& derive,
                                  const cv::Mat& mask);

 private:
  void Convolution(const cv::Mat& image, const std::vector<cv::KeyPoint>& pts,
                   const cv::Mat& derive, std::vector<short>& x_c,
                   std::vector<short>& y_c);

  const FeatureDetectOption options_;
  //
  const cv::Mat kernal_x =
      (cv::Mat_<double>(3, 3) << -1, 0, 1, -2, 0, 2, -1, 0, 1);
  const cv::Mat kernal_y =
      (cv::Mat_<double>(3, 3) << -1, -2, -1, 0, 0, 0, 1, 2, 1);
};

}  // namespace vins
}  // namespace internal


#endif