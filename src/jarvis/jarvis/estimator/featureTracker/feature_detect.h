#ifndef _JARVIS_ESTIMATIOR_FEATURE_DETECT_
#define _JARVIS_ESTIMATIOR_FEATURE_DETECT_
#include <thread>
#include <vector>

#include "Eigen/Core"
#include "opencv2/opencv.hpp"
namespace jarvis {
namespace estimator {
struct FeatureDetectOption {
  int fast_thresh_hold = 10;
  int min_distance = 30;
  int mask_min_dist =40;
  Eigen::Vector2i imag_size;
  Eigen::Vector2i grid_size{64, 68};
  int num_thread_ = 1;
};

struct GridOption {
  Eigen::Vector2i size;
  int resolution;
};
class Grid {
 public:
  explicit Grid(const GridOption&option);
  bool IsValid(const cv::Point2f& index);
  bool PushIndex(const cv::Point2f& index);
 private:
 const GridOption options_;
  const int width_ ;
  const int height_;
  std::vector<std::vector<cv::Point2f>> cells_;
};
class FeatureDetect {
 public:
  FeatureDetect(const FeatureDetectOption& options);
  std::vector<cv::Point2f> Detect(const cv::Mat& image,
  
const std::vector<cv::Point2f>&cur_points,
   int max_corners,
                                  const cv::Mat& derive, const cv::Mat& mask);

  void FastNeon(const cv::Mat& mage, std::vector<cv::KeyPoint>& out,
                int thresh_hodl,const cv::Mat&mask ,bool score = false);

 private:
  //
  std::vector<cv::KeyPoint> ExtractFastWithGrid(
      const cv::Mat& image, const cv::Mat& mask);
  
  std::vector<std::pair<int, double>> ComputeEigens(
      const cv::Point2i& offset, const std::vector<cv::KeyPoint>&,
      const cv::Mat& derive, const cv::Mat& mask);
  bool CheckGridValid(const std::vector<std::vector<cv::Point2f>>& grid,
                      const cv::Point2f& point, const cv::Mat& mask);
  void Convolution(const cv::Mat& image, const std::vector<cv::KeyPoint>& pts,
                   const cv::Mat& derive, std::vector<short>& x_c,
                   std::vector<short>& y_c);

  const FeatureDetectOption options_;
  //
  std::vector<std::thread> threads_;
  const cv::Mat kernal_x =
      (cv::Mat_<double>(3, 3) << -1, 0, 1, -2, 0, 2, -1, 0, 1);
  const cv::Mat kernal_y =
      (cv::Mat_<double>(3, 3) << -1, -2, -1, 0, 0, 0, 1, 2, 1);


  const int grid_width_;
  const int grid_height_;
  const int min_distance_=20; 
};


}  // namespace vins
}  // namespace internal


#endif