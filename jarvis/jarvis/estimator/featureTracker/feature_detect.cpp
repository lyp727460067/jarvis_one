#include "feature_detect.h"

#include "Eigen/Core"
#include "algorithm"
#include "glog/logging.h"

#include "Eigen/Eigenvalues"
namespace jarvis {
namespace estimator {

FeatureDetect::FeatureDetect(const FeatureDetectOption& options)
    : options_(options) {}

//
void FeatureDetect::Convolution(const cv::Mat& image,
                                const std::vector<cv::KeyPoint>& pts,
                                const cv::Mat& derive, std::vector<short>& x_c,
                                std::vector<short>& y_c) {
  //
  auto convolution = [image](const cv::Mat& kernal, int row, int col) {
    double pix_sum = 0;
    for (int k = 0; k < kernal.rows; k++) {
      for (int l = 0; l < kernal.cols; l++) {
        pix_sum += kernal.at<double>(k, l) *
                   double(image.at<uchar>(k + row - 1, l + col - 1));
      }
    }
    return pix_sum;
  };

  for (size_t i = 0; i < pts.size(); i++) {
    int row = floor(pts[i].pt.y);
    int col = floor(pts[i].pt.x);
    if (derive.empty()) {
      x_c.push_back(convolution(kernal_x, row, col));
      y_c.push_back(convolution(kernal_y, row, col));
    } else {
      x_c.push_back(derive.ptr<short>(row, col)[0]);
      y_c.push_back(derive.ptr<short>(row, col)[1]);
    }
  }
}
typedef std::pair<int, double> PAIR;
bool cmp_by_value(const PAIR& lhs, const PAIR& rhs) {
  return lhs.second > rhs.second;
}

std::vector<cv::Point2f> FeatureDetect::Detect(
    const cv::Mat& image, 
    const std::vector<cv::Point2f>& have_corners,int max_corners, const cv::Mat& derive,
    const cv::Mat& mask) {
  //
  std::vector<cv::Point2f> corners;
  // CHECK(!image.empty())<<"Ivalid image.";
  std::vector<cv::KeyPoint> keypoints;
  cv::FAST(image, keypoints, options_.fast_thresh_hold, true);
  std::vector<std::pair<int, double>> eigens;
  for (size_t i = 0; i < keypoints.size(); i++) {
    int row = floor(keypoints[i].pt.y);
    int col = floor(keypoints[i].pt.x);
    const auto& grad_x = derive.ptr<short>(row, col)[0];
    const auto& grad_y = derive.ptr<short>(row, col)[1];
    Eigen::Matrix2d cov;
    cov(0, 0) = grad_x * grad_x;
    cov(0, 1) = grad_x * grad_y;
    cov(1, 0) = grad_x * grad_y;
    cov(1, 1) = grad_y * grad_y;

    Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
    Eigen::Vector2cd eig_ = es.eigenvalues();
    Eigen::Vector2d eig = eig_.real();
    double eg1 = eig(0);
    double eg2 = eig(1);
    if (eg1 >= eg2)
      eigens.push_back(std::make_pair(i, eg1));
    else
      eigens.push_back(std::make_pair(i, eg2));
  }

  std::sort(eigens.begin(), eigens.end(), cmp_by_value);
  std::vector<cv::KeyPoint> keypoints_;
  for (size_t i = 0; i < eigens.size(); i++) {
    keypoints_.push_back(keypoints[eigens[i].first]);
  }

  if (options_.min_distance >= 1) {
    int ncorners = 0;
    int w = image.cols;
    int h = image.rows;

    const int cell_size = cvRound(options_.min_distance);
    const int grid_width = (w + cell_size - 1) / cell_size;
    const int grid_height = (h + cell_size - 1) / cell_size;

    std::vector<std::vector<cv::Point2f>> grid(grid_width * grid_height);

    // push the already exist feature points into the grid
    for (size_t i = 0; i < have_corners.size(); i++) {
      int y = (int)(have_corners[i].y);
      int x = (int)(have_corners[i].x);

      int x_cell = x / cell_size;
      int y_cell = y / cell_size;

      if (x_cell <= grid_width && y_cell <= grid_height)
        grid[y_cell * grid_width + x_cell].push_back(have_corners[i]);

      // corners.push_back(have_corners[i]);
      // ++ncorners;
    }

    for (size_t i = 0; i < keypoints_.size(); i++) {
      if (keypoints_[i].pt.y < 0 || keypoints_[i].pt.y > image.rows - 1)
        continue;
      if (keypoints_[i].pt.x < 0 || keypoints_[i].pt.x > image.cols - 1)
        continue;
      if (mask.at<uchar>(keypoints_[i].pt) == 0) continue;
      int y = (int)(keypoints_[i].pt.y);
      int x = (int)(keypoints_[i].pt.x);

      bool good = true;

      int x_cell = x / cell_size;
      int y_cell = y / cell_size;

      int x1 = x_cell - 1;
      int y1 = y_cell - 1;
      int x2 = x_cell + 1;
      int y2 = y_cell + 1;

      // boundary check
      x1 = std::max(0, x1);
      y1 = std::max(0, y1);
      x2 = std::min(grid_width - 1, x2);
      y2 = std::min(grid_height - 1, y2);

      // select feature points satisfy minDistance threshold
      for (int yy = y1; yy <= y2; yy++) {
        for (int xx = x1; xx <= x2; xx++) {
          std::vector<cv::Point2f>& m = grid[yy * grid_width + xx];

          if (m.size()) {
            for (size_t j = 0; j < m.size(); j++) {
              float dx = x - m[j].x;
              float dy = y - m[j].y;

              if (dx * dx + dy * dy <
                  options_.min_distance * options_.min_distance) {
                good = false;
                goto break_out;
              }
            }
          }
        }
      }

    break_out:

      if (good) {
        grid[y_cell * grid_width + x_cell].push_back(
            cv::Point2f((float)x, (float)y));

        corners.push_back(cv::Point2f((float)x, (float)y));
        ++ncorners;

        if (max_corners > 0 && (int)ncorners == max_corners) break;
      }
    }

  } else {
    return{};
  }
  return corners;
}

}  // namespace estimator
}  // namespace jarvis