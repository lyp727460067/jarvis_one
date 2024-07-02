#include "feature_detect.h"

#include "Eigen/Core"
#include "Eigen/Eigenvalues"
#include "algorithm"
#include "glog/logging.h"
namespace jarvis {
namespace estimator {

FeatureDetect::FeatureDetect(const FeatureDetectOption& options)
    : options_(options),
      grid_width_((options.imag_size.x() + options.grid_size.x() - 1) /
                  options.grid_size.x()),
      grid_height_((options.imag_size.y() + options.grid_size.y() - 1) /
                   options.grid_size.y()) {}

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

//
std::vector<std::pair<int, double>> FeatureDetect::ComputeEigens(
    const cv::Point2i& offset, const std::vector<cv::KeyPoint>& keypoints,
    const cv::Mat& derive) {
  std::vector<std::pair<int, double>> eigens;
  for (size_t i = 0; i < keypoints.size(); i++) {
    //
    int row = floor(keypoints[i].pt.y + offset.y);
    int col = floor(keypoints[i].pt.x + offset.x);
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
    if (eg1 >= eg2) {
      eigens.push_back(std::make_pair(i, eg1));
    } else {
      eigens.push_back(std::make_pair(i, eg2));
    }
  }
  return eigens;
}

std::vector<cv::KeyPoint> FeatureDetect::ExtractFastWithGrid(
    const cv::Mat& img, const cv::Mat& mask) {
  const int grid_size = grid_height_ * grid_width_;
  std::vector<cv::KeyPoint> point_collection;
  LOG(INFO)<< grid_size;
  point_collection.reserve(grid_size);
  //
  std::vector<std::vector<std::function<void()>>> tasks(options_.num_thread_);
  std::mutex mutex;
  for (int i = 0; i < grid_size; i++) {
    int index = i % options_.num_thread_;
    tasks[index].emplace_back([&, i]() {
      int x = i % grid_width_ * options_.grid_size.x();
      int y = i / grid_width_ * options_.grid_size.y();
      cv::Rect img_roi =
          cv::Rect(x, y, options_.grid_size.x(), options_.grid_size.y());
      std::vector<cv::KeyPoint> pts_new;
      cv::FAST(img(img_roi), pts_new, options_.fast_thresh_hold*2, false);
      if( pts_new.empty()){
        cv::FAST(img(img_roi), pts_new, options_.fast_thresh_hold/2, false);
      }
      // cv::FAST(img(img_roi), pts_new, options_.fast_thresh_hold, false);
      //
      for (size_t i = 0; i < pts_new.size(); i++) {
        cv::KeyPoint pt_cor = pts_new.at(i);
        pt_cor.pt.x += (float)x;
        pt_cor.pt.y += (float)y;
        if ((int)pt_cor.pt.x < 0 || (int)pt_cor.pt.x > img.cols ||
            (int)pt_cor.pt.y < 0 || (int)pt_cor.pt.y > img.rows) {
          continue;
        }
  
        if (mask.at<uint8_t>((int)pt_cor.pt.y, (int)pt_cor.pt.x) < 127) {
          continue;
        }
        std::lock_guard<std::mutex> lock(mutex);
        point_collection.push_back(pt_cor);
      }
      // const auto eigens = ComputeEigens(cv::Point2i(x, y), pts_new,derive);
      // std::sort(eigens.begin(), eigens.end(), cmp_by_value);
      // std::vector<cv::KeyPoint> keypoints;
      // for (size_t i = 0; i < eigens.size(); i++) {
      //   cv::KeyPoint pt_cor = pts_new.at(i);
      //   pt_cor.pt.x += (float)x;
      //   pt_cor.pt.y += (float)y;
      //   if ((int)pt_cor.pt.x < 0 || (int)pt_cor.pt.x > img.cols ||
      //       (int)pt_cor.pt.y < 0 || (int)pt_cor.pt.y > img.rows)
      //     continue;
      //   if (mask.at<uint8_t>((int)pt_cor.pt.y, (int)pt_cor.pt.x) > 127)
      //     continue;
      //   std::lock_guard<std::mutex> lock(mutex);
      //   point_collection[i].push_back(pt_cor.pt);
      // }
    });
  }
  threads_.resize(options_.num_thread_);
  for (int i = 0; i < options_.num_thread_; i++) {
    threads_[i] = std::thread([&tasks, i]() {
      for (auto f : tasks[i]) {
        f();
      }
    });
  }
  for (int i = 0; i < options_.num_thread_; i++) {
    threads_[i].join();
  }
  LOG(INFO)<<point_collection.size();
  return point_collection;
}
//
//
bool FeatureDetect::CheckGridValid(
    const std::vector<std::vector<cv::Point2f>>& grid,
    const cv::Point2f& point) {
  int x_cell = point.x / options_.grid_size.x();
  int y_cell = point.y / options_.grid_size.y();
  int x1 = x_cell - 1;
  int y1 = y_cell - 1;
  int x2 = x_cell + 1;
  int y2 = y_cell + 1;
  // boundary check
  x1 = std::max(0, x1);
  y1 = std::max(0, y1);
  x2 = std::min(grid_width_ - 1, x2);
  y2 = std::min(grid_height_ - 1, y2);
  // select feature points satisfy minDistance threshold
  for (int yy = y1; yy <= y2; yy++) {
    for (int xx = x1; xx <= x2; xx++) {
      const std::vector<cv::Point2f>& m = grid[yy * grid_width_ + xx];
      if (m.size()) {
        for (size_t j = 0; j < m.size(); j++) {
          float dx = point.x - m[j].x;
          float dy = point.y - m[j].y;
          if (dx * dx + dy * dy <
              options_.min_distance * options_.min_distance*4) {
            return false;
          }
        }
      }
    }
  }
  return true;
}
std::vector<cv::Point2f> FeatureDetect::Detect(const cv::Mat& image,
                                               int max_corners,
                                               const cv::Mat& derive,
                                               const cv::Mat& mask) {
  //
  CHECK(options_.min_distance >= 1);
  auto keypoints = ExtractFastWithGrid(image, mask);
  auto eigens = ComputeEigens(cv::Point2i(0, 0), keypoints, derive);
  //
  std::sort(
      eigens.begin(), eigens.end(),
      [](const std::pair<int, double>& lhs, const std::pair<int, double>& rhs) {
        return lhs.second > rhs.second;
      });
  std::vector<cv::KeyPoint> keypoints_;
  for (size_t i = 0; i < eigens.size(); i++) {
    keypoints_.push_back(keypoints[eigens[i].first]);
  }

  int ncorners = 0;
  int w = image.cols;
  int h = image.rows;
  std::vector<cv::Point2f> corners;
  std::vector<std::vector<cv::Point2f>> grid(grid_width_ * grid_height_);
  for (size_t i = 0; i < keypoints_.size(); i++) {
    int y = (int)(keypoints_[i].pt.y);
    int x = (int)(keypoints_[i].pt.x);
    int x_cell = x / options_.grid_size.x();
    int y_cell = y / options_.grid_size.y();
    if (!CheckGridValid(grid, keypoints_[i].pt)) continue;

    grid[y_cell * grid_width_ + x_cell].push_back(
        cv::Point2f((float)x, (float)y));
    corners.push_back(keypoints_[i].pt);
    ++ncorners;
    if (max_corners > 0 && (int)ncorners == max_corners) break;
  }
  return corners;
}

Grid::Grid(const GridOption& option)
    : options_(option),
      width_((option.size.x() + option.resolution - 1) / option.resolution),
      height_((option.size.y() + option.resolution - 1) / option.resolution) {
  cells_.resize(width_ * height_);
}

bool Grid::IsValid(const cv::Point2f& index) {
  int x = cvRound(index.x);
  int y = cvRound(index.y);
  const int x_cell = x / options_.resolution;
  const int y_cell = y / options_.resolution;
  const int x1 = x_cell - 1;
  const int y1 = y_cell - 1;
  const int x2 = x_cell + 1;
  const int y2 = y_cell + 1;
  return false;
}
bool Grid::PushIndex(const cv::Point2f& index) { return false; }
}  // namespace estimator
}  // namespace jarvis