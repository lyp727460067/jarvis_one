
#include <vector>
//
#include "jarvis/mapping/des/des_data_type.h"
#include "jarvis/mapping/key_point_exract.h"
#include "opencv2/opencv.hpp"
//
namespace jarvis {
namespace mapping {
namespace {
std::vector<cv::KeyPoint> CvtoStrut(const std::vector<cv::Point2f>& points) {
  std::vector<cv::KeyPoint> result;
  for (auto const& point : points) {
    result.push_back({{point.x, point.y}, 0, 0});
  }
  return result;
}

const int PATCH_SIZE = 31;
const int HALF_PATCH_SIZE = 15;
const int EDGE_THRESHOLD = 19;
//
std::vector<int> Umax() {
  std::vector<int> umax;
  umax.resize(HALF_PATCH_SIZE + 1);
  int v, v0, vmax = std::floor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);
  int vmin = std::ceil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
  const double hp2 = HALF_PATCH_SIZE * HALF_PATCH_SIZE;
  for (v = 0; v <= vmax; ++v) {
    umax[v] = std::round(sqrt(hp2 - v * v));
  }
  // Make sure we are symmetric
  for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v) {
    while (umax[v0] == umax[v0 + 1]) ++v0;
    umax[v] = v0;
    ++v0;
  }
  return umax;
};
//

//
static float IC_Angle(const cv::Mat& image, cv::Point2f pt,
                      const std::vector<int>& u_max) {
  int m_01 = 0, m_10 = 0;

  int y = cvRound(pt.y);
  int x = cvRound(pt.x);
  const uchar* center = &image.at<uchar>(cvRound(pt.y), cvRound(pt.x));
  if (x - HALF_PATCH_SIZE < 0 || x + HALF_PATCH_SIZE >= image.cols ||
      y - HALF_PATCH_SIZE < 0 || y + HALF_PATCH_SIZE >= image.rows) {
    return 0;
  }
  // Treat the center line differently, v=0
  for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
    m_10 += u * center[u];

  // Go line by line in the circuI853lar patch
  int step = (int)image.step1();
  for (int v = 1; v <= HALF_PATCH_SIZE; ++v) {
    // Proceed over the two lines
    int v_sum = 0;
    int d = u_max[v];
    for (int u = -d; u <= d; ++u) {
      int val_plus = center[u + v * step], val_minus = center[u - v * step];
      v_sum += (val_plus - val_minus);
      m_10 += u * (val_plus + val_minus);
    }
    m_01 += v * v_sum;
  }

  return common::atan2(Eigen::Vector2f{(float)m_10, (float)m_01});
}

}  // namespace

std::vector<cv::KeyPoint> KeyPointExtract::Extract(const cv::Mat& pyramid,
                                                   int num,
                                                   const cv::Mat& mask) {
  extend_key_points_num_ = num; 
  // 多层金字塔提取和其他mask的预留处理
  return StrategyExtract(pyramid, mask);
}
//
std::unique_ptr<KeyPointExtract> KeyPointExtract::Create(
    const KeyPointExtractOption& option) {
  return std::make_unique<KeyPointExtract>(option);
}
//
std::vector<cv::KeyPoint> KeyPointExtract::StrategyExtract(
    const cv::Mat& pyramid, const cv::Mat& mask) {
  std::vector<cv::Point2f> tmp_pts;
  //
  cv::Mat mask_temp(mask.size(), CV_8UC1, cv::Scalar::all(0));
  cv::Rect roi(0, 0, pyramid.cols, pyramid.rows / 2);
  mask_temp(roi) = cv::Scalar(255);
  mask_temp &=mask;
  //
  // cv::imshow("mask",mask_temp);
  // cv::waitKey(0);
  cv::goodFeaturesToTrack(pyramid, tmp_pts, extend_key_points_num_,
                          options_.minimal_accepted_quality_corners,
                          options_.min_distance, mask_temp);
  //
  {
    std::vector<cv::Point2f> tmp_pts1;
    cv::Mat mask_temp(mask.size(), CV_8UC1, cv::Scalar::all(0));
    cv::Rect roi(0, pyramid.rows / 2, pyramid.cols, pyramid.rows / 2);
    mask_temp(roi) = cv::Scalar(255);
    mask_temp &=mask;
    // cv::imshow("mask",mask_temp);
    // cv::waitKey(0);
    cv::goodFeaturesToTrack(pyramid, tmp_pts1, extend_key_points_num_/3,
                            options_.minimal_accepted_quality_corners,
                            options_.min_distance, mask_temp);
    //

    tmp_pts.insert(tmp_pts.end(), tmp_pts1.begin(), tmp_pts1.end());
  }
  if (tmp_pts.empty()) {
    LOG(WARNING)<<"goodFeaturesToTrack empty!!!";
    std::vector<cv::KeyPoint> pts_new;
    cv::FAST(pyramid, pts_new, 30, false);
    return pts_new;
  }
  return CvtoStrut(tmp_pts);
}

//
//

//
DescriptorExtract::DescriptorExtract(const DescriptorExtractOption& option)
    :options_(option),
      brief_(std::make_unique<des::ComputeBriefDescriptror>(
          option.des_option)),umax_(Umax()) {}
//
std::vector<BrifBitset> DescriptorExtract::Extract(
    const cv::Mat& img, const std::vector<cv::KeyPoint>& key_points) {
  std::vector<cv::KeyPoint> muteble_key_points = key_points;
  FillKeyPointAngle(img, &muteble_key_points);
  return brief_->Compute(img, muteble_key_points);
}
//
void DescriptorExtract::FillKeyPointAngle(
    const cv::Mat& image, std::vector<cv::KeyPoint>* key_points) {
  for (auto& point : *key_points) {
    point.angle = (IC_Angle(image, point.pt, umax_));
    LOG(INFO)<<point.angle;
    point.octave = 0;
  }
}
DescriptorExtract::~DescriptorExtract() {}
//
}  // namespace mapping
}  // namespace jarvis