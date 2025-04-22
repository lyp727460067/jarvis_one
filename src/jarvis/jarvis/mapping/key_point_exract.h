#ifndef _KEY_POINT_EXTRACT_INTERFACE_H
#define _KEY_POINT_EXTRACT_INTERFACE_H
#include <vector>
//
#include "jarvis/mapping/des/compute_brief_descriptror.h"
#include "jarvis/mapping/des/des_data_type.h"
#include "opencv2/opencv.hpp"
//
namespace jarvis {
namespace mapping {

//
struct KeyPointExtractOption {
  int type = 0;
  int extend_key_points_num = 1000;
  double minimal_accepted_quality_corners = 0.01;
  double min_distance = 2;
};
//
//
class KeyPointExtract {
 public:
  //
  KeyPointExtract(const KeyPointExtractOption& option)
      : options_(option),
        extend_key_points_num_(options_.extend_key_points_num) {}
  std::vector<cv::KeyPoint> Extract(const cv::Mat& pyramid, int num,
                                    const cv::Mat& mask = cv::Mat());
  //
  ~KeyPointExtract() {}
  virtual void Distribution(){}
  static std::unique_ptr<KeyPointExtract> Create(
      const KeyPointExtractOption& option) ;

 protected:
  virtual std::vector<cv::KeyPoint> StrategyExtract(const cv::Mat& pyramid,
                                                    const cv::Mat& mask);

 private:
  KeyPointExtractOption options_;
  int  extend_key_points_num_ =0;
};
//
struct DescriptorExtractOption {
  des::ComputeBriefDescriptrorOption des_option;
};
//
class DescriptorExtract {
 public:
  //
  DescriptorExtract(const DescriptorExtractOption& option);
  std::vector<BrifBitset> Extract(const cv::Mat& img,
                                  const std::vector<cv::KeyPoint>& key_points);
  virtual void FillKeyPointAngle(const cv::Mat& image,
                                 std::vector<cv::KeyPoint>* key_point);

  virtual ~DescriptorExtract();

 private:
  DescriptorExtractOption options_;
  std::unique_ptr<des::ComputeBriefDescriptror> brief_;
  const std::vector<int> umax_;
};

}  // namespace mapping
}  // namespace jarvis
#endif