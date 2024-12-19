//
#include "jarvis/mapping/des/compute_brief_descriptror.h"

#include "glog/logging.h"
#include "jarvis/mapping/des/data/brief_descriptor_pattern.h"
#include "transform/rigid_transform.h"

namespace jarvis {
namespace mapping {
namespace des {
ComputeBriefDescriptror::ComputeBriefDescriptror(
    const ComputeBriefDescriptrorOption &options)
    : options_(options),
      pattern_(data::GenerateBriiefPattern(options.patter_type)) {}

//
std::vector<BrifBitset> ComputeBriefDescriptror::Compute(
    const cv::Mat &image, const std::vector<cv::KeyPoint> key_points) {
  CHECK(!image.empty());
  CHECK(!key_points.empty());
  CHECK(image.type() == CV_8UC1);
  cv::Mat img;  //;
  if (options_.gaussian_blur_option.has_value()) {
    cv::GaussianBlur(
        image, img,
        {options_.gaussian_blur_option->kernel_size.x(),
         options_.gaussian_blur_option->kernel_size.y()},
        options_.gaussian_blur_option->kernel_standard_deviation.x(),
        options_.gaussian_blur_option->kernel_standard_deviation.y());
  } else {
    img = image;
  }
  //
  std::vector<BrifBitset> result;
  result.reserve(key_points.size());
  //
  auto RotationPoint = [](const cv::Point2f &pt, const Eigen::Vector2i &point,
                          const Eigen::Vector2f &cos_sine, bool rotated) {
    if (!rotated) {
      return Eigen::Vector2i{cvRound(pt.x + point.x()),
                             cvRound(pt.y + point.y())};
    }
    const float &a = cos_sine.x();
    const float &b = cos_sine.y();
    return Eigen::Vector2i{cvRound(pt.x + a * point.x() - b * point.y()),
                           cvRound(pt.y + b * point.x() + a * point.y())};
  };
  //

  const Eigen::AlignedBox2i image_box(
      Eigen::Vector2i(0, 0), Eigen::Vector2i(img.cols - 1, img.rows - 1));
  //

  for (const auto &point : key_points) {
    const Eigen::Vector2f cos_sine{cos(point.angle), sin(point.angle)};
    BrifBitset des;
    for (size_t i = 0; i < pattern_.size(); i++) {
      //
      const auto p1 =
          RotationPoint(point.pt, pattern_[i].p1, cos_sine, options_.rotated);
      const auto p2 =
          RotationPoint(point.pt, pattern_[i].p2, cos_sine, options_.rotated);
      //
      if (image_box.contains(p1) && image_box.contains(p2)) {
        if (img.at<uint8_t>(p1.y(), p1.x()) < img.at<uint8_t>(p2.y(), p2.x())) {
          des.set(i);
          continue;
        }
      }
      des.reset(i);
    }
    result.push_back(des);
  }

  return result;
}
}  // namespace des
//
}  // namespace mapping
}  // namespace jarvis