#ifndef _JARVIS_MAPPING_DES_DATA_COMPUTE_BRIEF_DESCRIPTROR
#define _JARVIS_MAPPING_DES_DATA_COMPUTE_BRIEF_DESCRIPTROR
#include <bitset>
#include <optional>
#include <vector>

#include "jarvis/mapping/des/data/brief_descriptor_pattern.h"
#include "jarvis/mapping/mapping_data.h"
#include "opencv2/opencv.hpp"
namespace jarvis {
namespace mapping {
namespace des {
struct ComputeBriefDescriptrorOption {
  int patter_type = 1;
  bool rotated = true;
  struct GaussianBlurOption {
    Eigen::Vector2i kernel_size{9, 9};
    Eigen::Vector2f kernel_standard_deviation{2.0, 2.0};
  };
  std::optional<GaussianBlurOption>
      gaussian_blur_option= GaussianBlurOption{};
};
class ComputeBriefDescriptror {
 public:
  explicit ComputeBriefDescriptror(
      const ComputeBriefDescriptrorOption &options);
  std::vector<BrifBitset> Compute(const cv::Mat &image,
                                  const std::vector<cv::KeyPoint> key_points);

 private:
  const ComputeBriefDescriptrorOption options_;
  const std::vector<data::BriefPoint> pattern_;
};
}
}  // namespace mapping
}  // namespace jarvis
#endif