#ifndef _JARVIS_MAPPING_DES_DATA_DESCRIPTOR_PATTERN
#define _JARVIS_MAPPING_DES_DATA_DESCRIPTOR_PATTERN
#include <vector>

#include "Eigen/Core"
namespace jarvis {
namespace mapping {
namespace des {
namespace data {
struct BriefPoint {
  Eigen::Vector2i p1;
  Eigen::Vector2i p2;
};

std::vector<BriefPoint> GenerateBriiefPattern(const int type);
}  // namespace data
}  // namespace des
}  // namespace mapping

}  // namespace jarvis

#endif