#include "jarvis/mapping/mapping_data.h"
#include "glog/logging.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "glog/logging.h"
namespace jarvis {
namespace mapping {

//
const std::vector<std::vector<int>> track_sequence{{1, 2}, {3}, {4}};

void MapPoint::ComputeMapPointDistinctiveDescriptors(
    const std::vector<mapping::Descriptor> &descriptors) {
  //
  if (descriptors.empty()) {
    LOG(WARNING) << "descritproes empty.";
    return;
  }
  const int obs_size = descriptors.size();
  // Compute distances between them
  std::vector<std::vector<int>> distances(obs_size, std::vector<int>(obs_size));
  for (size_t i = 0; i < obs_size; i++) {
    distances[i][i] = 0;
    for (size_t j = i + 1; j < obs_size; j++) {
      int distij = HammingDis(descriptors[i], descriptors[j]);
      distances[i][j] = distij;
      distances[j][i] = distij;
    }
  }
  //

  // Take the descriptor with least median distance to the rest
  int best_median = INT_MAX;
  int best_idx = 0;
  for (size_t i = 0; i < obs_size; i++) {
    auto median = distances[i].begin() + obs_size / 2;
    std::nth_element(distances[i].begin(), median, distances[i].end());
    if (*median < best_median) {
      best_median = *median;
      best_idx = i;
    }
  }
  //
  *descriptor_ = descriptors[best_idx];
}

}  // namespace mapping
}  // namespace jarvis
