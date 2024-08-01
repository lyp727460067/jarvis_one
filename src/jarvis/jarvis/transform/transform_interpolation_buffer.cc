#include "transform/transform_interpolation_buffer.h"

#include <algorithm>
#include <fstream>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "glog/logging.h"
#include "transform/transform.h"
namespace jarvis {
namespace transform {

TransformInterpolationBuffer::TransformInterpolationBuffer(
    const std::string& txt) {
  std::ifstream file;
  file.open(txt);
  std::string line;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    double x, y, z, qw, qx, qy, qz;
    double t;
    if (!(iss >> t >> x >> y >> z >> qx >> qy >> qz >> qw)) {
      break;
    }  // error
    const common::Time time = common::Time(common::FromSeconds(t));
    const transform::Rigid3d pose{{x, y, z}, {qw, qx, qy, qz}};
    // std::cout << "Adding time: " << time << std::endl;
    Push(time, pose);
  }
}

void TransformInterpolationBuffer::Push(const common::Time time,
                                        const transform::Rigid3d& transform) {
  if (!timestamped_transforms_.empty()) {
    CHECK_GE(time, latest_time()) << "New transform is older than latest.";
  }
  timestamped_transforms_.push_back(TimestampedTransform{time, transform});
  RemoveOldTransformsIfNeeded();
}

void TransformInterpolationBuffer::SetSizeLimit(
    const size_t buffer_size_limit) {
  buffer_size_limit_ = buffer_size_limit;
  RemoveOldTransformsIfNeeded();
}

void TransformInterpolationBuffer::Clear() { timestamped_transforms_.clear(); }

bool TransformInterpolationBuffer::Has(const common::Time time) const {
  if (timestamped_transforms_.empty()) {
    LOG(INFO) << "timestamped_transforms_ empty!!!!!!!!!!!!";
    return false;
  }
  return earliest_time() <= time && time <= latest_time();
}

transform::Rigid3d TransformInterpolationBuffer::Lookup(
    const common::Time time) const {
  CHECK(Has(time)) << "Missing transform for: " << time;
  const auto end = std::lower_bound(
      timestamped_transforms_.begin(), timestamped_transforms_.end(), time,
      [](const TimestampedTransform& timestamped_transform,
         const common::Time t) { return timestamped_transform.time < t; });
  if (end->time == time) {
    return end->transform;
  }
  const auto start = std::prev(end);
  return Interpolate(*start, *end, time).transform;
}

void TransformInterpolationBuffer::RemoveOldTransformsIfNeeded() {
  while (timestamped_transforms_.size() > buffer_size_limit_) {
    timestamped_transforms_.pop_front();
  }
}

common::Time TransformInterpolationBuffer::earliest_time() const {
  CHECK(!empty()) << "Empty buffer.";
  return timestamped_transforms_.front().time;
}

common::Time TransformInterpolationBuffer::latest_time() const {
  CHECK(!empty()) << "Empty buffer.";
  return timestamped_transforms_.back().time;
}

bool TransformInterpolationBuffer::empty() const {
  return timestamped_transforms_.empty();
}

size_t TransformInterpolationBuffer::size_limit() const {
  return buffer_size_limit_;
}

size_t TransformInterpolationBuffer::size() const {
  return timestamped_transforms_.size();
}
}  // namespace transform
}  // namespace jarvis
