#include "slip_detection/slip_detect.h"
#include "slip_detection/simple_vo.h"
#include <algorithm>
#include "option_parse.h"
#include "jarvis/estimator/featureTracker/feature_tracker.h"
using namespace jarvis;
namespace slip_detect {

//
SlipDetect::SlipDetect(const SlipDetectOption& option)
    : options_(option), feature_tracker_(new estimator::FeatureTracker({})) {}
//
void SlipDetect::AddOdometry(const jarvis::sensor::OdometryData& odom) {
  odometry_datas_.push_back(odom);
}

void SlipDetect::AddImage(const jarvis::sensor::ImageData& image_data) {
  if (options_.type == 0) {
    std::map<int, int> track_num;
    double d_time = common::ToSeconds(image_data.time - common::FromUniversal(0));
    auto feature_frame =
        feature_tracker_->trackImage(d_time, *image_data.image[0],
                                     *image_data.image[1], &track_num, 0);
    for (auto const& point : feature_frame.data->features) {
      key_point_datas_[static_cast<uint64_t>(point.first)].emplace(
          image_data.time,
          point.second.camera_features[0].uv);
    }
  }
}

bool SlipDetect::Detect(const jarvis::common::Time&time) {
  if (options_.type == 0) {
    return ZeroVelocityDetect(time);
  } else if (options_.type == 1) {
    return SimpleDetect(time);
  } else {
    LOG(FATAL) << "Not support type.";
  }
}
template <typename T>
double SlipDetect::ComputePosesS(std::deque<T>* datas) {
  double delta_s = 0;
  for (int i = 1; i < datas->size(); i++) {
    // delta_s += (datas->at(i-1).pose.inverse() * datas->at(i).pose)
    //                .translation()
    //                .head<2>()
    //                .norm();
  }
}
bool SlipDetect::SimpleDetect(const jarvis::common::Time&time) {
  DropData(time - common::FromSeconds(options_.que_time_duration),
           &odometry_datas_);
  DropData(time - common::FromSeconds(options_.que_time_duration),
           &pose_datas_);
  if (std::abs(ComputePosesS(&odometry_datas_) - ComputePosesS(&pose_datas_)) >
      options_.pose_odom_err_s_threash_hold) {
    return true;
  }
  return false;
}
//
bool SlipDetect::ZeroVelocityDetect(const jarvis::common::Time&time) {
  DropData(time - common::FromSeconds(options_.que_time_duration),
           &odometry_datas_);
  DropData(time - common::FromSeconds(options_.que_time_duration),
           &key_point_datas_);
  if (!IsZeroVelocity()) return false;
  if (ComputePosesS(&odometry_datas_) >
      options_.zero_velocity_odom_delte_s_threash_hold) {
    return true;
  }
  return false;
}
//
//
template <typename T>
void SlipDetect::DropData(const common::Time& time, std::deque<T>* deque) {
  while (!deque->empty() && deque->front().time < time) {
    deque->pop_front();
  }
}
void SlipDetect::DropData(const common::Time& time, KeyPointData* deque) {
  for (auto it = deque->begin(); it != deque->end();) {
    if (it->second.empty()) {
      it = deque->erase(it);
    } else {
      it->second.erase(it->second.begin(), it->second.lower_bound(time));
      ++it;
    }
  }
}
//
//
bool SlipDetect::IsZeroVelocity() {
  double disparity = 0;
  std::vector<float> disparitys;
  for (auto const key_points : key_point_datas_) {
    if (key_points.second.size() >= 2) {
      auto it = key_points.second.begin();
      ++it;
      Eigen::Vector2d sum{0, 0};
      for (; it != key_points.second.end(); it++) {
        sum += it->second;
      }
      disparitys.push_back((sum / key_points.second.size()).squaredNorm());
    }
  }
  if (disparitys.size() < options_.min_disparity_num) {
#ifdef __DEBUG__
    LOG(INFO) << "zupt failed - Disparity-based Detection (disparitys.size() "
              << disparitys.size() << " < min_disparity_num "
              << options_.min_disparity_num << ")";
#endif
    return false;
  }
  disparity = std::accumulate(disparitys.begin(), disparitys.end(), 0.0) /
              disparitys.size();

  if (disparity > options_.max_disparity * options_.max_disparity) {
#ifdef __DEBUG__
    LOG(INFO) << "zupt failed - Disparity-based Detection (disparity "
              << disparity << " > max_disparity " << options_.max_disparity
              << ", " << disparitys.size() << " features)";
#endif
    return false;
  }
#ifdef __DEBUG__
  LOG(INFO) << "zupt accepted - Disparity-based Detection (disparity "
            << disparity << " <= max_disparity " << options_.max_disparity
            << ", " << disparitys.size() << " features)";
#endif
  return true;
}

SlipDetect::~SlipDetect() {}

std::unique_ptr<SlipDetect> FactorSlipDetect(const std::string& file) {
  SlipDetectOption option;
  ParseYAMLOption(file, static_cast<void*>(&option));
  return std::make_unique<SlipDetect>(option);
}

}  // namespace slip_detect