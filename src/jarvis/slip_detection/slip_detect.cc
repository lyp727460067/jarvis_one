#include "slip_detection/slip_detect.h"

#include <algorithm>

#include "jarvis/estimator/featureTracker/feature_tracker.h"
#include "option_parse.h"
#include "slip_detection/simple_vo.h"
#include "jarvis/estimator/parameters.h"
namespace jarvis {
namespace slip_detect {

//
SlipDetect::SlipDetect(const SlipDetectOption& option)
    : options_(option),
      feature_tracker_(new estimator::FeatureTracker(
          *static_cast<estimator::FeatureTrackerOption*>(
              option.feat_tracker_option))) {
  Eigen::Matrix3d rotaion;
  rotaion << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  // LOG(INFO) << rotaion;
  transform_cam_to_odom_map_ =
      transform::Rigid3d::Rotation(transform::RollPitchYaw(
          0, 0, transform::GetYaw(options_.transform_cam_to_odom.rotation())));

  // transform_cam_to_odom_map_ = transform::Rigid3d(
  //     Eigen::Vector3d(0, 0, 0),
  //     Eigen::Quaterniond(-0.00189292, -0.188614, 0.00131981, -0.982049));
}
//
void SlipDetect::AddOdometry(const jarvis::sensor::OdometryData& odom) {
  std::lock_guard<std::mutex> lock(mutex_);
  odometry_datas_.push_back(odom);
  if (odometry_datas_.size() > KMaxDataLenth) {
    odometry_datas_.pop_front();
  }
}
//
jarvis::transform::Rigid3d SlipDetect::ToPoseInOdom(
    const jarvis::transform::Rigid3d& pose1) {
  // auto transform_cam_to_odom_map_1 = transform::Rigid3d(
  //     Eigen::Vector3d(0.382489, -0.00321091, 0.19029),
  //     Eigen::Quaterniond(-0.00189292, -0.188614, 0.00131981, -0.982049));
      // Eigen::Quaterniond(-0.108145, 0.94084, -0.00176135, -0.321126));
  const transform::Rigid3d transform_odom_to_imu =
      options_.transform_cam_to_odom;
  const transform::Rigid3d pose =
      transform_cam_to_odom_map_ * pose1 * transform_odom_to_imu.inverse();
  //
  return jarvis::transform::Rigid3d(
      Eigen::Vector3d(pose.translation().x(), pose.translation().y(),
                      pose.translation().z()),
      pose.rotation());
}
//
void SlipDetect::ClearData() {
  std::lock_guard<std::mutex> lock(mutex_);
  key_point_datas_.clear();
  pose_datas_.clear();
  odometry_datas_.clear();
}
void SlipDetect::AddPose(const TimePose& pose) {
  std::lock_guard<std::mutex> lock(mutex_);
  pose_datas_.push_back({pose.time, ToPoseInOdom(pose.pose)});
  if (pose_datas_.size() > KMaxDataLenth) {
    pose_datas_.pop_front();
  }
}
//
void SlipDetect::AddImage(const jarvis::sensor::ImageData& image_data) {
  if (options_.type == 0) {
    std::map<int, int> track_num;
    double d_time =
        common::ToSeconds(image_data.time - common::FromUniversal(0));
    auto feature_frame = feature_tracker_->trackImage(
        d_time, *image_data.image[0], cv::Mat(), &track_num, 0);
    LOG(INFO) << feature_frame.data->features.size();
    for (auto const& point : feature_frame.data->features) {
      key_point_datas_[static_cast<uint64_t>(point.first)].emplace(
          image_data.time, point.second.camera_features[0].uv);
    }
  }
}

bool SlipDetect::Detect(const jarvis::common::Time& time) {
  if (options_.type == 0) {
    return ZeroVelocityDetect(time);
  } else if (options_.type == 1 || options_.type == 2) {
    return SimpleDetect(time);
  } else {
    LOG(FATAL) << "Not support type.";
  }
  return false;
}
template <typename T>
double SlipDetect::ComputePosesS(std::deque<T>* datas,
                                 const jarvis::common::Time& time) {
  double delta_s = 0;
  if (datas->size() < 2) return 0;
  for (size_t i = 1; i < datas->size(); i++) {
    if (datas->at(i).time > time) break;
    delta_s += abs((datas->at(i - 1).pose.inverse() * datas->at(i).pose)
                       .translation()
                       .norm());
  }
  // LOG(INFO)<<delta_s ;
  return delta_s;
}
template <typename T>
double SlipDetect::ComputePosesTheta(std::deque<T>* datas,
                                     const jarvis::common::Time& time) {
  double delta = 0;
  if (datas->size() < 2) return 0;
  for (size_t i = 1; i < datas->size(); i++) {
    if (datas->at(i).time > time) break;
    delta += abs(transform::GetAngle(
        (datas->at(i - 1).pose.inverse() * datas->at(i).pose)));
  }
  return common::RadToDeg(delta);
}

template <typename T>
int SlipDetect::ComputePosesCount(std::deque<T>* datas,
                                  const jarvis::common::Time& time) {
  int count = 0;
  for (size_t i = 1; i < datas->size(); i++) {
    // LOG(INFO)<< datas->at(i).pose;
    if (datas->at(i).time > time) break;
    ++count;
  }
  return count;
}
//

bool SlipDetect::SimpleDetect(const jarvis::common::Time& time) {
  std::lock_guard<std::mutex> lock(mutex_);
  DropData(time - common::FromSeconds(options_.que_time_duration),
           &odometry_datas_);
  DropData(time - common::FromSeconds(options_.que_time_duration),
           &pose_datas_);
  //
  const double delta_odom_s = ComputePosesS(&odometry_datas_, time);
  const double delta_pose_s = ComputePosesS(&pose_datas_, time);
  const auto delta_s = delta_odom_s - delta_pose_s;
  // LOG(INFO) << ComputePosesTheta(&odometry_datas_,time);
  // LOG(INFO) << ComputePosesTheta(&pose_datas_,time);
  const auto delta_theta = std::abs(ComputePosesTheta(&odometry_datas_, time) -
                                    ComputePosesTheta(&pose_datas_, time));

  if (delta_s > options_.pose_odom_err_s_threash_hold ||
      delta_theta > options_.pose_odom_err_theta_threash_hold) {
    LOG(WARNING) << "Detect Slip at time " << time << " With ds: " << delta_s
                 << " dtheta: " << delta_theta << "delta_odom_s "
                 << delta_odom_s << " delta_pose_s " << delta_pose_s
                 << " odom count : "
                 << ComputePosesCount(&odometry_datas_, time)
                 << "pose count: " << ComputePosesCount(&pose_datas_, time);
    return true;
  }
  return false;
}
//
bool SlipDetect::ZeroVelocityDetect(const jarvis::common::Time& time) {
  DropData(time - common::FromSeconds(options_.que_time_duration),
           &odometry_datas_);
  DropData(time - common::FromSeconds(options_.que_time_duration),
           &key_point_datas_);
  if (!IsZeroVelocity()) return false;
  if (ComputePosesS(&odometry_datas_, time) >
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
  for (auto const& key_points : key_point_datas_) {
    if (key_points.second.size() >= 2) {
      auto it = key_points.second.begin();
      ++it;
      Eigen::Vector2d sum{0, 0};
      for (; it != key_points.second.end(); it++) {
        sum += it->second - (std::prev(it)->second);
      }
      disparitys.push_back((sum / key_points.second.size()).squaredNorm());
    }
  }
  if (disparitys.size() < size_t(options_.min_disparity_num)) {
    VLOG(kGlogLevel)
        << "zupt failed - Disparity-based Detection (disparitys.size() "
        << disparitys.size() << " < min_disparity_num "
        << options_.min_disparity_num << ")";
    return false;
  }
  disparity = std::accumulate(disparitys.begin(), disparitys.end(), 0.0) /
              disparitys.size();

  if (disparity > options_.max_disparity * options_.max_disparity) {
    VLOG(kGlogLevel) << "zupt failed - Disparity-based Detection (disparity "
                     << disparity << " > max_disparity "
                     << options_.max_disparity << ", " << disparitys.size()
                     << " features)";
    return false;
  }
  VLOG(kGlogLevel) << "zupt accepted - Disparity-based Detection (disparity "
                   << disparity << " <= max_disparity "
                   << options_.max_disparity << ", " << disparitys.size()
                   << " features)";
  return true;
}

SlipDetect::~SlipDetect() {}

std::unique_ptr<SlipDetect> FactorSlipDetect(const std::string& file) {
  SlipDetectOption option;
  ParseYAMLOption(file, &option);
  LOG(INFO) << "Start slip detect : " << option.type;
  //
  jarvis::estimator::FeatureTrackerOption feat_option;
  ParseYAMLOption(file, &feat_option);
  option.feat_tracker_option = static_cast<void*>(&feat_option);
  return std::make_unique<SlipDetect>(option);
}

}  // namespace slip_detect
}  // namespace jarvis