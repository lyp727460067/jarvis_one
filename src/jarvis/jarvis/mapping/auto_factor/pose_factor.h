#ifndef _JARVIS_MAPPING_POSE_INIT_FACTOR_H
#define _JARVIS_MAPPING_POSE_INIT_FACTOR_H

namespace jarvis {
namespace mapping {
template <typename T>
inline T NormalizeAngle(const T& angle_radians) {
  // Use ceres::floor because it is specialized for double and Jet types.
  T two_pi(2.0 * M_PI);
  return angle_radians -
         two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}

//

class PoseGraphCostFunctor {
 public:
  static ceres::CostFunction* Create(const transform::Rigid3d& relative_pose,
                                     const std::array<double, 2>& factor) {
    return new ceres::AutoDiffCostFunction<PoseGraphCostFunctor, 6, 3, 4, 3, 4>(
        new PoseGraphCostFunctor(relative_pose, factor));
  }

  template <typename T>
  bool operator()(const T* const t0, const T* const q0, const T* const t1,
                  const T* const q1, T* residual) const {
    //
    
    Eigen::Map<const Eigen::Quaternion<T>> q0_(q0);
    Eigen::Map<const Eigen::Quaternion<T>> q1_(q1);
    const Eigen::Quaternion<T> h_rotation_inverse = q1_.conjugate() * q0_;
    //
    const Eigen::Matrix<T, 3, 1> delta(t1[0] - t0[0], t1[1] - t0[1],
                                       t1[2] - t0[2]);
    const Eigen::Matrix<T, 3, 1> h_translation = q0_.conjugate() * delta;

    const Eigen::Matrix<T, 3, 1> angle_axis_difference =
        transform::RotationQuaternionToAngleAxisVector(
            h_rotation_inverse * relative_pose_.rotation().cast<T>());
    const Eigen::Matrix<T, 3, 1> relative_translation =
        relative_pose_.translation().cast<T>();
    residual[0] = (h_translation[0] - relative_translation[0]) * factor_[0];
    residual[1] = (h_translation[1] - relative_translation[1]) * factor_[0];
    residual[2] = (h_translation[2] - relative_translation[2]) * factor_[0];
    residual[3] = factor_[1] * angle_axis_difference[0];
    residual[4] = factor_[1] * angle_axis_difference[1];
    residual[5] = factor_[1] * angle_axis_difference[2];
    return true;
  }

 private:
  explicit PoseGraphCostFunctor(const transform::Rigid3d& relative_pose,
                                const std::array<double, 2>& factor)
      :relative_pose_(relative_pose), factor_(factor) {}

  std::array<double, 2> factor_;
  const transform::Rigid3d relative_pose_;
};

//
class TranslationCostFunctor {
 public:
  static ceres::CostFunction* Create(const Eigen::Vector3d& translation,
                                     const double& factor) {
    return new ceres::AutoDiffCostFunction<TranslationCostFunctor, 3, 3>(
        new TranslationCostFunctor(translation, factor));
  }

  template <typename T>
  bool operator()(const T* const translation, T* residual) const {
    residual[0] = factor_ * (translation[0]-T(x_));
    residual[1] = factor_ * (translation[1]-T(y_));
    residual[2] = factor_ * (translation[2]-T(z_));
    return true;
  }

 private:
  explicit TranslationCostFunctor(const Eigen::Vector3d& translation,
                                  const double& factor)
      : factor_(factor),
        x_(translation.x()),
        y_(translation.y()),
        z_(translation.z()) {}

  const double factor_;
  const double x_;
  const double y_;
  const double z_;
};

class RotationDeltaCostFunctor {
 public:
  static ceres::CostFunction* Create(const Eigen::Quaterniond& rotation,
                                     const double factor) {
    return new ceres::AutoDiffCostFunction<RotationDeltaCostFunctor, 3, 4>(
        new RotationDeltaCostFunctor(rotation, factor));
  }

  template <typename T>
  bool operator()(const T* const q1_, T* residual) const {
    Eigen::Map<const Eigen::Quaternion<T>> q1(q1_);
    Eigen::Matrix<T, 3, 1> delta =
        T(2.0) * (q1.inverse() * rotaion_.template cast<T>()).vec();
    residual[0] = factor_ * delta.x();
    residual[1] = factor_ * delta.y();
    residual[2] = factor_ * delta.z();
    return true;
  }

 private:
  explicit RotationDeltaCostFunctor(const Eigen::Quaterniond& rotation,
                                    const double& factor)
      : factor_(factor), rotaion_(rotation) {}

  const double factor_;
  const Eigen::Quaterniond rotaion_;
};
//
class YawRotationDeltaCostFunctor {
 public:
  static ceres::CostFunction* Create(const double& rotation,
                                     const double factor) {
    return new ceres::AutoDiffCostFunction<YawRotationDeltaCostFunctor, 1, 1>(
        new YawRotationDeltaCostFunctor(rotation, factor));
  }

  template <typename T>
  bool operator()(const T* const q1_, T* residual) const {
    residual[0] = factor_ * NormalizeAngle(T(yaw_) - q1_[0]);
    return true;
  }

 private:
  explicit YawRotationDeltaCostFunctor(const double& yaw, const double& factor)
      : factor_(factor), yaw_(yaw) {}

  const double factor_;
  const double yaw_;
};

}  // namespace mapping
}  // namespace jarvis

#endif
