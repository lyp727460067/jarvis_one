

#include "pose_optimization.h"

#include <Eigen/Core>

#include "ceres/ceres.h"
#include "jarvis/transform/timestamped_transform.h"
#include "jarvis/transform/transform.h"
//
using namespace jarvis;

namespace jarvis_pic {
//
struct NodePose {
  Eigen::Vector3d p;
  Eigen::Quaterniond q;
};
//
transform::Rigid3d NodePoseToTransform(const NodePose& node_pose) {
  return transform::Rigid3d(node_pose.p, node_pose.q);
}

//
template <typename T>
inline T NormalizeAngle(const T& angle_radians) {
  T two_pi(2.0 * M_PI);
  return angle_radians - two_pi * floor((angle_radians + T(M_PI)) / two_pi);
}

class AngleLocalParameterization {
 public:
  template <typename T>
  bool operator()(const T* theta_radians, const T* delta_theta_radians,
                  T* theta_radians_plus_delta) const {
    *theta_radians_plus_delta =
        NormalizeAngle(*theta_radians + *delta_theta_radians);
    return true;
  }

  static ceres::LocalParameterization* Create() {
    return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,
                                                     1, 1>);
  }
};
class GpsCostFunction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GpsCostFunction(const Eigen::Vector2d p_local, const Eigen::Vector2d p_gps,
                  const std::array<double, 2>& weitht)
      : p_local_(p_local), p_gps_(p_gps), weigth_(weitht) {}
  template <typename T>
  bool operator()(const T* const pose, T* residuals) const {
    Eigen::Matrix<T, 2, 1> t{pose[0], pose[1]};
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    const Eigen::Matrix<T, 2, 1> err =
        p_gps_.cast<T>() - (rotation_matrix * p_local_.cast<T>() + t);
    residuals[0] = err[0] * T(weigth_[0]);
    residuals[1] = err[1] * T(weigth_[1]);
    return true;
  }
  static ceres::CostFunction* Creat(const Eigen::Vector2d p_local,
                                    const Eigen::Vector2d p_gps,
                                    const std::array<double, 2>& weitht) {
    return new ceres::AutoDiffCostFunction<GpsCostFunction, 2, 3>(
        new GpsCostFunction(p_local, p_gps, weitht));
  }

 private:
  const Eigen::Vector2d p_local_;
  const Eigen::Vector2d p_gps_;
  const std::array<double, 2> weigth_;
};

template <typename T>
static std::array<T, 6> ComputeUnscaledError(
    const transform::Rigid3d& relative_pose, const T* const start_rotation,
    const T* const start_translation, const T* const end_rotation,
    const T* const end_translation) {
  const Eigen::Quaternion<T> R_i_inverse(start_rotation[0], -start_rotation[1],
                                         -start_rotation[2],
                                         -start_rotation[3]);

  const Eigen::Matrix<T, 3, 1> delta(end_translation[0] - start_translation[0],
                                     end_translation[1] - start_translation[1],
                                     end_translation[2] - start_translation[2]);
  const Eigen::Matrix<T, 3, 1> h_translation = R_i_inverse * delta;

  const Eigen::Quaternion<T> h_rotation_inverse =
      Eigen::Quaternion<T>(end_rotation[0], -end_rotation[1], -end_rotation[2],
                           -end_rotation[3]) *
      Eigen::Quaternion<T>(start_rotation[0], start_rotation[1],
                           start_rotation[2], start_rotation[3]);

  const Eigen::Matrix<T, 3, 1> angle_axis_difference =
      transform::RotationQuaternionToAngleAxisVector(
          h_rotation_inverse * relative_pose.rotation().cast<T>());

  return {{T(relative_pose.translation().x()) - h_translation[0],
           T(relative_pose.translation().y()) - h_translation[1],
           T(relative_pose.translation().z()) - h_translation[2],
           angle_axis_difference[0], angle_axis_difference[1],
           angle_axis_difference[2]}};
}
template <typename T>
std::array<T, 6> ScaleError(const std::array<T, 6>& error,
                            double translation_weight, double rotation_weight) {
  // clang-format off
  return {{
      error[0] * translation_weight,
      error[1] * translation_weight,
      error[2] * translation_weight,
      error[3] * rotation_weight,
      error[4] * rotation_weight,
      error[5] * rotation_weight
  }};
  // clang-format on
}

class GpsWindCostFunction {
 public:
  GpsWindCostFunction(const transform::Rigid3d& pose,
                      const std::array<double, 2>& weitht)
      : mearemnt_pose_(pose), weigth_(weitht) {
      }
  template <typename T>
  bool operator()(const T* const c_i_rotation, const T* const c_i_translation,
                  const T* const c_j_rotation, const T* const c_j_translation,
                  T* const e) const {
    const std::array<T, 6> error = ScaleError(
        ComputeUnscaledError(mearemnt_pose_, c_i_rotation, c_i_translation,
                             c_j_rotation, c_j_translation),
        weigth_[0], weigth_[1]);
    e[0] =  error[0];
    e[1] =  error[1];
    e[2] =  error[2];
    // std::copy(std::begin(error), std::begin(error)+3, e);
    return true;
  }
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const transform::Rigid3d& pose, const std::array<double, 2>& weitht) {
    return new ceres::AutoDiffCostFunction<GpsWindCostFunction, 3, 4, 3, 4, 3>(
        new GpsWindCostFunction(pose, weitht));
  }

 private:
  const transform::Rigid3d mearemnt_pose_;
  const std::array<double, 2> weigth_;
};
class PoseGraphExtricCostFunction {
 public:
  PoseGraphExtricCostFunction(const transform::Rigid3d& pose,
                              const std::array<double, 2>& weitht)
      : mearemnt_pose_(pose), weigth_(weitht) {
        LOG(INFO)<< mearemnt_pose_;
      }
  template <typename T>
  bool operator()(const T* const c_i_rotation, const T* const c_i_translation,
                  const T* const c_j_rotation, const T* const c_j_translation,
                  const T* const extric_translation, T* const e) const {
    //
    const Eigen::Quaternion<T> r_i(c_i_rotation[0], c_i_rotation[1],
                                   c_i_rotation[2], c_i_rotation[3]);
    //
    const Eigen::Quaternion<T> r_j(c_j_rotation[0], c_j_rotation[1],
                                   c_j_rotation[2], c_j_rotation[3]);
    //

    Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_i(c_i_translation);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_j(c_j_translation);
    const Eigen::Matrix<T, 3, 1> extric(extric_translation[0],extric_translation[1],T(0));

    //
    const Eigen::Matrix<T, 3, 1> p_i_e = r_i * extric+ p_i;
    const Eigen::Matrix<T, 3, 1> p_j_e = r_j * extric+ p_j;

    const Eigen::Quaternion<T> r_i_inverse(c_i_rotation[0], -c_i_rotation[1],
                                           -c_i_rotation[2], -c_i_rotation[3]);

    const Eigen::Matrix<T, 3, 1> delta(p_j_e[0] - p_i_e[0], p_j_e[1] - p_i_e[1],
                                       p_j_e[2] - p_i_e[2]);
    const Eigen::Matrix<T, 3, 1> h_translation = r_i_inverse * delta;

    const Eigen::Quaternion<T> h_rotation_inverse =
        Eigen::Quaternion<T>(c_j_rotation[0], -c_j_rotation[1],
                             -c_j_rotation[2], -c_j_rotation[3]) *
        Eigen::Quaternion<T>(c_i_rotation[0], c_i_rotation[1], c_i_rotation[2],
                             c_i_rotation[3]);

    const Eigen::Matrix<T, 3, 1> angle_axis_difference =
        transform::RotationQuaternionToAngleAxisVector(
            h_rotation_inverse * mearemnt_pose_.rotation().cast<T>());

    //
    e[0] = T(weigth_[0]) *
           (T(mearemnt_pose_.translation().x()) - h_translation[0]);
    e[1] =
        T(weigth_[0]) * ((mearemnt_pose_.translation().y()) - h_translation[1]);
    e[2] =
        T(weigth_[0]) * ((mearemnt_pose_.translation().z()) - h_translation[2]);
    e[3] = T(weigth_[1]) * angle_axis_difference[0];
    e[4] = T(weigth_[1]) * angle_axis_difference[1];
    e[5] = T(weigth_[1]) * angle_axis_difference[2];

    return true;
  }
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const transform::Rigid3d& pose, const std::array<double, 2>& weitht) {
    return new ceres::AutoDiffCostFunction<PoseGraphExtricCostFunction, 6, 4, 3,
                                           4, 3, 2>(
        new PoseGraphExtricCostFunction(pose, weitht));
  }

 private:
  const transform::Rigid3d mearemnt_pose_;
  const std::array<double, 2> weigth_;
};

std::pair<std::array<double, 3>, std::array<double, 4>> ToCeresPose(
    const transform::Rigid3d& pose) {
  return {{
              {pose.translation().x(), pose.translation().y(),
               pose.translation().z()},
          },
          {
              {pose.rotation().w(), pose.rotation().x(), pose.rotation().y(),
               pose.rotation().z()},

          }};
}
transform::Rigid3d ArrayToRigid(
    const std::pair<std::array<double, 3>, std::array<double, 4>>& pose) {
  return transform::Rigid3d(
      Eigen::Vector3d(pose.first[0], pose.first[1], pose.first[2]),
      Eigen::Quaterniond(pose.second[0], pose.second[1], pose.second[2],
                         pose.second[3]));
}

std::unique_ptr<jarvis::transform::Rigid3d> PoseOptimization::Optimization() {
  //
}

std::unique_ptr<transform::Rigid3d> PoseOptimization::AddFixedFramePoseData(
    const sensor::FixedFramePoseData& fix_data) {
  // if (rtk_motion_filter_->IsSimilar(fix_data.time, fix_data.pose)) {
  //   return nullptr;
  // }
  if (!rtk_interpolateion_) {
    rtk_pose_.push_back(fix_data);
  } else {
    rtk_interpolateion_->Push(fix_data.time, fix_data.pose);
  }
  Alignment();
  return nullptr;
}

void PoseOptimization::Alignment() {
  if (rtk_interpolateion_) return;
  while (!odom_pose_.empty() && (rtk_pose_.back().time > odom_pose_.back().time)) {
    odom_pose_.pop_back();
  }
  //
  if (!odom_pose_.empty() && !rtk_pose_.empty()) {
    rtk_interpolateion_ =
        std::make_unique<jarvis::transform::TransformInterpolationBuffer>();
    for (const auto& r : rtk_pose_) {
      rtk_interpolateion_->Push(r.time, r.pose);
    }
  }
  
}
//
//
std::unique_ptr<jarvis::transform::Rigid3d>
PoseOptimization::AlignmentOptimization() {
  //

  std::vector<NodePose> node_poses;
  //
  LOG(INFO) << "Using odom size :" << odom_pose_.size();
  LOG(INFO) << "Using rtk size :" << rtk_interpolateion_->size();
  for (int i = 0; i < odom_pose_.size(); i++) {
    if (!rtk_interpolateion_->Has(odom_pose_[i].time)) {
      LOG(INFO) << "At odom time not has rtk " << odom_pose_[i].time;
      return nullptr;
    }

    node_poses.push_back(NodePose{odom_pose_[i].pose.translation(),
                                  odom_pose_[i].pose.rotation()});
    // auto init = rtk_interpolateion_->Lookup(odom_pose_[i].time);
    // node_poses.push_back(NodePose{init.translation(),
    //                               init.rotation() });
  }
  //
  ceres::LocalParameterization* quaternion_local =
      new ceres::EigenQuaternionParameterization;
  ceres::Problem problem;
  //
  std::array<double, 3> local_to_fix_translation{0,0,0};
  std::array<double, 4> local_to_fix_rotation{1,0,0,0};
  std::array<double, 3> fix_imu_extri{0,0,0};
  //

  for (int i = 0; i < node_poses.size(); i++) {
    problem.AddResidualBlock(
        GpsWindCostFunction::CreateAutoDiffCostFunction(
            rtk_interpolateion_->Lookup(odom_pose_[i].time),
            std::array<double, 2>{options_.fix_weitht_traslation,
                                  options_.fix_weitht_rotation}),
        nullptr, local_to_fix_rotation.data(),
        local_to_fix_translation.data(), node_poses[i].q.coeffs().data(),
        node_poses[i].p.data());
    problem.SetParameterization(node_poses[i].q.coeffs().data(),
                                quaternion_local);
    problem.SetParameterBlockConstant(node_poses[i].q.coeffs().data());
    problem.SetParameterBlockConstant(node_poses[i].p.data());
  }
  //
  // for (int i = 1; i < node_poses.size(); i++) {
  //   problem.AddResidualBlock(
  //       PoseGraphExtricCostFunction::CreateAutoDiffCostFunction(
  //           odom_pose_[i - 1].pose.inverse() * odom_pose_[i].pose,
  //           std::array<double, 2>{100000,100000}),
  //       new ceres::HuberLoss(1), node_poses[i - 1].q.coeffs().data(),
  //       node_poses[i-1].p.data(), node_poses[i].q.coeffs().data(),
  //       node_poses[i].p.data(), fix_imu_extri.data());
  // }
  // problem.SetParameterBlockConstant(node_poses[0].q.coeffs().data());
  // problem.SetParameterBlockConstant(node_poses[0].p.data());


  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout =true;
  options.max_num_iterations = 20;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  LOG(INFO) << "Eetric: " << fix_imu_extri[0] << " " << fix_imu_extri[1] << " "
            << fix_imu_extri[2]; 
  pose_local_to_fix_ =
      ArrayToRigid({local_to_fix_translation, local_to_fix_rotation});
  LOG(INFO)<<"Local To fix transform: "<<pose_local_to_fix_ ;
  for(const auto&p:node_poses){
    optimazation_poses_.push_back(jarvis::transform::Rigid3d(p.p,p.q));
  }
  return std::make_unique<transform::Rigid3d>(pose_local_to_fix_);
  //
}
//
PoseOptimization::PoseOptimization(const PoseOptimizationOption& option)
    : options_(option),
      pose_motion_filter_(new jarvis::MotionFilter(MotionFilterOptions{1000,0.2,0.5})),
      rtk_motion_filter_(new jarvis::MotionFilter(MotionFilterOptions{1000,0.2,0.5})) {}

//
void PoseOptimization::AddPose(const PoseData& pose) {
  if (pose_motion_filter_->IsSimilar(pose.time, pose.pose)) {
    return;
  }
  if (odom_pose_.size() > options_.win_size) odom_pose_.pop_back();
  // LOG(INFO) << pose.time;
  odom_pose_.push_front(pose);
}

}  // namespace jarvis_pic
