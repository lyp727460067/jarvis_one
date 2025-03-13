#include "optimization.h"

#include "jarvis/estimator/factor/imu_factor.h"
#include "jarvis/estimator/factor/integration_base.h"
#include "jarvis/estimator/factor/marginalization_factor.h"
#include "jarvis/estimator/factor/pose_local_parameterization.h"
#include "jarvis/estimator/factor/projectionOneFrameTwoCamFactor.h"
#include "jarvis/estimator/factor/projectionTwoFrameOneCamFactor.h"
#include "jarvis/estimator/factor/projectionTwoFrameTwoCamFactor.h"
#include "marginalization.h"
#include "jarvis/estimator/factor/initial_pose_factor.h"
namespace jarvis {
namespace estimator {
//
namespace {

constexpr int kMaxFeatureNum = 1000;

#define para_Pose (data_.pose)
#define para_SpeedBias (data_.speed_bias)
#define para_Ex_Pose (data_.ex_pose)
#define para_Ex_Pose_Odom (data_.ex_pose_odom)
#define para_Td (data_.td)
#define para_Feature (data_.feature)

}  // namespace
// #define  AUTO_RE_PROEJCT
#ifdef AUTO_RE_PROEJCT

struct ReProjectionErrProblem {
 public:
  ReProjectionErrProblem(const Eigen::Vector2d &nor_poit,
                         const Eigen::Vector3d &map_point, const double &factor)
      : nor_point_(nor_poit), map_point_(map_point), factor_(factor) {}

  template <typename T>
  bool operator()(const T *parameters, const T *ex, T *residul) const {
    //
    Eigen::Matrix<T, 3, 1> t1(parameters[0], parameters[1],
                                      parameters[2]);
    //
    Eigen::Quaternion<T> q1(parameters[6], parameters[3], parameters[4],
                             parameters[5]);

    Eigen::Matrix<T, 3, 1> te(ex[0], ex[1], ex[2]);
    Eigen::Quaternion<T> qe(ex[6], ex[3], ex[4], ex[5]);
    Eigen::Quaternion<T> q_inverse = (q1 * qe).inverse();
    //
    Eigen::Matrix<T, 3, 1> t = q1 * te + t1;
    Eigen::Matrix<T, 3, 1> t_inverse =  q_inverse * t;

    Eigen::Matrix<T, 3, 1> project_p =
        q_inverse * map_point_.template cast<T>() - t_inverse;
    T x_normal = project_p[0] / project_p[2];
    T y_normal = project_p[1] / project_p[2];
    residul[0] = T(factor_) * (x_normal - T(nor_point_.x()));
    residul[1] = T(factor_) * (y_normal - T(nor_point_.y()));
    return true;
  }
  static ceres::CostFunction *Creat(const Eigen::Vector2d &nor_poit,
                                    const Eigen::Vector3d &map_point,
                                    double factor) {
    return new ceres::AutoDiffCostFunction<ReProjectionErrProblem, 2, 7, 7>(
        new ReProjectionErrProblem(nor_poit.head<2>(), map_point, factor));
  }

 private:
  const Eigen::Vector2d nor_point_;
  const Eigen::Vector3d map_point_;
  const double factor_;
};

#else 
struct ReProjectionErrProblem : public ceres::SizedCostFunction<2, 7, 7> {
  ReProjectionErrProblem(const Eigen::Vector2d &nor_poit,
                         const Eigen::Vector3d &map_point, const double &factor)
      : nor_point_(nor_poit), map_point_(map_point), factor_(factor) {
    sqrt_info = Eigen::Matrix2d::Identity() * factor_;
    t_sqrt_info = Eigen::Matrix2d::Identity() * factor_*100;
  }
  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const {
    //

    Eigen::Vector3d t1(parameters[0][0], parameters[0][1], parameters[0][2]);
    //
    Eigen::Quaterniond q1(parameters[0][6], parameters[0][3], parameters[0][4],
                          parameters[0][5]);
    Eigen::Vector3d te(parameters[1][0], parameters[1][1], parameters[1][2]);
    //
    Eigen::Quaterniond qe(parameters[1][6], parameters[1][3], parameters[1][4],
                          parameters[1][5]);

    Eigen::Vector3d pts_imu_1 = q1.inverse() * (map_point_ - t1);
    Eigen::Vector3d project_p = qe.inverse() * (pts_imu_1 - te);

    // Eigen::Quaterniond q_inverse = (q1 * qe).inverse();
    // //
    // Eigen::Vector3d t = q1 * te + t1;
    // Eigen::Vector3d t_inverse = -(q_inverse * t);

    // Eigen::Vector3d project_p = q_inverse * map_point_ + t_inverse;
    double x_normal = project_p[0] / project_p[2];
    double y_normal = project_p[1] / project_p[2];
    residuals[0] = (factor_) * (x_normal - (nor_point_.x()));
    residuals[1] = (factor_) * (y_normal - (nor_point_.y()));
    //
    if (jacobians) {
      Eigen::Matrix<double, 2, 3> reduce(2, 3);
      double dep_j = project_p.z();
      reduce << 1. / dep_j, 0, -project_p(0) / (dep_j * dep_j), 0, 1. / dep_j,
          -project_p(1) / (dep_j * dep_j);
      Eigen::Matrix3d r1 =  q1.toRotationMatrix();
      Eigen::Matrix3d re1 =  qe.toRotationMatrix();
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobians_(
            jacobians[0]);
        jacobians_.setZero();
        //忽略平移方向，只在旋转方向给约束
        //
        jacobians_.block<2, 3>(0, 0) =
            sqrt_info * reduce *
            re1.transpose() *  -r1.transpose();
        jacobians_.block<2, 3>(0, 3) = sqrt_info * reduce * re1.transpose() *
                                       Utility::skewSymmetric(pts_imu_1);
      }
      if (jacobians[1]) {
        //不参与外参的优化
        Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobians_(
            jacobians[1]);
        jacobians_.setZero();
      }
    }

    return true;
  }
  static ceres::CostFunction *Creat(const Eigen::Vector2d &nor_poit,
                                    const Eigen::Vector3d &map_point,
                                    double factor) {
    return new ReProjectionErrProblem(nor_poit, map_point, factor);
  }

 private:
  const Eigen::Vector2d nor_point_;
  const Eigen::Vector3d map_point_;
  Eigen::Matrix2d sqrt_info;
  Eigen::Matrix2d t_sqrt_info;
  const double factor_;
};
#endif

Optimization::Optimization(int win_size1, const OptimizationOption &option)
    : win_size_(win_size1), options_(option) {
  //
  LOG(INFO) << "Optimizaiton init ,w_size :" << win_size_;
  int win_size = win_size1 + 1;
  data_.pose = new double *[win_size];
  data_.speed_bias = new double *[win_size];

  //
  for (int i = 0; i < win_size; i++) {
    data_.pose[i] = new double[SIZE_POSE];
    memset(data_.pose[i], 0, sizeof(double) * SIZE_POSE);
    data_.speed_bias[i] = new double[SIZE_SPEEDBIAS];
    memset(data_.speed_bias[i], 0, sizeof(double) * SIZE_SPEEDBIAS);
  }
  CHECK_LE(option.CamNum(), 4);
  data_.ex_pose = new double *[option.CamNum()];
  for (int i = 0; i < option.CamNum(); i++) {
    data_.ex_pose[i] = new double[SIZE_POSE];
    memset(data_.ex_pose[i], 0, sizeof(double) * SIZE_POSE);
  }
  //
  //
  data_.feature = new double **[options_.TrackNum()];
  for (int i = 0; i < options_.TrackNum(); i++) {
    data_.feature[i] = new double *[kMaxFeatureNum];
    for (int j = 0; j < kMaxFeatureNum; j++) {
      data_.feature[i][j] = new double[1];
      data_.feature[i][j][0] = 0;
    }
  }

  //

  data_.ex_pose_odom = new double *[1];
  data_.ex_pose_odom[0] = new double[SIZE_POSE];
  memset(data_.ex_pose_odom[0], 0, sizeof(double) * SIZE_POSE);
  //

  data_.td = new double *[1];
  data_.td[0] = new double[1];
  data_.td[0][0] = 0;

  // para_Pose = data_.pose;
  // para_SpeedBias = data_.speed_bias;
  // para_Ex_Pose = data_.ex_pose;
  // para_Ex_Pose_Odom = data_.ex_pose_odom;
  // para_Td = data_.td;
  // para_Feature = data_.feature;

  //
}

int Optimization::AddCameraFactor(int id, ceres::Problem *problem,
                                   ceres::LossFunction *loss_function,
                                   ceres::ParameterBlockOrdering *ordering,
                                   FeatureManager *feature_managers) {
  //
  // const auto &f_managers = feature_managers->GetFeatureManagers();
  //
  auto feature_manager = feature_managers;
  int f_m_cnt = 0;
  // std::stringstream info;
  // for (const auto &feature_manager : f_managers) {

  const double cam_weight = options_.camera_weight;
  std::stringstream info1;
  //
  feature_manager->CreateFactor(
      [&](const Eigen::Vector3d &pts_i, const Eigen::Vector3d &pts_j,
          const Eigen::Vector2d &imu_i_velocity,
          const Eigen::Vector2d &imu_j_velocity, const double td_i,
          const double td_j, const std::tuple<int, int, int> &index) {
        //
        ProjectionTwoFrameOneCamFactor *f_td =
            new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, imu_i_velocity,
                                               imu_j_velocity, td_i, td_j,
                                               cam_weight);
        //

        // info1 << "[" << std::get<0>(index) << std::get<1>(index)
        //       << std::get<2>(index) << "]" << pts_i.transpose() << " "
        //       << pts_j.transpose() << imu_i_velocity.transpose()
        //       << imu_j_velocity.transpose() << td_i << td_j << cam_weight <<
        //       "\n";
        // for (int i = 0; i < 7; i++) {
        //   info1 << para_Pose[std::get<0>(index)][i] << " ";
        //   info1 << para_Pose[std::get<1>(index)][i] << " ";
        //   info1 << para_Ex_Pose[0][i] << "\n";
        // }
        // info1 << *para_Feature[id][std::get<2>(index)] << "\n";

        problem->AddResidualBlock(
            f_td, loss_function, para_Pose[std::get<0>(index)],
            para_Pose[std::get<1>(index)],
            para_Ex_Pose[options_.trace_sequence[id][0]],
            para_Feature[id][std::get<2>(index)], para_Td[0]);
        f_m_cnt++;
        ordering->AddElementToGroup(para_Feature[id][std::get<2>(index)], 0);
      },
      [&](const Eigen::Vector3d &pts_i, const Eigen::Vector3d &pts_j,
          const Eigen::Vector2d &velocity_i, const Eigen::Vector2d &velocity_j,
          const double td_i, const double td_j,
          const std::tuple<int, int, int> &index) {
        //
        ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(
            pts_i, pts_j, velocity_i, velocity_j, td_i, td_j, cam_weight);
        problem->AddResidualBlock(
            f, loss_function, para_Pose[std::get<0>(index)],
            para_Pose[std::get<1>(index)],
            para_Ex_Pose[options_.trace_sequence[id][0]],
            para_Ex_Pose[options_.trace_sequence[id][1]],
            para_Feature[id][std::get<2>(index)], para_Td[0]);
        // info1 << "ste[" << std::get<0>(index) << std::get<1>(index)
        //       << std::get<2>(index) << "]" << pts_i.transpose() << " "
        //       << pts_j.transpose() << velocity_i.transpose()
        //       << velocity_j.transpose() << td_i << td_j << cam_weight
        //       << "\n";
        // for (int i = 0; i < 7; i++) {
        //   info1 << para_Pose[std::get<0>(index)][i] << " ";
        //   info1 << para_Pose[std::get<1>(index)][i] << " ";
        //   info1 << para_Ex_Pose[options_.trace_sequence[id][0]][i]<<" "
        //         << para_Ex_Pose[options_.trace_sequence[id][1]][i] << "\n";
        // }
        f_m_cnt++;
      },
      [&](const Eigen::Vector3d &pts_i, const Eigen::Vector3d &pts_j,
          const Eigen::Vector2d &velocity_i, const Eigen::Vector2d &velocity_j,
          const double td_i, const double td_j,
          const std::tuple<int, int, int> &index) {
        //
        ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(
            pts_i, pts_j, velocity_i, velocity_j, td_i, td_j, cam_weight);

        problem->AddResidualBlock(
            f, loss_function, para_Ex_Pose[options_.trace_sequence[id][0]],
            para_Ex_Pose[options_.trace_sequence[id][1]],
            para_Feature[id][std::get<2>(index)], para_Td[0]);
        f_m_cnt++;
      }

  );
  // LOG(INFO)<<info1.str();
  LOG_EVERY_N(INFO, 10) << "cam " << id << " Adding factor feature size "
                         << f_m_cnt;
  VLOG(kGlogLevel) << "cam " << id << " Adding factor feature size " << f_m_cnt;
  return f_m_cnt;
}

void Optimization::AddFrameFactor(ceres::Problem *problem,
                                  ceres::LossFunction *loss_function,
                                  ceres::ParameterBlockOrdering *ordering,
                                  OptimizationData *sw_data) {
  //

  //
  if (options_.use_odom) {
    problem->AddParameterBlock(para_Ex_Pose_Odom[0], SIZE_POSE,
                              new PoseLocalParameterization());
    // /
    ordering->AddElementToGroup(para_Ex_Pose_Odom[0], 1);
  }
  //
  for (int i = 0; i < win_size_; i++) {
    int j = i + 1;
    // if (j == win_size_ + 1) {
    //   auto &update_zero_velocity =
    //       sw_data->frame_data[j].data->update_zero_velocity_data;
    //   if (update_zero_velocity) {
    //     LOG(INFO)<<"add velocity";
    //     if (update_zero_velocity->IsZeroVelocity()) {
    //       //
    //       for (int k = 0; k < 7; k++) {
    //         para_Pose[j][k] = para_Pose[i][k];
    //       }
    //       update_zero_velocity->AddToProblem(
    //           problem, nullptr,
    //           std::array<double *, 3>{para_Pose[i], para_Pose[j],
    //                                   para_SpeedBias[i]});
    //     }
    //   }
    // }
    if (options_.use_odom && sw_data->odom_factors[j]) {
      sw_data->odom_factors[j]->AddToProblem(
          problem, nullptr,
          std::array<double *, 3>{para_Pose[i], para_Pose[j],
                                  para_Ex_Pose_Odom[0]});
    }
    auto pre_integration = sw_data->imu_factors[j];
    //
    if (!pre_integration || !pre_integration->IsValid()) {
      LOG(WARNING)<< j << " Imu avalid..";
      continue;
    }
    // LOG(INFO)<<common::RadToDeg( transform::GetYaw(pre_integration->delta_q));
    // LOG(INFO)<<pre_integration->delta_p.transpose();
    IMUFactor *imu_factor = new IMUFactor(pre_integration);
    problem->AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i],
                              para_Pose[j], para_SpeedBias[j]);
  }
}

OptimizationStateData *Optimization::Solve(Marginalization *marg,
                                           OptimizationData *frames_data) {
  //
  //
  TicToc Optimization_result_t_t;
  std::stringstream info;
  //   FrameDataToState(frames_data);
  ceres::Problem problem;
  ceres::LossFunction *loss_function =
      new ceres::HuberLoss(options_.huber_loss);
  ceres::ParameterBlockOrdering *ordering = new ceres::ParameterBlockOrdering();
  for (int i = 0; i < win_size_ + 1; i++) {
    // LOG(INFO)<<frames_data->frame_data[i].data->imu_state;
    // LOG(INFO)<<frames_data->frame_data[i].data->imu_data->State();

    ceres::LocalParameterization *local_parameterization =
        new PoseLocalParameterization();
    ordering->AddElementToGroup(para_Pose[i], 1);
    problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
    problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    // for (int j = 0; j < 6; j++) {
    //   problem.SetParameterLowerBound(para_SpeedBias[i], j + 3, -0.5);
    //   problem.SetParameterUpperBound(para_SpeedBias[i], j + 3, 0.5);
    // }
    ordering->AddElementToGroup(para_SpeedBias[i], 1);
  }


  //
  Eigen::Vector3d vs(para_SpeedBias[0][0], para_SpeedBias[0][1],
                     para_SpeedBias[0][2]);

  auto pre_integration = frames_data->imu_factors.back();
  for (int i = 0; i < options_.CamNum(); i++) {
    ceres::LocalParameterization *local_parameterization =
        new PoseLocalParameterization();

    problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE,
                              local_parameterization);

    ordering->AddElementToGroup(para_Ex_Pose[i], 1);
    // LOG(INFO)<<vs.norm() ;
    if (options_.estimate_extrinsic == 0 || vs.norm() < 0.2) {
      problem.SetParameterBlockConstant(para_Ex_Pose[i]);
    }
    if (!slide_came_extirc) {
      if (i >= 2) {
        LOG(INFO)<<"FIX"<<i;
        problem.SetParameterBlockConstant(para_Ex_Pose[i]);
      }
    }
    //
    // for (int k = 0; k < 3; k++) {
    //   problem.SetParameterUpperBound(
    //       para_Ex_Pose[i], k,
    //       options_.extric_camera_to_imu[i].translation()[k] + 0.05);
    //   problem.SetParameterLowerBound(
    //       para_Ex_Pose[i], k,
    //       options_.extric_camera_to_imu[i].translation()[k] - 0.05);
    // }

    if (pre_integration == nullptr || !pre_integration->IsValid() ||  
        abs(common::RadToDeg(transform::GetYaw(pre_integration->delta_q)) > 4)) {
      // LOG(WARNING)<< common::RadToDeg(transform::GetYaw(pre_integration->delta_q) );
      // problem.SetParameterBlockConstant(para_Ex_Pose[i]);
    };
  }
  problem.AddParameterBlock(para_Td[0], 1);
  ordering->AddElementToGroup(para_Td[0], 1);
  if (options_.estimate_td == 0) {
    problem.SetParameterBlockConstant(para_Td[0]);
  }

  //
  if (marg) {
    TicToc t_t;
    marg->AddToProblem(&problem, nullptr);
    VLOG(kGlogCostTimeLevel) << "add marg factor costs " << t_t.toc() << " ms";
  } else {
    problem.SetParameterBlockConstant(para_Pose[0]);
  }
  {
    TicToc t_t;
    AddFrameFactor(&problem, nullptr, ordering, frames_data);
    VLOG(kGlogCostTimeLevel) << "add frame factor costs " << t_t.toc() << " ms";
  }
  //

  int camera_factor_num = 0;
  {
    TicToc t_t;
    for (int i = 0; i < options_.TrackNum(); i++) {
      if (frames_data->feat_manager_factors->Exist(i)) {
        camera_factor_num += AddCameraFactor(
            i, &problem, loss_function, ordering,
            frames_data->feat_manager_factors->MutableFeatureManager(i).get());
      } else {
        problem.SetParameterBlockConstant(
            para_Ex_Pose[options_.trace_sequence[i][0]]);
      }
      // LOG(INFO)<<options_.trace_sequence[i][0];
    }

    // 视觉约束过少时固定odo-imu外参
    if (options_.use_odom&& camera_factor_num < options_.camera_factor_num_th) {
      LOG(WARNING) << "Camera fator num : " << camera_factor_num;
      problem.SetParameterBlockConstant(para_Ex_Pose_Odom[0]);
      // problem.SetParameterBlockConstant(para_Ex_Pose[0]);
      // problem.SetParameterBlockConstant(para_Ex_Pose[1]);
      // problem.SetParameterBlockConstant(para_Ex_Pose[2]);
    }

    VLOG(kGlogCostTimeLevel)
        << "add camera factor costs " << t_t.toc() << " ms";
  }
  {
    if (prior_pose_.has_value()) {
      if (prior_pose_.value().second->matchs.empty()) {
        const transform::Rigid3d pose = prior_pose_.value().second->pose;
        const int k = prior_pose_.value().first;
        InitialPoseFactor *f = new InitialPoseFactor(
            options_.prio_pose_weight, pose.translation(), pose.rotation());
        problem.AddResidualBlock(f, nullptr, para_Pose[k]);
      } else {
        LOG(INFO) << "Add prio local map match:"
                  << prior_pose_.value().second->matchs.size();
        const int k = prior_pose_.value().first;
        bool need_add_init_factory = false;
        if (camera_factor_num > options_.camera_factor_num_th) {
          for (const auto &match : prior_pose_.value().second->matchs) {
            //
            problem.AddResidualBlock(
                ReProjectionErrProblem::Creat(match.normal, match.map_point,
                                              options_.prio_pose_weight),
                new ceres::HuberLoss(3.0), para_Pose[k],
                para_Ex_Pose[options_.trace_sequence[match.s][0]]);
          }
          need_add_init_factory = true;
        }
        if (need_add_init_factory) {
          const transform::Rigid3d pose = prior_pose_.value().second->pose;
          InitialPoseFactor *f =
              new InitialPoseFactor(100, pose.translation(), pose.rotation());
          problem.AddResidualBlock(f, nullptr, para_Pose[0]);
        }
      }
      prior_pose_.reset();
    }
  }
  VLOG(kGlogCostTimeLevel) << "opti factor costs "
                           << Optimization_result_t_t.toc() << " ms";
  ceres::Solver::Options options;
  options.linear_solver_ordering.reset(ordering);
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.num_threads = 6;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
  // options.dynamic_sparsity =true;
  options.use_explicit_schur_complement = true;
  // options.minimizer_progress_to_stdout = true;
  options.use_nonmonotonic_steps = true;
  // if (marginalization_flag == MARGIN_OLD)
  //   options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
  // else
  //   options.max_solver_time_in_seconds = SOLVER_TIME;
  options.max_num_iterations = 1;
  TicToc t_solver;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  final_cost_ = summary.final_cost;
  LOG_EVERY_N(INFO, 5) << summary.BriefReport();
  LOG_EVERY_N(INFO, 100) << "\n" << summary.FullReport();
  return &data_;
}
Optimization::~Optimization() {
  int win_size = win_size_ + 1;
  //
  for (int i = 0; i < win_size; i++) {
    delete[] data_.pose[i];
    delete[] data_.speed_bias[i];
  }
  delete[] data_.pose;
  delete[] data_.speed_bias;

  for (int i = 0; i < options_.CamNum(); i++) {
    delete[] data_.ex_pose[i];
  }
  delete[] data_.ex_pose;
  //
  for (int i = 0; i < options_.TrackNum(); i++) {
    for (int j = 0; j < kMaxFeatureNum; j++) {
      delete[] data_.feature[i][j];
    }
    delete[] data_.feature[i];
  }
  delete[] data_.feature;
  //
  // CHECK(false);
  delete[] data_.ex_pose_odom[0];
  delete[] data_.ex_pose_odom;
  //

  delete[] data_.td[0];
  delete data_.td;
}
}  // namespace estimator
}  // namespace jarvis