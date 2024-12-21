#include "jarvis/estimator/initial/initialization_stero_imu.h"
#include "jarvis/estimator/factor/projectionOneFrameTwoCamFactor.h"
#include "jarvis/estimator/factor/projectionTwoFrameOneCamFactor.h"
#include "jarvis/estimator/factor/projectionTwoFrameTwoCamFactor.h"
#include "ceres/problem.h"
#include "jarvis/estimator/factor/imu_factor.h"
#include "jarvis/estimator/factor/pose_local_parameterization.h"
#include "jarvis/estimator/factor/projectionTwoFrameOneCamFactor.h"
#include "key_frame_data.h"
namespace jarvis {
namespace estimator {
constexpr int KDataBaseLenth = 2;
//

SteroImuInitialization::SteroImuInitialization(
    const SteroImuInitializationOption& option, DataBase* data_base)
    : InitializationImu(data_base), options_(option),init_bgs_(0,0,0) {
  //
  CHECK_EQ(option.extric_camera_to_imu.size(), size_t(2));
  //
  for (size_t i = 0; i < options_.extric_camera_to_imu.size(); i++) {
    LOG(INFO) << "cam to imu " << options_.extric_camera_to_imu[i];
  }
  //
  feature_manager_ =
      std::make_shared<FeatureManager>(options_.feature_manager_option);
  // /
  initial_alignment_ =
      std::make_unique<InitialAlignment>(InitialAlignmentOption{});
  // OptimizationOption opti_optio = options_.opti_option;
  // opti_optio.camera_num = 2;
  // opti_optio.convin_used_num =
  // options_.feature_manager_option.convin_used_num;
  // ///
  // optimization_ = std::make_unique<Optimization>(options_.sw_size,
  // opti_optio);
  LOG(INFO) << "Init with Stero";
  initial_ex_rotation_ =  std::make_unique<InitialEXRotation>();
}
//

void SteroImuInitialization::Reset() {
  feature_manager_ =
      std::make_shared<FeatureManager>(options_.feature_manager_option);
  init_pnp_states_.clear();
  sw_pose_.clear();
  image_frames_.clear();
  integration_bases_.clear();
  frames_continuously_track_num.clear();
  init_bgs_.setZero();
  init_imu_rotation_.reset();
}

//
void SteroImuInitialization::RemoveBack() {
  if (image_frames_.empty()) return;
  image_frames_.erase(image_frames_.begin());
  //

  transform::Rigid3d marg_pose = sw_pose_[0] * options_.extric_camera_to_imu[0];
  sw_pose_.erase(sw_pose_.begin());
  init_pnp_states_.erase(init_pnp_states_.begin());
  //
  transform::Rigid3d new_pose = sw_pose_[0] * options_.extric_camera_to_imu[0];
  feature_manager_->RemoveBackShiftDepth(marg_pose, new_pose);
  integration_bases_.erase(integration_bases_.begin());

  // feature_manager_->RemoveBack();
}
//
std::array<double, 7> PoseToAarr(const transform::Rigid3d& pose) {
  return {pose.translation().x(), pose.translation().y(),
          pose.translation().z(), pose.rotation().x(),
          pose.rotation().y(),    pose.rotation().z(),
          pose.rotation().w()};
}

std::unique_ptr<InitializationResult>
SteroImuInitialization::OptimizationResult() {
  //
  std::array<double, 7> para_pose[options_.sw_size + 1];
  std::array<double, 9> para_speed[options_.sw_size + 1];
  std::array<double, 7> para_ex_pose[options_.extric_camera_to_imu.size()];
  //
  //
  for (int i = 0; i <= options_.sw_size; i++) {
    para_pose[i] = (std::array<double, 7>{PoseToAarr(sw_pose_[i])});
    para_speed[i] = std::array<double, 9>(
        {0, 0, 0, 0, 0, 0, init_bgs_.x(), init_bgs_.y(), init_bgs_.z()});
  }
  std::vector<double> para_depth = feature_manager_->GetDepthVector();
  // double para_depth[depth.size()];
  // for (int i = 0; i < depth.size(); i++) {
  //   para_depth[i] = depth[i];
  //   LOG(INFO)<<para_depth[i];
  // }
  //
  for (size_t i = 0; i < options_.extric_camera_to_imu.size(); i++) {
    para_ex_pose[i] = (PoseToAarr(options_.extric_camera_to_imu[i]));
  }
  double para_dt = 0;
  ceres::Problem problem;
  ceres::LossFunction* loss_function;
  // loss_function = NULL;
  loss_function = new ceres::HuberLoss(1.0);

  for (int i = 0; i < options_.sw_size + 1; i++) {
    ceres::LocalParameterization* local_parameterization =
        new PoseLocalParameterization();
    problem.AddParameterBlock(para_pose[i].data(), SIZE_POSE,
                              local_parameterization);
    problem.AddParameterBlock(para_speed[i].data(), SIZE_SPEEDBIAS);
    for (int j = 0; j < 6; j++) {
      problem.SetParameterLowerBound(para_speed[i].data(), j + 3, -1);
      problem.SetParameterUpperBound(para_speed[i].data(), j + 3, 1);
    }

  }
  ceres::LocalParameterization* local_parameterization =
      new PoseLocalParameterization();

  problem.AddParameterBlock(para_ex_pose[0].data(), SIZE_POSE,
                            local_parameterization);
  problem.AddParameterBlock(para_ex_pose[1].data(), SIZE_POSE,
                            local_parameterization);

  //
  // for (size_t i = 0; i < options_.opti_option.trace_sequence.size(); i++) {
  //   ceres::LocalParameterization* local_parameterization =
  //       new PoseLocalParameterization();

  //   problem.AddParameterBlock(para_ex_pose[i].data(), SIZE_POSE,
  //                             local_parameterization);

  //     LOG(INFO)<<options_.opti_option.estimate_extrinsic;
  //   if (options_.opti_option.estimate_extrinsic == 0) {
  //     LOG(INFO)<<options_.opti_option.estimate_extrinsic;
  //     problem.SetParameterBlockConstant(para_ex_pose[i].data());
  //   }
  // }
  //

  // 固定第一帧位姿和相机相对IMU外参
  problem.SetParameterBlockConstant(para_pose[0].data());
  problem.SetParameterBlockConstant(para_ex_pose[0].data());
  problem.SetParameterBlockConstant(para_ex_pose[1].data());
  
  problem.AddParameterBlock(&para_dt, 1);
  problem.SetParameterBlockConstant(&para_dt);
  for (int i = 0; i < options_.sw_size; i++) {
    int j = i + 1;
    if (!integration_bases_[j]->IsValid()) {
      LOG(WARNING) << j << " Imu avalid..";
      continue;
    }
    LOG(INFO) << integration_bases_[j]->delta_p.transpose();
    IMUFactor* imu_factor = new IMUFactor(integration_bases_[j].get());
    //
    problem.AddResidualBlock(imu_factor, NULL, para_pose[i].data(),
                             para_speed[i].data(), para_pose[j].data(),
                             para_speed[j].data());
  }
  //
  std::stringstream info1;
  const double cam_weight = options_.opti_option.camera_weight;
  feature_manager_->CreateFactor(
      [&](const Eigen::Vector3d& pts_i, const Eigen::Vector3d& pts_j,
          const Eigen::Vector2d& imu_i_velocity,
          const Eigen::Vector2d& imu_j_velocity, const double td_i,
          const double td_j, const std::tuple<int, int, int>& index) {
        //
        ProjectionTwoFrameOneCamFactor* f_td =
            new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, imu_i_velocity,
                                               imu_j_velocity, td_i, td_j,
                                               cam_weight);
        //
        // info1 << "[" << std::get<0>(index) << std::get<1>(index)
        //       << std::get<2>(index) << "]" << pts_i.transpose() << " "
        //       << pts_j.transpose() << imu_i_velocity.transpose()
        //       << imu_j_velocity.transpose() << td_i << td_j << cam_weight
        //       << "\n";
        // ///
        // for (int i = 0; i < 7; i++) {
        //   info1 << para_pose[std::get<0>(index)][i] << " ";
        //   info1 << para_pose[std::get<1>(index)][i] << " ";
        //   info1 << para_ex_pose[0][i] << "\n";
        // }
        // info1 << para_depth[std::get<2>(index)] << "\n";
       
        //
        problem.AddResidualBlock(
            f_td, loss_function, para_pose[std::get<0>(index)].data(),
            para_pose[std::get<1>(index)].data(), para_ex_pose[0].data(),
            &para_depth[std::get<2>(index)], &para_dt);

        // problem.SetParameterBlockConstant(&para_depth[std::get<2>(index)]);
      },
      [&](const Eigen::Vector3d& pts_i, const Eigen::Vector3d& pts_j,
          const Eigen::Vector2d& velocity_i, const Eigen::Vector2d& velocity_j,
          const double td_i, const double td_j,
          const std::tuple<int, int, int>& index) {
        //
        ProjectionTwoFrameTwoCamFactor* f = new ProjectionTwoFrameTwoCamFactor(
            pts_i, pts_j, velocity_i, velocity_j, td_i, td_j, cam_weight);
        problem.AddResidualBlock(
            f, loss_function, para_pose[std::get<0>(index)].data(),
            para_pose[std::get<1>(index)].data(), para_ex_pose[0].data(),
            para_ex_pose[1].data(), &para_depth[std::get<2>(index)], &para_dt);
      },
      [&](const Eigen::Vector3d& pts_i, const Eigen::Vector3d& pts_j,
          const Eigen::Vector2d& velocity_i, const Eigen::Vector2d& velocity_j,
          const double td_i, const double td_j,
          const std::tuple<int, int, int>& index) {
        //

        ProjectionOneFrameTwoCamFactor* f = new ProjectionOneFrameTwoCamFactor(
            pts_i, pts_j, velocity_i, velocity_j, td_i, td_j, cam_weight);

        problem.AddResidualBlock(f, loss_function, para_ex_pose[0].data(),
                                 para_ex_pose[1].data(),
                                 &para_depth[std::get<2>(index)], &para_dt);
      });
  //
  // std::cout<<info1.str()<<std::endl;

  //

  ceres::Solver::Options options;
  // options.linear_solver_ordering.reset(ordering);
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.num_threads = 4;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
  options.use_explicit_schur_complement = true;
  options.use_nonmonotonic_steps = true;
  // options.max_solver_time_in_seconds =
  //     options_.opti_option.max_solver_time * 4.0 / 5.0;
  options.max_num_iterations = options_.opti_option.max_num_iterations;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  Eigen::Vector3d bas =
      Eigen::Vector3d(para_speed[0][3], para_speed[0][4], para_speed[0][5]);
  //
  Eigen::Vector3d bgs =
      Eigen::Vector3d(para_speed[0][6], para_speed[0][7], para_speed[0][8]);
  std::vector<transform::Rigid3d> extric_camera_to_imu;
  //
  for (int i = 0; i < camera_num_; i++) {
    LOG(INFO) << para_ex_pose[i][0] << " " << para_ex_pose[i][1] << " "
              << para_ex_pose[i][2];
    extric_camera_to_imu.push_back(transform::Rigid3d(
        Eigen::Vector3d(para_ex_pose[i][0], para_ex_pose[i][1],
                        para_ex_pose[i][2]),
        Eigen::Quaterniond(para_ex_pose[i][6], para_ex_pose[i][3],
                           para_ex_pose[i][4], para_ex_pose[i][5])));
  }

  std::stringstream info;
  info << "Init optimization info:\n";
  std::vector<ImuState> imu_state;
  double max_velocity = .0;
  for (int i = 0; i < options_.sw_size + 1; i++) {
    imu_state.push_back(ImuState{
        Eigen::Vector3d(para_pose[i][0], para_pose[i][1], para_pose[i][2]),
        Eigen::Quaterniond(para_pose[i][6], para_pose[i][3], para_pose[i][4],
                           para_pose[i][5]),
        Eigen::Vector3d(para_speed[i][0], para_speed[i][1], para_speed[i][2]),
        Eigen::Vector3d(para_speed[i][3], para_speed[i][4], para_speed[i][5]),
        Eigen::Vector3d(para_speed[i][6], para_speed[i][7], para_speed[i][8])});
    imu_state.back().time = image_frames_[i].time;

    Eigen::Vector3d vi(para_speed[i][0], para_speed[i][1], para_speed[i][2]);
    double vin = vi.norm();
    if (vin > max_velocity) max_velocity = vin;
  }

  for (size_t i = 0; i < imu_state.size(); i++) {
    info << "state " << std::to_string(i) << imu_state[i] << "\n";
  }
  for (int i = 0; i < camera_num_; i++) {
    info << "cam " << std::to_string(i) << extric_camera_to_imu[i] << "\n";
  }
  info << "max velocity: " << std::to_string(max_velocity) << "\n";

  if (bgs.norm() > options_.init_bg_th || bas.norm() > options_.init_ba_th) {
    info << "---------opitimization bias error--------------";
    LOG(ERROR) << info.str();
    return nullptr;
  }
  if (max_velocity > options_.init_v_th){
    info << "---------opitimization velocity error--------------";
    LOG(ERROR) << info.str();
    return nullptr;
  }

  feature_manager_->SetDepth(para_depth);
  info << "---------opitimization bias ok--------------";
  LOG(INFO) << info.str();
  LOG(INFO) << "\n" << summary.FullReport();
  // feature_manager_->RemoveFailures();
  //
  // RemoveBack();
  // imu_state.erase(imu_state.begin());
  return std::make_unique<InitializationResult>(InitializationResult{
      0, std::move(imu_state), feature_manager_, extric_camera_to_imu});
};

std::unique_ptr<InitializationResult> SteroImuInitialization::AddFeatureData(
    const ImageFeatureTrackerData& track_frame) {
  //
  const common::Time& cur_time = track_frame.data->time;
  if (!init_imu_rotation_.has_value()) {
    auto init_rotation = InitImuRotaion(cur_time);
    if (init_rotation) {
      init_imu_rotation_ = *init_rotation;
      // LOG(INFO) << "Init roation imu:" << *init_imu_rotation_;
    } else {
      last_time = cur_time;
      return nullptr;
    }
  }

  const int frame_count = image_frames_.size();
  //
  feature_manager_->AddFeatureCheckParallax(image_frames_.size(), track_frame,
                                            0);

  LOG(INFO) << "Stero init frame: " << frame_count
            << "new feature count:" << feature_manager_->GetFeatureCount();
  //
  // frames_continuously_track_num.push_back();
  //
  //
  //




  //
  if (sw_pose_.empty()) {
    sw_pose_.push_back(
        transform::Rigid3d::Rotation(init_imu_rotation_.value()));
  } else {
    sw_pose_.push_back(sw_pose_.back());
  }
  //
  bool pnp_state = feature_manager_->InitFramePoseByPnP(
                       frame_count, options_.extric_camera_to_imu, sw_pose_);
  //
  if (sw_pose_.back().translation().norm() > 2) {
    pnp_state = false;
  }
  init_pnp_states_.push_back(pnp_state);
  feature_manager_->Triangulate(frame_count, sw_pose_,
                                options_.extric_camera_to_imu);

  //
  //
  
  const auto imu_datas = data_base_->GetImuIntervalData(last_time, cur_time);
  //
  //
  LOG(INFO) << "image interval [" << last_time << "," << cur_time
            << ",peri: " << common::ToSeconds(cur_time - last_time)
            << "],imu num: " << imu_datas.size();

  integration_bases_.push_back(nullptr);
  if (!imu_datas.empty()) {
    integration_bases_.back() = std::make_shared<IntegrationBase>(
        ImuState{}, options_.imu_option, imu_datas);
  }
  //

  // if (frame_count != 0) {
  //     LOG(INFO) << "calibrating extrinsic param, rotation movement is needed";
  //     std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres =
  //         feature_manager_->GetCorresponding(frame_count - 1, frame_count);
  //     Eigen::Matrix3d calib_ric;
  //     if (initial_ex_rotation_->CalibrationExRotation(
  //             corres, integration_bases_.back()->delta_q, calib_ric)) {
  //       LOG(WARNING) << "initial extrinsic rotation calib success";
  //       LOG(WARNING) << "initial extrinsic rotation: "
  //                    << Eigen::Quaterniond(calib_ric)
  //                    << options_.extric_camera_to_imu[0].rotation();
  //       // RIC[0] = calib_ric;
  //     }
  //   }


  image_frames_.emplace_back(ImageFrame{cur_time, sw_pose_.back(), track_frame,
                                        integration_bases_.back().get()});
  //
  //
  if (frame_count == options_.sw_size) {
    if ((std::count(init_pnp_states_.begin(), init_pnp_states_.end(), true) ==
         options_.sw_size + 1)) {
      const Eigen::Vector3d bgs =
          initial_alignment_->SolveGyroscopeBias(image_frames_);
      //

      if (bgs.norm() < options_.init_bg_th) {
        init_bgs_ += bgs;  ///??????
        for (size_t i = 0; i < image_frames_.size(); i++) {
          if (image_frames_[i].pre_integration) {
            image_frames_[i].pre_integration->repropagate(
                Eigen::Vector3d::Zero(), init_bgs_);
          }
        }
        LOG(INFO) << "init bgs finish: " << bgs.transpose();

        std::unique_ptr<InitializationResult> resut = OptimizationResult();
        if (resut) {
          resut->time = track_frame.data->time;
          //
          for (size_t i = 0; i < integration_bases_.size(); i++) {
            if (integration_bases_[i]) {
              integration_bases_[i]->repropagate(resut->states.back().ba,
                                                 resut->states.back().bg);
            }
          }
          resut->integration_base = std::move(integration_bases_);
          LOG(INFO) << "optimization done.";
          return resut;
        }
      }

      Reset();
    }
    Reset();
    //RemoveBack();
  }
  last_time = cur_time;
  return nullptr;
}

}  // namespace estimator
}  // namespace jarvis
