#include "marginalization.h"

#include "jarvis/estimator/factor/imu_factor.h"
#include "jarvis/estimator/factor/pose_local_parameterization.h"
#include "jarvis/estimator/factor/projectionOneFrameTwoCamFactor.h"
#include "jarvis/estimator/factor/projectionTwoFrameOneCamFactor.h"
#include "jarvis/estimator/factor/projectionTwoFrameTwoCamFactor.h"
namespace jarvis {
namespace estimator {

namespace {
#define para_Pose opt_data->pose
#define para_SpeedBias opt_data->speed_bias
#define para_Ex_Pose opt_data->ex_pose
#define para_Ex_Pose_Odom opt_data->ex_pose_odom
#define para_Td opt_data->td
#define para_Feature opt_data->feature
// std::array<int,3> ParaExPoseIndex { 0, 2, 3 };
}  // namespace
//
void Marginalization::MergeFrameData(const OptimizationStateData *opt_data,
                                     MarginalizationFactorData *frame_data,
                                     MarginalizationInfo *margina_info) {
  //
  //
  //
  CHECK_EQ(int(frame_data->odom_factors.size()), options_.win_size + 1);
  CHECK_EQ(int(frame_data->imu_factors.size()), options_.win_size + 1);
  if (options_.use_odom && frame_data->odom_factors[1]) {
    ceres::CostFunction *cost_function =
        frame_data->odom_factors[1]->CostFunction();
    if (cost_function) {
      ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
          cost_function, NULL,
          std::vector<double *>{para_Pose[0], para_Pose[1],
                                para_Ex_Pose_Odom[0]},
          std::vector<int>{0});
      margina_info->addResidualBlockInfo(residual_block_info);
    }
  }

  // /
  auto &pre_integration = frame_data->imu_factors[1];
  if (pre_integration && pre_integration->IsValid()) {
    IMUFactor *imu_factor = new IMUFactor(pre_integration);
    CHECK(para_Pose[0]);
    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
        imu_factor, NULL,
        std::vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1],
                              para_SpeedBias[1]},
        std::vector<int>{0, 1});
    margina_info->addResidualBlockInfo(residual_block_info);
  }

  // if (update_zero_velocity_) {
  //   if (is_velocity_updates_[1]) {
  //     ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
  //         update_zero_velocity_->CostFunction(), NULL,
  //         std::vector<double *>{para_Pose[0], para_Pose[1],
  //                               para_SpeedBias[0]},
  //         std::vector<int>{0,2});
  //     marginalization_info->addResidualBlockInfo(residual_block_info);
  //   }
  // }
}

void Marginalization::MergeCameraData(int id,
                                      const OptimizationStateData *opt_data,
                                      FeatureManager *feature_manager,
                                      MarginalizationInfo *margina_info,
                                      ceres::LossFunction *loss_function) {
  //
  const double cam_weight = options_.camera_weight;
  feature_manager->CreateFactor(
      [&](const Eigen::Vector3d &pts_i, const Eigen::Vector3d &pts_j,
          const Eigen::Vector2d &imu_i_velocity,
          const Eigen::Vector2d &imu_j_velocity, const double td_i,
          const double td_j, const std::tuple<int, int, int> &index) {
        //
        if (std::get<0>(index) == 0) {
          ProjectionTwoFrameOneCamFactor *f_td =
              new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, imu_i_velocity,
                                                 imu_j_velocity, td_i, td_j,
                                                 cam_weight);

          ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
              f_td, loss_function,
              std::vector<double *>{
                  para_Pose[std::get<0>(index)], para_Pose[std::get<1>(index)],
                  para_Ex_Pose[options_.trace_sequence[id][0]],
                  para_Feature[id][std::get<2>(index)], para_Td[0]},
              std::vector<int>{0, 3});
          margina_info->addResidualBlockInfo(residual_block_info);
        }
      },

      [&](const Eigen::Vector3d &pts_i, const Eigen::Vector3d &pts_j,
          const Eigen::Vector2d &velocity_i, const Eigen::Vector2d &velocity_j,
          const double td_i, const double td_j,
          const std::tuple<int, int, int> &index) {
        //
        ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(
            pts_i, pts_j, velocity_i, velocity_j, td_i, td_j,cam_weight);

        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
            f, loss_function,
            std::vector<double *>{
                para_Pose[std::get<0>(index)], para_Pose[std::get<1>(index)],
                para_Ex_Pose[options_.trace_sequence[id][0]],
                para_Ex_Pose[options_.trace_sequence[id][1]],
                para_Feature[id][std::get<2>(index)], para_Td[0]},
            std::vector<int>{0, 4});
        margina_info->addResidualBlockInfo(residual_block_info);
      },
      [&](const Eigen::Vector3d &pts_i, const Eigen::Vector3d &pts_j,
          const Eigen::Vector2d &velocity_i, const Eigen::Vector2d &velocity_j,
          const double td_i, const double td_j,
          const std::tuple<int, int, int> &index) {
        //
        ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(
            pts_i, pts_j, velocity_i, velocity_j, td_i, td_j,cam_weight);

        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
            f, loss_function,
            std::vector<double *>{para_Ex_Pose[options_.trace_sequence[id][0]],
                                  para_Ex_Pose[options_.trace_sequence[id][1]],
                                  para_Feature[id][std::get<2>(index)],
                                  para_Td[0]},
            std::vector<int>{2});
        margina_info->addResidualBlockInfo(residual_block_info);
      }

  );
}

//
//
std::unordered_map<long, double *> Marginalization::ShiftStateAdrrOld(
    const OptimizationStateData *opt_data) {
  std::unordered_map<long, double *> addr_shift;
  //
  for (int i = 1; i <= options_.win_size; i++) {
    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] =
        para_SpeedBias[i - 1];
  }
  //
  for (int i = 0; i < options_.camera_num; i++) {
    addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
  }
  addr_shift[reinterpret_cast<long>(para_Ex_Pose_Odom[0])] =
      para_Ex_Pose_Odom[0];
  addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

  return addr_shift;
}
//
//
std::unordered_map<long, double *> Marginalization::ShiftStateAdrrNew(
    const OptimizationStateData *opt_data) {
  std::unordered_map<long, double *> addr_shift;
  for (int i = 0; i <= options_.win_size; i++) {
    if (i == options_.win_size - 1){
      continue;
    }
    else if (i == options_.win_size) {
      addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
      addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
    } else {
      addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
      addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
    }
  }
  for (int i = 0; i < options_.camera_num; i++) {
    addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
  }
  addr_shift[reinterpret_cast<long>(para_Ex_Pose_Odom[0])] =
      para_Ex_Pose_Odom[0];
  addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
  return addr_shift;
}
//
void Marginalization::Marginalize(const OptimizationStateData *opt_data,
                                  MarginalizationFactorData *data, bool flag) {
   TicToc factor_pre_margin;
  std::unique_ptr<ceres::LossFunction> loss_function(
      new ceres::HuberLoss(options_.huber_loss));
  std::unique_ptr<MarginalizationInfo> marginalization_info(
      new MarginalizationInfo());
  bool margina_valid = false; 
  if (!flag) {
    if (last_marginalization_info_ && last_marginalization_info_->valid) {
      std::vector<int> drop_set;
      for (int i = 0;
           i < static_cast<int>(last_marginalization_parameter_blocks_.size());
           i++) {
        if (last_marginalization_parameter_blocks_[i] == para_Pose[0] ||
            last_marginalization_parameter_blocks_[i] == para_SpeedBias[0])
          drop_set.push_back(i);
      }
      // construct new marginlization_factor
      MarginalizationFactor *marginalization_factor =
          new MarginalizationFactor(last_marginalization_info_.get());
      ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
          marginalization_factor, NULL, last_marginalization_parameter_blocks_,
          drop_set);
      marginalization_info->addResidualBlockInfo(residual_block_info);
    }
    for (size_t i = 0; i < options_.trace_sequence.size(); i++) {
      if (data->feat_manager_factors->Exist(i)) {
        MergeCameraData(
            i, opt_data,
            data->feat_manager_factors->MutableFeatureManager(i).get(),
            marginalization_info.get(), loss_function.get());
      }
    }
    MergeFrameData(opt_data, data, marginalization_info.get());
    margina_valid = true;
  } else {
    if (last_marginalization_info_ &&
        std::count(std::begin(last_marginalization_parameter_blocks_),
                   std::end(last_marginalization_parameter_blocks_),
                   para_Pose[options_.win_size - 1])) {
      //
      if (last_marginalization_info_ && last_marginalization_info_->valid) {
        std::vector<int> drop_set;
        for (int i = 0; i < static_cast<int>(
                                last_marginalization_parameter_blocks_.size());
             i++) {
          CHECK(last_marginalization_parameter_blocks_[i] !=
                para_SpeedBias[options_.win_size - 1]);
          if (last_marginalization_parameter_blocks_[i] ==
              para_Pose[options_.win_size - 1]) {
            drop_set.push_back(i);

          }
        }
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor =
            new MarginalizationFactor(last_marginalization_info_.get());
        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
            marginalization_factor, NULL,
            last_marginalization_parameter_blocks_, drop_set);

        marginalization_info->addResidualBlockInfo(residual_block_info);
      }
      margina_valid = true;
    }
  }
   VLOG(kGlogCostTimeLevel) << "marginalization factor costs " << factor_pre_margin.toc();
  if (margina_valid) {
    TicToc t_pre_margin;
    marginalization_info->preMarginalize();
    VLOG(kGlogCostTimeLevel) << "pre marginalization " << t_pre_margin.toc();

    TicToc t_margin;
    marginalization_info->marginalize();
    VLOG(kGlogCostTimeLevel) << "marginalization " << t_margin.toc();
    std::unordered_map<long, double *> addr_shift;
    if (!flag) {
      addr_shift = ShiftStateAdrrOld(opt_data);
    } else {
      addr_shift = ShiftStateAdrrNew(opt_data);
    }
    last_marginalization_parameter_blocks_ =
        marginalization_info->getParameterBlocks(addr_shift);

    last_marginalization_info_ = std::move(marginalization_info);
  }
}

//
void Marginalization::AddToProblem(ceres::Problem *problem,
                                   ceres::LossFunction *loss_function) const {
  if (last_marginalization_info_ && last_marginalization_info_->valid) {
    // construct new marginlization_factor

    MarginalizationFactor *marginalization_factor =
        new MarginalizationFactor(last_marginalization_info_.get());
    problem->AddResidualBlock(marginalization_factor, loss_function,
                              last_marginalization_parameter_blocks_);
  }
}

}  // namespace estimator
}  // namespace jarvis