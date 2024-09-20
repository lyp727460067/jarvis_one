#include "jarvis/estimator/slide_window.h"

#include <memory>

#include "marginalization.h"
namespace jarvis {
namespace estimator {
#define para_Pose (opt_data_->pose)
#define para_SpeedBias (opt_data_->speed_bias)
#define para_Ex_Pose (opt_data_->ex_pose)
#define para_Ex_Pose_Odom (opt_data_->ex_pose_odom)
#define para_Td (opt_data_->td)
#define para_Feature (opt_data_->feature)
std::array<int,3> ParaExPoseIndex { 0, 2, 3 };
//
SlideWindow::SlideWindow(const SlideWindowOption& option, DataBase* data_base,
                         const std::unique_ptr<InitializationResult>& init_data)
    : options_(option), data_base_(data_base) {
  //
  CHECK(init_data);
  CHECK_EQ(options_.win_size + 1, init_data->states.size());
  extric_camera_to_imu_ = option.extric_camera_to_imu;
  imu_states_ = init_data->states;
  feature_managers_ = std::make_unique<FeatureManagers>();
  feature_managers_->AddFeatureManger(init_data->cam_id,
                                      init_data->feat_manager);
  //
  //
  integration_base_ = init_data->integration_base;
  //
  for (int i = 0; i < options_.win_size + 1; i++) {
    odoms_factor_.push_back(nullptr);
  }
  //
  if (options_.enable_zero_velocity) {
    update_zero_velocity_ = std::make_unique<UpdataZeroVelocity>(
        options_.updata_zerovelocity_option);
  }
  //
  //
  //
  int camera_num = options_.use_stereo ? 1 : 0;
  camera_num += options_.track_cam_num;
  //
 
  //
  //for init...
  for (int i = 0; i < options_.track_cam_num; i++) {
    init_feature_managers_.emplace(
        i, std::make_shared<FeatureManager>(options_.feature_manager_option));
  }

  //
  OptimizationOption opti_option = option.opti_option;
  opti_option.camera_num = camera_num;
  opti_option.track_cam_num = options_.track_cam_num;
  //
  //
  optimization_ =
      std::make_unique<Optimization>(options_.win_size, opti_option);
  options_.opti_option = opti_option;
  //
  opt_data_ = optimization_->MutableData();

  marginalizer_ = std::make_unique<Marginalization>(MarginalizationOption{
      options_.win_size,     camera_num, options_.track_cam_num, 
      options_.opti_option.camera_weight});
  // /
  last_feature_time_ = init_data->time;
  CHECK_EQ(imu_states_.size(),options_.win_size+1);
  SlideData(true);
}

FrameData SlideWindow::AddFeatureData(const FrameData& frame) {
  //

  //
  const double dt = camera_imu_time_offset_;
  std::string info;
  //
  const int frame_count = imu_states_.size();
  //
  //
  for (auto& f : frame.data->features_datas) {
    if (feature_managers_->Exist(f.first)) {
      LOG(INFO)<<"feature id: "<<f.first;
      feature_managers_->MutableFeatureManager(f.first)
          ->AddFeatureCheckParallax(
              frame_count, frame.data->features_datas[f.first].features, dt);
    }
  }
  //
  bool is_keyframe = feature_managers_->CheckParallax();
  for (auto& f : frame.data->features_datas) {
    if (!feature_managers_->Exist(f.first)) {
      CHECK(init_feature_managers_.count(f.first));
      if (is_keyframe) {
        init_feature_managers_[f.first]->AddFeatureCheckParallax(
            init_feature_managers_[f.first]->FrameCount() + 1,
            frame.data->features_datas[f.first].features, dt);
        if (init_feature_managers_[f.first]->FrameCount() ==
            options_.win_size) {
          std::vector<transform::Rigid3d> triang_pose;
          for (int i = 0; i < imu_states_.size(); i++) {
            triang_pose.push_back(imu_states_[i].Pose());
          }
          //
          
          // init_feature_managers_[f.first]->Triangulate(
          //     options_.win_size, triang_pose,
          //     {extric_camera_to_imu_[ParaExPoseIndex[f.first]]});
          feature_managers_->AddFeatureManger(f.first,
                                              init_feature_managers_[f.first]);
        }
      }
    }
  }
  //
  // bool is_keyframe = feature_manager_->CheckParallax();
  LOG(INFO) << "Add incoming feature "
            << (is_keyframe ? "Keyframe" : "Non-keyframe,");
  //
  const common::Time current_time =
      frame.data->time + common::FromSeconds(camera_imu_time_offset_);
  //
  const std::vector<sensor::ImuData> imu_datas =
      data_base_->GetImuIntervalData(last_feature_time_, current_time);
  //
  imu_states_.push_back(frame.data->imu_state);
  //

  integration_base_.push_back(nullptr);
  if (!imu_datas.empty()) {
    Eigen::Vector3d ba = imu_states_.back().ba;
    Eigen::Vector3d bg = imu_states_.back().bg;
    integration_base_.back() = std::make_shared<IntegrationBase>(
        ImuState{Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                 Eigen::Vector3d::Zero(), ba, bg},
        options_.imu_option, imu_datas);
  };
  //
  //
  odoms_factor_.push_back(
      std::make_shared<OdomFactor>(options_.odom_factor_option, data_base_));
  odoms_factor_.back()->ComputeObserve(last_feature_time_, current_time);
  //

  // frame.data->update_zero_velocity_data = update_zero_velocity_.get();
  // sw_data_.frame_data.emplace_back(mute_frame_data);
  //
  std::vector<transform::Rigid3d> triang_pose;
  for (int i = 0; i < imu_states_.size(); i++) {
    triang_pose.push_back(imu_states_[i].Pose());
  }
  //
  feature_managers_->Triangulate(frame_count,triang_pose,
                                 extric_camera_to_imu_);
  //
  OptimizationData opt_data;
  for (int i = 0; i < options_.win_size + 1; i++) {
    opt_data.odom_factors.push_back(odoms_factor_[i].get());
    opt_data.imu_factors.push_back(integration_base_[i].get());
  }
  //
  opt_data.feat_manager_factors = feature_managers_.get();
  //
  FrameDataToState();
  optimization_->Solve(marginalizer_.get(), &opt_data);
  StateToFrameData();
  {
    FrameDataToState();
    MarginalizationFactorData marg_data;
    for (int i = 0; i < options_.win_size + 1; i++) {
      marg_data.odom_factors.push_back(odoms_factor_[i].get());
      marg_data.imu_factors.push_back(integration_base_[i].get());
    }
    marg_data.feat_manager_factors = feature_managers_.get();
    marginalizer_->Marginalize(opt_data_, &marg_data, !is_keyframe);
  }

  //
  for (int i = 0; i < extric_camera_to_imu_.size(); i++) {
    LOG(INFO) << "extric_camera_to_imu_ " << i << " " <<
        extric_camera_to_imu_[i];
  }
  //
  SlideData(is_keyframe);

  feature_managers_->RemoveFailures();
  //
  //
  for (auto& cam_feature_data : frame.data->features_datas) {
    const CameraId cam_id = cam_feature_data.first;
    FrameData::FeatureData cam_features_data = cam_feature_data.second;
    if(!feature_managers_->Exist(cam_id))continue;
    auto feat_manager = feature_managers_->MutableFeatureManager(cam_id);
    std::set<TrackingId> remove_id;
    for (auto& features : cam_features_data.features.data->features) {
      TrackingId track_id = features.first;
      const double depth = feat_manager->GetDepth(track_id);
      if (depth > 0) {
        cam_features_data.depths[track_id] = depth;
        cam_features_data.key_points[track_id] =
            cv::KeyPoint(features.second.camera_features[0].uv.x(),
                         features.second.camera_features[0].uv.y(), 2);
      } else {
        remove_id.insert(track_id);
      }
    }
  }
  //
  last_feature_time_ = frame.data->time;
  FrameData fram_result = frame;
  //

  fram_result.data->imu_state = imu_states_.back();
  return fram_result;
}
//
void SlideWindow::SlideNew() {
  //
  //
  std::swap(imu_states_[imu_states_.size() - 2], imu_states_.back());
  imu_states_.erase(imu_states_.end());
  integration_base_[imu_states_.size() - 2]->Merge(*integration_base_.back());
  integration_base_.erase(integration_base_.end());
  odoms_factor_[odoms_factor_.size() - 2]->Merge(*odoms_factor_.back());
  odoms_factor_.erase(odoms_factor_.end());
  //
  //
}

void SlideWindow::SlideData(bool is_keyframe) {
  //
  CHECK_EQ(imu_states_.size() - 1, options_.win_size);
  //
  if (is_keyframe) {
    //

    transform::Rigid3d marg_pose = imu_states_[0].Pose();
    imu_states_.erase(imu_states_.begin());
    transform::Rigid3d new_pose = imu_states_[0].Pose();
    //


    for (size_t i = 0; i < options_.opti_option.track_cam_num; i++) {
      //
      if (feature_managers_->Exist(i)) {
        feature_managers_->MutableFeatureManager(i)->RemoveBackShiftDepth(
            marg_pose * extric_camera_to_imu_[ParaExPoseIndex[i]],
            new_pose * extric_camera_to_imu_[ParaExPoseIndex[i]]);
      }
    }

    odoms_factor_.erase(odoms_factor_.begin());
    integration_base_.erase(integration_base_.begin());
    // feature_managers_->RemoveBack();
  } else {
    CHECK(false);
    //
    SlideNew();
    feature_managers_->RemoveFront(options_.win_size);
  }
}

// double Estimator::reprojectionError(Eigen::Matrix3d &Ri, Eigen::Vector3d &Pi,
//                                     Eigen::Matrix3d &rici,
//                                     Eigen::Vector3d &tici, Eigen::Matrix3d &Rj,
//                                     Eigen::Vector3d &Pj, Eigen::Matrix3d &ricj,
//                                     Eigen::Vector3d &ticj, double depth,
//                                     Eigen::Vector3d &uvi,
//                                     Eigen::Vector3d &uvj) {
//   Eigen::Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
//   Eigen::Vector3d pts_cj =
//       ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
//   Eigen::Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
//   double rx = residual.x();
//   double ry = residual.y();
//   return sqrt(rx * rx + ry * ry);
// }

// void Estimator::outliersRejection(std::set<int> &removeIndex,const int convin_used_num) {
//   // return;
//   int feature_index = -1;
//   for (auto &it_per_id : f_manager->feature) {
//     double err = 0;
//     int errCnt = 0;
//     it_per_id.used_num = it_per_id.feature_per_frame.size();
//     if (it_per_id.used_num < convin_used_num) continue;
//     feature_index++;
//     int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
//     Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;
//     double depth = it_per_id.estimated_depth;
//     for (auto &it_per_frame : it_per_id.feature_per_frame) {
//       imu_j++;
//       if (imu_i != imu_j) {
//         Eigen::Vector3d pts_j = it_per_frame.point;
//         double tmp_error =
//             reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], Rs[imu_j],
//                               Ps[imu_j], ric[0], tic[0], depth, pts_i, pts_j);
//         err += tmp_error;
//         errCnt++;
//         // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
//       }
//         // need to rewrite projecton factor.........
//         if (it_per_frame.is_stereo) {
//           Eigen::Vector3d pts_j_right = it_per_frame.pointRight;
//           if (imu_i != imu_j) {
//             double tmp_error = reprojectionError(
//                 Rs[imu_i], Ps[imu_i], ric[0], tic[0], Rs[imu_j], Ps[imu_j],
//                 ric[1], tic[1], depth, pts_i, pts_j_right);
//             err += tmp_error;
//             errCnt++;
//             // LOG(INFO)<<"tmp_error "<< FOCAL_LENGTH / 1.5 * tmp_error;
//           } else {
//             double tmp_error = reprojectionError(
//                 Rs[imu_i], Ps[imu_i], ric[0], tic[0], Rs[imu_j], Ps[imu_j],
//                 ric[1], tic[1], depth, pts_i, pts_j_right);
//             err += tmp_error;
//             errCnt++;
//             // LOG(INFO)<<"tmp_error "<< FOCAL_LENGTH / 1.5 * tmp_error;
//           }
//         }
//     }
//     double ave_err = err / errCnt;
//     if (ave_err * FOCAL_LENGTH > options_.optimazation_outliers_rejection_th ||
//         depth < 0 ||
//         depth > options_.rejection_points_depth_max_th) {
//       removeIndex.insert(it_per_id.feature_id);
//     }
//   }
//   LOG(INFO)<<removeIndex.size();
// }


void SlideWindow::StateToFrameData() {
  //
  const ImuState imu_state0 = imu_states_[0];
  //
  //
  Eigen::Quaterniond rotation0 = imu_state0.q;
  Eigen::Vector3d origin_R0 = Utility::R2ypr(rotation0.toRotationMatrix());
  Eigen::Vector3d origin_P0 = imu_state0.p;
  std::stringstream info;
  // double **pose = data_.pose;
  // double **speed_bias = data_.speed_bias;
  // double **ex_pose = data_.ex_pose;
  // double **ex_pose_odom = data_.ex_pose_odom;
  //
  Eigen::Vector3d origin_R00 =
      Utility::R2ypr(Eigen::Quaterniond(para_Pose[0][6], para_Pose[0][3],
                                        para_Pose[0][4], para_Pose[0][5])
                         .toRotationMatrix());
  double y_diff = origin_R0.x() - origin_R00.x();

  // TODO
  Eigen::Matrix3d rot_diff = Utility::ypr2R(Eigen::Vector3d(y_diff, 0, 0));

  if (abs(abs(origin_R0.y()) - 90) < 1.0 ||
      abs(abs(origin_R00.y()) - 90) < 1.0) {
    VLOG(kGlogLevel) << "euler singular point!";
    rot_diff = rotation0 * Eigen::Quaterniond(para_Pose[0][6], para_Pose[0][3],
                                              para_Pose[0][4], para_Pose[0][5])
                               .toRotationMatrix()
                               .transpose();
  }

  for (int i = 0; i <= options_.win_size; i++) {
    const Eigen::Quaterniond r =
        (Eigen::Quaterniond(rot_diff) *
         Eigen::Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4],
                            para_Pose[i][5]))
            .normalized();

    const Eigen::Vector3d p =
        rot_diff * Eigen::Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                   para_Pose[i][1] - para_Pose[0][1],
                                   para_Pose[i][2] - para_Pose[0][2]) +
        origin_P0;

    //
    ImuState& imu_state = imu_states_[i];
    imu_state.p = p;
    imu_state.q = r;
    imu_state.v =
        rot_diff * Eigen::Vector3d(para_SpeedBias[i][0], para_SpeedBias[i][1],
                                   para_SpeedBias[i][2]);
    imu_state.ba = Eigen::Vector3d(para_SpeedBias[i][3], para_SpeedBias[i][4],
                                   para_SpeedBias[i][5]);

    imu_state.bg = Eigen::Vector3d(para_SpeedBias[i][6], para_SpeedBias[i][7],
                                   para_SpeedBias[i][8]);
    // LOG(INFO) << imu_state;
  }
  //

  for (int i = 0; i < options_.opti_option.camera_num; i++) {
    const Eigen::Vector3d t = Eigen::Vector3d(
        para_Ex_Pose[i][0], para_Ex_Pose[i][1], para_Ex_Pose[i][2]);
    const Eigen::Quaterniond q =
        Eigen::Quaterniond(para_Ex_Pose[i][6], para_Ex_Pose[i][3],
                           para_Ex_Pose[i][4], para_Ex_Pose[i][5])
            .normalized();
    extric_camera_to_imu_[i] = transform::Rigid3d(t, q);
    // LOG(INFO) << extric_camera_to_imu_[i];
  }
  if (options_.opti_option.use_odom) {
    odo_to_imu_extric_ = transform::Rigid3d(
        Eigen::Vector3d(para_Ex_Pose_Odom[0][0], para_Ex_Pose_Odom[0][1],
                        para_Ex_Pose_Odom[0][2]),
        Eigen::Quaterniond(para_Ex_Pose_Odom[0][6], para_Ex_Pose_Odom[0][3],
                           para_Ex_Pose_Odom[0][4], para_Ex_Pose_Odom[0][5])
            .normalized());
  }
  //
  //
  for (size_t i = 0; i < options_.opti_option.track_cam_num; i++) {
    //
    if (feature_managers_->Exist(i)) {
      int feat_manger_depth_lenth =
          feature_managers_->MutableFeatureManager(i)->GetFeatureCount();
      std::vector<double> dephts(feat_manger_depth_lenth);
      for (int j = 0; j < feat_manger_depth_lenth; j++) {
        dephts[j] = para_Feature[i][j][0];
      }
      feature_managers_->MutableFeatureManager(i)->SetDepth(dephts);
    }
  }
  //
  camera_imu_time_offset_ = para_Td[0][0];
}

void SlideWindow::FrameDataToState() {
  //
  CHECK_EQ(imu_states_.size() - 1, options_.win_size);
  for (int i = 0; i <= options_.win_size; i++) {
    ImuState& imu_state = imu_states_[i];
    for (int j = 0; j < 3; j++) {
      para_Pose[i][j] = imu_state.p(j);
    }
    const Eigen::Quaterniond q = imu_state.q;
    para_Pose[i][3] = q.x();
    para_Pose[i][4] = q.y();
    para_Pose[i][5] = q.z();
    para_Pose[i][6] = q.w();
    for (int j = 0; j < 3; j++) {
      para_SpeedBias[i][j] = imu_state.v[j];
      para_SpeedBias[i][j + 3] = imu_state.ba[j];
      para_SpeedBias[i][j + 6] = imu_state.bg[j];
    }
  }
  if (options_.opti_option.use_odom) {
    for (int j = 0; j < 3; j++) {
      para_Ex_Pose_Odom[0][j] = odo_to_imu_extric_.translation()[j];
      LOG(INFO) << para_Ex_Pose_Odom[0][j];
    }
    para_Ex_Pose_Odom[0][3] = odo_to_imu_extric_.rotation().x();
    para_Ex_Pose_Odom[0][4] = odo_to_imu_extric_.rotation().y();
    para_Ex_Pose_Odom[0][5] = odo_to_imu_extric_.rotation().z();
    para_Ex_Pose_Odom[0][6] = odo_to_imu_extric_.rotation().w();
  }
  //
  //
  // CHECK_EQ(extric_camera_to_imu_.size(), options_.opti_option.camera_num);
  LOG(INFO)<<options_.opti_option.camera_num;
  for (int i = 0; i < options_.opti_option.camera_num; i++) {
    for (int j = 0; j < 3; j++) {
      para_Ex_Pose[i][j] = extric_camera_to_imu_[i].translation()[j];
    }
    //
    const Eigen::Quaterniond q = extric_camera_to_imu_[i].rotation();
    para_Ex_Pose[i][3] = q.x();
    para_Ex_Pose[i][4] = q.y();
    para_Ex_Pose[i][5] = q.z();
    para_Ex_Pose[i][6] = q.w();
    // if (IsStereo()) break;
    LOG(INFO)<<extric_camera_to_imu_[i];
  }
  
  for (size_t i = 0; i < options_.opti_option.track_cam_num; i++) {
    //
    if (feature_managers_->Exist(i)) {
      std::vector<double> dephts =
          feature_managers_->MutableFeatureManager(i)->GetDepthVector();
      for (int j = 0; j < dephts.size(); j++) {
        para_Feature[i][j][0] = dephts[j];
      }
    }
  }
  para_Td[0][0] = camera_imu_time_offset_;
}

}  // namespace estimator
}  // namespace jarvis