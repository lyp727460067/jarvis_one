#include "jarvis/estimator/slide_window.h"

#include <memory>
#include <sstream>
#include "marginalization.h"
namespace jarvis {
namespace estimator {
#define para_Pose (opt_data_->pose)
#define para_SpeedBias (opt_data_->speed_bias)
#define para_Ex_Pose (opt_data_->ex_pose)
#define para_Ex_Pose_Odom (opt_data_->ex_pose_odom)
#define para_Td (opt_data_->td)
#define para_Feature (opt_data_->feature)
// std::array<int,3> ParaExPoseIndex { 0, 2, 3 };
//
SlideWindow::SlideWindow(const SlideWindowOption& option, DataBase* data_base,
                         const std::unique_ptr<InitializationResult>& init_data,
                         PriorFactorFunction factor)
    : options_(option),
      data_base_(data_base),
      prior_factor_(factor) {
  //
  CHECK(init_data);
  CHECK_EQ(options_.win_size + 1, int(init_data->states.size()));
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
  odo_to_imu_extric_ = transform::Rigid3d::Identity();
  //
  //

  //
  // for init...
  for (size_t i = 0; i < options_.track_sequence.size(); i++) {
    init_feature_managers_.emplace(
        i, std::make_shared<FeatureManager>(options_.feature_manager_option));
  }

  //
  OptimizationOption opti_option = option.opti_option;
  opti_option.trace_sequence = options_.track_sequence;
  // opti_option.track_cam_num = options_.track_cam_num;
  opti_option.extric_camera_to_imu = options_.extric_camera_to_imu;
  optimization_ =
      std::make_unique<Optimization>(options_.win_size, opti_option);
  options_.opti_option = opti_option;
  //
  opt_data_ = optimization_->MutableData();

  marginalizer_ = std::make_unique<Marginalization>(MarginalizationOption{
      options_.win_size, options_.track_sequence,
      options_.opti_option.camera_weight, options_.opti_option.CamNum(),
      options_.opti_option.use_odom, options_.opti_option.huber_loss});
  // /
  last_feature_time_ = init_data->time;
  CHECK_EQ(int(imu_states_.size()), options_.win_size + 1);
  SlideData(true);
  // feature_managers_->RemoveOutliersRejection(
  //       imu_states_, extric_camera_to_imu_);
  camera_imu_time_offset_ =  options_.camera_imu_time_offset;
}
//
TrackingData SlideWindow::GetratePriorData(bool generate_point, int k) {
  if (images_.count(imu_states_[k].time) == 0) return {};
  TrackingData result;
  result.data = std::make_shared<TrackingData::Data>();
  result.data->imu_state = imu_states_[k];
  result.data->time = imu_states_[k].time;
  // result.data->extric_camera_to_imu = options_.extric_camera_to_imu;
  result.data->extric_camera_to_imu = extric_camera_to_imu_;
  result.data->images = images_[imu_states_[k].time];
  if (generate_point) {
    auto const feature_managers = feature_managers_->GetFeatureManagers();
    for (auto const &f_manger : feature_managers) {
      auto feat_ids = f_manger.second->GetBack();
      if (feat_ids.empty()) continue;
      result.data->features_datas[f_manger.first].features.data =
          std::make_shared<ImageFeatureTrackerData::Data>();
      //
      for (const auto& feat_id : feat_ids) {
        const auto feat_data = f_manger.second->Features().at(feat_id);

        const double depth = f_manger.second->GetDepth(feat_id);
        // if(depth <0)continue;
        result.data->features_datas[f_manger.first]
            .features.data->features[feat_id] =
            feat_data.feature_per_frame[0].feature;

        //
        const Eigen::Vector3d cam_map_point = feat_data.feature_per_frame[0]
                                                  .feature.camera_features[0]
                                                  .normal_points *
                                              depth;
        result.data->features_datas[f_manger.first].map_points[feat_id] =
            imu_states_[0].Pose() *
            extric_camera_to_imu_[options_.opti_option
                                      .trace_sequence[f_manger.first][0]] *
            cam_map_point;
      }
    }
  }
  return result;
}
//
std::unique_ptr<SlideWindowResult> SlideWindow::AddFeatureData(
    const FrameData& frame) {
  //

  //
  const double dt = camera_imu_time_offset_;
  //
  const int frame_count = imu_states_.size();
  //
  const common::Time current_time =
      frame.data->time + common::FromSeconds(camera_imu_time_offset_);
  //
  images_.emplace(current_time, frame.data->images);
  //
  //
  TicToc feature_t_t;
  for (auto& f : frame.data->features_datas) {
    if (feature_managers_->Exist(f.first)) {
      const int conti_cout =
          feature_managers_->MutableFeatureManager(f.first)->GetFeatureCount();
      feature_managers_->MutableFeatureManager(f.first)
          ->AddFeatureCheckParallax(
              frame_count, frame.data->features_datas[f.first].features, dt);
      const int conti_cout_after =
          feature_managers_->MutableFeatureManager(f.first)->GetFeatureCount();
      LOG_IF(WARNING, conti_cout < 10)
          << "Cam:" << f.first
          << " Continuously track feature points greater than 4-->" << conti_cout
          << ",Add feature after trackpoints: " << conti_cout_after<<" ";
    }
  }


  //
  bool is_keyframe = feature_managers_->CheckParallax();

  if (is_keyframe) {
  
    if (prior_factor_) {
       TicToc t_t;
      //  int k  = options_.win_size;
       int k  = 0;
       const auto prior_pose =
           prior_factor_(GetratePriorData(false,k ));
       if (prior_pose) {
         LOG(WARNING) << "Prior pose: " << *prior_pose << ",fisrt imu pose:"
                      << imu_states_[k].Pose();
         optimization_->SetPrior(*prior_pose, k);
         has_prio_pose = true;

         // CHECK(false);
      }
      VLOG(kGlogCostTimeLevel) << "Local match cost: " << t_t.toc() << " ms";
      LOG(INFO) << "Local match cost: " << t_t.toc() << " ms";
    }
  }
  for (auto& f : frame.data->features_datas) {
    if (!feature_managers_->Exist(f.first)) {
      CHECK(init_feature_managers_.count(f.first));
      init_feature_datas_[frame.data->time].emplace(f);
    }
  }
  if (!init_feature_datas_.empty()) {
    if (int(init_feature_datas_.size()) > options_.win_size + 1) {
      init_feature_datas_.erase(init_feature_datas_.begin());
    }

    if (init_feature_datas_.begin()->first == imu_states_.begin()->time) {
      for (auto& t_f : init_feature_datas_) {
        for (auto& f : t_f.second) {
          init_feature_managers_[f.first]->AddFeatureCheckParallax(
              init_feature_managers_[f.first]->FrameCount() + 1,
              f.second.features, dt);
        }
      }

      for (auto& f : init_feature_datas_.begin()->second) {
        feature_managers_->AddFeatureManger(f.first,
                                            init_feature_managers_[f.first]);
        LOG(INFO) << "Add FeatureManger " << f.first << ",init size "
                  << init_feature_datas_.size();
      }

      init_feature_datas_.clear();
    }
  }

  //
  // bool is_keyframe = feature_manager_->CheckParallax();
  VLOG(kGlogLevel) << "Add incoming feature "
                   << (is_keyframe ? "Keyframe" : "Non-keyframe,");
  //

  const std::vector<sensor::ImuData> imu_datas =
      data_base_->GetImuIntervalData(last_feature_time_, current_time);
  //
  // for (auto& i : imu_datas) {
  //   LOG(INFO) << i.angular_velocity.transpose();
  // }
  // for (auto& i : imu_datas) {
  //   LOG(INFO) << i.linear_acceleration.transpose()<<" "<<i.linear_acceleration.norm();
  // }
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
  for (size_t i = 0; i < imu_states_.size(); i++) {
    triang_pose.push_back(imu_states_[i].Pose());
  }
  //

  VLOG(kGlogCostTimeLevel) << "feature_t_t costs " << feature_t_t.toc()
                           << " ms";

  TicToc tran_t_t;
  feature_managers_->Triangulate(frame_count, triang_pose,
                                 extric_camera_to_imu_);
  //

  VLOG(kGlogCostTimeLevel) << "Triangulate costs " << tran_t_t.toc() << " ms";
  OptimizationData opt_data;
  TicToc opt_sum_t_t;
  //  CHECK_EQ(odoms_factor_.size(),options_.win_size + 1);
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

  VLOG(kGlogCostTimeLevel) << "optisum costs " << opt_sum_t_t.toc() << " ms";

  TicToc marg_sum_t_t;
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

  VLOG(kGlogCostTimeLevel) << "margsum costs " << marg_sum_t_t.toc() << " ms";
  TicToc fram_result_t_t;
  rejection_outliers_ = feature_managers_->RemoveOutliersRejection(
      imu_states_, extric_camera_to_imu_);
  TrackingData front_data;
  if (is_keyframe) {
    front_data = GetratePriorData(true);
  }
  SlideData(is_keyframe);

  feature_managers_->RemoveFailures(&rejection_outliers_);
  //
  FrameData fram_result = frame;
  has_prio_pose =  false;
  //
  for (auto& cam_feature_data : fram_result.data->features_datas) {
    const CameraId cam_id = cam_feature_data.first;
    FrameData::FeatureData& cam_features_data = cam_feature_data.second;
    if (!feature_managers_->Exist(cam_id)) continue;
    auto feat_manager = feature_managers_->MutableFeatureManager(cam_id);
    std::set<TrackingId> remove_id;
    for (auto& features : cam_features_data.features.data->features) {
      TrackingId track_id = features.first;
      const double depth = feat_manager->GetDepth(track_id);
      if (depth > 0) {
        int start_frame = feat_manager->Features().at(track_id).start_frame;
        const Eigen::Vector3d cam_map_point =
            features.second.camera_features[0].normal_points * depth;

        //
        cam_features_data.map_points[track_id] =
            imu_states_[start_frame].Pose() *
            extric_camera_to_imu_[options_.opti_option
                                      .trace_sequence[cam_id][0]] *
            cam_map_point;

      } else {
        remove_id.insert(track_id);
      }
      // cam_features_data.key_points[track_id] =
      //     cv::KeyPoint(features.second.camera_features[0].uv.x(),
      //                  features.second.camera_features[0].uv.y(), 2);
    }
  }
  //
  last_feature_time_ = frame.data->time;
  fram_result.data->extric_camera_to_imu = extric_camera_to_imu_;
  //
  fram_result.data->imu_state = imu_states_.back();

  VLOG(kGlogCostTimeLevel) << "fram_result costs " << fram_result_t_t.toc()
                           << " ms";

  return std::make_unique<SlideWindowResult>(
      SlideWindowResult{fram_result, optimization_->FinalCost(),
                        feature_managers_->GetFeatTrackInfo(),
                        odoms_factor_.back() != nullptr
                            ? odoms_factor_.back()->GetObserveDistance()
                            : 100,
                        front_data});
}
 //
 void SlideWindow::SlideNew() {
   //
   //
   std::swap(imu_states_[imu_states_.size() - 2], imu_states_.back());

   if (integration_base_[imu_states_.size() - 2] && integration_base_.back()) {
     integration_base_[imu_states_.size() - 2]->Merge(
         *integration_base_.back());
   }
   if (odoms_factor_[odoms_factor_.size() - 2] && odoms_factor_.back()) {
     odoms_factor_[odoms_factor_.size() - 2]->Merge(*odoms_factor_.back());
   }

   imu_states_.erase(imu_states_.end());
   integration_base_.erase(integration_base_.end());
   odoms_factor_.erase(odoms_factor_.end());
   //
   //
}

void SlideWindow::SlideData(bool is_keyframe) {
  //
  CHECK_EQ(int(imu_states_.size() - 1), options_.win_size);
  //
  if (is_keyframe /*&& init_slide_new_num==options_.win_size+1*/) {
    //

   if (images_.count(imu_states_[0].time)==1) {
      images_.erase(imu_states_[0].time);
  }
    transform::Rigid3d marg_pose = imu_states_[0].Pose();
    imu_states_.erase(imu_states_.begin());
    transform::Rigid3d new_pose = imu_states_[0].Pose();
    //
    for (size_t i = 0; i < options_.track_sequence.size(); i++) {
      //
      if (feature_managers_->Exist(i)) {
        feature_managers_->MutableFeatureManager(i)->RemoveBackShiftDepth(
            marg_pose * extric_camera_to_imu_[options_.track_sequence[i][0]],
            new_pose * extric_camera_to_imu_[options_.track_sequence[i][0]]);
      }
    }

    odoms_factor_.erase(odoms_factor_.begin());
    integration_base_.erase(integration_base_.begin());
    // feature_managers_->RemoveBack();
  } else {
     if (images_.count(imu_states_[imu_states_.size() - 2].time)==1) {
        images_.erase(imu_states_[imu_states_.size() - 2].time);
      }
    // CHECK(false);
    SlideNew();
    feature_managers_->RemoveFront(options_.win_size);
  }
}



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
  if (has_prio_pose) {
    LOG(INFO)<<y_diff ;
    rot_diff = Eigen::Matrix3d::Identity();
    origin_P0 =
        Eigen::Vector3d(para_Pose[0][0], para_Pose[0][1], para_Pose[0][2]);
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

  for (int i = 0; i < options_.opti_option.CamNum(); i++) {
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
  for (size_t i = 0; i < options_.opti_option.trace_sequence.size(); i++) {
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
  CHECK_EQ(int(imu_states_.size() - 1), options_.win_size);
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

  std::stringstream extric_info;
  if (options_.opti_option.use_odom) {
    for (int j = 0; j < 3; j++) {
      para_Ex_Pose_Odom[0][j] = 0;//odo_to_imu_extric_.translation()[j];
    }
    // para_Ex_Pose_Odom[0][3] = odo_to_imu_extric_.rotation().x();
    // para_Ex_Pose_Odom[0][4] = odo_to_imu_extric_.rotation().y();
    // para_Ex_Pose_Odom[0][5] = odo_to_imu_extric_.rotation().z();
    // para_Ex_Pose_Odom[0][6] = odo_to_imu_extric_.rotation().w();
    para_Ex_Pose_Odom[0][3] = 0;
    para_Ex_Pose_Odom[0][4] = 0;
    para_Ex_Pose_Odom[0][5] = 0;
    para_Ex_Pose_Odom[0][6] = 1;
    extric_info << "Odom to imu extric:" << odo_to_imu_extric_ << " ";
  }
  //
  //
  // CHECK_EQ(extric_camera_to_imu_.size(), options_.opti_option.camera_num);
  for (int i = 0; i < options_.opti_option.CamNum(); i++) {
    extric_info << "cam(" << std::to_string(i) << ")"
                << extric_camera_to_imu_[i] << " ";
    for (int j = 0; j < 3; j++) {
      para_Ex_Pose[i][j] = extric_camera_to_imu_[i].translation()[j];
    }
    const Eigen::Quaterniond q = extric_camera_to_imu_[i].rotation();
    para_Ex_Pose[i][3] = q.x();
    para_Ex_Pose[i][4] = q.y();
    para_Ex_Pose[i][5] = q.z();
    para_Ex_Pose[i][6] = q.w();
  }
  // LOG(INFO) << extric_info.str();
  LOG_EVERY_N(INFO,5)<<extric_info.str();
  VLOG(kGlogLevel)<<extric_info.str();

  for (size_t i = 0; i < options_.opti_option.trace_sequence.size(); i++) {
    //
    if (feature_managers_->Exist(i)) {
      std::vector<double> dephts =
          feature_managers_->MutableFeatureManager(i)->GetDepthVector();
      for (size_t j = 0; j < dephts.size(); j++) {
        para_Feature[i][j][0] = dephts[j];
      }
    }
  }
  para_Td[0][0] = camera_imu_time_offset_;
}

}  // namespace estimator
}  // namespace jarvis