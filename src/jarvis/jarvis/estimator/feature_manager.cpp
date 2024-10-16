#include "feature_manager.h"

#include "glog/logging.h"
#include "opencv2/core/eigen.hpp"
#include "pnp_solver.h"
namespace jarvis {
namespace estimator {
namespace {
using namespace std;
using namespace Eigen;
}  // namespace

// FeatureManager::FeatureManager(Eigen::Matrix3d _Rs[]) : Rs(_Rs) {
//   for (int i = 0; i < NUM_OF_CAM; i++) ric[i].setIdentity();
// }

// void FeatureManager::setRic(Eigen::Matrix3d _ric[]) {
//   for (int i = 0; i < NUM_OF_CAM; i++) {
//     ric[i] = _ric[i];
//   }
// }
FeatureManager::FeatureManager(const FeatureManagerOption &options)
    : options_(options) {}
//

int FeatureManager::GetFeatureCount() {
  int cnt = 0;
  for (auto &pair_it : features_) {
    auto it = pair_it.second;
    if (it.UsedNum() >= options_.convin_used_num) {
      cnt++;
    }
  }
  return cnt;
}

//
bool FeatureManager::IsParallax(int frame_count,
                                const ImageFeatureTrackerData &image) {
  double parallax_sum = 0;
  int parallax_num = 0;
  int last_track_num = 0;
  int new_feature_num = 0;
  int long_track_num = 0;
  //
  for (const auto &id_pts : image.data->features) {
    const int track_num = features_[id_pts.first].feature_per_frame.size();
    if (track_num == 1) {
      new_feature_num++;
    } else {
      last_track_num++;
      if (features_[id_pts.first].feature_per_frame.size() >
          size_t(options_.convin_used_num)) {
        long_track_num++;
      }
    }
    //
  }
  info = FeatTrackInfo{last_track_num, new_feature_num, long_track_num};
  if (frame_count < options_.parallax_option.start_frame ||
      last_track_num < options_.parallax_option.last_track_num ||
      long_track_num < options_.parallax_option.long_track_num ||
      new_feature_num >
          options_.parallax_option.new_feature_ration * last_track_num) {
    return true;
  }
  //
  //
  for (const auto &pair_it_per_id : features_) {
    const auto &it_per_id = pair_it_per_id.second;
    if (it_per_id.start_frame <= frame_count - 2 &&
        it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >=
            frame_count - 1) {
      parallax_sum += compensatedParallax2(it_per_id, frame_count);
      parallax_num++;
    }
  }
  if (parallax_num == 0) {
    LOG(WARNING)<<"parallax_num :"<<parallax_num;
    return true;
  } else {
    VLOG(kGlogLevel) << "parallax_sum: " << parallax_sum
                     << ",parallax_num: " << parallax_num
                     << ",current parallax: ";
    // LOG(INFO)<< parallax_sum / parallax_num<<" " << options_.min_parallax;
    // LOG(INFO)<<options_.min_parallax;
    return parallax_sum / parallax_num >= options_.min_parallax;
  }

  return false;
}
///

bool FeatureManager::AddFeatureCheckParallax(
    int frame_count, const ImageFeatureTrackerData &image, double td) {
  //
  frame_count_= frame_count;
  const int conti_cout = GetFeatureCount();
  LOG_IF(INFO, conti_cout < 10)
      << "Continuously track feature points greater than 4-->" << conti_cout;
  //
  //
  //
  // std::stringstream info;
  for (const auto &id_pts : image.data->features) {
    if (features_.count(id_pts.first)) {
      // info<< id_pts.first<<" d: "<< features_[id_pts.first].estimated_depth<<" ,";
      // info<<" "<<id_pts.second.camera_features[0].normal_points.transpose()<<" ";
      // if(id_pts.second.camera_features.size()==2){
      // info<<" "<<id_pts.second.camera_features[1].normal_points.transpose()<<" ";
      // }
      features_[id_pts.first].feature_per_frame.push_back(
          FeaturePerFrame{id_pts.second, td});
    } else {
      // info<<"*"<< id_pts.first<<" ";
      // info<<" "<<id_pts.second.camera_features[0].normal_points.transpose()<<" ";
      // if(id_pts.second.camera_features.size()==2){
      // info<<" "<<id_pts.second.camera_features[1].normal_points.transpose()<<" ";
      // }
      features_.emplace(
          id_pts.first,
          FeaturePerId{frame_count, {FeaturePerFrame{id_pts.second, td}}});
    }
  }
  // VLOG(kGlogLevel)<<info.str();
  parallax_ = IsParallax(frame_count, image);
  return parallax_;
}

//
std::vector<pair<Eigen::Vector3d, Eigen::Vector3d>>
FeatureManager::GetCorresponding(int frame_count_l, int frame_count_r) {
  //
  std::vector<pair<Eigen::Vector3d, Eigen::Vector3d>> corres;
  for (auto &f : features_) {
    auto &it = f.second;
    if (it.start_frame <= frame_count_l && it.EndFrame() >= frame_count_r) {
      Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
      int idx_l = frame_count_l - it.start_frame;
      int idx_r = frame_count_r - it.start_frame;

      a = it.feature_per_frame[idx_l]
              .feature.camera_features.front()
              .normal_points;
      b = it.feature_per_frame[idx_r]
              .feature.camera_features.front()
              .normal_points;
      corres.push_back(make_pair(a, b));
    }
  }
  return corres;
}
//
 std::vector<double> FeatureManager::GetDepthVector() {
  std::vector<double> dep_vec(GetFeatureCount());
  int feature_index = -1;
  for (auto &pair_it_per_id : features_) {
    //
    auto &it_per_id = pair_it_per_id.second;
    if (it_per_id.UsedNum() < options_.convin_used_num) continue;
    dep_vec[++feature_index] = 1. / it_per_id.estimated_depth;
  }
  return dep_vec;
}

//
void FeatureManager::SetDepth(const std::vector<double> &x) {
  int feature_index = -1;
  // std::stringstream info;
  for (auto &pair_it_per_id : features_) {
    auto &it_per_id = pair_it_per_id.second;
    if (it_per_id.UsedNum() < options_.convin_used_num) continue;
    //
    it_per_id.estimated_depth = 1.0 / x[++feature_index];

    // info << "[" << pair_it_per_id.first << "]"
    //      << it_per_id.estimated_depth<<" ";
    if (it_per_id.estimated_depth < 0) {
      it_per_id.solve_flag = 2;
    } else
      it_per_id.solve_flag = 1;
  }
  // LOG(INFO)<<info.str();
  CHECK_EQ(feature_index,int(x.size()-1));
}
//
//
void FeatureManager::RemoveFailures() {
  std::stringstream info;
  for (auto it = features_.begin(), it_next = features_.begin();
       it != features_.end(); it = it_next) {
    it_next++;
    if (it->second.solve_flag == 2) {
      info<<it->first<<' ';
      features_.erase(it);
    }
  }
  VLOG(kGlogLevel) <<  info.str();
}

//
void FeatureManager::ClearDepth() {
  for (auto &it_per_id : features_) {
    it_per_id.second.estimated_depth = -1;
  }
}
std::set<TrackFeatureId> FeatureManager::OutliersRejection(
    const std::vector<ImuState> &pose,
    const std::vector<transform::Rigid3d> &ex) {
  std::set<TrackFeatureId> remove_index;
  auto ReprojectionError = [](const Eigen::Vector3d world_point_i,
                              const transform::Rigid3d &pose_j,
                              Eigen::Vector3d &uvj) {
    //
    const Eigen::Vector3d pts_cj =
         pose_j.inverse() * world_point_i;
    Eigen::Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);
  };
    std::stringstream info;
  for (auto &pair_it_per_id : features_) {
    double err = 0;
    int errCnt = 0;
    auto &it_per_id = pair_it_per_id.second;

    if (it_per_id.UsedNum() < options_.convin_used_num) continue;
    int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
    Eigen::Vector3d pts_i =
        it_per_id.feature_per_frame[0].feature.camera_features[0].normal_points;
    double depth = it_per_id.estimated_depth;
    const Eigen::Vector3d world_point_i =
        pose[imu_i].Pose() *( ex[0] * (pts_i * depth));
    //
    for (auto &it_per_frame : it_per_id.feature_per_frame) {
      imu_j++;
      if (imu_i != imu_j) {
        Eigen::Vector3d pts_j =
            it_per_frame.feature.camera_features[0].normal_points;
        double tmp_error = ReprojectionError(
            world_point_i, pose[imu_j].Pose() * ex[0], pts_j);
        err += tmp_error;
        errCnt++;
        // printf("tmp_error %f\n", tmp_error);
      }
      // need to rewrite projecton factor.........
      if (it_per_frame.IsStereo()) {
        // Eigen::Vector3d pts_j =
        //     it_per_frame.feature.camera_features[1].normal_points;
        // double tmp_error =
        //     ReprojectionError(world_point_i, pose[imu_j].Pose() * ex[1], pts_j);
        // err += tmp_error;
        // printf("right tmp_error %f\n", tmp_error);
        // errCnt++;
      }
    }
    double ave_err = err / errCnt;
    // LOG(INFO)<<pair_it_per_id.first <<" "<<ave_err<<" " <<options_.optimazation_outliers_rejection_th;
    // LOG(INFO)<<options_.optimazation_outliers_rejection_th;
    if (ave_err > options_.optimazation_outliers_rejection_th) {
      info<<pair_it_per_id.first<<" ";
      remove_index.insert(pair_it_per_id.first);
    }
  }
  VLOG(kGlogLevel) <<info.str();
  RemoveOutlier(remove_index);
  return remove_index;
}
//

void FeatureManager::CreateFactor(
    const std::function<void(
        const Eigen::Vector3d &, const Eigen::Vector3d &,
        const Eigen::Vector2d &, const Eigen::Vector2d &, double, double,
        const std::tuple<int, int, int> &index)> &projection_two_frame_one_cam,
    const std::function<void(
        const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j,
        const Eigen::Vector2d &_velocity_i, const Eigen::Vector2d &_velocity_j,
        const double _td_i, const double _td_j,
        const std::tuple<int, int, int> &index)> &projection_two_frametwocam,
    const std::function<void(
        const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j,
        const Eigen::Vector2d &_velocity_i, const Eigen::Vector2d &_velocity_j,
        const double _td_i, const double _td_j,
        const std::tuple<int, int, int> &index)> &projection_one_frame_twocam) {
  //
  std::stringstream info;
  int feature_index = -1;
  for (const auto &pair_it_per_id : features_) {
    const FeaturePerId it_per_id = pair_it_per_id.second;
    if (it_per_id.UsedNum() < options_.convin_used_num) continue;
    ++feature_index;
    int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

    const Eigen::Vector3d &pts_i =
        it_per_id.feature_per_frame[0].feature.camera_features[0].normal_points;
    //
    // info<<pts_i.transpose()<<" ";
    const Eigen::Vector2d imu_i_velocity =
        it_per_id.feature_per_frame[0].feature.camera_features[0].uv_velocity;
    //
    info << "{" << imu_i << "}" << " " << it_per_id.feature_per_frame.size()
         << " ";
    for (auto it_per_frame : it_per_id.feature_per_frame) {
      imu_j++;
      if (imu_i != imu_j) {
        const Eigen::Vector3d &pts_j =
            it_per_frame.feature.camera_features[0].normal_points;
        const Eigen::Vector2d &imu_j_velocity =
            it_per_frame.feature.camera_features[0].uv_velocity;
        //
        if (projection_two_frame_one_cam) {
          projection_two_frame_one_cam(
              pts_i, pts_j, imu_i_velocity, imu_j_velocity,
              it_per_id.feature_per_frame[0].td, it_per_frame.td,
              {imu_i, imu_j, feature_index});
        }
      }

      if (it_per_frame.IsStereo()) {
        // // /
        const Eigen::Vector3d &pts_j_right =
            it_per_frame.feature.camera_features[1].normal_points;
        //

        const Eigen::Vector2d &imu_j_velocity =
            it_per_frame.feature.camera_features[1].uv_velocity;

        CHECK(!isnan(imu_j_velocity.y()))
            << imu_j_velocity.transpose() << " " << pair_it_per_id.first;
        if (imu_i != imu_j) {
          if (projection_two_frametwocam) {
            projection_two_frametwocam(
                pts_i, pts_j_right, imu_i_velocity, imu_j_velocity,
                it_per_id.feature_per_frame[0].td, it_per_frame.td,
                {imu_i, imu_j, feature_index});
          }
        } else {
          if (projection_one_frame_twocam) {
            projection_one_frame_twocam(
                pts_i, pts_j_right, imu_i_velocity, imu_j_velocity,
                it_per_id.feature_per_frame[0].td, it_per_frame.td,
                {imu_i, imu_j, feature_index});
          }
        }
      }
    }
  }
}
  //
  //
  //
  Eigen::Vector3d TriangulatePoint(
      const std::vector<transform::Rigid3d> &poses,
      const std::vector<Eigen::Vector2d> &key_point_normal) {
    Eigen::MatrixXd H(poses.size() * 2, 4);

    // CHECK_EQ(poses.size(), 2) << "Function Just adoptor 2 size pose";
    // Eigen::MatrixXd H;
    for (size_t i = 0; i < poses.size(); i++) {
      Eigen::Matrix<double, 3, 4> pose_matrix;
      pose_matrix.block<3, 3>(0, 0) = poses[i].rotation().toRotationMatrix();
      pose_matrix.block<3, 1>(0, 3) = poses[i].translation();
      const Eigen::Vector2d &point = key_point_normal[i];
      H.row(i * 2 + 0) = point[0] * pose_matrix.row(2) - pose_matrix.row(0);
      H.row(i * 2 + 1) = point[1] * pose_matrix.row(2) - pose_matrix.row(1);
    }
    //
    Eigen::Vector4d triangulated_point;
    triangulated_point =
        H.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    return (triangulated_point / triangulated_point(3)).head<3>();
  }
  //
  //
  bool FeatureManager::SolvePoseByPnP(const std::vector<cv::Point2f> &pts2D,
                                      const std::vector<cv::Point3f> &pts3D,
                                      transform::Rigid3d *p_initial) {
    // // printf("pnp size %d \n",(int)pts2D.size() );
    // LOG(INFO) << options_.init_pnp_inlier_num;
    if (int(pts2D.size()) < options_.init_pnp_inlier_num) {
      LOG(ERROR)
          << "feature tracking not enough, please slowly move you device! "
          << pts2D.size() << " < " << options_.init_pnp_inlier_num;
      return false;
    }
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(p_initial->rotation().inverse().toRotationMatrix(), tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(p_initial->translation(), t);
    //
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    bool pnp_succ;
    // pnp_succ = cv::solvePnP(pts3D, pts2D, K, D, rvec, t, 1);
    cv::Mat inliers;
    pnp_succ = solvePnPRansac(pts3D, pts2D, K, D, rvec, t, true, 100, 4.0 / 377,
                              0.999, inliers);
    int n = 0;
    for (int i = 0; i < inliers.rows; i++) {
      if (inliers.at<int>(i)) {
        n++;
      }
    }
    if (!pnp_succ && n >= options_.init_pnp_inlier_num) {
      LOG(ERROR) << "pnp failed ! ";
      return false;
    }
    cv::Rodrigues(rvec, r);
    // cout << "r " << endl << r << endl;
    Eigen::Matrix3d R_pnp;
    cv::cv2eigen(r, R_pnp);
    Eigen::Vector3d T_pnp;
    cv::cv2eigen(t, T_pnp);

    // cam_T_w ---> w_T_cam
    *p_initial = transform::Rigid3d(Eigen::Vector3d(T_pnp),
                                    Eigen::Quaterniond(R_pnp).normalized())
                     .inverse();
    return true;

    // PnpSolver pnp_solver(PnpSolverOption{});
    // //
    // std::vector<Eigen::Vector2d> normal_2d_temp;
    // std::vector<Eigen::Vector3d> map_points_temp;
    // for (int i = 0; i < pts3D.size(); i++) {
    //   map_points_temp.push_back(
    //       Eigen::Vector3d(pts3D[i].x, pts3D[i].y, pts3D[i].z));
    //   normal_2d_temp.push_back(Eigen::Vector2d(pts2D[i].x, pts2D[i].y));
    // }
    // auto result = pnp_solver.Solve(map_points_temp, normal_2d_temp,
    //                                transform::Rigid3d::Identity());
    // std::pair<transform::Rigid3d, std::vector<bool>> pnp_pose;
    // if (result && !result->extend) {
    //   pnp_pose = std::make_pair<transform::Rigid3d, std::vector<bool>>(
    //       std::move(result->pose), std::move(result->inliers));
    // } else {
    //   LOG(ERROR) << "pnp failed ! ";
    //   return false;
    // }
    // transform::Rigid3d pose = pnp_pose.first.inverse();
    // R = pose.rotation().toRotationMatrix();
    // P = pose.translation();
    // return true;
  }

  bool FeatureManager::InitFramePoseByPnP(
      int frameCnt, const std::vector<transform::Rigid3d> &ex_came_to_imu,
      std::vector<transform::Rigid3d> &sw_pose) {
    if (frameCnt <= 0) return true;

    std::vector<cv::Point2f> pts2D;
    std::vector<cv::Point2f> pts2D_r;
    std::vector<cv::Point3f> pts3D;
    for (auto &pair_it_per_id : features_) {
      const auto &it_per_id = pair_it_per_id.second;
      if (it_per_id.estimated_depth > 0) {
        int index = frameCnt - it_per_id.start_frame;
        CHECK_GE(index, 0);
        if ((int)it_per_id.feature_per_frame.size() >= index + 1) {
          const Eigen::Vector3d &point = it_per_id.feature_per_frame[0]
                                             .feature.camera_features[0]
                                             .normal_points;
          //

          CHECK_LE(it_per_id.start_frame, int(sw_pose.size() - 1));
          Eigen::Vector3d ptsInWorld = sw_pose[it_per_id.start_frame] *
                                       ex_came_to_imu[0] *
                                       (point * it_per_id.estimated_depth);
          cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());
          //
          const Eigen::Vector3d &frame_point =
              it_per_id.feature_per_frame[index]
                  .feature.camera_features[0]
                  .normal_points;
          const Eigen::Vector3d &frame_right_point =
              it_per_id.feature_per_frame[index]
                  .feature.camera_features[1]
                  .normal_points;
          //
          cv::Point2f point2d(frame_point.x(), frame_point.y());
          cv::Point2f point2d_r(frame_right_point.x(), frame_right_point.y());
          pts3D.push_back(point3d);
          pts2D.push_back(point2d);
          pts2D_r.push_back(point2d_r);
        }
      }
    }
    // LOG(INFO) << pts3D.size();
    // trans to w_T_cam
    transform::Rigid3d RCam = (sw_pose[frameCnt - 1] * ex_came_to_imu[0]);
    //
    if (!SolvePoseByPnP(pts2D, pts3D, &RCam)) return false;

    // transform::Rigid3d r_pose =
    //     transform::Rigid3d(PCam, Eigen::Quaterniond(RCam)) *
    //     transform_cam1_to_cam0_;

    // int out_reprejct_outlier_num = 0;
    // for (int i = 0; i < pts3D.size(); i++) {
    //   Eigen::Vector3d p =
    //       r_pose.inverse() * Eigen::Vector3d(pts3D[i].x, pts3D[i].y,
    //       pts3D[i].z);
    //   p = p / p.z();
    //   double err =(p - Eigen::Vector3d(pts2D_r[i].x, pts2D_r[i].y,
    //   1)).squaredNorm(); LOG(INFO) << err; if (err < 3.0/FOCAL_LENGTH ) {
    //     out_reprejct_outlier_num++;
    //   }
    // }
    // LOG(INFO)<<out_reprejct_outlier_num<<" "<<pts3D.size();
    // // if(out_reprejct_outlier_num<=pts3D.size()*0.90)return false;
    // LOG(INFO)<<ex_came_to_imu[0];
    // trans to w_T_imu
    sw_pose[frameCnt] = RCam * ex_came_to_imu[0].inverse();
    LOG(INFO) << "pnp pose:" << sw_pose[frameCnt] << "ypr: "
              << Utility::R2ypr(sw_pose[frameCnt].rotation().toRotationMatrix())
                     .transpose();
    return true;
  }
  //
  //
  void FeatureManager::TriangulateStero(
      uint64_t it_per_id, const std::vector<transform::Rigid3d> &sw_pose,
      const std::vector<transform::Rigid3d> &ex_came_to_imu) {
    //
    CHECK(features_.count(it_per_id));
    auto &features_id = features_[it_per_id];
    //
    const Eigen::Vector2d &point0 = features_id.feature_per_frame[0]
                                        .feature.camera_features[0]
                                        .normal_points.head<2>();
    const Eigen::Vector2d &point1 = features_id.feature_per_frame[0]
                                        .feature.camera_features[1]
                                        .normal_points.head<2>();
    //

    // LOG(INFO) << point0.transpose() << point1.transpose();
    int imu_i = features_id.start_frame;
    CHECK_LE(imu_i, int(sw_pose.size() - 1));
    const transform::Rigid3d &frame_left_pose =
        sw_pose[imu_i] * ex_came_to_imu[0];

    const transform::Rigid3d &frame_right_pose =
        sw_pose[imu_i] * ex_came_to_imu[1];

    const Eigen::Vector3d point3d = TriangulatePoint(
        {frame_left_pose.inverse(), frame_right_pose.inverse()},
        {point0, point1});
    // //
    Eigen::Vector3d localPoint = frame_left_pose.inverse() * point3d;
    // //
    // LOG(INFO) << localPoint.transpose();
    double depth = localPoint.z();
    // LOG(INFO)<<depth;
    const Eigen::Vector3d localPoint_r = frame_right_pose.inverse() * point3d;
    if (depth > 0.5 && localPoint_r.z() > 0.5 && depth < 20 &&
        localPoint_r.z() < 20)
      features_id.estimated_depth = depth;
    else {
      features_id.estimated_depth = options_.init_depth;
    }
  }
  void FeatureManager::TriangulateCurAfter(
      uint64_t it_per_id, const std::vector<transform::Rigid3d> &sw_pose,
      const std::vector<transform::Rigid3d> &ex_came_to_imu) {
    auto &feature = features_[it_per_id];
    //
    int imu_i = feature.start_frame;
    CHECK_LE(imu_i, int(sw_pose.size() - 1));
    const transform::Rigid3d &frame_left_pose =
        sw_pose[imu_i] * ex_came_to_imu[0];
    const int next_imu_i = imu_i + 1;

    // LOG(INFO) << imu_i << " " << next_imu_i;
    CHECK_LE(next_imu_i, int(sw_pose.size() - 1));
    const transform::Rigid3d &frame_right_pose =
        sw_pose[next_imu_i] * ex_came_to_imu[0];

    const Eigen::Vector2d &point0 = feature.feature_per_frame[0]
                                        .feature.camera_features[0]
                                        .normal_points.head<2>();

    const Eigen::Vector2d &point1 = feature.feature_per_frame[1]
                                        .feature.camera_features[0]
                                        .normal_points.head<2>();
    const Eigen::Vector3d point3d = TriangulatePoint(
        {frame_left_pose.inverse(), frame_right_pose.inverse()},
        {point0, point1});
    Eigen::Vector3d localPoint = frame_left_pose.inverse() * point3d;
    //
    double depth = localPoint.z();
    // LOG(INFO) << localPoint.transpose();
    const Eigen::Vector3d localPoint_r = frame_right_pose.inverse() * point3d;
    if (depth > 0.5 && localPoint_r.z() > 0.5 && depth < 20 &&
        localPoint_r.z() < 20)
      feature.estimated_depth = depth;
    else {
      feature.estimated_depth = options_.init_depth;
    }
  }


  void FeatureManager::Triangulate(
      int frameCnt, const std::vector<transform::Rigid3d> &sw_pose,
      const std::vector<transform::Rigid3d> &ex_came_to_imu) {
     for (auto &pair_it_per_id : features_) {
      auto &it_per_id = pair_it_per_id.second;
      if (it_per_id.estimated_depth > 0) continue;
      //
      if (options_.use_stereo && it_per_id.feature_per_frame[0].IsStereo()) {
        TriangulateStero(pair_it_per_id.first, sw_pose, ex_came_to_imu);
        continue;
      } else if (it_per_id.feature_per_frame.size() > 1) {
        TriangulateCurAfter(pair_it_per_id.first, sw_pose, ex_came_to_imu);
        continue;
      }
      //
      if (it_per_id.UsedNum() < options_.convin_used_num) continue;
      //

      int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
      Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
      //
      CHECK_LE(imu_i, int(sw_pose.size() - 1));
      //
      const transform::Rigid3d &frame_pose0 =
          sw_pose[imu_i] * ex_came_to_imu[0];
      std::vector<transform::Rigid3d> sw_poses;
      // sw_poses.push_back(frame_pose0);
      //
      std::vector<Eigen::Vector2d> normal_points;
      for (auto &it_per_frame : it_per_id.feature_per_frame) {
        imu_j++;
        const transform::Rigid3d &frame_posei =
            sw_pose[imu_j] * ex_came_to_imu[0];
        sw_poses.push_back(frame_posei);

        const Eigen::Vector3d &point0 =
            it_per_frame.feature.camera_features[0].normal_points.normalized();
        normal_points.push_back(point0.head<2>());
        if (imu_i == imu_j) continue;
      }
      const Eigen::Vector3d point3d = TriangulatePoint(sw_poses, normal_points);
      const Eigen::Vector3d local_points = frame_pose0 * point3d;
      it_per_id.estimated_depth = local_points.z();

      if (it_per_id.estimated_depth < 0.1) {
        it_per_id.estimated_depth = options_.init_depth;
      }
    }
  }

  //
  void FeatureManager::RemoveOutlier(
      const std::set<TrackFeatureId> &outlierIndex) {
    for (const TrackFeatureId &id : outlierIndex) {
      features_.erase(id);
    }
  }
  //
  void FeatureManager::RemoveBackShiftDepth(const transform::Rigid3d &marg_p,
                                            const transform::Rigid3d &new_p) {
    for (auto it = features_.begin(); it != features_.end();) {
      auto &feature_per_id = it->second;
      //
      if (feature_per_id.start_frame != 0) {
        feature_per_id.start_frame--;
        it++;
        continue;
      }

      auto &feature_per_frame = feature_per_id.feature_per_frame;
      CHECK(!feature_per_frame.empty())
          << it->first << feature_per_frame.size();
      Eigen::Vector3d uv_i =
          feature_per_frame[0].feature.camera_features[0].normal_points;
      //
      // uv_i.head<2>() = feature_per_frame[0].feature.camera_features[0].uv;
      // uv_i.z() = 1;
      feature_per_frame.erase(feature_per_frame.begin());
      //
      if (feature_per_frame.size() < 2) {
        it = features_.erase(it);
        continue;
      } else {
        Eigen::Vector3d pts_i{uv_i * it->second.estimated_depth};
        //
        Eigen::Vector3d w_pts_i = marg_p * pts_i;
        Eigen::Vector3d pts_j = new_p.inverse() * w_pts_i;
        double dep_j = pts_j(2);
        if (dep_j > 0)
          it->second.estimated_depth = dep_j;
        else
          it->second.estimated_depth = options_.init_depth;
      }
      it++;
    }
  }

  void FeatureManager::RemoveBack() {
    for (auto it = features_.begin(), it_next = features_.begin();
         it != features_.end(); it = it_next) {
      it_next++;
      if (it->second.start_frame != 0) {
        it->second.start_frame--;
      } else {
        it->second.feature_per_frame.erase(
            it->second.feature_per_frame.begin());
        if (it->second.feature_per_frame.size() == 0) {
          features_.erase(it);
        }
      }
    }
  }
  //

  void FeatureManager::RemoveFront(int frame_count) {
    for (auto it = features_.begin(), it_next = features_.begin();
         it != features_.end(); it = it_next) {
      it_next++;

      if (it->second.start_frame == frame_count) {
        it->second.start_frame--;
      } else {
        //
        int j = options_.sw_size - 1 - it->second.start_frame;
        if (it->second.EndFrame() < frame_count - 1) continue;
        //
        it->second.feature_per_frame.erase(
            it->second.feature_per_frame.begin() + j);
        //

        if (it->second.feature_per_frame.size() == 0) {
          features_.erase(it);
        }
      }
    }
  }

  double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id,
                                              int frame_count) {
    // check the second last frame is keyframe or not
    // parallax betwwen seconde last frame and third last frame
    const FeaturePerFrame &frame_i =
        it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j =
        it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.feature.camera_features[0].normal_points;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.feature.camera_features[0].normal_points;
    Vector3d p_i_comp;

    // int r_i = frame_count - 2;
    // int r_j = frame_count - 1;
    // p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] *
    // ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv,
                            du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
  }

  // std::vector<double> FeatureManagers::GetDepthVector() {
  //   std::vector<double> result;
  //   for (auto &f_m : feature_managers_) {
  //     const auto depths = f_m.second.GetDepthVector();
  //     result.insert(result.end(), depths.begin(), depths.end());
  //   }
  //   return result;
  // }
  // //

  // void FeatureManagers::AddFeaturesWithData(
  //     int id, const std::map<TrackFeatureId, FeaturePerId> &features) {
  //   CHECK(feature_managers_.count(id));
  //   feature_managers_.erase(id);
  //   feature_managers_.emplace(id, FeatureManager(feat_option_, features));
  // }

  // //
  // void FeatureManagers::SetDepth(const std::vector<double> &x) {
  //   auto it = x.begin();
  //   for (auto &f_m : feature_managers_) {
  //     int index = f_m.second.GetFeatureCount();
  //     f_m.second.SetDepth(std::vector<double>(it, it + index));
  //     it = it + index;
  //   }
  // }

  //
  // FeatureManagers::FeatureManagers(int cam_trajector,
  //                                  FeatureManagerOption &feat_option)
  //     : feat_option_(feat_option) {
  //   for (int i = 0; i < cam_trajector; i++) {
  //     feature_managers_.emplace(i, feat_option);
  //   }
  // }

  //
  bool FeatureManagers::CheckParallax() const {
    for (const auto &f_m : feature_managers_) {
      if (f_m.second->IsParallax()) {
        return true;
      }
    }
    return false;
  }
  //
  //
  std::map<CameraId, std::set<TrackFeatureId>>
  FeatureManagers::RemoveOutliersRejection(
      const std::vector<ImuState> &pose,
      const std::vector<transform::Rigid3d> &ex) {
    std::array<std::vector<int>, 3> ParaExPoseIndex{
        std::vector<int>{0, 1}, std::vector<int>{2}, std::vector<int>{3}};

    std::map<CameraId, std::set<TrackFeatureId>> result;
    for (auto &f_m : feature_managers_) {
      std::vector<transform::Rigid3d> ex_came_to_imu_tmp;
      for (size_t i = 0; i < ParaExPoseIndex[f_m.first].size(); i++) {
        ex_came_to_imu_tmp.push_back(ex[ParaExPoseIndex[f_m.first][i]]);
      }
      result[f_m.first] =
          std::move(f_m.second->OutliersRejection(pose, ex_came_to_imu_tmp));
    }
    return result;
  }

  //

  void FeatureManagers::Triangulate(
      int fram_cout, const std::vector<transform::Rigid3d> &sw_pose,
      const std::vector<transform::Rigid3d> &ex_came_to_imu) {
    std::array<std::vector<int>, 3> ParaExPoseIndex{
        std::vector<int>{0, 1}, std::vector<int>{2}, std::vector<int>{3}};

    for (auto &f_m : feature_managers_) {
       std::vector<transform::Rigid3d> ex_came_to_imu_tmp;
      for (size_t i = 0; i < ParaExPoseIndex[f_m.first].size(); i++) {
        ex_came_to_imu_tmp.push_back(
            ex_came_to_imu[ParaExPoseIndex[f_m.first][i]]);
      }
      f_m.second->Triangulate(fram_cout, sw_pose,ex_came_to_imu_tmp );
    }
  }

  //
  void FeatureManagers::AddFeatureManger(int cam_track_id,
                                         std::shared_ptr<FeatureManager> fm) {
    feature_managers_.emplace(cam_track_id, fm);
  }

  void FeatureManagers::RemoveBackShiftDepth(const transform::Rigid3d &marg_p,
                                             const transform::Rigid3d &new_p) {
    for (auto &f_m : feature_managers_) {
      f_m.second->RemoveBackShiftDepth(marg_p, new_p);
    }
  }

  void FeatureManagers::RemoveBack() {
    for (auto &f_m : feature_managers_) {
      f_m.second->RemoveBack();
    }
  }

  void FeatureManagers::RemoveFront(int frame_count) {
    for (auto &f_m : feature_managers_) {
      f_m.second->RemoveFront(frame_count);
    }
  }
  void FeatureManagers::RemoveFailures() {
    for (auto &f_m : feature_managers_) {
      f_m.second->RemoveFailures();
    }
  }

  FeatTrackInfo FeatureManagers::GetFeatTrackInfo() {
    FeatTrackInfo info;
    for (auto &f_m : feature_managers_) {
      info += f_m.second->GetFeatTrackInfo();
    }
    return info;
  }

}  // namespace estimator
}  // namespace jarvis
