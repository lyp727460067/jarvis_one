/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <algorithm>
#include <list>
#include <numeric>
#include <set>
#include <vector>

#include "Eigen/Dense"
#include "jarvis/estimator/featureTracker/feature_tracker.h"
#include "jarvis/utility/tic_toc.h"
#include "parameters.h"
#include "jarvis/key_frame_data.h"
namespace jarvis {
namespace estimator {
//
//
struct FeaturePerFrame {
  FeatureData feature;
  double td = 0.0;
  bool IsStereo() const { return feature.camera_features.size() == 2; }
};

struct FeaturePerId {
  int start_frame;
  std::vector<FeaturePerFrame> feature_per_frame;
  double estimated_depth = -1.;
  int solve_flag = 0;  // 0 haven't solve yet; 1 solve succ; 2 solve fail;
  int UsedNum() const { return feature_per_frame.size(); }
  int EndFrame() { return start_frame + feature_per_frame.size() - 1; }
};
//
//
struct FeatureManagerOption {
  int sw_size=6;
  bool use_stereo = true;
  double init_depth = 5.0;
  double min_parallax = 1. / 377;
  int init_pnp_inlier_num = 15;
  int convin_used_num = 4;
  int keyframe_parallax=1;
  bool predit_all_sw_frame =true;
  double optimazation_outliers_rejection_th = 5.0 / 377;
  double rejection_points_depth_max_th =30;
  struct ParallaxOption {
    int start_frame = 2;
    int last_track_num = 20;
    int long_track_num = 10;
    double new_feature_ration = 0.5;
  } parallax_option;
};

struct FeatTrackInfo {
  int frame  =0;
  int last_track_num = 0;
  int new_feature_num = 0;
  int long_track_num = 0;
  double parallax_sum = 0;
  int parallax_num =0;
  FeatTrackInfo &operator+=(const FeatTrackInfo &rhs) {
    frame  = rhs.frame;
    last_track_num += rhs.last_track_num;
    new_feature_num += rhs.new_feature_num;
    long_track_num += rhs.long_track_num;
    parallax_sum += rhs.parallax_sum;
    parallax_num += rhs.parallax_num;

    return *this;
  }
};
class FeatureManager {
 public:

  FeatureManager(const FeatureManagerOption &options);
  FeatureManager(const FeatureManagerOption &options,
                 const std::map<TrackFeatureId, FeaturePerId>& features)
      : options_(options), features_(features){}

  int FrameCount() { return frame_count_; }
  //
  int GetFeatureCount();
  bool AddFeatureCheckParallax(int frame_count,
                               const ImageFeatureTrackerData &image, double td);
    //
  std::map<int, Eigen::Vector3d> GetPredictionInPose(const transform::Rigid3d&pose,int frame_count,
 const   std::vector<transform::Rigid3d>& sw_poses 
  );
  //
  const FeatTrackInfo &GetFeatTrackInfo() { return info; }
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> GetCorresponding(
      int frame_count_l, int frame_count_r);
  // void updateDepth(const VectorXd &x);
  void SetDepth(const std::vector<double> &x);
  std::set<TrackFeatureId> RemoveFailures();
  void ClearDepth();
  std::vector<double> GetDepthVector();

  void Triangulate(int frameCnt, const std::vector<transform::Rigid3d> &sw_pose,
                   const std::vector<transform::Rigid3d> &ex_came_to_imu);
  //

  //
  //
  bool InitFramePoseByPnP(int frameCnt,
                          const std::vector<transform::Rigid3d> &ex_came_to_imu,
                          std::vector<transform::Rigid3d> &sw_pose);
  //
  bool SolvePoseByPnP(const std::vector<cv::Point2f> &pts2D,
                      const std::vector<cv::Point3f> &pts3D,
                      transform::Rigid3d *p_initial);

  //
  void CreateFactor(
      const std::function<void(const Eigen::Vector3d &, const Eigen::Vector3d &,
                               const Eigen::Vector2d &, const Eigen::Vector2d &,
                               double, double,
                               const std::tuple<int, int, int> &index)>
          &projection_two_frame_one_cam,
      const std::function<
          void(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j,
               const Eigen::Vector2d &_velocity_i,
               const Eigen::Vector2d &_velocity_j, const double _td_i,
               const double _td_j, const std::tuple<int, int, int> &index)>
          &projection_two_frametwocam = nullptr,
      const std::function<
          void(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j,
               const Eigen::Vector2d &_velocity_i,
               const Eigen::Vector2d &_velocity_j, const double _td_i,
               const double _td_j, const std::tuple<int, int, int> &index)>
          &projection_one_frame_twocam = nullptr);

  void RemoveBackShiftDepth(const transform::Rigid3d &marg_p,
                            const transform::Rigid3d &new_p);
  // /
  void RemoveBack();
  std::vector<TrackFeatureId> GetBack();
  void RemoveFront(int frame_count);
  void RemoveOutlier(const std::set<TrackFeatureId> &outlierIndex);
  std::set<TrackFeatureId> OutliersRejection(const std::vector<ImuState> &pose,
                                  const std::vector<transform::Rigid3d> &ex);
  //
  //
  const std::map<TrackFeatureId, FeaturePerId> &Features() const {
    return features_;
  }
  bool IsParallax() const { return parallax_; }

  //
  double GetDepth(const TrackFeatureId &id) {
    if (!features_.count(id)) {
      // LOG(WARNING) << "Feat id " << id << "not exist.";
      return -1;
    }
    return features_[id].estimated_depth;
  }
  //
  void SetDepth(const TrackFeatureId &id, double depth) {
    if (!features_.count(id)) {
      LOG(WARNING) << "Feat id " << id << "not exist.";
      return;
    }
    features_[id].estimated_depth = depth;
  }
//
 const FeatureManagerOption&  Options(){
  return  options_;
 }
 private:
  bool IsParallax(int frame_count, const ImageFeatureTrackerData &image);
  void TriangulateStero(uint64_t it_per_id,
                        const std::vector<transform::Rigid3d> &sw_pose,
                        const std::vector<transform::Rigid3d> &ex_came_to_imu);
  //
  void TriangulateCurAfter(
      uint64_t it_per_id, const std::vector<transform::Rigid3d> &sw_pose,
      const std::vector<transform::Rigid3d> &ex_came_to_imu);

  FeatureManagerOption options_;
  std::map<TrackFeatureId, FeaturePerId> features_;
  double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
  // const Eigen::Matrix3d *Rs = nullptr;
  // Eigen::Matrix3d ric[2];
  bool parallax_ = false;
  int frame_count_ =-1;
  FeatTrackInfo info;
};

//
class FeatureManagers {
 public:
  //

  // FeatureManagers(int cam_trajector, FeatureManagerOption &feat_option);
  // FeatureManagers(const std::map<uint64_t, FeatureManager> &feat_ms)
  //     : feature_managers_(feat_ms) {}
  //
  const std::map<uint64_t, std::shared_ptr<FeatureManager>> &
  GetFeatureManagers() {
    return feature_managers_;
  }
  //
  //
  void AddFeatureManger(int cam_track_id, std::shared_ptr<FeatureManager> fm);
  std::shared_ptr<FeatureManager> MutableFeatureManager(int cam_id) {
    //
    if (feature_managers_.count(cam_id)) {
      return feature_managers_.at(cam_id);
    }
    LOG(INFO) << "Featuremanger not construct." << cam_id;
    return nullptr;
    // CHECK(feature_managers_.count(cam_id))
    //     << "Featuremanger not construct." << cam_id;
    // //
  }
  //
  std::map<CameraId, std::set<TrackFeatureId>> RemoveOutliersRejection(
      const std::vector<ImuState> &pose,
      const std::vector<transform::Rigid3d> &ex);
  //
  bool Exist(CameraId id) { return feature_managers_.count(id) != 0; }
  void Triangulate(int fram_cout,
                   const std::vector<transform::Rigid3d> &sw_pose,
                   const std::vector<transform::Rigid3d> &ex_came_to_imu);
  //
  FeatTrackInfo GetFeatTrackInfo();
  void RemoveFailures(std::map<CameraId, std::set<TrackFeatureId>>*ids);
  void RemoveBack();
  void RemoveBackShiftDepth(const transform::Rigid3d &marg_p,
                            const transform::Rigid3d &new_p);
  void RemoveFront(int frame_count);
  // std::vector<double> GetDepthVector();
  // void SetDepth(const std::vector<double> &x);
  bool CheckParallax() const;
 private:
   FeatureManagerOption feat_option_;
   std::map<uint64_t, std::shared_ptr<FeatureManager>> feature_managers_;
};

}  // namespace estimator
}  // namespace jarvis

#endif