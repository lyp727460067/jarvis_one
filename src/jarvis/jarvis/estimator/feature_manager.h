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
  double estimated_depth = -1;
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
  struct ParallaxOption {
    int start_frame = 2;
    int last_track_num = 30;
    int long_track_num = 20;
    double new_feature_ration = 0.5;
  } parallax_option;
};

class FeatureManager {
 public:

  FeatureManager(const FeatureManagerOption &options);
  FeatureManager(const FeatureManagerOption &options,
                 const std::map<TrackFeatureId, FeaturePerId>& features)
      : options_(options), features_(features){}

  int GetFeatureCount();
  //
  bool AddFeatureCheckParallax(int frame_count,
                               const ImageFeatureTrackerData &image, double td);

  //
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> GetCorresponding(
      int frame_count_l, int frame_count_r);
  // void updateDepth(const VectorXd &x);
  void SetDepth(const std::vector<double> &x);
  void RemoveFailures();
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
      const std::function<void(
          const Eigen::Vector3d &, const Eigen::Vector3d &,
          const Eigen::Vector2d &, const Eigen::Vector2d &, double, double,
          const std::tuple<int, int, int>  &index)> &projection_two_frame_one_cam);

  void RemoveBackShiftDepth(const transform::Rigid3d &marg_p,
                            const transform::Rigid3d &new_p);
  // /
  void RemoveBack();
  void RemoveFront(int frame_count);
  void RemoveOutlier(const std::set<TrackFeatureId> &outlierIndex);
  //
  //
  const std::map<TrackFeatureId, FeaturePerId> &Features() const {
    return features_;
  }
  bool IsParallax() const { return parallax_; }
  //
  double FeatDepth(const TrackFeatureId &id) {
    if (!features_.count(id)) {
      LOG(WARNING) << "Feat id " << id << "not exist.";
      return 0.0;
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
 private:
  bool IsParallax(int frame_count, const ImageFeatureTrackerData &image);
  void TriangulateStero(uint64_t it_per_id,
                        const std::vector<transform::Rigid3d> &sw_pose,
                        const std::vector<transform::Rigid3d> &ex_came_to_imu);
  //
  void TriangulateCurAfter(
      uint64_t it_per_id, const std::vector<transform::Rigid3d> &sw_pose,
      const std::vector<transform::Rigid3d> &ex_came_to_imu);

  std::map<TrackFeatureId, FeaturePerId> features_;
  double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
  // const Eigen::Matrix3d *Rs = nullptr;
  // Eigen::Matrix3d ric[2];
  FeatureManagerOption options_;
  bool parallax_ = false;
};

//
class FeatureManagers {
 public:
  //
  FeatureManagers(int cam_trajector, FeatureManagerOption &feat_option);
  FeatureManagers(const std::map<uint64_t, FeatureManager> &feat_ms)
      : feature_managers_(feat_ms) {}
  //
  void AddFeaturesWithData(int id,
                           const std::map<TrackFeatureId, FeaturePerId>& features);
  void RemoveFailures();
  void RemoveBack();
  void RemoveBackShiftDepth(const transform::Rigid3d &marg_p,
                            const transform::Rigid3d &new_p);
  void RemoveFront(int frame_count);
  std::vector<double> GetDepthVector();
  void SetDepth(const std::vector<double> &x);
  FeatureManager *MutableFeatureManager(int cam_id) {
    //
    CHECK(feature_managers_.count(cam_id))
        << "Featuremanger not construct." << cam_id;
    //
    return &feature_managers_.at(cam_id);
  }
  bool CheckParallax() const;
  const std::map<uint64_t, FeatureManager> &GetFeatureManagers() const {
    return feature_managers_;
  }

 private:
   FeatureManagerOption feat_option_;
  std::map<uint64_t, FeatureManager> feature_managers_;
};

}  // namespace estimator
}  // namespace jarvis

#endif