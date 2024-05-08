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

#include <Eigen/Dense>
#include <algorithm>
#include <list>
#include <numeric>
#include <vector>

#include "jarvis/estimator/featureTracker/feature_tracker.h"
#include "jarvis/utility/tic_toc.h"
#include "parameters.h"
namespace jarvis {
namespace estimator {
// using namespace std;
// using namespace Eigen;
class FeaturePerFrame {
 public:
  FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td) {
    point.x() = _point(0);
    point.y() = _point(1);
    point.z() = _point(2);
    uv.x() = _point(3);
    uv.y() = _point(4);
    velocity.x() = _point(5);
    velocity.y() = _point(6);
    cur_td = td;
    is_stereo = false;
  }
  void rightObservation(const Eigen::Matrix<double, 7, 1> &_point) {
    pointRight.x() = _point(0);
    pointRight.y() = _point(1);
    pointRight.z() = _point(2);
    uvRight.x() = _point(3);
    uvRight.y() = _point(4);
    velocityRight.x() = _point(5);
    velocityRight.y() = _point(6);
    is_stereo = true;
  }
  double cur_td;
  Eigen::Vector3d point, pointRight;
  Eigen::Vector2d uv, uvRight;
  Eigen::Vector2d velocity, velocityRight;
  bool is_stereo;
};

class FeaturePerId {
 public:
  const int feature_id;
  int start_frame;
  vector<FeaturePerFrame> feature_per_frame;
  int used_num;
  double estimated_depth;
  int solve_flag;  // 0 haven't solve yet; 1 solve succ; 2 solve fail;

  FeaturePerId(int _feature_id, int _start_frame)
      : feature_id(_feature_id),
        start_frame(_start_frame),
        used_num(0),
        estimated_depth(-1.0),
        solve_flag(0) {}

  int endFrame();
};

class FeatureManager {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FeatureManager(Eigen::Matrix3d _Rs[]);

  void setRic(Eigen::Matrix3d _ric[]);
  void clearState();
  int getFeatureCount();
  bool addFeatureCheckParallax(int frame_count,
                               const ImageFeatureTrackerResult &image,
                               double td);
  std::vector<pair<Eigen::Vector3d, Eigen::Vector3d>> getCorresponding(
      int frame_count_l, int frame_count_r);
  // void updateDepth(const VectorXd &x);
  void setDepth(const Eigen::VectorXd &x);
  void removeFailures();
  void clearDepth();
  Eigen::VectorXd getDepthVector();
  void triangulate(int frameCnt, Eigen::Vector3d Ps[], Eigen::Matrix3d Rs[],
                   Eigen::Vector3d tic[], Eigen::Matrix3d ric[]);
  void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0,
                        Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1,
                        Eigen::Vector3d &point_3d);
  void initFramePoseByPnP(int frameCnt, Eigen::Vector3d Ps[],
                          Eigen::Matrix3d Rs[], Eigen::Vector3d tic[],
                          Eigen::Matrix3d ric[]);
  bool solvePoseByPnP(Eigen::Matrix3d &R_initial, Eigen::Vector3d &P_initial,
                      vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D);
  void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P,
                            Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
  void removeBack();
  void removeFront(int frame_count);
  void removeOutlier(set<int> &outlierIndex);
  list<FeaturePerId> feature;
  int last_track_num = 0;
  double last_average_parallax = 0.0;
  int new_feature_num = 0;
  int long_track_num = 0;

 private:
  double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
  const Eigen::Matrix3d *Rs = nullptr;
  Eigen::Matrix3d ric[2];
};
}  // namespace estimator
}  // namespace jarvis

#endif