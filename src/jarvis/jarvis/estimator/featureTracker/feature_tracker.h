/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/
#ifndef JARVIS_ESTIMATOR_FEATURE_TRACKER_H
#define JARVIS_ESTIMATOR_FEATURE_TRACKER_H

#include <execinfo.h>

#include <csignal>
#include <cstdio>
#include <iostream>
#include <map>
#include <queue>

#include "Eigen/Core"
#include "jarvis/camera_models/camera_models/CataCamera.h"
#include "jarvis/camera_models/camera_models/PinholeCamera.h"
#include "jarvis/camera_models/camera_models/camera_factory.h"
#include "jarvis/common/time.h"
#include "jarvis/estimator/featureTracker/feature_detect.h"
#include "jarvis/estimator/featureTracker/pyramid_image.h"
#include "jarvis/estimator/parameters.h"
#include "jarvis/utility/tic_toc.h"
#include "opencv2/opencv.hpp"
namespace jarvis {
namespace estimator {

//
using TrackFeatureId = uint64_t;
//
struct FeatureData {
  TrackFeatureId id;
  struct CameraFeature {
    Eigen::Vector3d normal_points{0, 0, 0};
    Eigen::Vector2d uv{0, 0};
    Eigen::Vector2d uv_velocity{0, 0};
  };
  std::vector<CameraFeature> camera_features;  // 只在双目当中
};

struct ImageFeatureTrackerData {
  struct Data {
    common::Time time;
    std::map<TrackFeatureId, FeatureData> features;  // feature_id
    std::map<TrackFeatureId, int> tracker_features_num;
    std::vector<cv::Mat> images;
  };
  std::shared_ptr<Data> data;
};

bool inBorder(const cv::Point2f &pt);
void reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status);
void reduceVector(std::vector<int> &v, std::vector<uchar> status);

struct FeatureTrackerOption {
  PyramidImageOption pyrmid_option;
  FeatureDetectOption feature_detect_option;
  std::vector<camera_models::CameraPtr> cameras;
  cv::Mat mask;
  int track_back = 0;
  int max_feat_cnt = 100;
  double ransac_threshold = 1;
  double back_flow_min_distance = 0.5;
  int try_recalc_min_num = 30;
  int  klt_type =0;
  std::vector<std::shared_ptr<PyramidImage>>pyramid_image;
};
struct CalcOpticalFlowPyrLKOption {
  int level = 4;
  cv::Size win_size{7, 7};
  Eigen::Vector2i image_size;
  cv::TermCriteria criteria;
};
//
struct PointCnt {
  cv::Point2f pt;
  int track_cnt;
};
//
class CalcOpticalFlowPyrLK {
 public:
  CalcOpticalFlowPyrLK(const CalcOpticalFlowPyrLKOption &option)
      : options_(option) {}
  virtual void operator()(const std::vector<cv::Mat> &pre_image,
                          const std::vector<cv::Mat> &cur_image,
                          const std::map<uint64_t, PointCnt> &prev_pts,
                          std::map<uint64_t, PointCnt> &cur_pts, int flags = 0);
  virtual ~CalcOpticalFlowPyrLK() {}

 protected:
  bool InBorder(const cv::Point2f &pt);

  CalcOpticalFlowPyrLKOption options_;
};
//
class XpCalcOpticalFlowPyrLK : public CalcOpticalFlowPyrLK {
 public:
  XpCalcOpticalFlowPyrLK(const CalcOpticalFlowPyrLKOption &option)
      : CalcOpticalFlowPyrLK(option) {};
  void operator()(const std::vector<cv::Mat> &pre_image,
                  const std::vector<cv::Mat> &cur_image,
                  const std::map<uint64_t, PointCnt> &prev_pts,
                  std::map<uint64_t, PointCnt> &cur_pts, int flags = 0

  );
};

class FeatureTracker {
 public:
  explicit FeatureTracker(const FeatureTrackerOption &option);
  //
  ImageFeatureTrackerData TrackImage(
      const common::Time &, const cv::Mat &_img,
      const cv::Mat &_img1 = cv::Mat());

  //

  //
  //
  cv::Mat UpdatePointAndMask(std::map<uint64_t, PointCnt> &points);
  ImageFeatureTrackerData TransToTrackerData(
      const std::map<uint64_t, PointCnt> &cur_point,
      const std::map<uint64_t, PointCnt> &cur_r_point);
  //
  FeatureData::CameraFeature FillAndUndistortedPt(
      const std::pair<uint64_t, PointCnt> &pointid,
      const std::map<uint64_t, Eigen::Vector3d> &pre_pointid,
      camera_models::CameraPtr cam, double dt);

  //
  std::vector<uchar> rejectWithF(std::vector<cv::Point2f> &cur_pts,
                                 std::vector<cv::Point2f> &prev_pts);
  //
  //
  //
  std::map<uint64_t, Eigen::Vector2d> UndistortedPts(
      const std::map<uint64_t, PointCnt> &pts, camera_models::CameraPtr cam);
  std::map<uint64_t, Eigen::Vector2d> PtsVelocity(
      const std::map<uint64_t, PointCnt> &pts,
      const std::map<uint64_t, PointCnt> &pre_pts);
  //
  void SetPrediction(const std::map<int, Eigen::Vector3d> &predictPts);
  void RemoveOutliers(const std::set<uint64_t> &removePtsIds);

 private:
  cv::Mat getTrackImage();
  std::map<uint64_t, PointCnt> TrackImage(
      const std::vector<cv::Mat> &pre_image,
      const std::vector<cv::Mat> &cur_image,
      const std::map<uint64_t, PointCnt> &prev_pts,
      const std::map<uint64_t, PointCnt> &init_cur_pts, int flags = 0);
  const FeatureTrackerOption options_;

  std::vector<camera_models::CameraPtr> m_camera;
  std::shared_ptr<PyramidImage> pyramid_image_;
  std::shared_ptr<PyramidImage> r_pyramid_image_;
  std::unique_ptr<FeatureDetect> feature_detect_;
  //
  std::map<uint64_t, PointCnt> prev_pts_;
  std::map<uint64_t, PointCnt> predit_pts_;
  std::map<uint64_t, Eigen::Vector3d> prev_un_pts_;
  std::map<uint64_t, Eigen::Vector3d> prev_un_right_pts_;
  std::unique_ptr<CalcOpticalFlowPyrLK> calc_optical_flow_pyrlk_;
  common::Time prev_time_;
  common::Time curr_time_;
  uint64_t tranck_id_ = 0;
};

}  // namespace estimator
}  // namespace jarvis
#endif