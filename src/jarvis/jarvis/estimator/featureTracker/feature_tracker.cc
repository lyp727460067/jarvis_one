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

#include "feature_tracker.h"

#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/core/types_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgcodecs/legacy/constants_c.h>
#include <opencv2/imgproc/types_c.h>

#include "glog/logging.h"
#include "jarvis/estimator/featureTracker/xppyramid.hpp"
#include "random"
namespace jarvis {
namespace estimator {
namespace {}



//
FeatureTracker::FeatureTracker(const FeatureTrackerOption &option)
    : options_(option) {
  m_camera = options_.cameras;

  VLOG(kGlogLevel) << option.pyrmid_option.image_size;
  feature_detect_ =
      std::make_unique<FeatureDetect>(option.feature_detect_option);
  if (options_.pyramid_image.empty()) {
    pyramid_image_ = std::make_unique<PyramidImage>(option.pyrmid_option);
    r_pyramid_image_ = std::make_unique<PyramidImage>(option.pyrmid_option);
  } else {
    pyramid_image_ = options_.pyramid_image[0];
    if (options_.pyramid_image.size()==2) {
      r_pyramid_image_ = options_.pyramid_image[1];
    }
  }
  cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                            0.01);
  CalcOpticalFlowPyrLKOption klt_option{
      options_.pyrmid_option.layer,
      cv::Size{options_.pyrmid_option.lk_win_size,
               options_.pyrmid_option.lk_win_size},
      options_.pyrmid_option.image_size, criteria};
  if (options_.klt_type == 0) {
    calc_optical_flow_pyrlk_ =
        std::make_unique<CalcOpticalFlowPyrLK>(klt_option);
    //  calc_optical_flow_pyrlk_ =
    //     std::make_unique<CalcOpticalFlowPyrLK>(klt_option);
 
  } else {
    calc_optical_flow_pyrlk_ =
        std::make_unique<XpCalcOpticalFlowPyrLK>(klt_option);
    // calc_optical_flow_pyrlk_r_ =
    //     std::make_unique<XpCalcOpticalFlowPyrLK>(klt_option);


  }
}
//
//
cv::Mat FeatureTracker::UpdatePointAndMask(
    std::map<uint64_t, PointCnt> &points) {
  //
  cv::Mat mask = options_.mask.clone();
  // prefer to keep features that are tracked for long time
  std::vector<std::pair<int, std::pair<cv::Point2f, int>>> cnt_pts_id;
  //
  for (const auto &point : points) {
    cnt_pts_id.push_back(std::make_pair(
        point.second.track_cnt, std::make_pair(point.second.pt, point.first)));
  }
  std::sort(cnt_pts_id.begin(), cnt_pts_id.end(),
            [](const std::pair<int, std::pair<cv::Point2f, int>> &a,
               const std::pair<int, std::pair<cv::Point2f, int>> &b) {
              return a.first > b.first;
            });

  points.clear();

  for (auto &it : cnt_pts_id) {
    if (mask.at<uchar>(it.second.first) >= 128) {
      points[it.second.second].pt = it.second.first;
      points[it.second.second].track_cnt = it.first;
      cv::circle(mask, it.second.first,
                 options_.feature_detect_option.mask_min_dist, 0, -1);
    }
  }
  return mask;
}

double distance(const cv::Point2f &pt1,
                                const cv::Point2f &pt2) {
  // printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
  double dx = pt1.x - pt2.x;
  double dy = pt1.y - pt2.y;
  return sqrt(dx * dx + dy * dy);
}

#if 1
cv::Mat GenerateImageWithKeyPoint(const cv::Mat &l_img,
                                  std::map<uint64_t, PointCnt> l_key_points,
                                  const cv::Mat &r_img,
                                  std::map<uint64_t, PointCnt> r_key_points) {
  int col = l_img.cols;
  int row = l_img.rows;
  cv::Mat l_img_tmp;
  cv::Mat r_img_tmp;

  // cv::Mat l_img_feat;
  // cv::cvtColor(l_img, l_img_feat, cv::COLOR_GRAY2RGB);
  // cv::Mat r_img_feat;
  // cv::cvtColor(r_img, r_img_feat, cv::COLOR_GRAY2RGB);
  // //
  const int gap = 10;
  const int v_gap = 0;

  cv::Mat gap_image(row + v_gap, gap, CV_8UC1, cv::Scalar(255, 255, 255));
  cv::Mat v_gap_image(v_gap, col, CV_8UC1, cv::Scalar(0, 0, 0));
  cv::vconcat(v_gap_image, l_img, l_img_tmp);
  //

  for (auto &p : l_key_points) {
    p.second.pt.y += v_gap;
  }
    cv::Mat gray_img, loop_match_img;  
  if (!r_img.empty()) {
    cv::vconcat(r_img, v_gap_image, r_img_tmp);

    cv::hconcat(l_img_tmp, gap_image, gap_image);
    cv::hconcat(gap_image, r_img_tmp, gray_img);
  }else {

    gray_img = l_img;
  }
  // common::FixedRatioSampler sampler(0.1);
  cvtColor(gray_img, loop_match_img, cv::COLOR_GRAY2RGB);
  std::mt19937 rng(42);
  std::uniform_int_distribution r_bound_distribution(1, 255);
  std::uniform_int_distribution b_bound_distribution(1, 255);
  std::uniform_int_distribution g_ound_distribution(1, 255);

  //
  for (auto &keypoint : (l_key_points)) {
    cv::Scalar color = cv::Scalar(255, 0, 0);
    // if (class_id.count(keypoint.class_id)) {
    //   cv::circle(loop_match_img, keypoint.pt, 2 ,cv::Scalar(0, 0, 255), 2);
    // } else {
    cv::circle(loop_match_img, keypoint.second.pt, 2, color, 2);
    // }
    // cv::putText(loop_match_img, std::to_string(keypoint.class_id),
    // keypoint.pt,
    //             cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
  }
  for (auto &&keypoint : r_key_points) {
    cv::circle(
        loop_match_img,
        {(int)keypoint.second.pt.x + (col + gap), (int)keypoint.second.pt.y}, 1,
        cv::Scalar(0, 255, 0), 2);
  }

  for (auto &p : r_key_points) {
    cv::Point2f old_pt = p.second.pt;
    old_pt.x += (col + gap);
    cv::line(loop_match_img, l_key_points[p.first].pt, old_pt,
             cv::Scalar(r_bound_distribution(rng), b_bound_distribution(rng),
                        g_ound_distribution(rng)),
             1, 8, 0);
  }
  // cv::Mat notation(50, col + gap + col, CV_8UC3, cv::Scalar(255, 255, 255));
  // putText(notation, l_name, cv::Point2f(20, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
  //         cv::Scalar(255), 3);
  // putText(notation, r_name, cv::Point2f(20 + col + gap, 30),
  //         cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 3);
  // cv::vconcat(notation, loop_match_img, loop_match_img);
  return loop_match_img;
}
#endif
bool CalcOpticalFlowPyrLK::InBorder(const cv::Point2f &pt) {
  const int BORDER_SIZE = 1;
  int img_x = cvRound(pt.x);
  int img_y = cvRound(pt.y);
  return BORDER_SIZE <= img_x &&
         img_x < options_.image_size.x() - BORDER_SIZE &&
         BORDER_SIZE <= img_y &&
         img_y < options_.image_size.y() - BORDER_SIZE;
}
//
void CalcOpticalFlowPyrLK::operator()(
    const std::vector<cv::Mat> &pre_image,
    const std::vector<cv::Mat> &cur_image,
    const std::map<uint64_t, PointCnt> &prev_pts,
    std::map<uint64_t, PointCnt> &cur_pts, int flags) {
  // CHECK_EQ(int(pre_image.size()), (options_.level+1) * 2)
      // << "Image need deriv image";
  //
  std::vector<uchar> status;
  std::vector<float> err;
  std::vector<cv::Point2f> v_prev_pts(prev_pts.size());
  std::vector<cv::Point2f> v_cur_pts;
  //

  int i = 0;
  std::vector<uint64_t> ids(prev_pts.size());
  if (!cur_pts.empty()) {
    v_cur_pts.resize(cur_pts.size());
  }

  for (const auto &pt : prev_pts) {
    v_prev_pts[i] = pt.second.pt;
    ids[i] = pt.first;
    if (!cur_pts.empty()) {
      CHECK(cur_pts.count(pt.first));
      v_cur_pts[i] = cur_pts[pt.first].pt;
    }
    i++;
  }
  //
  // cv::imshow("2",cur_image[0]);
  // cv::imshow("1",pre_image[0]);
  // cv::waitKey(0);
  //
  // cv::calcOpticalFlowPyrLK(pre_image, cur_image, v_prev_pts, v_cur_pts, status,
  //                          err, options_.win_size, options_.level,
  //                          options_.criteria, flags);
  cur_pts.clear();
  for (int i = 0; i < int(status.size()); i++) {
    if (status[i] && InBorder(v_cur_pts[i])) {
      cur_pts[ids[i]].pt = v_cur_pts[i];
      cur_pts[ids[i]].track_cnt = prev_pts.at(ids[i]).track_cnt;
    }
  }
}

void XpCalcOpticalFlowPyrLK::operator()(const std::vector<cv::Mat> &pre_image,
                                        const std::vector<cv::Mat> &cur_image,
                                        const std::map<uint64_t, PointCnt> &prev_pts ,
                                        std::map<uint64_t, PointCnt> &cur_pts,
                                        int flags) {
  CHECK_LE(options_.level, 4);
  std::vector<float> err;
  std::vector<bool> status;
  const int start_level = 0;
  //
  std::vector<XP::XP_OPTICAL_FLOW::XPKeyPoint> pre_xp_kp_small;
  pre_xp_kp_small.reserve(prev_pts.size());
  std::vector<Point2f> v_cur_pts;
  if (!cur_pts.empty()) {
    v_cur_pts.resize(cur_pts.size());
    CHECK_EQ(v_cur_pts.size(), prev_pts.size());
  }
  //
  std::vector<uint64_t> ids(prev_pts.size());
  int i = 0;
  for (const auto &p : prev_pts) {
    pre_xp_kp_small.push_back(XP::XP_OPTICAL_FLOW::XPKeyPoint(p.second.pt));
    ids[i] = p.first;
    if (!cur_pts.empty()) {
      v_cur_pts[i] = cur_pts[p.first].pt;
    }
    i++;
  }

  XP::XP_OPTICAL_FLOW::XPcalcOpticalFlowPyrLKWithDeriv(
      pre_image, cur_image, &pre_xp_kp_small, &v_cur_pts, &status, &err,
      options_.win_size, options_.level, start_level, options_.criteria, flags);

  cur_pts.clear();
  for (int i = 0; i < int(status.size()); i++) {
    if (status[i] && InBorder(v_cur_pts[i])) {
      cur_pts[ids[i]].pt = v_cur_pts[i];
      cur_pts[ids[i]].track_cnt = prev_pts.at(ids[i]).track_cnt;
    }
  }
}

//
//
template <typename A, typename B>
void MapIntersection(const std::map<A, B> &a, const std::map<A, B> &b,
                     std::map<A, B> &resulta) {
  auto a_it = a.begin();
  auto b_it = b.begin();
  while (a_it != a.end() && b_it != b.end()) {
    if (a_it->first == b_it->first) {
      // *result = *a_it;
      resulta.insert(*a_it);
      ++a_it;
      ++b_it;
    } else if (a_it->first < b_it->first) {
      a_it = a.lower_bound(b_it->first);
    } else {
      b_it = b.lower_bound(a_it->first);
    }
  }
}
std::map<uint64_t, PointCnt> FeatureTracker::TrackImage(
    const std::vector<cv::Mat> &pre_image,
    const std::vector<cv::Mat> &cur_image,
    const std::map<uint64_t, PointCnt> &prev_pts,
    const std::map<uint64_t, PointCnt> &init_cur_pts,
    int flags) {
  if (prev_pts.empty()) return {};
  std::map<uint64_t, PointCnt> cur_pts = init_cur_pts;
  auto &calc_optical_flow_pyrlk = *calc_optical_flow_pyrlk_;
  calc_optical_flow_pyrlk(pre_image, cur_image, prev_pts, cur_pts, flags);
  const int succ_num = cur_pts.size();
  // LOG(INFO)<<"pre pts size:"<<prev_pts.size();
  if (succ_num < options_.try_recalc_min_num && flags != 0) {
    cur_pts.clear();
    calc_optical_flow_pyrlk(pre_image, cur_image, prev_pts, cur_pts, flags);
  }
  // LOG(INFO)<<"cur pts size:"<< cur_pts.size();
  //
  if(cur_pts.empty())return {};
  if (options_.track_back) {
    std::map<uint64_t, PointCnt> prev_pts_tmp;
    //
    MapIntersection(prev_pts, cur_pts, prev_pts_tmp);
    calc_optical_flow_pyrlk(cur_image, pre_image, cur_pts, prev_pts_tmp,
                            cv::OPTFLOW_USE_INITIAL_FLOW);

    // LOG(INFO)<<"back pts size:"<<  prev_pts_tmp.size();
    for (auto it = cur_pts.begin(); it != cur_pts.end();) {
      if (prev_pts_tmp.count(it->first) == 0) {
        it = cur_pts.erase(it);
        continue;
      }
      if (distance(prev_pts.at(it->first).pt, prev_pts_tmp[it->first].pt) >
          options_.back_flow_min_distance) {
        it = cur_pts.erase(it);
        continue;
      }
      ++it;
    }
  }
  return cur_pts;
}
//
ImageFeatureTrackerData FeatureTracker::TrackImage(
    const common::Time &time, const cv::Mat &_img,
    const cv::Mat &_img1) {
  curr_time_ =time;
  //
  TicToc t_t;
  pyramid_image_->Build(_img);
  //
  std::map<uint64_t, PointCnt> cur_pts;
  if (!predit_pts_.empty()) {
    cur_pts =
        TrackImage(pyramid_image_->PrePyram(), pyramid_image_->CurrPyram(),
                   prev_pts_,predit_pts_, cv::OPTFLOW_USE_INITIAL_FLOW);
  } else {
    cur_pts = TrackImage(pyramid_image_->PrePyram(),
                         pyramid_image_->CurrPyram(), prev_pts_, prev_pts_);
  }
  //
  VLOG(kGlogCostTimeLevel) << "TrackImage costs " << t_t.toc() << " ms";
  for (auto &p : cur_pts) {
    p.second.track_cnt++;
  }
  //

  TicToc mask_t_t;
  const cv::Mat mask = UpdatePointAndMask(cur_pts);

  VLOG(kGlogCostTimeLevel) << "set mask costs " << mask_t_t.toc() << " ms";
  std::vector<cv::Point2f> v_cur_pts;
  //

  std::map<TrackFeatureId, int> tracker_features_num;

  for (auto &p : cur_pts) {
    tracker_features_num.emplace(p.first, p.second.track_cnt);
    v_cur_pts.push_back(p.second.pt);
  }
  // auto shwo_image1 =
  //     GenerateImageWithKeyPoint(_img, cur_pts, cv::Mat(),{});

  // cv::imshow("shwo_image1 ", shwo_image1);
  int n_max_cnt = options_.max_feat_cnt - static_cast<int>(cur_pts.size());
  auto n_pts = feature_detect_->Detect(_img, v_cur_pts, n_max_cnt,
                                       pyramid_image_->CurrPyram()[1], mask);

  VLOG(kGlogLevel) << "Feature detect new num " << n_pts.size();

  for (int i = 0; i < int(n_pts.size()); i++) {
    cur_pts.emplace(tranck_id_, PointCnt{n_pts[i], 1});
    tranck_id_+=1;
  }
  
  std::map<uint64_t, PointCnt> cur_right_pts ;
  if (!_img1.empty()) {

    TicToc t_t;
    r_pyramid_image_->Build(_img1);
    cur_right_pts = TrackImage(pyramid_image_->CurrPyram(),
                               r_pyramid_image_->CurrPyram(), cur_pts, {});


    VLOG(kGlogLevel) << "Track r  num:" << cur_right_pts.size();
    VLOG(kGlogCostTimeLevel) << "Track r Image costs " << t_t.toc() << " ms";
  }
  
  // auto shwo_image =
  //     GenerateImageWithKeyPoint(_img, cur_pts, _img1, cur_right_pts);

  // cv::imshow("shwo_image", shwo_image);
  // cv::waitKey(0);
  TicToc tran_t_t;
  auto result_data = TransToTrackerData(cur_pts, cur_right_pts);
  // result_data.data->images.push_back(_img);
  // if (_img1.empty()) {
  //   result_data.data->images.push_back(_img1);
  // }
  result_data.data->tracker_features_num = std::move(tracker_features_num);
  result_data.data->time = curr_time_;
  prev_time_ = curr_time_;
  prev_pts_ = std::move(cur_pts);
  predit_pts_.clear();
  //

    VLOG(kGlogCostTimeLevel) << "tranck other costs " << tran_t_t.toc() << " ms";
  return result_data;
}

//
ImageFeatureTrackerData FeatureTracker::TransToTrackerData(
    const std::map<uint64_t, PointCnt> &cur_point,
    const std::map<uint64_t, PointCnt> &cur_r_point) {
  ImageFeatureTrackerData::Data result_data;
  //
  double dt = common::ToSeconds(curr_time_ - prev_time_);
  //
  std::map<uint64_t, Eigen::Vector3d> prev_un_pts;
  std::map<uint64_t, Eigen::Vector3d> prev_un_right_pts;
  for (auto &p : cur_point) {

    //
    FeatureData::CameraFeature feature =
        FillAndUndistortedPt(p, prev_un_pts_, m_camera[0], dt);
    result_data.features[p.first] =
        FeatureData{TrackFeatureId(p.first), {feature}};
    prev_un_pts[p.first] = feature.normal_points;
  }
  for (auto &p : cur_r_point) {
    //
    
    FeatureData::CameraFeature feature =
        FillAndUndistortedPt(p, prev_un_right_pts_, m_camera[1], dt);
    CHECK( cur_point.count(p.first))<<p.first;
    result_data.features[p.first].camera_features.push_back(feature);
    prev_un_right_pts[p.first] = feature.normal_points;

  }

  prev_un_right_pts_ = std::move(prev_un_right_pts);
  prev_un_pts_ = std::move(prev_un_pts);
  return ImageFeatureTrackerData{
      std::make_shared<ImageFeatureTrackerData::Data>(result_data)};
}
//
std::vector<uchar> FeatureTracker::rejectWithF(
    std::vector<cv::Point2f> &cur_pts, std::vector<cv::Point2f> &prev_pts) {
  // if (cur_pts.size() >= 8) {
  //   VLOG(kGlogLevel) << "FM ransac begins";
  //   TicToc t_f;
  //   std::vector<cv::Point2f> un_cur_pts(cur_pts.size()),
  //       un_prev_pts(prev_pts.size());
  //   for (unsigned int i = 0; i < cur_pts.size(); i++) {
  //     Eigen::Vector3d tmp_p;
  //     const std::vector<double> &cam0_intric =
  //         options_.calibrate_option.camera_options[0].intrinsics;

  //     m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x,
  //     cur_pts[i].y),
  //                                 tmp_p);
  //     //
  //     tmp_p.x() = cam0_intric[0] * tmp_p.x() / tmp_p.z() + cam0_intric[2];
  //     tmp_p.y() = cam0_intric[1] * tmp_p.y() / tmp_p.z() + cam0_intric[3];
  //     un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

  //     const std::vector<double> &cam1_intric =
  //         options_.calibrate_option.camera_options[1].intrinsics;

  //     m_camera[1]->liftProjective(Eigen::Vector2d(prev_pts[i].x,
  //     prev_pts[i].y),
  //                                 tmp_p);
  //     tmp_p.x() = cam1_intric[0] * tmp_p.x() / tmp_p.z() + cam1_intric[2];
  //     tmp_p.y() = cam1_intric[1] * tmp_p.y() / tmp_p.z() + cam1_intric[3];
  //     un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
  //   }

  //   std::vector<uchar> status;
  //   cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC,
  //                          options_.ransac_threshold, 0.99, status);
  //   int size_a = cur_pts.size();

  //   // reduceVector(prev_pts, status);
  //   // reduceVector(cur_pts, status);
  //   // reduceVector(cur_un_pts, status);
  //   // reduceVector(ids, status);
  //   // reduceVector(track_cnt, status);
  //   const int ransac_inli_cnt = std::count(status.begin(), status.end(), 1);
  //   VLOG(kGlogLevel) << "FM ransac: " << size_a << " -> " << ransac_inli_cnt
  //                    << " " << 1.0 * ransac_inli_cnt / size_a;
  //   return status;
  // }
  return {};
}

//
FeatureData::CameraFeature FeatureTracker::FillAndUndistortedPt(
    const std::pair<uint64_t, PointCnt> &pointid,
    const std::map<uint64_t, Eigen::Vector3d> &pre_pointid,
    camera_models::CameraPtr cam, double dt) {
  Eigen::Vector2d a(pointid.second.pt.x, pointid.second.pt.y);
  Eigen::Vector3d b;
  cam->liftProjective(a, b);
  Eigen::Vector2d pts_velocity{0, 0};
  Eigen::Vector3d norm_points = b / b.z();
  // LOG(INFO)<<pointid.first<<" "<<norm_points.transpose(); 
  if (pre_pointid.count(pointid.first)) {
    pts_velocity =
        ((norm_points - pre_pointid.at(pointid.first)) / dt).head<2>();
  }
  return FeatureData::CameraFeature{norm_points, a, pts_velocity};
}

std::map<uint64_t, Eigen::Vector2d> FeatureTracker::UndistortedPts(
    const std::map<uint64_t, PointCnt> &pts, camera_models::CameraPtr cam) {
  std::map<uint64_t, Eigen::Vector2d> un_pts;
  for (const auto &point : pts) {
    Eigen::Vector2d a(point.second.pt.x, point.second.pt.y);
    Eigen::Vector3d b;
    cam->liftProjective(a, b);
    un_pts.emplace(point.first, (b / b.z()).head<2>());
  }
  return un_pts;
}

//
std::map<uint64_t, Eigen::Vector2d> FeatureTracker::PtsVelocity(
    const std::map<uint64_t, PointCnt> &pts,
    const std::map<uint64_t, PointCnt> &pre_pts) {
  std::map<uint64_t, Eigen::Vector2d> pts_velocity;
  if (pre_pts.empty()) {
    for (auto &p : pts) {
      pts_velocity.emplace(p.first, Eigen::Vector2d::Zero());
    }
    return pts_velocity;
  }
  double dt = common::ToSeconds(curr_time_ - prev_time_);

  for (auto &p : pts) {
    std::map<int, cv::Point2f>::iterator it;
    if (pre_pts.count(p.first)) {
      const double v_x = (p.second.pt.x - pre_pts.at(p.first).pt.x) / dt;
      const double v_y = (p.second.pt.y - pre_pts.at(p.first).pt.y) / dt;

      pts_velocity.emplace(p.first, Eigen::Vector2d(v_x, v_y));

    } else {
      pts_velocity[p.first] = Eigen::Vector2d::Zero();
    }
  }

  return pts_velocity;
}
//
void FeatureTracker::SetPrediction(
    const std::map<int, Eigen::Vector3d> &predictPts) {
  for (auto &point : predit_pts_) {
    if (predictPts.count(point.first)) {
      Eigen::Vector2d tmp_uv;
      m_camera[0]->spaceToPlane(predictPts.at(point.first), tmp_uv);
      //
      predit_pts_.emplace(point.first,
                          PointCnt{cv::Point2f(tmp_uv.x(), tmp_uv.y()),
                                   point.second.track_cnt});
    } else {
      predit_pts_[point.first] = point.second;
    }
  }
}

void FeatureTracker::RemoveOutliers(const std::set<uint64_t> &removePtsIds) {
  for (auto &id : removePtsIds) {
    prev_pts_.erase(id);
    predit_pts_.erase(id);
    prev_un_pts_.erase(id);
    prev_un_right_pts_.erase(id);
  }
}

}  // namespace estimator
}  // namespace jarvis