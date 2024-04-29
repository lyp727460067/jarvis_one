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
#include "Eigen/Eigenvalues"
#include "glog/logging.h"
namespace jarvis {
namespace estimator {
namespace {
constexpr uint8_t kGlogLevel = 1;
}

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace std::chrono;
vector<double> convolution(const cv::Mat &image,
                           const vector<cv::KeyPoint> &pts,
                           const cv::Mat &kernal) {
  double pixSum = 0;
  vector<double> grad;
  for (size_t i = 0; i < pts.size(); i++) {
    int row = floor(pts[i].pt.y);
    int col = floor(pts[i].pt.x);
    // the pixel locates in the middle of the kernal
    for (int k = 0; k < kernal.rows; k++)
      for (int l = 0; l < kernal.cols; l++)
        pixSum += kernal.at<double>(k, l) *
                  double(image.at<uchar>(k + row - 1, l + col - 1));
    grad.push_back(abs(pixSum));
    pixSum = 0;
  }

  return grad;
}

typedef pair<int, double> PAIR;
bool cmp_by_value(const PAIR &lhs, const PAIR &rhs) {
  return lhs.second > rhs.second;
}

// have_corners：图像已有特征点， corners：用来存放额外检测到的特征点，
// maxCorners:图像最多特征点数量
void efficientGoodFeaturesToTrack(InputArray _image,
                                  vector<cv::Point2f> &have_corners,
                                  vector<cv::Point2f> &corners, int maxCorners,
                                  double minDistance) {
  corners.clear();
  Mat image = _image.getMat();
  if (image.empty()) return;

  vector<cv::KeyPoint> keypoints;
  cv::FAST(image, keypoints, 20, true);

  if (keypoints.empty()) return;

  cv::Mat kernal_x = (cv::Mat_<double>(3, 3) << -1, 0, 1, -2, 0, 2, -1, 0, 1);
  cv::Mat kernal_y = (cv::Mat_<double>(3, 3) << -1, -2, -1, 0, 0, 0, 1, 2, 1);
  vector<double> grad_x;
  vector<double> grad_y;
  grad_x = convolution(image, keypoints, kernal_x);
  grad_y = convolution(image, keypoints, kernal_y);

  // 2. minMaxLoc
  std::vector<pair<int, double>> eigens;
  for (size_t i = 0; i < grad_x.size(); i++) {
    Eigen::Matrix2d cov;
    cov(0, 0) = grad_x[i] * grad_x[i];
    cov(0, 1) = grad_x[i] * grad_y[i];
    cov(1, 0) = grad_x[i] * grad_y[i];
    cov(1, 1) = grad_y[i] * grad_y[i];

    Eigen::EigenSolver<Matrix2d> es(cov);
    Eigen::Vector2cd eig_ = es.eigenvalues();
    Vector2d eig = eig_.real();
    double eg1 = eig(0);
    double eg2 = eig(1);
    if (eg1 >= eg2)
      eigens.push_back(make_pair(i, eg1));
    else
      eigens.push_back(make_pair(i, eg2));
  }

  sort(eigens.begin(), eigens.end(), cmp_by_value);
  vector<cv::KeyPoint> keypoints_;
  for (size_t i = 0; i < eigens.size(); i++)
    keypoints_.push_back(keypoints[eigens[i].first]);

  if (minDistance >= 1) {
    int ncorners = 0;
    int w = image.cols;
    int h = image.rows;

    const int cell_size = cvRound(minDistance);
    const int grid_width = (w + cell_size - 1) / cell_size;
    const int grid_height = (h + cell_size - 1) / cell_size;

    std::vector<std::vector<cv::Point2f>> grid(grid_width * grid_height);

    minDistance *= minDistance;
    // push the already exist feature points into the grid
    for (size_t i = 0; i < have_corners.size(); i++) {
      int y = (int)(have_corners[i].y);
      int x = (int)(have_corners[i].x);

      int x_cell = x / cell_size;
      int y_cell = y / cell_size;

      if (x_cell <= grid_width && y_cell <= grid_height)
        grid[y_cell * grid_width + x_cell].push_back(have_corners[i]);

      corners.push_back(have_corners[i]);
      ++ncorners;
    }

    for (size_t i = 0; i < keypoints_.size(); i++) {
      if (keypoints_[i].pt.y < 0 || keypoints_[i].pt.y > image.rows - 1)
        continue;
      if (keypoints_[i].pt.x < 0 || keypoints_[i].pt.x > image.cols - 1)
        continue;
      int y = (int)(keypoints_[i].pt.y);
      int x = (int)(keypoints_[i].pt.x);

      bool good = true;

      int x_cell = x / cell_size;
      int y_cell = y / cell_size;

      int x1 = x_cell - 1;
      int y1 = y_cell - 1;
      int x2 = x_cell + 1;
      int y2 = y_cell + 1;

      // boundary check
      x1 = std::max(0, x1);
      y1 = std::max(0, y1);
      x2 = std::min(grid_width - 1, x2);
      y2 = std::min(grid_height - 1, y2);

      // select feature points satisfy minDistance threshold
    for (int  yy = y1; yy <= y2; yy++) {
        for (int xx = x1; xx <= x2; xx++) {
          std::vector<cv::Point2f> &m = grid[yy * grid_width + xx];

          if (m.size()) {
            for (size_t j = 0; j < m.size(); j++) {
              float dx = x - m[j].x;
              float dy = y - m[j].y;

              if (dx * dx + dy * dy < minDistance) {
                good = false;
                goto break_out;
              }
            }
          }
        }
      }

    break_out:

      if (good) {
        grid[y_cell * grid_width + x_cell].push_back(
            cv::Point2f((float)x, (float)y));

        corners.push_back(cv::Point2f((float)x, (float)y));
        ++ncorners;

        if (maxCorners > 0 && (int)ncorners == maxCorners) break;
      }
    }

  } else {
    return;
  }

  if (have_corners.size() != 0)
    corners.erase(corners.begin(), corners.end() + have_corners.size());
}

bool FeatureTracker::inBorder(const cv::Point2f &pt) {
  const int BORDER_SIZE = 1;
  int img_x = cvRound(pt.x);
  int img_y = cvRound(pt.y);
  return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE &&
         BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;
}

double distance(cv::Point2f pt1, cv::Point2f pt2) {
  // printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
  double dx = pt1.x - pt2.x;
  double dy = pt1.y - pt2.y;
  return sqrt(dx * dx + dy * dy);
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status) {
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i]) v[j++] = v[i];
  v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status) {
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i]) v[j++] = v[i];
  v.resize(j);
}

FeatureTracker::FeatureTracker() {
  mask_ = cv::imread(
      "/oem/mowpack/vslam/configuration/mask.png",
      cv::IMREAD_GRAYSCALE);
  stereo_cam = 0;
  n_id = 0;
  hasPrediction = false;
}

void FeatureTracker::setMask() {
  mask = mask_.clone();

  // prefer to keep features that are tracked for long time
  vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

  for (unsigned int i = 0; i < cur_pts.size(); i++)
    cnt_pts_id.push_back(
        make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));

  std::sort(cnt_pts_id.begin(), cnt_pts_id.end(),
            [](const pair<int, pair<cv::Point2f, int>> &a,
               const pair<int, pair<cv::Point2f, int>> &b) {
              return a.first > b.first;
            });

  cur_pts.clear();
  ids.clear();
  track_cnt.clear();

  for (auto &it : cnt_pts_id) {
    if (mask.at<uchar>(it.second.first) == 255) {
      cur_pts.push_back(it.second.first);
      ids.push_back(it.second.second);
      track_cnt.push_back(it.first);
      cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
    }
  }
}

double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2) {
  // printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
  double dx = pt1.x - pt2.x;
  double dy = pt1.y - pt2.y;
  return sqrt(dx * dx + dy * dy);
}

ImageFeatureTrackerResult
FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img,
                           const cv::Mat &_img1,std::map<int,int>* track_num  ) {
  TicToc t_r;
  cur_time = _cur_time;
  cur_img = _img;
  row = cur_img.rows;
  col = cur_img.cols;
  cv::Mat rightImg = _img1;
  /*
  {
      cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
      clahe->apply(cur_img, cur_img);
      if(!rightImg.empty())
          clahe->apply(rightImg, rightImg);
  }
  */
  cur_pts.clear();

  if (prev_pts.size() > 0) {
    TicToc t_o;
    vector<uchar> status;
    vector<float> err;
    if (hasPrediction) {
      cur_pts = predict_pts;
      cv::calcOpticalFlowPyrLK(
          prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21),
          2,
          cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                           0.01),
          cv::OPTFLOW_USE_INITIAL_FLOW);

      int succ_num = 0;
      for (size_t i = 0; i < status.size(); i++) {
        if (status[i]) succ_num++;
      }
      if (succ_num < 10)
        cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status,
                                 err, cv::Size(21, 21), 4);
    } else
      cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status,
                               err, cv::Size(21, 21), 4);
    // reverse check
    if (FLOW_BACK) {
      vector<uchar> reverse_status;
      vector<cv::Point2f> reverse_pts = prev_pts;
      cv::calcOpticalFlowPyrLK(
          cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err,
          cv::Size(21, 21), 1,
          cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                           0.01),
          cv::OPTFLOW_USE_INITIAL_FLOW);
      // cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts,
      // reverse_status, err, cv::Size(21, 21), 3);
      for (size_t i = 0; i < status.size(); i++) {
        if (status[i] && reverse_status[i] &&
            distance(prev_pts[i], reverse_pts[i]) <= 0.5) {
          status[i] = 1;
        } else
          status[i] = 0;
      }
    }

    for (int i = 0; i < int(cur_pts.size()); i++)
      if (status[i] && !inBorder(cur_pts[i])) status[i] = 0;
    reduceVector(prev_pts, status);
    reduceVector(cur_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);
    VLOG(kGlogLevel) << "temporal optical flow costs:" << t_o.toc() << "ms";
    // printf("track cnt %d\n", (int)ids.size());
  }

  for (auto &n : track_cnt) n++;

  if (1) {
    // rejectWithF();
    VLOG(kGlogLevel) << "set mask begins";
    TicToc t_m;
    setMask();
    VLOG(kGlogLevel) << "set mask costs " << t_m.toc() << "ms";
    VLOG(kGlogLevel) << "detect feature begins";
    TicToc t_t;
    int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
    if (n_max_cnt > 0) {
      if (mask.empty()) cout << "mask is empty " << endl;
      if (mask.type() != CV_8UC1) cout << "mask type wrong " << endl;
      // cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01,
      //                         MIN_DIST, mask);
      vector<cv::Point2f> forw_pts;
      efficientGoodFeaturesToTrack(cur_img, forw_pts, n_pts,
                                   MAX_CNT - cur_pts.size(), MIN_DIST);
    } else {
      n_pts.clear();
    }
    VLOG(kGlogLevel) << "detect feature costs: " << t_t.toc() << " ms";

    for (auto &p : n_pts) {
      cur_pts.push_back(p);
      ids.push_back(n_id++);
      track_cnt.push_back(1);
    }
    // printf("feature cnt after add %d\n", (int)ids.size());
  }

  cur_un_pts = undistortedPts(cur_pts, m_camera[0]);
  pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

  if (!_img1.empty() && stereo_cam) {
    ids_right.clear();
    cur_right_pts.clear();
    cur_un_right_pts.clear();
    right_pts_velocity.clear();
    cur_un_right_pts_map.clear();
    if (!cur_pts.empty()) {
      // printf("stereo image; track feature on right image\n");
      vector<cv::Point2f> reverseLeftPts;
      vector<uchar> status, statusRightLeft;
      vector<float> err;
      // cur left ---- cur right
      cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts,
                               status, err, cv::Size(21, 21), 4);
      // reverse check cur right ---- cur left
      if (FLOW_BACK) {
        cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts,
                                 reverseLeftPts, statusRightLeft, err,
                                 cv::Size(21, 21), 4);
        for (size_t i = 0; i < status.size(); i++) {
          if (status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) &&
              distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
            status[i] = 1;
          else
            status[i] = 0;
        }
      }

      ids_right = ids;
      reduceVector(cur_right_pts, status);
      reduceVector(ids_right, status);
      // only keep left-right pts
      /*
      reduceVector(cur_pts, status);
      reduceVector(ids, status);
      reduceVector(track_cnt, status);
      reduceVector(cur_un_pts, status);
      reduceVector(pts_velocity, status);
      */
      cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
      right_pts_velocity =
          ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map,
                      prev_un_right_pts_map);
    }
    prev_un_right_pts_map = cur_un_right_pts_map;
  }
  if (SHOW_TRACK)
    drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);

  prev_img = cur_img;
  prev_pts = cur_pts;
  prev_un_pts = cur_un_pts;
  prev_un_pts_map = cur_un_pts_map;
  prev_time = cur_time;
  hasPrediction = false;

  prevLeftPtsMap.clear();
  for (size_t i = 0; i < cur_pts.size(); i++)
    prevLeftPtsMap[ids[i]] = cur_pts[i];

  // map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
  ImageFeatureTrackerResult::Data result_data;
  result_data.time = _cur_time;
  for (size_t i = 0; i < ids.size(); i++) {
    int feature_id = ids[i];
    double x, y, z;
    x = cur_un_pts[i].x;
    y = cur_un_pts[i].y;
    z = 1;
    double p_u, p_v;
    p_u = cur_pts[i].x;
    p_v = cur_pts[i].y;
    int camera_id = 0;
    double velocity_x, velocity_y;
    velocity_x = pts_velocity[i].x;
    velocity_y = pts_velocity[i].y;

    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
    xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
    result_data.features[feature_id] =
        ImageFeatureTrackerResult::FeatureTrackerResult{
            feature_id,
            {ImageFeatureTrackerResult::FeatureTrackerResult::CameraFeature{
                camera_id, Eigen::Vector3d{x, y, z}, Eigen::Vector2d{p_u, p_v},
                Eigen::Vector2d{velocity_x, velocity_y}}},
        };
    result_data.tracker_features_num[feature_id] = track_cnt[i];
    // featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
    // if (track_num) {
    //   (*track_num)[feature_id] = track_cnt[i];
    // }
  }
  
  result_data.images[0]= cur_img;
  if (!_img1.empty() && stereo_cam) {
    int camera_id = 1;
    for (size_t i = 0; i < ids_right.size(); i++) {
      int feature_id = ids_right[i];
      double x, y, z;
      x = cur_un_right_pts[i].x;
      y = cur_un_right_pts[i].y;
      z = 1;
      double p_u, p_v;
      p_u = cur_right_pts[i].x;
      p_v = cur_right_pts[i].y;
      double velocity_x, velocity_y;
      velocity_x = right_pts_velocity[i].x;
      velocity_y = right_pts_velocity[i].y;

      Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
      xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
      result_data.features[feature_id].camera_features.emplace_back(
          ImageFeatureTrackerResult::FeatureTrackerResult::CameraFeature{
              camera_id, Eigen::Vector3d{x, y, z}, Eigen::Vector2d{p_u, p_v},
              Eigen::Vector2d{velocity_x, velocity_y}});
      // featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
    }
    result_data.images[camera_id]= rightImg;
  }
  return ImageFeatureTrackerResult {
    std::make_shared<ImageFeatureTrackerResult::Data>(result_data)
  };
  // printf("feature track whole time %f\n", t_r.toc());
  // return featureFrame;
}

void FeatureTracker::rejectWithF() {
  if (cur_pts.size() >= 8) {
    VLOG(kGlogLevel) << "FM ransac begins";
    TicToc t_f;
    vector<cv::Point2f> un_cur_pts(cur_pts.size()),
        un_prev_pts(prev_pts.size());
    for (unsigned int i = 0; i < cur_pts.size(); i++) {
      Eigen::Vector3d tmp_p;
      m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y),
                                  tmp_p);
      tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
      tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
      un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

      m_camera[0]->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y),
                                  tmp_p);
      tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
      tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
      un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
    }

    vector<uchar> status;
    cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC, F_THRESHOLD,
                           0.99, status);
    int size_a = cur_pts.size();
    reduceVector(prev_pts, status);
    reduceVector(cur_pts, status);
    reduceVector(cur_un_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);
    VLOG(kGlogLevel) << "FM ransac: " << size_a << " -> " << cur_pts.size()
                     << " " << 1.0 * cur_pts.size() / size_a;
    VLOG(kGlogLevel) << "FM ransac costs: " << t_f.toc() << " ms";
  }
}

void FeatureTracker::readIntrinsicParameter(const vector<string> &calib_file) {
  for (size_t i = 0; i < calib_file.size(); i++) {
    VLOG(kGlogLevel) << "reading paramerter of camera" << calib_file[i].c_str();
    camera_models::CameraPtr camera =
        camera_models::CameraFactory::instance()->generateCameraFromYamlFile(
            calib_file[i]);
    m_camera.push_back(camera);
  }
  if (calib_file.size() == 2) stereo_cam = 1;
}

void FeatureTracker::showUndistortion(const string &name) {
  cv::Mat undistortedImg(row + 600, col + 600, CV_8UC1, cv::Scalar(0));
  vector<Eigen::Vector2d> distortedp, undistortedp;
  for (int i = 0; i < col; i++)
    for (int j = 0; j < row; j++) {
      Eigen::Vector2d a(i, j);
      Eigen::Vector3d b;
      m_camera[0]->liftProjective(a, b);
      distortedp.push_back(a);
      undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
      // printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
    }
  for (int i = 0; i < int(undistortedp.size()); i++) {
    cv::Mat pp(3, 1, CV_32FC1);
    pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + col / 2;
    pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + row / 2;
    pp.at<float>(2, 0) = 1.0;
    // cout << trackerData[0].K << endl;
    // printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
    // printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
    if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < row + 600 &&
        pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < col + 600) {
      undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300,
                               pp.at<float>(0, 0) + 300) =
          cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
    } else {
      // ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x,
      // pp.at<float>(1, 0), pp.at<float>(0, 0));
    }
  }
  // turn the following code on if you need
  // cv::imshow(name, undistortedImg);
  // cv::waitKey(0);
}

vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts,
                                                   camera_models::CameraPtr cam) {
  vector<cv::Point2f> un_pts;
  for (unsigned int i = 0; i < pts.size(); i++) {
    Eigen::Vector2d a(pts[i].x, pts[i].y);
    Eigen::Vector3d b;
    cam->liftProjective(a, b);
    un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
  }
  return un_pts;
}

vector<cv::Point2f> FeatureTracker::ptsVelocity(
    vector<int> &ids, vector<cv::Point2f> &pts,
    map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts) {
  vector<cv::Point2f> pts_velocity;
  cur_id_pts.clear();
  for (unsigned int i = 0; i < ids.size(); i++) {
    cur_id_pts.insert(make_pair(ids[i], pts[i]));
  }

  // caculate points velocity
  if (!prev_id_pts.empty()) {
    double dt = cur_time - prev_time;

    for (unsigned int i = 0; i < pts.size(); i++) {
      std::map<int, cv::Point2f>::iterator it;
      it = prev_id_pts.find(ids[i]);
      if (it != prev_id_pts.end()) {
        double v_x = (pts[i].x - it->second.x) / dt;
        double v_y = (pts[i].y - it->second.y) / dt;
        pts_velocity.push_back(cv::Point2f(v_x, v_y));
      } else
        pts_velocity.push_back(cv::Point2f(0, 0));
    }
  } else {
    for (unsigned int i = 0; i < cur_pts.size(); i++) {
      pts_velocity.push_back(cv::Point2f(0, 0));
    }
  }
  return pts_velocity;
}

void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight,
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts,
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap) {
  // int rows = imLeft.rows;
  int cols = imLeft.cols;
  if (!imRight.empty() && stereo_cam)
    cv::hconcat(imLeft, imRight, imTrack);
  else
    imTrack = imLeft.clone();
  cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);

  for (size_t j = 0; j < curLeftPts.size(); j++) {
    double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
    cv::circle(imTrack, curLeftPts[j], 2,
               cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
  }
  if (!imRight.empty() && stereo_cam) {
    for (size_t i = 0; i < curRightPts.size(); i++) {
      cv::Point2f rightPt = curRightPts[i];
      rightPt.x += cols;
      cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
      // cv::Point2f leftPt = curLeftPtsTrackRight[i];
      // cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
    }
  }

  map<int, cv::Point2f>::iterator mapIt;
  for (size_t i = 0; i < curLeftIds.size(); i++) {
    int id = curLeftIds[i];
    mapIt = prevLeftPtsMap.find(id);
    if (mapIt != prevLeftPtsMap.end()) {
      cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second,
                      cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
    }
  }

  // draw prediction
  /*
  for(size_t i = 0; i < predict_pts_debug.size(); i++)
  {
      cv::circle(imTrack, predict_pts_debug[i], 2, cv::Scalar(0, 170, 255), 2);
  }
  */
  // printf("predict pts size %d \n", (int)predict_pts_debug.size());

  // cv::Mat imCur2Compress;
  // cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));
}

void FeatureTracker::setPrediction(map<int, Eigen::Vector3d> &predictPts) {
  hasPrediction = true;
  predict_pts.clear();
  predict_pts_debug.clear();
  map<int, Eigen::Vector3d>::iterator itPredict;
  for (size_t i = 0; i < ids.size(); i++) {
    // printf("prevLeftId size %d prevLeftPts size
    // %d\n",(int)prevLeftIds.size(), (int)prevLeftPts.size());
    int id = ids[i];
    itPredict = predictPts.find(id);
    if (itPredict != predictPts.end()) {
      Eigen::Vector2d tmp_uv;
      m_camera[0]->spaceToPlane(itPredict->second, tmp_uv);
      predict_pts.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
      predict_pts_debug.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
    } else
      predict_pts.push_back(prev_pts[i]);
  }
}

void FeatureTracker::removeOutliers(set<int> &removePtsIds) {
  std::set<int>::iterator itSet;
  vector<uchar> status;
  for (size_t i = 0; i < ids.size(); i++) {
    itSet = removePtsIds.find(ids[i]);
    if (itSet != removePtsIds.end())
      status.push_back(0);
    else
      status.push_back(1);
  }

  reduceVector(prev_pts, status);
  reduceVector(ids, status);
  reduceVector(track_cnt, status);
}

cv::Mat FeatureTracker::getTrackImage() { return imTrack; }
}  // namespace vins
}  // namespace internal