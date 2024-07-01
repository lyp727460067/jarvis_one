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

#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <map>

#include "jarvis/estimator/factor/imu_factor.h"
#include "jarvis/estimator/feature_manager.h"
#include "jarvis/utility/utility.h"
namespace jarvis {
namespace estimator {

class ImageFrame {
 public:
  ImageFrame(){};
  ImageFrame(const std::map<
                 int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>
                 &_points,
             double _t)
      : points(_points), t{_t}, is_key_frame{false} {};
  std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> points;
  double t = 0.0;
  Eigen::Matrix3d R;
  Eigen::Vector3d T;
  IntegrationBase *pre_integration = nullptr;
  bool is_key_frame = false;
};
struct AlignmentOption {
  transform::Rigid3d cam_to_imu;
  Eigen::Vector3d gravity;
};

class Alignment {
 public:
   explicit Alignment(const AlignmentOption &option) : options_(option) {}
  void solveGyroscopeBias(std::map<double, ImageFrame> &all_image_frame,
                          Eigen::Vector3d *Bgs);
  bool VisualIMUAlignment(std::map<double, ImageFrame> &all_image_frame,
                          Eigen::Vector3d *Bgs, Eigen::Vector3d &g,
                          Eigen::VectorXd &x);

 private:
  AlignmentOption options_;
  void RefineGravity(std::map<double, ImageFrame> &all_image_frame,
                     Eigen::Vector3d &g, Eigen::VectorXd &x);
                     bool LinearAlignment(std::map<double, ImageFrame> &all_image_frame,
                     Eigen::Vector3d &g, Eigen::VectorXd &x);
};

}  // namespace estimator
}  // namespace jarvis