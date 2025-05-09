/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

/**
* Copyright (c) 2009, V. Lepetit, EPFL
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* The views and conclusions contained in the software and documentation are those
* of the authors and should not be interpreted as representing official policies,
*   either expressed or implied, of the FreeBSD Project
*/

#ifndef PNPSOLVER_H
#define PNPSOLVER_H

#include <map>
#include <vector>
#include <Eigen/Dense>
#include <algorithm>

#include "jarvis/camera_models/camera_models/camera.h"
#include "jarvis/transform/rigid_transform.h"
#include "jarvis/common/id.h"


namespace jarvis {
namespace mapping {
namespace match {

struct PnPOption {
  double mRansacProb;     // RANSAC 置信度
  int mRansacMinInliers;  // 最小內点数
  int mRansacMaxIts;      // 最大迭代数
  int mRansacMinSet;      // 求解问题要求最小样本数,6以上
  double mRansacEpsilon;  // 希望得到的 内点数/总体数 的比值
  double mvMaxError;      // 內外点判定的距离阀值
  std::map<int, camera_models::CameraPtr> cameras;
  std::map<int, transform::Rigid3d> extric_imu_to_camera_;
};

class PnPsolver {
 public:
  PnPsolver(const std::vector<FeatureId> &feature_ids,
            const std::vector<Eigen::Vector2d> &match_keypoints,
            const std::vector<Eigen::Vector3d> &match_mappoints,
            const PnPOption option);

  ~PnPsolver();

  void SetRansacParameters();

  int find(std::vector<bool> &vbInliers, int &nInliers,
           transform::Rigid3d &result);

  int iterate(int nIterations, bool &bNoMore, std::vector<bool> &vbInliers,
              int &nInliers, transform::Rigid3d &result);

 private:

  void CheckInliers();
  bool Refine();

  // Functions from the original EPnP code
  void set_maximum_number_of_correspondences(const int n);
  void reset_correspondences(void);
  void add_correspondence(const double X, const double Y, const double Z,
                          const double u, const double v,
                          const int sequence_id);

  double compute_pose(Eigen::Matrix3d &R, Eigen::Vector3d &t);

  double reprojection_error(const Eigen::Matrix3d &R, const Eigen::Vector3d &t);

  void choose_control_points(void);
  void compute_barycentric_coordinates(void);
  void compute_cis(const Eigen::VectorXd &x);
  void compute_pis(void);

  void solve_for_sign(void);

  void construct_M_b(Eigen::MatrixXd &M, Eigen::VectorXd &b, double *aplhas, const double *us);

  double compute_R_and_t(const Eigen::VectorXd &x, Eigen::Matrix3d &R,
                         Eigen::Vector3d &t);

  void estimate_R_and_t(Eigen::Matrix3d &R, Eigen::Vector3d &t);

  void qr_solve(Eigen::MatrixXd &A, Eigen::VectorXd &b, Eigen::VectorXd &X);




  double * pws, * us, * alphas, * pis;
  int *ids;
  int maximum_number_of_correspondences;
  int number_of_correspondences;

  // cws和ccs分别是四个控制点在世界坐标系和imu坐标系下的坐标
  double cws[4][3], cis[4][3];
  double cws_determinant;

  // 2D Points
  std::vector<Eigen::Vector2d> mvP2D;

  // 3D Points
  std::vector<Eigen::Vector3d> mvP3Dw;
  std::vector<Eigen::Vector3d> mvP3D_undis; // 归一化平面

  // 2D Points对应的FeatureId
  std::vector<FeatureId> mvFId;

  // Index in Frame
  std::vector<size_t> mvKeyPointIndices;

  std::map<int, transform::Rigid3d> extric_camera_to_imu_;
  std::map<int, transform::Rigid3d> extric_imu_to_camera_;
  std::map<int, double> fu;
  std::map<int, double> fv;
  std::map<int, double> cx;
  std::map<int, double> cy;

  // Current Estimation
  Eigen::Matrix3d mRi; // 将世界坐标系上的点转换到IMU坐标系
  Eigen::Vector3d mti;
  cv::Mat mTcwi;
  std::vector<bool> mvbInliersi;
  int mnInliersi;

  // Current Ransac State
  int mnIterations;
  std::vector<bool> mvbBestInliers;
  int mnBestInliers;
  transform::Rigid3d mBest_T_imu_to_world; // 将世界坐标系上的点转换到IMU坐标系

  // Refined
  transform::Rigid3d mRefined_T_imu_to_world;
  std::vector<bool> mvbRefinedInliers;
  int mnRefinedInliers;

  // Number of Correspondences
  int N;

  // Indices for random selection [0 .. N-1]
  std::vector<size_t> mvAllIndices;

  PnPOption option_;
};


} // namespace match
} // namespace mapping
} // namespace jarvis
#endif //PNPSOLVER_H
