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

#include <iostream>

#include "PnPsolver.h"
#include "jarvis/DUtils/Random.h"


namespace jarvis {
namespace mapping {
namespace match {

// pcs表示3D点在camera坐标系下的坐标
// pws表示3D点在世界坐标系下的坐标
// us表示图像坐标系下的2D点坐标
// alphas为真实3D点用4个虚拟控制点表达时的系数
// 构造函数
PnPsolver::PnPsolver(const std::vector<FeatureId> &feature_ids,
                     const std::vector<Eigen::Vector2d> &match_keypoints,
                     const std::vector<Eigen::Vector3d> &match_mappoints,
                     const PnPOption option)
    : option_(option),
      pws(0),
      us(0),
      alphas(0),
      pis(0),
      ids(0),
      maximum_number_of_correspondences(0),
      number_of_correspondences(0),
      mnInliersi(0),
      mnIterations(0),
      mnBestInliers(0),
      N(0) {
  CHECK_EQ(feature_ids.size(), match_keypoints.size());
  CHECK_EQ(feature_ids.size(), match_mappoints.size());
  mvFId = feature_ids;
  mvP2D = match_keypoints;
  mvP3Dw = match_mappoints;
  N = feature_ids.size();

  mvP3D_undis.clear();
  mvP3D_undis.reserve(N);
  for (int i = 0; i < N; i++) {
    Eigen::Vector3d P3D_undis;
    option_.cameras[mvFId[i].sequence_id]->liftProjective(mvP2D[i], P3D_undis);
    mvP3D_undis.push_back(P3D_undis);
  }

  mvAllIndices.resize(N);
  for (int i = 0; i < N; i++) {
    mvAllIndices[i] = i;
  }

  // for (auto it = option_.cameras.begin(); it != option_.cameras.end(); it++) {
  //   camera_models::EquidistantCamera::Parameters parameter_i =
  //       it->second->getParameters();
  //   fu[it->first] = parameter_i.mu();
  //   fv[it->first] = parameter_i.mv();
  //   cx[it->first] = parameter_i.u0();
  //   cy[it->first] = parameter_i.v0();
  // }

  SetRansacParameters();
}

// 析构函数
PnPsolver::~PnPsolver()
{
  // 释放堆内存
  delete [] pws;
  delete [] us;
  delete [] alphas;
  delete [] pis;
  delete [] ids;
}


void PnPsolver::SetRansacParameters() {
  mvbInliersi.resize(N);  // inlier index, mvbInliersi记录每次迭代inlier的点

  // Adjust Parameters according to number of correspondences
  // NOTICE 实际在计算的过程中使用的 option_.mRansacMinInliers =
  // min(给定内点数,最小集,理论内点数)
  int nMinInliers = N * option_.mRansacEpsilon;
  if (nMinInliers < option_.mRansacMinInliers)
    nMinInliers = option_.mRansacMinInliers;
  if (nMinInliers < option_.mRansacMinSet) nMinInliers = option_.mRansacMinSet;
  option_.mRansacMinInliers = nMinInliers;

  // Step 3 根据敲定的"最小内点数"来调整 内点数/总体数 这个比例 epsilon
  if (option_.mRansacEpsilon < (double)option_.mRansacMinInliers / N)
    option_.mRansacEpsilon = (double)option_.mRansacMinInliers / N;

  // Set RANSAC iterations according to probability, epsilon, and max iterations
  int nIterations;
  if (option_.mRansacMinInliers ==
      N)  //根据期望的残差大小来计算RANSAC需要迭代的次数
    nIterations = 1;
  else
    nIterations = ceil(log(1 - option_.mRansacProb) /
                       log(1 - pow(option_.mRansacEpsilon, 3)));

  option_.mRansacMaxIts = std::max(1, std::min(nIterations, option_.mRansacMaxIts));
}

int PnPsolver::find(std::vector<bool> &vbInliers, int &nInliers,
                    transform::Rigid3d &result) {
  bool bFlag;
  return iterate(option_.mRansacMaxIts, bFlag, vbInliers, nInliers, result);
}

/**
 * @brief EPnP迭代计算
 * 
 * @param[in] nIterations   迭代次数
 * @param[in] bNoMore       达到最大迭代次数的标志
 * @param[in] vbInliers     内点的标记
 * @param[in] nInliers      总共内点数
 * @return 0: 失败, 1: 失败但变换稍微能用, 2: 成功
 */
int PnPsolver::iterate(int nIterations, bool &bNoMore,
                       std::vector<bool> &vbInliers, int &nInliers,
                       transform::Rigid3d &result) {
  bNoMore = false;  //已经达到最大迭代次数的标志
  vbInliers.clear();
  nInliers = 0;  // 当前次迭代时的内点数

  // option_.mRansacMinSet 为每次RANSAC需要的特征点数，默认为4组3D-2D对应点
  set_maximum_number_of_correspondences(option_.mRansacMinSet);

  if (N < option_.mRansacMinInliers) {
    bNoMore = true;
    return 0;
  }

  // vAvailableIndices为每次从mvAllIndices中随机挑选mRansacMinSet组3D-2D对应点进行一次RANSAC
  std::vector<size_t> vAvailableIndices;

  // 当前的迭代次数id
  int iter_num = 0;
  while (iter_num < nIterations) {
    iter_num++;
    // std::cout << "iter num: " << iter_num << std::endl;
    // 清空已有的匹配点的计数,为新的一次迭代作准备
    reset_correspondences();

    vAvailableIndices = mvAllIndices;

    // Get min set of points
    // 随机选取4组（默认数目）最小集合
    CHECK_LE(option_.mRansacMinSet, N);
    for (short i = 0; i < option_.mRansacMinSet; ++i) {
      int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size() - 1);

      // 将生成的这个索引映射到给定帧的特征点id
      int idx = vAvailableIndices[randi];

      // 将对应的3D-2D压入到pws和us.
      // 这个过程中需要知道将这些点的信息存储到数组中的哪个位置,这个就由变量
      // number_of_correspondences 来指示了
      // add_correspondence(mvP3Dw[idx].x, mvP3Dw[idx].y, mvP3Dw[idx].z,
      //                    mvP2D[idx].x, mvP2D[idx].y, mvFId[idx].sequence_id);
      add_correspondence(mvP3Dw[idx].x(), mvP3Dw[idx].y(), mvP3Dw[idx].z(),
                         mvP3D_undis[idx].x(), mvP3D_undis[idx].y(), mvFId[idx].sequence_id);

      vAvailableIndices[randi] = vAvailableIndices.back();
      vAvailableIndices.pop_back();
    }

    // 计算相机的位姿
    compute_pose(mRi, mti);

    // Check inliers
    CheckInliers();
    std::cout << "inliers num: " << mnInliersi << std::endl;

    if (mnInliersi >= option_.mRansacMinInliers) {
      // If it is the best solution so far, save it
      if (mnInliersi > mnBestInliers) {
        mvbBestInliers = mvbInliersi;
        mnBestInliers = mnInliersi;

        Eigen::Quaterniond mqi(mRi);
        mBest_T_imu_to_world = transform::Rigid3d(mti, mqi);
      }

      // 如果求精成功直接返回结果,否则重新迭代
      if (Refine()) {
        nInliers = mnRefinedInliers;
        vbInliers = std::vector<bool>(N, false);
        for (int i = 0; i < N; i++) {
          if (mvbRefinedInliers[i]) vbInliers[i] = true;
        }
        result = mRefined_T_imu_to_world;
        return 2;
      }
    }
  }

  if (mnIterations >= option_.mRansacMaxIts) {
    bNoMore = true;
    if (mnBestInliers >= option_.mRansacMinInliers) {
      nInliers = mnBestInliers;
      vbInliers = std::vector<bool>(N, false);
      for (int i = 0; i < N; i++) {
        if (mvbBestInliers[i]) vbInliers[i] = true;
      }
      result = mBest_T_imu_to_world;
      return 1;
    }
  }

  return 0;
}

// 使用新的内点来继续对位姿进行精求解
bool PnPsolver::Refine()
{
    std::vector<int> vIndices;
    vIndices.reserve(mvbBestInliers.size());

    for(size_t i=0; i<mvbBestInliers.size(); i++)
    {
        if(mvbBestInliers[i])
        {
            vIndices.push_back(i);
        }
    }

    set_maximum_number_of_correspondences(vIndices.size());
    reset_correspondences();
    for (size_t i = 0; i < vIndices.size(); i++) {
      int idx = vIndices[i];
      // add_correspondence(mvP3Dw[idx].x, mvP3Dw[idx].y, mvP3Dw[idx].z,
      //                    mvP2D[idx].x, mvP2D[idx].y,
      //                    mvFId[idx].sequence_id);
      add_correspondence(mvP3Dw[idx].x(), mvP3Dw[idx].y(), mvP3Dw[idx].z(),
                         mvP3D_undis[idx].x(), mvP3D_undis[idx].y(),
                         mvFId[idx].sequence_id);
    }

    // Compute camera pose
    compute_pose(mRi, mti);

    // Check inliers
    CheckInliers();
    // std::cout << "inliers num: " << mnInliersi << " / " << option_.mRansacMinInliers << std::endl;

    mnRefinedInliers = mnInliersi;
    mvbRefinedInliers = mvbInliersi;

    // 如果达到了要求
    if (mnInliersi >= option_.mRansacMinInliers) {
      Eigen::Quaterniond mqi(mRi);
      mRefined_T_imu_to_world = transform::Rigid3d(mti, mqi);
      return true;
    }

    return false;
}

/**
 * @brief 通过之前求解的位姿来进行3D-2D投影，统计内点数目
 * 
 */
// void PnPsolver::CheckInliers()
// {
//     mnInliersi=0;
    
//     std::map<int, Eigen::Matrix3d> R_i_to_c;
//     std::map<int, Eigen::Vector3d> t_i_to_c;
//     for (auto it = option_.extric_imu_to_camera_.begin();
//          it != option_.extric_imu_to_camera_.end(); it++) {
//       R_i_to_c[it->first] = it->second.rotation().toRotationMatrix();
//       t_i_to_c[it->first] = it->second.translation()
//     }

//     // 遍历当前帧中所有的匹配点
//     for(int i=0; i<N; i++)
//     {
//         // 取出对应的3D点和2D点
//         Eigen::Vector3d P3Dw = mvP3Dw[i];
//         Eigen::Vector2d P2D = mvP2D[i];

//         // 将3D点由世界坐标系旋转到相机坐标系
//         int id = mvFId[i];
//         CHECK(R_i_to_c.count(id));

//         Eigen::Vector3d P3Dc = R_i_to_c[id] * (mRi * P3Dw + mti) + t_i_to_c[id];
//         if (P3Dc(2) < 0) {
//           mvbInliersi[i] = false;
//           continue;
//         }

//         double Xc = P3Dc(0);
//         double Yc = P3Dc(1);
//         double invZc = 1.0 / P3Dc(2);

//         // 将相机坐标系下的3D进行针孔投影
//         double ue = cx[id] + fu[id] * Xc * invZc;
//         double ve = cy[id] + fv[id] * Yc * invZc;

//         // 计算特征点和投影点的残差大小
//         double distX = P2D.x - ue;
//         double distY = P2D.y - ve;

//         double error2 = distX * distX + distY * distY;

//         if (error2 < option_.mvMaxError) {
//           mvbInliersi[i] = true;
//           mnInliersi++;
//         } else {
//           mvbInliersi[i] = false;
//         }
//     }
// }

/**
 * @brief 通过之前求解的位姿来进行到归一化平面的投影,在归一化平面计算误差统计内点数目
 * 
 */
void PnPsolver::CheckInliers()
{
    mnInliersi=0;
    
    std::map<int, Eigen::Matrix3d> R_i_to_c;
    std::map<int, Eigen::Vector3d> t_i_to_c;
    for (auto it = option_.extric_imu_to_camera_.begin();
         it != option_.extric_imu_to_camera_.end(); it++) {
      R_i_to_c[it->first] = it->second.rotation().toRotationMatrix();
      t_i_to_c[it->first] = it->second.translation();
    }

    // std::cout << "check inliers" << std::endl;
    // 遍历当前帧中所有的匹配点
    for(int i=0; i<N; i++)
    {
        // 取出对应的3D点和2D点
        Eigen::Vector3d P3Dw = mvP3Dw[i];
        Eigen::Vector3d P3D_undist = mvP3D_undis[i];

        // 将3D点由世界坐标系旋转到相机坐标系
        int id = mvFId[i].sequence_id;
        CHECK(R_i_to_c.count(id));

        Eigen::Vector3d P3Dc = R_i_to_c[id] * (mRi * P3Dw + mti) + t_i_to_c[id];
        if (P3Dc(2) < 0) {
          mvbInliersi[i] = false;
          continue;
        }

        double Xc = P3Dc(0);
        double Yc = P3Dc(1);
        double invZc = 1.0 / P3Dc(2);

        // 将相机坐标系下的3D点投影到归一化平面
        double ue = Xc * invZc;
        double ve = Yc * invZc;

        // 计算特征点和投影点的残差大小
        double distX = P3D_undist.x() - ue;
        double distY = P3D_undist.y() - ve;

        double error2 = distX * distX + distY * distY;

        // std::cout << ue << " / " << P3D_undist.x() << ", " << ve << " / "
        //           << P3D_undist.y() << ", " << error2 << " / "
        //           << option_.mvMaxError << std::endl;

        if (error2 < option_.mvMaxError) {
          mvbInliersi[i] = true;
          mnInliersi++;
        } else {
          mvbInliersi[i] = false;
        }
    }
}

/**
 * @brief 设置EPnP 相关的参数
 * 
 * @param[in] n     EPnP 最小集合数目，默认是6
 */

void PnPsolver::set_maximum_number_of_correspondences(int n)
{
  // number_of_correspondences为RANSAC每次PnP求解时时3D点和2D点匹配对数
  if (maximum_number_of_correspondences < n) {
    if (pws != 0) delete [] pws;
    if (us != 0) delete [] us;
    if (alphas != 0) delete [] alphas;
    if (pis != 0) delete [] pis;
    if (ids != 0 ) delete[] ids;


    // 更新
    maximum_number_of_correspondences = n;
    pws = new double[3 * maximum_number_of_correspondences];    // 每个3D点有(X Y Z)三个值
    us = new double[2 * maximum_number_of_correspondences];     // 每个图像2D点有(u v)两个值
    alphas = new double[4 * maximum_number_of_correspondences]; // 每个3D点由四个控制点拟合，有四个系数
    pis = new double[3 * maximum_number_of_correspondences];    // 每个3D点有(X Y Z)三个值
    ids = new int[maximum_number_of_correspondences];
  }
}

// 清空当前已有的匹配点计数,为进行新的一次迭代作准备
void PnPsolver::reset_correspondences(void)
{
  number_of_correspondences = 0;
}

/**
 * @brief 将给定的3D,2D点的数据压入到数组中
 * 
 * @param[in] X       3D点X坐标
 * @param[in] Y       3D点Y坐标
 * @param[in] Z       3D点Z坐标
 * @param[in] u       3D点对应2D点的横坐标
 * @param[in] v       3D点对应2D点的纵坐标
 */
void PnPsolver::add_correspondence(const double X, const double Y,
                                   const double Z, const double u,
                                   const double v, const int sequence_id) {
  // std::cout << "correspondence:" << std::endl;
  // std::cout << X << ", " << Y << ", " << Z << std::endl;
  // std::cout << u << ", " << v << std::endl;
  // std::cout << sequence_id << std::endl;

  pws[3 * number_of_correspondences] = X;
  pws[3 * number_of_correspondences + 1] = Y;
  pws[3 * number_of_correspondences + 2] = Z;

  us[2 * number_of_correspondences] = u;
  us[2 * number_of_correspondences + 1] = v;

  ids[number_of_correspondences] = sequence_id;

  // 当前次迭代中,已经采样的匹配点的个数;也用来指导这个"压入到数组"的过程中操作
  number_of_correspondences++;
}

/**
 * @brief 从给定的匹配点中计算出四个控制点
 * 
 */
void PnPsolver::choose_control_points(void)
{
  // Step 1：第一个控制点：参与PnP计算的参考3D点的质心（均值）
  // 计算前先把第1个控制点坐标清零
  cws[0][0] = cws[0][1] = cws[0][2] = 0;

  for(int i = 0; i < number_of_correspondences; i++)
    for(int j = 0; j < 3; j++)
      cws[0][j] += pws[3 * i + j];

  for(int j = 0; j < 3; j++)
    cws[0][j] /= number_of_correspondences;

  // Take C1, C2, and C3 from PCA on the reference points:
  Eigen::MatrixXd PW0(number_of_correspondences, 3);
  Eigen::Matrix3d PW0tPW0;
  Eigen::Vector3d DC;   // 特征值
  Eigen::Matrix3d UCt;  // 特征向量

  // 将存在pws中的参考3D点减去第一个控制点(均值中心)的坐标（相当于把第一个控制点作为原点）, 并存入PW0
  for (int i = 0; i < number_of_correspondences; i++)
    for (int j = 0; j < 3; j++) 
      PW0(i, j) = pws[3 * i + j] - cws[0][j];

  PW0tPW0 = PW0.transpose() * PW0;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(PW0tPW0);
  DC = es.eigenvalues();
  UCt = es.eigenvectors();

  // std::cout << "control point eigenvalues" << std::endl << DC << std::endl;

  // Step 2.3：得到C1, C2, C3三个3D控制点，最后加上之前减掉的第一个控制点这个偏移量
  for (int i = 1; i < 4; i++) {
    double k = sqrt(DC[i - 1] / number_of_correspondences);
    for (int j = 0; j < 3; j++) {
      cws[i][j] = cws[0][j] + k * UCt(i-1, j);
    }
  }
}

/**
 * @brief 求解世界坐标系下四个控制点的系数alphas，在IMU坐标系下系数不变
 * 
 */
void PnPsolver::compute_barycentric_coordinates(void) {
  // alphas 四个控制点的系数，每一个pws，都有一组alphas与之对应
  Eigen::Matrix3d CC(3, 3);
  Eigen::Matrix3d CC_inv(3, 3);

  // Step
  // 1：第一个控制点在质心的位置，后面三个控制点减去第一个控制点的坐标（以第一个控制点为原点）
  // 减去质心后得到x y z轴
  //
  // cws的排列 |cws1_x cws1_y cws1_z|  ---> |cws1|
  //          |cws2_x cws2_y cws2_z|       |cws2|
  //          |cws3_x cws3_y cws3_z|       |cws3|
  //          |cws4_x cws4_y cws4_z|       |cws4|
  //
  // cc的排列  |cc2_x cc3_x cc4_x|  --->|cc2 cc3 cc4|
  //          |cc2_y cc3_y cc4_y|
  //          |cc2_z cc3_z cc4_z|

  for (int i = 0; i < 3; i++)
    for (int j = 1; j < 4; j++) {
      CC(i, j - 1) = cws[j][i] - cws[0][i];
    }

  CC_inv = CC.inverse();
  for (int i = 0; i < number_of_correspondences; i++) {
    double *pi = pws + 3 * i;  // pi指向第i个3D点的首地址
    double *a = alphas + 4 * i;  // a指向第i个控制点系数alphas的首地址

    // pi[]-cws[0][]表示去质心
    // a0,a1,a2,a3 对应的是四个控制点的齐次重心坐标
    for (int j = 0; j < 3; j++)
      /*    这里的原理基本上是这个样子：(这里公式的下标和程序中的不一样，是从1开始的)
       *    cp=p_i-c1
       *    cp=a1(c1-c1)+a2(c2-c1)+a3(c3-c1)+a4(c4-c1)
       *      => a2*cc2+a3*cc3+a4*cc4
       *    [cc2 cc3 cc4] * [a2 a3 a4]^T = cp
       *  => [a2 a3 a4]^T = [cc2 cc3 cc4]^(-1) * cp
       */
      a[1 + j] = CC_inv(j, 0) * (pi[0] - cws[0][0]) +
                 CC_inv(j, 1) * (pi[1] - cws[0][1]) +
                 CC_inv(j, 2) * (pi[2] - cws[0][2]);
    // 最后计算用于进行归一化的a0
    a[0] = 1.0f - a[1] - a[2] - a[3];
  }
}

// 不考虑畸变的
// void PnPsolver::construct_M_b(Eigen::MatrixXd &M, Eigen::VectorXd &b, double *aplhas, const double *us) {
//   for (int i = 0; i < number_of_correspondences; i++) {
//     int id = ids[i];
//     Eigen::Matrix3d Ri = option_.extric_imu_to_camera_[id].rotation().toRotationMatrix();
//     Eigen::Vector3d ti = option_.extric_imu_to_camera_[id].translation();
//     for (int j = 0; j < 4; j++) {
//       M[i * 2][j * 3] =
//           alphas[4 * i + j] * (fu[id] * Ri[0][0] + (cx[id] - us[2 * i]) * Ri[2][0]);
//       M[i * 2][j * 3 + 1] =
//           alphas[4 * i + j] * (fu[id] * Ri[0][1] + (cx[id] - us[2 * i]) * Ri[2][1]);
//       M[i * 2][j * 3 + 2] =
//           alphas[4 * i + j] * (fu[id] * Ri[0][2] + (cx[id] - us[2 * i]) * Ri[2][2]);

//       M[i * 2 + 1][j * 3] = alphas[4 * i + j] *
//                             (fv[id] * Ri[1][0] + (cy[id] - us[2 * i + 1]) * Ri[2][0]);
//       M[i * 2 + 1][j * 3 + 1] =
//           alphas[4 * i + j] *
//           (fv[id] * Ri[1][1] + (cy[id] - us[2 * i + 1]) * Ri[2][1]);
//       M[i * 2 + 1][j * 3 + 2] =
//           alphas[4 * i + j] *
//           (fv[id] * Ri[1][2] + (cy[id] - us[2 * i + 1]) * Ri[2][2]);

//       b[i * 2] = -(fu[id] * ti[0] + (cx[id] - us[2 * i]) * ti[2]);
//       b[i * 2 + 1] = -(fv[id] * ti[1] + (cy[id] - us[2 * i + 1]) * ti[2]);
//     }
//   }
// };

// 考虑畸变的
void PnPsolver::construct_M_b(Eigen::MatrixXd &M, Eigen::VectorXd &b, double *aplhas, const double *us) {

  for (int i = 0; i < number_of_correspondences; i++) {
    int id = ids[i];
    Eigen::Matrix3d Ri =
        option_.extric_imu_to_camera_[id].rotation().toRotationMatrix();
    Eigen::Vector3d ti = option_.extric_imu_to_camera_[id].translation();
    // std::cout << id << std::endl << Ri << std::endl;

    // Eigen::Vector3d p(0, 0, 0);
    // double z = 0;
    // for (int j = 0; j < 4; j++) {
    //   p.x() += alphas[4 * i + j] * cws[j][0];
    //   p.y() += alphas[4 * i + j] * cws[j][1];
    //   p.z() += alphas[4 * i + j] * cws[j][2];

    //   z += Ri(2, 0) * alphas[4 * i + j] * cws[j][0] +
    //        Ri(2, 1) * alphas[4 * i + j] * cws[j][1] +
    //        Ri(2, 2) * alphas[4 * i + j] * cws[j][2];
    // }
    // z += ti(2);
    // p = Ri * p + ti;
    // std::cout << "p:" << std::endl;
    // std::cout << p << std::endl;
    // std::cout << "u: " << std::endl;
    // std::cout << us[2 * i] * z << ", " << us[2 * i + 1] * z << ", " << z << std::endl;

    for (int j = 0; j < 4; j++) {
      M(i * 2, j * 3) = alphas[4 * i + j] * (Ri(0, 0) - us[2 * i] * Ri(2, 0));
      M(i * 2, j * 3 + 1) =
          alphas[4 * i + j] * (Ri(0, 1) - us[2 * i] * Ri(2, 1));
      M(i * 2, j * 3 + 2) =
          alphas[4 * i + j] * (Ri(0, 2) - us[2 * i] * Ri(2, 2));

      M(i * 2 + 1, j * 3) =
          alphas[4 * i + j] * (Ri(1, 0) - us[2 * i + 1] * Ri(2, 0));
      M(i * 2 + 1, j * 3 + 1) =
          alphas[4 * i + j] * (Ri(1, 1) - us[2 * i + 1] * Ri(2, 1));
      M(i * 2 + 1, j * 3 + 2) =
          alphas[4 * i + j] * (Ri(1, 2) - us[2 * i + 1] * Ri(2, 2));
    }
    b(i * 2) = -ti(0) + us[2 * i] * ti(2);
    b(i * 2 + 1) = -ti(1) + us[2 * i + 1] * ti(2);
  }
};

void PnPsolver::compute_cis(const Eigen::VectorXd &x) {
  // std::cout << "control point in imu" << std::endl;
  for (int j = 0; j < 4; j++){  // j表示当前计算的是第几个控制点
    for (int k = 0; k < 3; k++)  // k表示当前计算的是控制点的哪个坐标
      cis[j][k] = x[j * 3 + k];
    
    // std::cout << cis[j][0] << ", " << cis[j][1] << ", " << cis[j][2] << std::endl;
  }
}

void PnPsolver::compute_pis(void)
{
  // std::cout << "p imu:" << std::endl;
  for(int i = 0; i < number_of_correspondences; i++) {
    double * a = alphas + 4 * i;
    double * pi = pis + 3 * i;

    for(int j = 0; j < 3; j++)
      pi[j] = a[0] * cis[0][j] + a[1] * cis[1][j] + a[2] * cis[2][j] + a[3] * cis[3][j];
    
    // std::cout << pi[0] << ", " << pi[1] << ", " << pi[2] << std::endl;
  }
}

/**
 * @brief 使用EPnP算法计算相机的位姿.其中匹配点的信息由类的成员函数给定 
 * @param[out] R    旋转
 * @param[out] T    平移
 * @return double   使用这对旋转和平移的时候, 匹配点对的平均重投影误差
 */
double PnPsolver::compute_pose(Eigen::Matrix3d &R, Eigen::Vector3d &t) {
  // Step 1：获得EPnP算法中的四个控制点
  choose_control_points();
  // std::cout << "control points" << std::endl;
  // for (int i = 0; i < 4; i++) {
  //   std::cout << cws[i][0]  << ", " << cws[i][1] << ", " << cws[i][2]
  //             << std::endl;
  // }

  // Step 2：计算世界坐标系下每个3D点用4个控制点线性表达时的系数alphas
  compute_barycentric_coordinates();

  // std::cout << "alphas" << std::endl;
  // for (int i = 0; i < number_of_correspondences; i++) {
  //   std::cout << alphas[i * number_of_correspondences] << ", "
  //             << alphas[i * number_of_correspondences + 1] << ", "
  //             << alphas[i * number_of_correspondences + 2] << ", "
  //             << alphas[i * number_of_correspondences + 3] << std::endl;
  // }

  // Step 3：构造M矩阵，EPnP原始论文中公式(3)(4)-->(5)(6)(7); 矩阵的大小为 2n*12
  // ,n 为使用的匹配点的对数
  Eigen::MatrixXd M(2 * number_of_correspondences, 12);
  Eigen::VectorXd b(2 * number_of_correspondences);

  // 根据每一对匹配点的数据来填充矩阵M中的数据
  construct_M_b(M, b, alphas, us);

  // 求解Mx =
  // b,此处为正定或超定方程组(要求M行数大于等于12),因此用正规方程法求解,可换其它方法
  // 正规方程法,求解MTMx=MTb的最小二乘解
  Eigen::MatrixXd MTM = M.transpose() * M;
  Eigen::VectorXd MTb = M.transpose() * b;

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es;
  es.compute(MTM);
  Eigen::VectorXd Sigma = es.eigenvalues();
  Eigen::MatrixXd Q = es.eigenvectors();
  // std::cout << "eigen value:" << std::endl;
  // std::cout << es.eigenvalues() << std::endl;
  // std::cout << "eigen vector:" << std::endl;
  // std::cout << es.eigenvectors() << std::endl;

  if(Sigma[1] < 1e-3){
    LOG(ERROR) << "the second min eigen value is too small: " << Sigma[1];
  }

  Eigen::VectorXd temp = Q.transpose() * MTb;
  for(int i=1; i<12; i++){
    temp[i] /= Sigma(i);
  }

  Eigen::VectorXd xs = Q * temp;
  Eigen::VectorXd v0 = Q.col(0);
  // std::cout << "result:" << std::endl;
  // std::cout << xs << std::endl;
  // std::cout << "v0" << std::endl;
  // std::cout << v0 << std::endl;

  Eigen::VectorXd x_del_2(6), q_del_2(6), c_del_2(6);
  int idx = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 4; j++) {
      x_del_2[idx] =
          (xs[i * 3] - xs[j * 3]) * (xs[i * 3] - xs[j * 3]) +
          (xs[i * 3 + 1] - xs[j * 3 + 1]) * (xs[i * 3 + 1] - xs[j * 3 + 1]) +
          (xs[i * 3 + 2] - xs[j * 3 + 2]) * (xs[i * 3 + 2] - xs[j * 3 + 2]);

      q_del_2[idx] =
          (v0[i * 3] - v0[j * 3]) * (v0[i * 3] - v0[j * 3]) +
          (v0[i * 3 + 1] - v0[j * 3 + 1]) * (v0[i * 3 + 1] - v0[j * 3 + 1]) +
          (v0[i * 3 + 2] - v0[j * 3 + 2]) * (v0[i * 3 + 2] - v0[j * 3 + 2]);

      c_del_2[idx++] = (cws[i][0] - cws[j][0]) * (cws[i][0] - cws[j][0]) +
                       (cws[i][1] - cws[j][1]) * (cws[i][1] - cws[j][1]) +
                       (cws[i][2] - cws[j][2]) * (cws[i][2] - cws[j][2]);
    }
  }

  double sum1 = 0, sum2 = 0, sum3 = 0;
  for (int i = 0; i < 6; i++) {
    sum1 += (sqrt(x_del_2[i]) + sqrt(c_del_2[i])) * sqrt(q_del_2[i]);
    sum2 += q_del_2[i];
    sum3 += (sqrt(x_del_2[i]) - sqrt(c_del_2[i])) * sqrt(q_del_2[i]);
  }
  double lambda1 = - sum1 / sum2;
  double lambda2 = - sum3 / sum2;
  // std::cout << "lambda" << std::endl;
  // std::cout << lambda1 << std::endl;
  // std::cout << lambda2 << std::endl;

  Eigen::VectorXd x = xs + lambda1 * v0;
  // std::cout << "x" << std::endl;
  // std::cout << x << std::endl;
  Eigen::VectorXd x2 = xs + lambda2 * v0;
  // std::cout << "x2" << std::endl;
  // std::cout << x2 << std::endl;

  // std::cout << "MTMx1" << std::endl;
  // std::cout << MTM * x << std::endl;
  // std::cout << "MTMx2" << std::endl;
  // std::cout << MTM * x2 << std::endl;
  // std::cout << "MTb" << std::endl;
  // std::cout << MTb << std::endl;

  // Eigen::VectorXd cwsv(12);
  // for (int i = 0; i < 4; i++)
  //   for (int j = 0; j < 3; j++) {
  //     cwsv[i * 3 + j] = cws[i][j];
  //   }

  // std::cout << "cws" << std::endl;
  // std::cout << cwsv << std::endl;

  // Eigen::VectorXd left1(6), left2(6);
  // idx = 0;
  // for (int i = 0; i < 4; i++) {
  //   for (int j = i + 1; j < 4; j++) {
  //     left1[idx] =
  //         (xs[i * 3] - xs[j * 3] + lambda1 * (v0[i * 3] - v0[j * 3])) *
  //             (xs[i * 3] - xs[j * 3] + lambda1 * (v0[i * 3] - v0[j * 3])) +
  //         (xs[i * 3 + 1] - xs[j * 3 + 1] +
  //          lambda1 * (v0[i * 3 + 1] - v0[j * 3 + 1])) *
  //             (xs[i * 3 + 1] - xs[j * 3 + 1] +
  //              lambda1 * (v0[i * 3 + 1] - v0[j * 3 + 1])) +
  //         (xs[i * 3 + 2] - xs[j * 3 + 2] +
  //          lambda1 * (v0[i * 3 + 2] - v0[j * 3 + 2])) *
  //             (xs[i * 3 + 2] - xs[j * 3 + 2] +
  //              lambda1 * (v0[i * 3 + 2] - v0[j * 3 + 2]));

  //     left2[idx++] =
  //         (xs[i * 3] - xs[j * 3] + lambda2 * (v0[i * 3] - v0[j * 3])) *
  //             (xs[i * 3] - xs[j * 3] + lambda2 * (v0[i * 3] - v0[j * 3])) +
  //         (xs[i * 3 + 1] - xs[j * 3 + 1] +
  //          lambda2 * (v0[i * 3 + 1] - v0[j * 3 + 1])) *
  //             (xs[i * 3 + 1] - xs[j * 3 + 1] +
  //              lambda2 * (v0[i * 3 + 1] - v0[j * 3 + 1])) +
  //         (xs[i * 3 + 2] - xs[j * 3 + 2] +
  //          lambda2 * (v0[i * 3 + 2] - v0[j * 3 + 2])) *
  //             (xs[i * 3 + 2] - xs[j * 3 + 2] +
  //              lambda2 * (v0[i * 3 + 2] - v0[j * 3 + 2]));
  //   }
  // }

  // std::cout << "left1" << std::endl;
  // std::cout << left1 << std::endl;
  // std::cout << "left2" << std::endl;
  // std::cout << left2 << std::endl;
  // std::cout << "c_del_2" << std::endl;
  // std::cout << c_del_2 << std::endl;


  // std::cout << "control point estimate:" << std::endl;
  // for (int i = 0; i < 4; i++) {
  //   std::cout << x(i * 3) / x(i * 3) << ", " << x(i * 3 + 1) / x(i * 3) << ", "
  //             << x(i * 3 + 2) / x(i * 3) << std::endl;
  // }

  // std::cout << "points" << std::endl;
  // for (int i = 0; i < number_of_correspondences; i++) {
  //   Eigen::Vector3d p(0.0, 0.0, 0.0);
  //   for (int j = 0; j < 4; j++) {
  //     p.x() += alphas[i * 4 + j] * x(j * 3);
  //     p.y() += alphas[i * 4 + j] * x(j * 3 + 1);
  //     p.z() += alphas[i * 4 + j] * x(j * 3 + 2);
  //   }
  //   std::cout << p.x() << ", " << p.y() << ", " << p.z() << std::endl;
  // }


  // 最后计算R t
  // x = MTM.inverse() * MTb;
  // std::cout << "MTM-1MTM" << std::endl;
  // std::cout << MTM.inverse()*MTM << std::endl;

  // std::cout << "result" << std::endl;
  // std::cout << MTM * x << std::endl;

  // std::cout << "MTb" << std::endl;
  // std::cout << MTb << std::endl;

  Eigen::Matrix3d R1, R2;
  Eigen::Vector3d t1, t2;
  double error1 = compute_R_and_t(x, R1, t1);
  double error2 = compute_R_and_t(x2, R2, t2);

  // std::cout << "error" << std::endl;
  // std::cout << error1 << " / " << error2 << std::endl;

  double error = error1;
  if(error1 < error2){
    R = R1;
    t = t1;
  }else{
    error = error2;
    x = x2;
    R = R2;
    t = t2;
  }

  // std::cout << "R inverse" << std::endl << R.inverse() << std::endl;
  // std::cout << "t inverse" << std::endl << -R.inverse() * t << std::endl;

  return error;
}



/**
 * @brief 计算在给定位姿的时候的3D点投影误差
 * @param[in] R      给定旋转
 * @param[in] t      给定平移
 * @return double    重投影误差,是平均到每一对匹配点上的误差
 */
double PnPsolver::reprojection_error(const Eigen::Matrix3d &R, const Eigen::Vector3d &t)
{
  // 统计其实并不是平方
  double sum2 = 0.0;

  // 遍历每个3D点
  // std::cout << "u estimate / u row" << std::endl;
  for(int i = 0; i < number_of_correspondences; i++) {
    int id = ids[i];
    Eigen::Vector3d pw(pws[3 * i], pws[3 * i + 1], pws[3 * i + 2]);
    Eigen::Matrix3d R_i_to_c =
        option_.extric_imu_to_camera_[id].rotation().toRotationMatrix();
    Eigen::Vector3d t_i_to_c = option_.extric_imu_to_camera_[id].translation();
    // 计算这个3D点在相机坐标系下的坐标,逆深度表示
    // Eigen::Vector3d pi = (R * pw + t);
    // std::cout << pi.x() << ", " << pi.y() << ", " << pi.z() << std::endl;
    Eigen::Vector3d pc = R_i_to_c * (R * pw + t) + t_i_to_c;
    // 深度不为负
    if (pc(2) < 0) {
      // std::cout << "depth < 0" << std::endl;
      // std::cout << pc(0) << ", " << pc(1) << ", " << pc(2) << std::endl;
      return 1e3;
    }

    double Xi = pc(0);
    double Yi = pc(1);
    double inv_Zi = 1.0 / pc(2);

    // 计算投影点
    double ui = Xi * inv_Zi;
    double vi = Yi * inv_Zi;
    // 计算投影点与匹配2D点的欧氏距离的平方
    double u = mvP3D_undis[i].x(), v = mvP3D_undis[i].y();
    // 得到其欧式距离并累加
    sum2 += sqrt( (u - ui) * (u - ui) + (v - vi) * (v - vi));
    // std::cout << u << " / " << ui << ", " << v << " / " << vi << std::endl;
  }
  // 返回平均误差
  return sum2 / number_of_correspondences;
}

/**
 * @brief 用3D点在世界坐标系和相机坐标系下对应的坐标，用ICP求取R t
 * @param[out] R   旋转
 * @param[out] t   平移
 */
void PnPsolver::estimate_R_and_t(Eigen::Matrix3d &R, Eigen::Vector3d &t) {
  // 计算3D点的质心
  Eigen::Vector3d pi0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d pw0 = Eigen::Vector3d::Zero();

  for (int i = 0; i < number_of_correspondences; i++) {
    const double *pi = pis + 3 * i;
    const double *pw = pws + 3 * i;

    for (int j = 0; j < 3; j++) {
      pi0(j) += pi[j];
      pw0(j) += pw[j];
    }
  }
  for (int j = 0; j < 3; j++) {
    pi0(j) /= number_of_correspondences;
    pw0(j) /= number_of_correspondences;
  }

  // 准备构造矩阵A,B以及B^T*A的SVD分解的值
  Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
  for (int i = 0; i < number_of_correspondences; i++) {
    double *pi = pis + 3 * i;
    double *pw = pws + 3 * i;

    for (int j = 0; j < 3; j++) {
      W(j, 0) += (pi[j] - pi0[j]) * (pw[0] - pw0[0]);
      W(j, 1) += (pi[j] - pi0[j]) * (pw[1] - pw0[1]);
      W(j, 2) += (pi[j] - pi0[j]) * (pw[2] - pw0[2]);
    }
  }
  // std::cout << "W" << std::endl << W << std::endl;

  // 对矩阵W进行SVD分解
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      W, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d W_U = svd.matrixU();
  Eigen::Vector3d W_D = svd.singularValues();
  Eigen::Matrix3d W_V = svd.matrixV();

  // std::cout << "W eigen value" << std::endl;
  // std::cout << W_D << std::endl;

  // Step 4 R=U*V^T, 并且进行合法性检查
  R = W_U * W_V.transpose();

  // 注意在得到了R以后,需要保证 det(R)=1>0
  const double det = R.determinant();
  if (det < 0) {
    // std::cout << "det: " << det << std::endl;
    R(2, 0) = -R(2, 0);
    R(2, 1) = -R(2, 1);
    R(2, 2) = -R(2, 2);
  }

  // Step 5 根据R计算t
  t = pi0 - R * pw0;

  // std::cout << "estimate R: " << std::endl << R << std::endl;
  // std::cout << "estimete t: " << std::endl << t << std::endl;
  // std::cout << "R inverse" << std::endl << R.inverse() << std::endl;
  // std::cout << "t inverse" << std::endl << -R.inverse() * t << std::endl;
}


// 保持所有点在相机坐标系下的深度为正,调整符号
void PnPsolver::solve_for_sign(void)
{
  // 根据第一个3D点在当前相机坐标系下的深度,调整所有的3D点的深度为正(因为正常地来讲,这些3D点都应该是在相机前面的)
  // 如果第一个点的深度是负的话
  if (pis[2] < 0.0) {
    // 先调整控制点的坐标
    for(int i = 0; i < 4; i++)
      for(int j = 0; j < 3; j++)
	      cis[i][j] = -cis[i][j];

    // 然后调整3D点的坐标
    for(int i = 0; i < number_of_correspondences; i++) {
      pis[3 * i    ] = -pis[3 * i];
      pis[3 * i + 1] = -pis[3 * i + 1];
      pis[3 * i + 2] = -pis[3 * i + 2];
    }
  }
}

/**
 * @brief 根据已经得到的控制点在当前相机坐标系下的坐标来恢复出相机的位姿
 * @param[in]  x          控制点坐标
 * @param[out] R          计算得到的相机旋转R
 * @param[out] t          计算得到的相机位置t
 * @return double         使用这个位姿,所得到的重投影误差
 */
double PnPsolver::compute_R_and_t(const Eigen::VectorXd &x, Eigen::Matrix3d &R,
                                  Eigen::Vector3d &t) {
  compute_cis(x);
  compute_pis();
  // 调整点坐标的符号,来保证在相机坐标系下点的深度为正
  // solve_for_sign();

  // Step 4 ICP计算R和t
  estimate_R_and_t(R, t);

  // Step 5 计算使用这个位姿,所得到的每对点平均的重投影误差,作为返回值
  return reprojection_error(R, t);
}

//

// void PnPsolver::qr_solve(Eigen::MatrixXd &A, Eigen::VectorXd &b, Eigen::VectorXd &X) {
//   double *A1, *A2;
//   const int nr = A.rows();
//   const int nc = A.cols();
//   A1 = new double[nr];
//   A2 = new double[nr];

//   double *pA = A.data(), *ppAkk = pA;
//   for (int k = 0; k < nc; k++) {
//     // 求eta,A每列最大元素
//     double *ppAik = ppAkk, eta = fabs(*ppAik);
//     for (int i = k + 1; i < nr; i++) {
//       double elt = fabs(*ppAik);
//       if (eta < elt) eta = elt;
//       ppAik += nc;
//     }

//     if (eta == 0) {
//       A1[k] = A2[k] = 0.0;
//       LOG(ERROR) << "God damnit, A is singular, this shouldn't happen.";
//       return;
//     } else {
//       double *ppAik = ppAkk, sum = 0.0, inv_eta = 1. / eta;
//       for (int i = k; i < nr; i++) {
//         *ppAik *= inv_eta;
//         sum += *ppAik * *ppAik;
//         ppAik += nc;
//       }
//       double sigma = sqrt(sum);
//       if (*ppAkk < 0) sigma = -sigma;
//       *ppAkk += sigma;
//       A1[k] = sigma * *ppAkk;
//       A2[k] = -eta * sigma;
//       for (int j = k + 1; j < nc; j++) {
//         double *ppAik = ppAkk, sum = 0;
//         for (int i = k; i < nr; i++) {
//           sum += *ppAik * ppAik[j - k];
//           ppAik += nc;
//         }
//         double tau = sum / A1[k];
//         ppAik = ppAkk;
//         for (int i = k; i < nr; i++) {
//           ppAik[j - k] -= tau * *ppAik;
//           ppAik += nc;
//         }
//       }
//     }
//     ppAkk += nc + 1;
//   }

//   // b <- Qt b
//   double *ppAjj = pA, *pb = b.data();
//   for (int j = 0; j < nc; j++) {
//     double *ppAij = ppAjj, tau = 0;
//     for (int i = j; i < nr; i++) {
//       tau += *ppAij * pb[i];
//       ppAij += nc;
//     }
//     tau /= A1[j];
//     ppAij = ppAjj;
//     for (int i = j; i < nr; i++) {
//       pb[i] -= tau * *ppAij;
//       ppAij += nc;
//     }
//     ppAjj += nc + 1;
//   }

//   // X = R-1 b
//   double *pX = X.data();
//   pX[nc - 1] = pb[nc - 1] / A2[nc - 1];
//   for (int i = nc - 2; i >= 0; i--) {
//     double *ppAij = pA + i * nc + (i + 1), sum = 0;

//     for (int j = i + 1; j < nc; j++) {
//       sum += *ppAij * pX[j];
//       ppAij++;
//     }
//     pX[i] = (pb[i] - sum) / A2[i];
//   }
// }

void PnPsolver::qr_solve(Eigen::MatrixXd &A, Eigen::VectorXd &b, Eigen::VectorXd &X) {
  // 施密特正交化
  Eigen::HouseholderQR<Eigen::MatrixXd> qr(A);
  Eigen::MatrixXd Q = qr.householderQ();
  Eigen::MatrixXd R = qr.matrixQR().triangularView<Eigen::Upper>();

  std::cout << "Q matrix:\n" << Q << std::endl;
  std::cout << "R matrix:\n" << R << std::endl;
  
  std::cout << "QTb matrix: \n" << Q.transpose()*b << std::endl;
  std::cout << "QTQ martix: \n" << Q.transpose()*Q << std::endl;
}

} // namespace match
} // namespace mapping
} // namespace jarvis