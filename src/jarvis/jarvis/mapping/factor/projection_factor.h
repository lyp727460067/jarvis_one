#pragma once

#include <ceres/ceres.h>

#include "Eigen/Dense"

#include "jarvis/estimator/parameters.h"
#include "jarvis/utility/tic_toc.h"
#include "jarvis/utility/utility.h"
namespace jarvis {
namespace mapping {
class ProjectionFactor{
 public:
     ProjectionFactor(const double observed_u, const double observed_v) : 
                      observed_u_(observed_u), observed_v_(observed_v) {}

     template <typename T>
     bool operator()(const T *const imu_pos, const T *const map_point,
                     const T *const ex_cam_to_imu, T *residual) const {
         Eigen::Quaternion<T> q_wi(imu_pos[3], imu_pos[4], imu_pos[5], imu_pos[6]);
         Eigen::Quaternion<T> q_ic(ex_cam_to_imu[3], ex_cam_to_imu[4], ex_cam_to_imu[5], ex_cam_to_imu[6]);
         Eigen::Vector3<T> t_wi(imu_pos[0], imu_pos[1], imu_pos[2]);
         Eigen::Vector3<T> t_ic(ex_cam_to_imu[0], ex_cam_to_imu[1], ex_cam_to_imu[2]);
         Eigen::Vector3<T> p_w(map_point[0], map_point[1], map_point[2]);

        // std::cout << "imu_pose: " << imu_pos[0] << ", " << imu_pos[1] << ", " 
        //           << imu_pos[2] << ", " << imu_pos[3] << ", " << imu_pos[4] << ", " 
        //           << imu_pos[5] << ", " << imu_pos[6] << std::endl; 
        //  std::cout << "q: " << q_wi << std::endl;
        //  std::cout << "t: " << t_wi.x() << ", " << t_wi.y() << ", " << t_wi.z() << std::endl;

         // 世界坐标系到相机坐标系的转换
         Eigen::Quaternion<T> q_wc = q_wi * q_ic;
         Eigen::Vector3<T> t_wc = q_wi * t_ic + t_wi;
         Eigen::Quaternion<T> q_cw = q_wc.inverse();
         Eigen::Vector3<T> t_cw = -(q_cw * t_wc);
         Eigen::Vector3<T> p_c = q_cw * p_w + t_cw;

         // 归一化
        //  Eigen::Vector2<T> p_normal = (p_c / p_c.z()).head<2>();
        Eigen::Vector2<T> p_normal(p_c.x() / p_c.z(), p_c.y() / p_c.z());

         // 传入观察坐标是归一化坐标,直接相减即可
         residual[0] = p_normal[0] - observed_u_;
         residual[1] = p_normal[1] - observed_v_;

        //  // Z = 1
        //  T x_normalized = -pt3_c[0] / pt3_c[2];
        //  T y_normalized = -pt3_c[1] / pt3_c[2];

        //  residual[0] = observed_u - x_normalized;
        //  residual[1] = observed_v - y_normalized;

         return true;
     }

     static ceres::CostFunction *Create(const double observed_u, const double observed_v){
         return (new ceres::AutoDiffCostFunction<ProjectionFactor, 2, 7, 3, 7>(
             new ProjectionFactor(observed_u, observed_v)));
     }

 private:
     double observed_u_;
     double observed_v_;
};
}  // namespace mapping
}  // namespace jarvis
