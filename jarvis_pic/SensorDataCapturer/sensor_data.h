/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OV_CORE_SENSOR_DATA_H
#define OV_CORE_SENSOR_DATA_H

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <vector>

namespace VSLAM 
{

typedef struct StereoImages
{
    cv::Mat mLeftImg;
    cv::Mat mRightImg;
    uint64_t mTimeStamp;
    int count;
}StereoImages;

typedef struct ImuData_NotAligned
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    uint64_t time_stamp;  ///<  MCU端时间戳 [us]
    uint32_t sync_count;   ///<  脉冲计数
      /// Gyroscope reading, angular velocity (rad/s)
    Eigen::Matrix<double, 3, 1> wm;

    /// Accelerometer reading, linear acceleration (m/s^2)
    Eigen::Matrix<double, 3, 1> am;
}ImuData_NotAligned;


/**
 * @brief Struct for a single imu measurement (time, wm, am)
 */
struct ImuData 
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// Timestamp of the reading
  double timestamp;

  /// Gyroscope reading, angular velocity (rad/s)
  Eigen::Matrix<double, 3, 1> wm;

  /// Accelerometer reading, linear acceleration (m/s^2)
  Eigen::Matrix<double, 3, 1> am;

  /// Sort function to allow for using of STL containers
  bool operator<(const ImuData &other) const { return timestamp < other.timestamp; }
};
} // namespace ov_core

#endif // OV_CORE_SENSOR_DATA_H