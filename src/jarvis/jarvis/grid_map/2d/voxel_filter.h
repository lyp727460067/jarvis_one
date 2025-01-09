/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_VOXEL_FILTER_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_VOXEL_FILTER_H_

#include <bitset>

#include "jarvis/grid_map/2d/point_cloud.h"

namespace jarvis {
namespace sensor {
struct AdaptiveVoxelFilterOptions {
  int min_num_points;
  float max_length = 0.5;
  float max_range = 5;
};
std::vector<RangefinderPoint> VoxelFilter(
    const std::vector<RangefinderPoint>& points, const float resolution);
PointCloud VoxelFilter(const PointCloud& point_cloud, const float resolution);
//
std::vector<Eigen::Vector3f > VoxelFilter(
    const std::vector<Eigen::Vector3f>& points, const float resolution) ;
 

TimedPointCloud VoxelFilter(const TimedPointCloud& timed_point_cloud,
                            const float resolution);
//
PointCloud AdaptiveVoxelFilter(const PointCloud& point_cloud,
                               const AdaptiveVoxelFilterOptions& options);

}  // namespace sensor
}  // namespace jarvis

#endif  // CARTOGRAPHER_SENSOR_INTERNAL_VOXEL_FILTER_H_
