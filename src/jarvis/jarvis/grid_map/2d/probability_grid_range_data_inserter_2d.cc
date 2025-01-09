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

#include "jarvis/grid_map/2d/probability_grid_range_data_inserter_2d.h"

#include <cstdlib>
#include <set>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "glog/logging.h"
#include "jarvis/grid_map/2d/probability_values.h"
#include "jarvis/grid_map/2d/ray_to_pixel_mask.h"
#include "jarvis/grid_map/2d/xy_index.h"
// namespace std{
// bool operator<(const Eigen::Array2i& lhs, const Eigen::Array2i& rhs) {
//   if (lhs(0) != rhs(0)) return lhs(0) < rhs(0);
//   return lhs(1) < rhs(1);
// }
// }

struct Array2iCompar {
  bool operator()(const Eigen::Array2i& lhs, const Eigen::Array2i& rhs)const {
    if (lhs(0) != rhs(0)) return lhs(0) < rhs(0);
    return lhs(1) < rhs(1);
  }
};

namespace jarvis {
namespace grid_map {
namespace {

// Factor for subpixel accuracy of start and end point for ray casts.
constexpr int kSubpixelScale = 1000;

void GrowAsNeeded(const sensor::RangeData& range_data,
                  ProbabilityGrid* const probability_grid) {
  Eigen::AlignedBox2f bounding_box(range_data.origin.head<2>());
  // Padding around bounding box to avoid numerical issues at cell boundaries.
  constexpr float kPadding = 1e-6f;
  for (const sensor::RangefinderPoint& hit : range_data.returns) {
    bounding_box.extend(hit.position.head<2>());
  }
  for (const sensor::RangefinderPoint& miss : range_data.misses) {
    bounding_box.extend(miss.position.head<2>());
  }
  if (!range_data.sector.end_points.empty()) {
    for (const auto &p : range_data.sector.end_points)
      bounding_box.extend(p.head<2>());
  }
  probability_grid->GrowLimits(bounding_box.min() -
                               kPadding * Eigen::Vector2f::Ones());
  probability_grid->GrowLimits(bounding_box.max() +
                               kPadding * Eigen::Vector2f::Ones());
}
template <typename T>
inline T NormalizeAngle(const T& angle_radians) {
  // Use ceres::floor because it is specialized for double and Jet types.
  T two_pi(2.0 * M_PI);
  return angle_radians -
         two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}
//
//

//
void CastRays(const sensor::RangeData& range_data,
              const std::vector<uint16>& hit_table,
              const std::vector<uint16>& miss_table,
              const float min_distance,
              const bool insert_free_space, ProbabilityGrid* probability_grid) {
  GrowAsNeeded(range_data, probability_grid);

  const MapLimits& limits = probability_grid->limits();
  const double superscaled_resolution = limits.resolution() / kSubpixelScale;
  const MapLimits superscaled_limits(
      superscaled_resolution, limits.max(),
      CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                 limits.cell_limits().num_y_cells * kSubpixelScale));
  const Eigen::Array2i begin =
      superscaled_limits.GetCellIndex(range_data.origin.head<2>());
  // Compute and add the end points.
  std::vector<Eigen::Array2i> ends;
  std::set<Eigen::Array2i,Array2iCompar> ends_set;
  ends.reserve(range_data.returns.size());
  for (const sensor::RangefinderPoint& hit : range_data.returns) {
    ends.push_back(superscaled_limits.GetCellIndex(hit.position.head<2>()));
    ends_set.insert(ends.back() / kSubpixelScale);
    probability_grid->ApplyLookupTable(ends.back() / kSubpixelScale, hit_table);
  }
  ends.clear();
  if (!range_data.sector.end_points.empty()) {
    Eigen::Vector2i index;
    Eigen::AlignedBox2f bounding_box(range_data.origin.head<2>());
    for (auto& p : range_data.sector.end_points) {
      bounding_box.extend(p.head<2>());
    }
    const MapLimits& limits = probability_grid->limits();
    
    auto max_index = limits.GetCellIndex(bounding_box.max());
    auto min_index = limits.GetCellIndex(bounding_box.min());
    //
    for (const Eigen::Array2i &xy_index :
         XYIndexRangeIterator(max_index, min_index)) {
      if (!probability_grid->IsKnown(xy_index)) continue;
      if(ends_set.count(xy_index))continue;
      const Eigen::Vector2f point =
          limits.GetCellCenter(xy_index) - range_data.origin.head<2>();
      Eigen::Vector3f pointxyz(point.x(), point.y(), 0);
      if( pointxyz.norm()>range_data.sector.r)continue;
      if( pointxyz.norm()<min_distance)continue;
      float cos = range_data.sector.centor.dot(pointxyz.normalized());
      Eigen::Vector3f cross_v =
          range_data.sector.centor.cross(pointxyz.normalized());
      // LOG(INFO)<<cross_v; 
      if (cross_v.z() < 0) {
        cos = -cos;
      }
      if (cos > range_data.sector.max_cos ||  (cos<0&& cos <-range_data.sector.min_cos)  ) {
        ends.push_back(xy_index);
      }
    }
    for (const Eigen::Array2i& cell_index : ends) {
      probability_grid->ApplyLookupTable(cell_index, miss_table);
    }
    return;
  }
  if (!insert_free_space) {
    return;
  }

  // Now add the misses.
  for (const Eigen::Array2i& end : ends) {
    std::vector<Eigen::Array2i> ray =
        RayToPixelMask(begin, end, kSubpixelScale);
    for (const Eigen::Array2i& cell_index : ray) {
      const Eigen::Vector2f point =
          limits.GetCellCenter(cell_index) - range_data.origin.head<2>();
      if( point.norm()<min_distance)continue;
      probability_grid->ApplyLookupTable(cell_index, miss_table);
    }
  }

  // Finally, compute and add empty rays based on misses in the range data.
  for (const sensor::RangefinderPoint& missing_echo : range_data.misses) {
    std::vector<Eigen::Array2i> ray = RayToPixelMask(
        begin, superscaled_limits.GetCellIndex(missing_echo.position.head<2>()),
        kSubpixelScale);
    for (const Eigen::Array2i& cell_index : ray) {
      const Eigen::Vector2f point =
          limits.GetCellCenter(cell_index) - range_data.origin.head<2>();
      if( point.norm()<min_distance)continue;
      probability_grid->ApplyLookupTable(cell_index, miss_table);
    }
  }
}
}  // namespace

//
ProbabilityGridRangeDataInserter2D::ProbabilityGridRangeDataInserter2D(
    const ProbabilityGridRangeDataInserterOptions2D& options)
    : options_(options),
      hit_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
          Odds(options.hit_probability))),
      miss_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
          Odds(options.miss_probability))) {}

void ProbabilityGridRangeDataInserter2D::Insert(
    const sensor::RangeData& range_data, Grid2D* const grid) const {
  ProbabilityGrid* const probability_grid = static_cast<ProbabilityGrid*>(grid);
  CHECK(probability_grid != nullptr);
  // By not finishing the update after hits are inserted, we give hits priority
  // (i.e. no hits will be ignored because of a miss in the same cell).
  CastRays(range_data, hit_table_, miss_table_,
           options_.min_free_distance, options_.insert_free_space,
           probability_grid);
  probability_grid->FinishUpdate();
}

}  // namespace mapping
}  // namespace cartographer
