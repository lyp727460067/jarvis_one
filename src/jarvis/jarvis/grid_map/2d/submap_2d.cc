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
#include <Eigen/Dense>
#include "jarvis/grid_map/2d/submap_2d.h"
#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>

#include "Eigen/Geometry"
#include "glog/logging.h"
#include "jarvis/common/port.h"
#include "jarvis/grid_map/2d/probability_grid_range_data_inserter_2d.h"

namespace jarvis {
namespace grid_map {

Submap2D::Submap2D(const Eigen::Vector2f& origin, std::unique_ptr<Grid2D> grid,
                   ValueConversionTables* conversion_tables)
    : Submap(transform::Rigid3d::Translation(
          Eigen::Vector3d(origin.x(), origin.y(), 0.))),
      conversion_tables_(conversion_tables) {
  grid_ = std::move(grid);
}

// void Submap2D::ToResponseProto(
//     const transform::Rigid3d&,
//     proto::SubmapQuery::Response* const response) const {
//   if (!grid_) return;
//   response->set_submap_version(num_range_data());
//   proto::SubmapQuery::Response::SubmapTexture* const texture =
//       response->add_textures();
//   grid()->DrawToSubmapTexture(texture, local_pose());
// }

void Submap2D::InsertRangeData(
    const sensor::RangeData& range_data,
    const ProbabilityGridRangeDataInserter2D* range_data_inserter) {
  CHECK(grid_);
  // CHECK(!insertion_finished());
  range_data_inserter->Insert(range_data, grid_.get());
  set_num_range_data(num_range_data() + 1);
}

void Submap2D::Finish() {
  CHECK(grid_);
  // CHECK(!insertion_finished());
  grid_ = grid_->ComputeCroppedGrid();
  set_insertion_finished(true);
}

ActiveSubmaps2D::ActiveSubmaps2D(const SubmapsOptions2DOption& options)
    : options_(options), range_data_inserter_(CreateRangeDataInserter()) {}

std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::submaps() const {
  return std::vector<std::shared_ptr<const Submap2D>>(submaps_.begin(),
                                                      submaps_.end());
}

std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::InsertRangeData(
    const sensor::RangeData& range_data) {
  //
  if (!submaps_.empty()) {
    Eigen::AlignedBox2i known_cells_box =
        submaps_.back()->grid()->known_cells_box();
    //
    // LOG(INFO) << known_cells_box.diagonal().transpose();
  }

  // if (submaps_.empty() ||
  //     submaps_.back()->num_range_data() == options_.num_range_data) {
  //   AddSubmap(range_data.origin.head<2>());
  // }
  if (submaps_.empty()) {
    AddSubmap(range_data.origin.head<2>());
  } else {
    Eigen::AlignedBox2i known_cells_box =
        submaps_.back()->grid()->known_cells_box();
    if (known_cells_box.sizes().x() > options_.min_y_map_size &&
        known_cells_box.sizes().y() > options_.min_x_map_size) {
      AddSubmap(range_data.origin.head<2>());
    }
  }

  for (auto& submap : submaps_) {
    submap->InsertRangeData(range_data, range_data_inserter_.get());
  }
  // Eigen::AlignedBox2i known_cells_box =
  //     submaps_.front()->grid()->known_cells_box();
  // if (known_cells_box.sizes().x() > 2 * options_.min_y_map_size &&
  //     known_cells_box.sizes().y() > 2 * options_.min_x_map_size) {
  //   submaps_.front()->Finish();
  // }

  // if (submaps_.front()->num_range_data() == 2 * options_.num_range_data) {
  //   submaps_.front()->Finish();
  // }
  return submaps();
}

//
std::unique_ptr<ProbabilityGridRangeDataInserter2D>
ActiveSubmaps2D::CreateRangeDataInserter() {
  return std::make_unique<ProbabilityGridRangeDataInserter2D>(
      options_.range_data_inserter_option);
}

//
std::unique_ptr<ProbabilityGrid> ActiveSubmaps2D::CreateGrid(
    const Eigen::Vector2f& origin) {
  constexpr int kInitialSubmapSize = 100;
  float resolution = options_.resolution;
  return std::make_unique<ProbabilityGrid>(
      MapLimits(resolution,
                origin.cast<double>() + 0.5 * kInitialSubmapSize * resolution *
                                            Eigen::Vector2d::Ones(),
                CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
      &conversion_tables_);
}

void ActiveSubmaps2D::AddSubmap(const Eigen::Vector2f& origin) {
  if (submaps_.size() >= 2) {
    // This will crop the finished Submap before inserting a new Submap to
    // reduce peak memory usage a bit.
    // CHECK(submaps_.front()->insertion_finished());
    submaps_.erase(submaps_.begin());
  }
  submaps_.push_back(std::make_unique<Submap2D>(
      origin,
      std::unique_ptr<Grid2D>(
          static_cast<Grid2D*>(CreateGrid(origin).release())),
      &conversion_tables_));
}

}  // namespace mapping
}  // namespace cartographer
