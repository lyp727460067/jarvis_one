#include "jarvis/grid_map/grid_impl.h"
//
#include "jarvis/grid_map/2d/voxel_filter.h"
#include "opencv2/opencv.hpp"
#include "jarvis/utility/tic_toc.h"
namespace jarvis {
namespace grid_map {
//

//
sensor::PointCloud ToLaserData(const GridMapOption& option,
                               const PointCloud& point_cloud) {
  const float& ang_size = option.angle_size;
  const float& min_angle = option.min_angle;
  const float& max_angle = option.max_angle;
  //
  const int index_size =
      static_cast<int>((max_angle - min_angle + 0.5) / ang_size);
  std::vector<int> his_index(index_size, -1);
  std::vector<double> xy_normal(index_size, 100);
  for (size_t i = 0; i < point_cloud.size(); i++) {
    
    const auto& p = point_cloud[i];
    float angle =
        common::RadToDeg(common::atan2(Eigen::Vector2f{p.x(), p.y()}));
    if (angle < min_angle) continue;
    if (angle > max_angle) continue;
    angle -= min_angle;
    //
    int ange_index = static_cast<int>(angle / ang_size);
    CHECK_GE(ange_index, 0);
    const double p_normal = p.head<2>().norm();
    if (xy_normal[ange_index] > p_normal) {
      xy_normal[ange_index] = p_normal;
      his_index[ange_index] = i;
    }
  }
  sensor::PointCloud result;
  for (size_t i = 0; i < his_index.size(); i++) {
    if (his_index[i] != -1) {
      result.push_back({point_cloud[his_index[i]]});
    } else {
      const Eigen::AngleAxisf rotation(
          common::DegToRad(i * ang_size + min_angle),
          Eigen::Vector3f::UnitZ());
      result.push_back({(rotation * ((option.max_distance + 0.5) *
                                               Eigen::Vector3f::UnitX()))});
    }
  }
  return result;
  //
}

//
sensor::RangeData ToRangeSensor(const GridMapOption& option,
                                const transform::Rigid3f pose,
                                const PointCloud& point_cloud) {
  //
  const Eigen::Vector3f& origin = pose.translation();
  if (option.insert_free_space && !option.insert_free_sector_space) {
    auto point_clouds = ToLaserData(option, point_cloud);
    sensor::PointCloud miss;
    sensor::PointCloud hit;
    for (const auto& p : point_clouds) {
      CHECK(!isnan(p.position.norm()));
      // CHECK(p.position.norm() < 1e4) << p.position.norm() << " point
      // valid!!!!";
      Eigen::Vector3f pos(p.position.x(), p.position.y(), 0);
      if (pos.norm() > option.max_distance) {
        miss.push_back({pose * pos});
      } else {
        hit.push_back({pose * pos});
      }
    }
    return sensor::RangeData{origin, hit, miss};
  }
  sensor::PointCloud result;
  sensor::RangeData::SectorPara sector;
  for (auto const& p : point_cloud) {
    CHECK(!isnan(p.norm())) << "nan";
    // CHECK(p.norm() < 1e4) << p.norm()<< " point valid!!!!";
    Eigen::Vector3f pos = p;
    pos.z() = 0.0;
    if (pos.norm() < option.max_distance) {
      result.push_back({pose * pos});
    }
  }
  if (option.insert_free_sector_space) {
    //
    // sector.end_angle = common::DegToRad(option.max_angle);
    // sector.start_angle = common::DegToRad(option.min_angle);
    //
    //
    sector.r =  option.max_distance;
    sector.min_cos = cos(common::DegToRad(option.min_angle));
    sector.max_cos = cos(common::DegToRad(option.max_angle));
    Eigen::AngleAxisf rotation(common::DegToRad(option.min_angle),
                               Eigen::Vector3f::UnitZ());
    sector.end_points.push_back(
        pose * (rotation * ((option.max_distance) * Eigen::Vector3f::UnitX())));
    rotation = Eigen::AngleAxisf(common::DegToRad(option.max_angle),
                                 Eigen::Vector3f::UnitZ());
    sector.end_points.push_back(
        pose * (rotation * ((option.max_distance) * Eigen::Vector3f::UnitX())));
    sector.end_points.push_back(
        pose * (((option.max_distance) * Eigen::Vector3f::UnitX())));

    sector.centor =
        pose.rotation() * Eigen::Vector3f::UnitX();
  }
  //
  if (result.size() > 200) {
    return sensor::RangeData{
        origin, sensor::VoxelFilter(result, option.point_votex), {},sector};
  }
  return sensor::RangeData{origin, result, {},sector};
}

//
//
void GridImpl::Insert(const AiObject& objects) {
  transform::Rigid3f pose(objects.pose.tanslation.cast<float>(),
                          objects.pose.rotaion.cast<float>());
  // LOG(INFO)<<pose;
  last_pose_ = transform::Project2D(pose);
  for (auto& submap : active_submaps_) {
    jarvis::estimator::TicToc feature_t_t;
    // CHECK(active_submaps_.count(submap.first))
    //     << "Need construct register type";
    // //
    if (objects.points_clouds.count(submap.first)) {
      // LOG(INFO) << "[" <<int(submap.first) << "]" << "insert point size :"
      //           << objects.points_clouds.at(submap.first).size();

      active_submaps_.at(submap.first)
          ->InsertRangeData(
              ToRangeSensor(options_[submap.first], pose,
                            objects.points_clouds.at(submap.first)));
      // LOG(INFO) << "Inser Cost " << feature_t_t.toc();
    } else {
      active_submaps_.at(submap.first)
          ->InsertRangeData(ToRangeSensor(options_[submap.first], pose, {}));
    }
  }
}
//
std::map<uint8_t, ObResultValue> GridImpl::IndexValue(
    const Eigen::Vector2f& index) {
  //
  std::map<uint8_t, ObResultValue> result;

  for (const auto& submap_pair : active_submaps_) {
    //
    auto grid = submap_pair.second->submaps()[0]->grid();
    auto index_xy = grid->limits().GetCellIndex(last_pose_ * index);
    float p = (1.0f - grid->GetCorrespondenceCost(index_xy));
    result[submap_pair.first].p = static_cast<uint8_t>(p * 255);
  }
  return result;
}
//
GridImpl::GridImpl(const std::map<int, GridMapOption>& option)
    : options_(option) {
  for (const auto& op : options_) {
    active_submaps_.emplace(
        op.first, std::make_unique<ActiveSubmaps2D>(SubmapsOptions2DOption{
                      ProbabilityGridRangeDataInserterOptions2D{
                          op.second.hit_probability, op.second.miss_probability,
                          op.second.insert_free_space,
                          op.second.insert_free_min_distance},
                      op.second.max_node_num,
                      op.second.resolution,
                      op.second.min_x_map_size,
                      op.second.min_y_map_size,
                  }));
  }
}
//
//
PointCloud GridImpl::Votex(const PointCloud& point_clous, float size) {
  return sensor::VoxelFilter(point_clous, size);
}
bool GridImpl::HasValue(const Eigen::Vector2i& index) { return false; }
GridImpl::~GridImpl() {}

//
std::unique_ptr<GridMap> GridMap::Create(
    const std::map<int, GridMapOption>& option) {
  return std::make_unique<GridImpl>(option);
};

void GridImpl::ToPgn(const std::string& dir) {
  LOG(INFO) << "write to png";
  for (const auto& submap_pair : active_submaps_) {
    LOG(INFO) << submap_pair.second->submaps().size();
    auto const grid = submap_pair.second->submaps()[0]->grid();
    int rows = grid->limits().cell_limits().num_y_cells;
    int cols = grid->limits().cell_limits().num_x_cells;
    LOG(INFO) << rows << cols;
    cv::Mat image(rows, cols, CV_8UC1);

    for (int i = 0; i < rows; i++) {
      for (int j = 0; j < cols; j++) {
        if (grid->IsKnown({rows - i, j})) {
          float p = (1.0f - grid->GetCorrespondenceCost({rows - i, j}));
          // if(p<0.5){
          //   image.at<uint8_t>(i, j) = 0.1*255;
          //   continue;
          // }
          image.at<uint8_t>(i, j) = 255 * p;
        } else {
          image.at<uint8_t>(i, j) = 0.1 * 255;
        }
      }
    }
    // cv::imshow(std::to_string(submap_pair.first),image);
    // cv::waitKey(0);
    cv::imwrite(dir + "_" + std::to_string(submap_pair.first) + ".png", image);
  }
}

}  // namespace grid_map
}  // namespace jarvis