#include "jarvis/mapping/match/area_search.h"

#include "glog/logging.h"
#include "jarvis/common/math.h"
#include "math.h"
namespace jarvis {
namespace mapping {
namespace match {
//
bool AreaSearchGrid::PosInGrid(const cv::KeyPoint& kp, int& posX, int& posY) {
  //
  posX = std::floor((kp.pt.x - image_box_.min().x()) * grid_element_width_inv_);
  posY =
      std::floor((kp.pt.y - image_box_.min().y()) * frid_element_height_inv_);
  // Keypoint's coordinates are undistorted, which could cause to go out of the
  // image
  if (posX < 0 || posX >= area_grid_num_.x() || posY < 0 ||
      posY >= area_grid_num_.y())
    return false;
  return true;
}
//

//
//
AreaSearchGrid::AreaSearchGrid(
    const Eigen::AlignedBox2i& image_box, const Eigen::Vector2i& area_grid_num,
    const Range<MapById<FeatureId, FeatureData>::ConstIterator>& target_points)
    : image_box_(image_box), area_grid_num_(area_grid_num) {
  grid_element_width_inv_ =
      static_cast<float>(area_grid_num.x()) / (image_box.sizes().x());
  frid_element_height_inv_ =
      static_cast<float>(area_grid_num.y()) / (image_box.sizes().y());
  //
  grid_.resize(area_grid_num.x());
  for (int i = 0; i < area_grid_num.x(); i++) {
    grid_[i].resize(area_grid_num.y());
  }
  for (const auto& pt : target_points) {
    const cv::KeyPoint& kp = pt.data.key_point;
    int nGridPosX, nGridPosY;
    if (PosInGrid(kp, nGridPosX, nGridPosY)) {
      grid_[nGridPosX][nGridPosY].push_back(GridValue{pt.id, kp});
    }
  }
}

//
//
Eigen::AlignedBox2f AreaSearchGrid::GetBound(float x, float y, double r) {
  const int nMinCellX =
      std::max(0, int(std::floor((x - image_box_.min().x() - r) *
                                 grid_element_width_inv_)));
  //
  if (nMinCellX >= area_grid_num_.x()) return Eigen::AlignedBox2f();
  //
  const int nMaxCellX =
      std::min((int)area_grid_num_.x() - 1,
               common::RoundToInt((x - image_box_.min().x() + r) *
                                  grid_element_width_inv_));
  //
  if (nMaxCellX < 0) return Eigen::AlignedBox2f();
  const int nMinCellY =
      std::max(0, int(std::floor(((y - image_box_.min().y() - r) *
                                  frid_element_height_inv_))));
  if (nMinCellY >= area_grid_num_.y()) return Eigen::AlignedBox2f();
  //
  const int nMaxCellY =
      std::min((int)area_grid_num_.y() - 1,
               common::RoundToInt((y - image_box_.min().y() + r) *
                                  frid_element_height_inv_));
  if (nMaxCellY < 0) return Eigen::AlignedBox2f();

  return Eigen::AlignedBox2f(Eigen::Vector2f{nMinCellX, nMinCellY},
                             Eigen::Vector2f{nMaxCellX, nMaxCellY});
}
//
//

//
AreaSearch::AreaSearch(const AreaSearchOption& options, int s,
                       const KeyFrameData& target_frame)
    : option_(options), target_points_(target_frame.data->features) {
  const auto one_sequence_feautes = target_points_.trajectory(s);
  grid_ = std::make_unique<AreaSearchGrid>(
      option_.image_box, option_.area_grid_num, one_sequence_feautes);
}

std::vector<FeatureId> AreaSearchGrid::GetNear(const cv::KeyPoint& point,
                                               double r) {
  auto feat_box = GetBound(point.pt.x, point.pt.y, r);
  //
  std::vector<FeatureId> result;
  if (feat_box.isEmpty()) return {};
  for (int ix = feat_box.min().x(); ix <= feat_box.max().x(); ix++) {
    for (int iy = feat_box.min().y(); iy <= feat_box.max().y(); iy++) {
      const std::vector<GridValue> vCell = grid_[ix][iy];
      for (size_t j = 0, jend = vCell.size(); j < jend; j++) {
        //
        const cv::KeyPoint& target_point = vCell[j].p;
        //
        const float distx = target_point.pt.x - point.pt.x;
        const float disty = target_point.pt.y - point.pt.y;
        if (fabs(distx) < r && fabs(disty) < r) result.push_back(vCell[j].id);
      }
    }
  }
  return result;
}

//
//
std::vector<FeatureId> AreaSearch::GetRadiusIndex(const cv::KeyPoint& point,
                                                  double r)const {
  return grid_->GetNear(point, r);
}
//

//
std::vector<FeatureId> AreaSearch::GetRadiusIndex(const Eigen::Vector2d& point,
                                                  double r)const {
  return grid_->GetNear(cv::KeyPoint(point.x(), point.y(), 2), r);
}

AreaSearch::~AreaSearch() {
}

std::map<int, std::unique_ptr<match::AreaSearch>>
AreaSearch::CreateAreaSearchFromeKeyFrameData(
    std::vector<Eigen::AlignedBox2i> image_bboxs, int grid_lenth,
    const KeyFrameData& data) {
  std::map<int, std::unique_ptr<match::AreaSearch>> area_searchs;

  auto sequence_feautes = data.data->features.trajectory_ids();
  for (const auto& sequence_id : sequence_feautes) {
    area_searchs[sequence_id] = std::make_unique<match::AreaSearch>(
        match::AreaSearchOption{image_bboxs[sequence_id],
                                image_bboxs[sequence_id].sizes() / grid_lenth},
        sequence_id, data);
  }
  return area_searchs;
}

}  // namespace match
}  // namespace mapping
}  // namespace jarvis