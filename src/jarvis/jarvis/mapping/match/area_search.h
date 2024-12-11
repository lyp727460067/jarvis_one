#ifndef _JARVIS_MAPPING_MATCH_AREASEARCH_
#define _JARVIS_MAPPING_MATCH_AREASEARCH_
#include <vector>

#include "Eigen/Eigen"
#include "jarvis/mapping/mapping_data.h"
#include "opencv2/opencv.hpp"
namespace jarvis {
namespace mapping {
namespace match {

//
struct AreaSearchOption {
  const Eigen::AlignedBox2i& image_box;
  Eigen::Vector2i area_grid_num{64, 40};
};
//

//
class AreaSearchGrid {
 public:
  AreaSearchGrid(
      const Eigen::AlignedBox2i& image_box,
      const Eigen::Vector2i& area_grid_num,
     const  Range<MapById<FeatureId, FeatureData>::ConstIterator>& target_points);
  //
  std::vector<FeatureId>GetNear(const cv::KeyPoint& point, double r);
 private:
  bool PosInGrid(const cv::KeyPoint& kp, int& posX, int& posY);
  Eigen::AlignedBox2f GetBound(float x, float y, double r);
  const Eigen::AlignedBox2i image_box_;
  const Eigen::Vector2i area_grid_num_;
  //
  struct GridValue {
    FeatureId id;
    cv::KeyPoint p;
  };
  std::vector<std::vector<std::vector<GridValue>>> grid_;
  //
  float grid_element_width_inv_ = 0.0;
  float frid_element_height_inv_ = 0.0;
};



class AreaSearch {
  public: 
  explicit AreaSearch(const AreaSearchOption& option,int s,
                      const KeyFrameData& target_points);
  //
  std::vector<FeatureId> GetRadiusIndex(const cv::KeyPoint& points,
                                        double r = 0.5)const ;
  std::vector<FeatureId> GetRadiusIndex(const Eigen::Vector2d& points,
                                  double r = 0.5)const ;
  Eigen::Vector2i GetGridNum() { return option_.area_grid_num; }
  ~AreaSearch();
  //
  static std::map<int, std::unique_ptr<match::AreaSearch>>
  CreateAreaSearchFromeKeyFrameData(
      std::vector<Eigen::AlignedBox2i> image_bboxs, int grid_lenth,
      const KeyFrameData& data);

 private:
  std::unique_ptr<AreaSearchGrid>grid_;
  AreaSearchOption option_;
  const  MapById<FeatureId, FeatureData>& target_points_;



};
}  // namespace match
}  // namespace mapping
}  // namespace jarvis
#endif