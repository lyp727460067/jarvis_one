#ifndef __JARVIS_MAPPING_MAP_POINT_CONSTRUCT_MAPMANAGER_H__
#define __JARVIS_MAPPING_MAP_POINT_CONSTRUCT_MAPMANAGER_H__


#include <map>
#include <memory>
#include <set>

#include "Eigen/Core"
#include "Eigen/Geometry"
//
#include "jarvis/camera_models/camera_models/camera.h"
//
#include "jarvis/key_frame_data.h"
#include "jarvis/mapping/key_frame_database.h"
#include "jarvis/mapping/key_point_exract.h"
#include "jarvis/mapping/mapping_data.h"
#include "jarvis/transform/transform.h"
#include "jarvis/common/id.h"
#include "jarvis/mapping/covisibility.h"
#include "jarvis/mapping/match/des_matcher.h"
namespace jarvis {
namespace mapping {
//
//维护一定规模大小的图，然后重建出当前的一部分的地图点
//由mapmanager维护
struct  MapPointConstructOption
{


};
class MapPointConstruct {
  //
 public:
  MapPointConstruct(const MapPointConstructOption& option);
  //
  void Construct(const KeyFrameId& kf_id, KeyFrameData* data);
  //
 private:
  //
  MapById<KeyFrameId, const KeyFrameData> key_frames_datas_;
  std::map<int, camera_models::CameraPtr> cameras_;
  std::unique_ptr<mapping::Covisibility> covisibility_;
  MapById<MapPointId, MapPointData> map_points_;
};

}  // namespace mapping
}  // namespace jarvis
#endif
