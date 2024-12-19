#include "jarvis/mapping/map_point_construct.h"

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

MapPointConstruct::MapPointConstruct(const MapPointConstructOption& option){}
//
void MapPointConstruct::Construct(const KeyFrameId& kf_id, KeyFrameData* data) {

}

}  // namespace mapping
}  // namespace jarvis
