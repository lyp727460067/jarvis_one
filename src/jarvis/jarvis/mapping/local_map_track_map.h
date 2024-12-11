#ifndef _JARVIS_LAOCAL_MAP_TRACK_MAP_H
#define _JARVIS_LAOCAL_MAP_TRACK_MAP_H
#include "jarvis/mapping/map_manger.h"
#include "jarvis/mapping/match/direct_match.h"
#include "jarvis/transform/rigid_transform.h"
#include "jarvis/transform/transform.h"
//
#include "jarvis/mapping/match/occupancy_grid_2d.h"
namespace jarvis {
namespace mapping {

struct LocalMapTrackMapOption {
  int kf_num = 100;
  
};

class LocalMapTrackMap {
 public:

};

}  // namespace mapping
}  // namespace jarvis
#endif