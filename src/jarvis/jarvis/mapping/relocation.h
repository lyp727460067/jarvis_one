#ifndef JARVIS_MAPPING_RELOCATION_H
#define  JARVIS_MAPPING_RELOCATION_H
//
#include "jarvis/mapping/loop_detect.h"
//
namespace jarvis {
namespace mapping {
//
//上电重定位的话，在全图上去做，比如合成一个大图
//如果跟踪重定位可以用直接发匹配去重定位
struct RelocationOption
{


};

//
class Relocation {
 public:
  Relocation(const RelocationOption& option);
  //
  //跟踪丢了后进行重定位
  std::unique_ptr<LoopDetctResult> DirectRelocation(
      const std::shared_ptr<LocalMap>& local_map,
      const KeyFrameData& track_data);
  //合成一个大的地图去重定位，比如重新上电了后
  std::unique_ptr<LoopDetctResult> FullRelocation();
  //
  std::unique_ptr<LoopDetctResult> LostRelocation();
 private:
  RelocationOption option;
};

}  // namespace mapping
}  // namespace jarvis
#endif