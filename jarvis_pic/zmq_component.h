#ifndef _JARVIS_PIC_ZMQ_COMPONET_H
#define _JARVIS_PIC_ZMQ_COMPONET_H
#include "jarvis/key_frame_data.h"
//
#include <zmq.h>
#include <memory>
//
namespace jarvis_pic {
class DevInterface;
class ZmqComponent {
 public:
  ZmqComponent();
  // for debug
  void PubLocalData(const jarvis::TrackingData& data);
 ~ZmqComponent();
 private:
 //有内存安全
  std::vector<std::unique_ptr<DevInterface>> device_;
};

void WriteMpc(const jarvis::TrackingData& data);

};  // namespace jarvis_pic

#endif