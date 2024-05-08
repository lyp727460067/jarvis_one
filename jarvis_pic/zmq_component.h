#ifndef _JARVIS_PIC_ZMQ_COMPONET_H
#define _JARVIS_PIC_ZMQ_COMPONET_H
#include "jarvis/key_frame_data.h"
//
#include <zmq.h>
#include "data_protocol.h"
#include <memory>

#include "shm_mod.h"
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
  // 有内存安全
  std::vector<std::unique_ptr<DevInterface>> device_;
};
class MpcComponent {
 public:
  MpcComponent() ;
  MpcComponent(const MpcComponent&) = delete;
  void Write(const jarvis::TrackingData& data);

 private:
  std::unique_ptr<ShmMod> shm_mod_;
  constexpr static int16_t vio_id_ = MOD_ID_LOCAL_POSE_FB;
};

};  // namespace jarvis_pic

#endif