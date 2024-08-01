
#ifndef _SOCKET_DEV_H
#define _SOCKET_DEV_H

#include <functional>
#include <memory>
#include <thread>
#include <mutex>
#include "dev_interface.h"

namespace jarvis_pic {
namespace internal {

class DevSocket : public DevInterface {
 public:
  using CallBack = std::function<void(std::vector<uint8_t>&&)>;
  DevSocket();
  DevSocket(const CallBack& callback);
  DevSocket(const std::pair<std::string, int> id, const CallBack& callback);

  virtual bool tx(const std::vector<uint8_t>& data) const override;
  virtual std::vector<uint8_t> rx() const override;
  ~DevSocket();

 private:
  class Socket;
 
  std::unique_ptr<Socket> SocketImpl;
};

}  // namespace internal
}  // namespace jarvis_pic

#endif
