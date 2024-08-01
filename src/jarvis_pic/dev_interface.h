#ifndef _JRVIS_PIC_DEV_INTERFACE_H
#define _JRVIS_PIC_DEV_INTERFACE_H_
#include <vector>

namespace jarvis_pic {
class DevInterface {
 public:
  virtual bool tx(const std::vector<uint8_t> &data) const = 0;
  virtual std::vector<uint8_t> rx() const = 0;
  virtual ~DevInterface(){};
};

}  // namespace jarvis_pic

#endif