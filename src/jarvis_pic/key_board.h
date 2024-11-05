

#ifndef JARVIS__VIO_IO_KEY_BOARD_
#define JARVIS_VIO_IO_KEY_BOARD_
#include <math.h>
#include <signal.h>
#include <signal.h>  // signal functions
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <termios.h>

#include <functional>
#include <thread>

namespace jarvis_pic {
namespace io {

class KeyBoard {
 public:
  KeyBoard(std::function<void(const char&)> call_back=nullptr);
  ~KeyBoard();

  char GetKey();
  std::string GetMultyKey();

 private:
  std::thread thread_;
  bool kill_thread_ = false;
  int kfd = 0;
  std::function<void(const char)> call_back_;
  struct termios cooked, raw;
  struct pollfd ufd;
};
}  // namespace io
}  // namespace jarvis_pic
#endif