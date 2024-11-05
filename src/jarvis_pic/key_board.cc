
#include "key_board.h"

#include <string.h>

#include <chrono>
#include <iostream>

#include "unistd.h"

namespace jarvis_pic {
namespace io {

KeyBoard::KeyBoard(std::function<void(const char&)> call_back)
    : call_back_(std::move(call_back)) {
  /**
   * 从终端中获取按键
   * int tcgetattr(int fd, struct termios *termios_p);
   */
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));

  /**
   * c_lflag : 本地模式标志，控制终端编辑功能
   * ICANON: 使用标准输入模式
   * ECHO: 显示输入字符
   */
  raw.c_lflag &= ~(ICANON | ECHO);

  /**
   * c_cc[NCCS]：控制字符，用于保存终端驱动程序中的特殊字符，如输入结束符等
   * VEOL: 附加的End-of-file字符
   * VEOF: End-of-file字符
   * */
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  puts("Reading from keyboard");

  ufd.fd = kfd;
  ufd.events = POLLIN;

  //
  //
  // thread_ = std::thread([this]() {
  //   while (!kill_thread_) {
  //     char key = Loop();
  //     if (key != -1) {
  //       call_back_(key);
  //     }
  //     std::this_thread::sleep_for(std::chrono::milliseconds(20));
  //   }
  // });
}

std::string KeyBoard::GetMultyKey() {
  std::string result;
  while (true) {
    usleep(1000);
    char c = GetKey();
    if (c == -1) continue;
    if (int(c) == 255) continue;
    if (c == 10) break;
    result.push_back(c);
  }

  puts(result.c_str());
  return result;
}
//
char KeyBoard::GetKey() {
  /* get the next event from the keyboard */
  int num;
  char c = -1;
  /**
   * poll:把当前的文件指针挂到设备内部定义的等待队列中。
   * unsigned int (*poll)(struct file * fp, struct poll_table_struct * table)
   */
  if ((num = poll(&ufd, 1, 2)) < 0) {
    /**
     * perror( ) 用来将上一个函数发生错误的原因输出到标准设备(stderr)。
     * 参数s所指的字符串会先打印出,后面再加上错误原因字符串。
     * 此错误原因依照全局变量errno 的值来决定要输出的字符串。
     * */
    perror("poll():");
    return -1;
  } else if (num > 0) {
    if (read(kfd, &c, 1) < 0) {
      perror("read():");
      return -1;
    }
  }
  return c;
}

KeyBoard::~KeyBoard() {
  // kill_thread_ = true;
  // thread_.join();
  tcsetattr(kfd, TCSANOW, &cooked);
  puts("terminal resume");
}
}  // namespace io
}  // namespace jarvis_pic
