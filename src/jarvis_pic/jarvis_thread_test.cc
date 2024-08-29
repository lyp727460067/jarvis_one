#include <iostream>
#include <string>
#include <thread>
#include <vector>

int main(int argc, char* argv[]) {
  //
  int thread_num = stoi(std::string(argv[1]));
  std::cout << "thread_num " << thread_num << std::endl;
  std::vector<std::thread> threads;
  for (int i = 0; i < thread_num; i++) {
    threads.emplace_back([]() {
      while (1) {
      }
    });
  }
  for(int i =0;i<thread_num;i++){
    threads[i].join();
  }

  return 0;
}
