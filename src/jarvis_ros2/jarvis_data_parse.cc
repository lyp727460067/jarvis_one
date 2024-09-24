
#include <fstream>
#include <iostream>
#include <memory>

#include "glog/logging.h"
// #define CHECK_DATA
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <optional>

#include "opencv2/opencv.hpp"
//
#pragma pack(1)
typedef struct {
  uint8_t type;         // 类型
  uint64_t count;       // 计数
  uint64_t time_stamp;  // 时间戳 [us]
  uint32_t len;         // 长度
} VioDataHead;
#pragma pack()

struct TimeStep {
  std::optional<uint64_t> last_time;
  std::optional<uint64_t> start_time;
  uint64_t image_count = 0;
};

int main(int argc, char* argv[]) {
  //
  const std::string image_file(argv[1]);
  auto it = image_file.find_last_of('/');
  std::string outdir = image_file.substr(0, it + 1);

  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  auto CreateDir = [](const std::string& dir) {
    if (access(dir.c_str(), F_OK) == -1) {
      mkdir(dir.c_str(), S_IRWXO | S_IRWXG | S_IRWXU);
    }
    if (access(dir.c_str(), F_OK) == -1) {
      mkdir(dir.c_str(), S_IRWXO | S_IRWXG | S_IRWXU);
    }
    LOG(INFO) << "mkdir " << dir;
    return true;
  };
  LOG(INFO) << "Parse file " << image_file;
  const std::string image_dir = outdir + "image/";
  const std::string image1_dir = outdir + "image1/";
  CreateDir(image_dir);
  std::ifstream image_stream(image_file,
                             std::ios_base::in | std::ios_base::binary);
  // image_stream.open(image_file, std::ios_base::in | std::ios_base::binary);

  CHECK(image_stream.is_open());

  std::map<int,TimeStep> time_step;
  while (!image_stream.eof()) {
    std::string head;
    // head.resize(7);
    for (int i = 0; i < 7; i++) {
      char c = image_stream.get();
      if (image_stream.eof()) {
        break;
      }
      head.push_back(c);
    }
    // image_stream.read((char*)head.data(),7);
    if (head != "VIODATA") {
      LOG(WARNING) << "data check false.";
      continue;
    }
    //
    char c = image_stream.get();
    if (image_stream.eof()) {
      break;
    }
    // c =  image_stream.get();
    // LOG(INFO)<<int(c)<<sizeof(VioDataHead);
    //
    // char head_data[sizeof(VioDataHead)];
    // for(int i  =0;i<sizeof(VioDataHead);i++){
    //     head_data[i] = image_stream.get();
    //     LOG(INFO)<<int(head_data[i]);
    // }
    // image_stream.seekg(7 , std::ios::cur);
    VioDataHead vio_head;
    char temp[sizeof(VioDataHead)];
    image_stream.read((char*)&temp, sizeof(VioDataHead));
    if (image_stream.eof()) {
      break;
    }
    memcpy((void*)&vio_head, (void*)&temp, sizeof(VioDataHead));
    auto &last_time  =  time_step[vio_head.type].last_time;
    auto &start_time  =  time_step[vio_head.type].start_time;
    auto &image_count = time_step[vio_head.type].image_count;
    LOG(INFO) << "t:" << int(vio_head.type) << ",c:" << (vio_head.count)
              << ",time:" << (vio_head.time_stamp) << ",l:" << (vio_head.len);

    if (!last_time.has_value()) {
      last_time = vio_head.time_stamp;
    }
    if (!start_time.has_value()) {
      start_time = vio_head.time_stamp;
    }
    if (abs(int(vio_head.time_stamp - last_time.value())) > 200 * 1000 * 1000) {
      LOG(WARNING) << "image data lost,last " << last_time.value() << ",cur "
                   << vio_head.time_stamp << ",detet "
                   << int(vio_head.time_stamp - last_time.value());
    }

    last_time = vio_head.time_stamp;
    // image_stream.seekg(sizeof(VioDataHead) , std::ios::cur);
    std::vector<uint8_t> image_data(vio_head.len);
    image_stream.read((char*)image_data.data(), vio_head.len);
    if (image_stream.eof()) {
      break;
    }
    // image_stream.seekg(vio_head.len , std::ios::cur);
    cv::Mat image = cv::imdecode(image_data, cv::IMREAD_GRAYSCALE);
    //

    if (image.empty()) {
      LOG(WARNING) << "Image parse error";
      continue;
    }
    const std::string image_file_name =
        image_dir + std::to_string(uint64_t(vio_head.time_stamp)) + "_" +
        std::to_string(vio_head.type) + ".jpg";    
    LOG_EVERY_N(INFO, 100) << "write " << image_file_name;

    cv::imwrite(image_file_name, image);
    image_count++;
    //
  }
  for (auto &index : time_step) {
    auto &last_time = index.second.last_time;
    auto &start_time = index.second.start_time;
    auto &image_count = index.second.image_count;
    CHECK(image_count) << "Parse lenth empty!!";
    LOG(INFO) << "Parse done,total image count " << image_count << ",start time"
              << start_time.value() << ",end time " << last_time.value()
              << ",total time prerid:"
              << (last_time.value() - start_time.value()) / 1000. / 1000. /
                     1000.;
  }

  return 0;
  // char c;
  // while((c = image_stream.get()) != EOF) {
  //   if(c=='V')
  //   if(c)
  // }
}