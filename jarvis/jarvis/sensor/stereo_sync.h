
#ifndef __METABOUNDS_VIO_STEREO_SYNC_H
#define __METABOUNDS_VIO_STEREO_SYNC_H
#include <memory>
#include <string>

#include "opencv2/opencv.hpp"
#include "jarvis/sensor/data_process.h"
#include <functional>
namespace jarvis {

namespace sensor {

class StereoSync {
 public:
  struct LocalImageData {
    double time;
    std::string topic;
    std::shared_ptr<cv::Mat> image;
  };
  StereoSync(const double &cam0_cam1_time_offset_thresh_hold, double sample,
             std::function<void(const sensor::ImageData &)> call_back)
      : cam0_cam1_time_offset_threash_hold_(cam0_cam1_time_offset_thresh_hold),
        call_back_(std::move(call_back)) {}

  void ProcessImage(LocalImageData img, std::string topic);
  void InitCacheImage(const std::string &topic, bool is_base);
  ~StereoSync();

 private:
  double cam0_cam1_time_offset_threash_hold_ = 0;
  std::map<std::string, std::vector<std::pair<double, LocalImageData>>>
      cache_images_;
  std::string main_base_topic_;
  std::vector<std::string> images_topics_;
  OrderedMultiQueue *order_queue_ =nullptr;
  // std::unique_ptr<common::FixedRatioSampler> image_sample_;
  std::function<void(const sensor::ImageData&)>
      call_back_=nullptr;
};

}  // namespace data_sync
}  // namespace jarvis
#endif