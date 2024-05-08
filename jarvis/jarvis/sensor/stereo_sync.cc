#include "sensor/stereo_sync.h"

#include "sensor/data_process.h"
namespace jarvis {

namespace sensor {
//
void StereoSync::InitCacheImage(const std::string &topic, bool is_base) {
  cache_images_[topic];
  if (is_base) {
    CHECK(main_base_topic_.empty()) << "Main base topic already seted.";
    main_base_topic_ = topic;
  }
}
//
void StereoSync::ProcessImage(LocalImageData img, std::string topic) {
   cache_images_[topic].emplace_back(img.time, img);
    if (topic == main_base_topic_) {
      auto &main_image_list = cache_images_[main_base_topic_];
      const double &main_image_time = main_image_list.front().first;
      bool main_image_list_earse = false;
      sensor::ImageData image_datas;
      image_datas.time = main_image_time;
      //
      image_datas.image.push_back(main_image_list.front().second.image);
      //
      for (auto &image_list : cache_images_) {
        if (image_list.first != main_base_topic_) {
          auto it = std::find_if(
              image_list.second.begin(), image_list.second.end(),
              [=](const std::pair<double, LocalImageData> &time) {
                return fabs(time.first - main_image_time) <
                       cam0_cam1_time_offset_threash_hold_;
              });
          if (it != image_list.second.end()) {
            main_image_list_earse = true;
            // std::cout << main_base_topic_ << "time stamp: " << std::fixed
            //           << std::setprecision(15) << main_image_time << ", "
            //           << it->first << std::endl;
            image_datas.image.push_back(it->second.image);
            image_list.second.erase(image_list.second.begin(), it + 1);
          }
        }
      }
      if (main_image_list_earse) {
        main_image_list.erase(main_image_list.begin());
        // if (image_sample_->Pulse()) {
          if (call_back_) {
            call_back_(image_datas);
          }
          // order_queue_->AddData(
          //     topic,
          //     std::make_unique<sensor::DispathcData<sensor::ImageData>>(
          //         image_datas));
        // }
      }
      if (main_image_list.size() > 2) {
        main_image_list.erase(main_image_list.begin());
        LOG(WARNING) << "Cache Main Data too big erase befor data.";
      }

    } else {
      for (auto &image_list : cache_images_) {
        if (image_list.first != main_base_topic_) {
          LOG_IF(WARNING, image_list.second.size() > 10)
              << "Cache Data too big erase befor data.";
        }
      }
    }
}

StereoSync::~StereoSync() {}
}  // namespace sensor
}  // namespace jarvis