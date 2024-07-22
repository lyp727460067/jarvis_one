#ifndef JARVIS_PIC_BRIGE_H
#define JARVIS_PIC_BRIGE_H
#include <string>
#include <functional>
#include "jarvis/common/fixed_ratio_sampler.h"
#include "jarvis/sensor/data_process.h"
#include "jarvis/trajectory_builder.h"
#include "slip_detection/slip_detect.h"


namespace jarvis_pic {
class DataCapture;
class JarvisBrige {
  public:
  JarvisBrige(const std::string& config,DataCapture* data_capture,
              std::function<void(const jarvis::TrackingData&)> call_back);
   
  ~JarvisBrige();
  private:
  DataCapture* data_capture_;
  std::unique_ptr<jarvis::TrajectorBuilder> builder_;
  std::unique_ptr<jarvis::sensor::OrderedMultiQueue> order_queue_;
  std::unique_ptr<jarvis::common::FixedRatioSampler> image_sample_;
  std::mutex mutex_;
  std::string  class_name_ =  "JarvisBrige";
};

}
#endif