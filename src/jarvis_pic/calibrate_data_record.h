#ifndef _JARVIS_PIC_C_DATA_RECORD_H
#define _JARVIS_PIC_C_DATA_RECORD_H
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "jarvis/transform/rigid_transform.h"
#include "data_capture.h"
#include "jarvis/common/time.h"
#include "jarvis/common/fixed_ratio_sampler.h"
//
//
//
namespace jarvis_pic {
  class CalibrateDataRecord {
   public:
    explicit CalibrateDataRecord(double sample_ration,const std::string& data_path);
    void AddFrame(const Frame& frame);
    void AddImu(const ImuData& imu);
   private:
    void Run();
    void CreateDataDir();
    std::unique_ptr<jarvis::common::FixedRatioSampler>  image_sample_; 
    std::string image_data_dir_;
    std::string data_path_;
    std::ofstream imu_file_;
  };
}  // namespace jarvis_pic

#endif