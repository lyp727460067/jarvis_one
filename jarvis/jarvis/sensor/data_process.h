
#ifndef __METABOUNDS_VIO_DATA_PROCESS_H
#define __METABOUNDS_VIO_DATA_PROCESS_H
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <optional>
#include "Eigen/Core"
#include "glog/logging.h"
#include "opencv2/opencv.hpp"
#include "jarvis/sensor/image_data.h"
#include "jarvis/sensor/imu_data.h"
#include "jarvis/sensor/odometry_data.h"
namespace jarvis {
namespace sensor {


using ImageFuction = std::function<void(const ImageData &)>;
using ImuFuction = std::function<void(const ImuData &)>;
using OdomFuction = std::function<void(const OdometryData &)>;

class Data {
 public:
  virtual common::Time GetTime() const = 0;
  virtual ~Data() {}


 private:
};
template <typename DataType>
class DispathcData : public Data {
 public:
  DispathcData(const DataType &data) : data_(data) {}
  const DataType &Data() const { return data_; }
  common::Time  GetTime() const { return data_.time; }
  ~DispathcData() {}
 private:
  const DataType data_;
};
using Callback = std::function<void(std::unique_ptr<Data>)>;
struct Queue {
  std::queue<std::unique_ptr<Data>> queue;
  Callback callback;
};

class OrderedMultiQueue {
 public:
  void AddData(const std::string &name, std::unique_ptr<Data> data);
  void AddQueue(std::string name, ImageFuction call_back);
  void AddQueue(std::string name, ImuFuction call_back);
  void AddQueue(std::string name, OdomFuction call_back);
  ~OrderedMultiQueue();
  void Start();

  void Stop() ;
 protected:
  bool kill_thread = false;
  std::mutex  mutex_;
  std::thread  dispath_thead_;
  virtual void Dispathch();
  common::Time GetStartCommontime();
  std::optional<common::Time>common_start_time_ ;
  common::Time last_dispatched_time_ ;
  std::unordered_map<std::string, Queue> queues_;
  bool circle_done = false;
};
}  // namespace sensor
}  // namespace jarvis
#endif