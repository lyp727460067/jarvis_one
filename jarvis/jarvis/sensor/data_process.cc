#include "jarvis/sensor/data_process.h"
// /
namespace jarvis {
namespace sensor {
//
void OrderedMultiQueue::AddQueue(std::string name, ImageFuction call_back) {
  queues_.emplace(
      name, Queue{{}, [=](std::unique_ptr<Data> data) {
                    DispathcData<ImageData> *image_data =
                        dynamic_cast<DispathcData<ImageData> *>(data.get());
                    CHECK(image_data) << "Invalid  dynamic_cast to 'ImageData'";
                    call_back(image_data->Data());
                  }});
}
//
void OrderedMultiQueue::AddQueue(std::string name, ImuFuction call_back) {
  queues_.emplace(
      name, Queue{{}, [=](std::unique_ptr<Data> data) {
                    DispathcData<ImuData> *imu_data =
                        dynamic_cast<DispathcData<ImuData> *>(data.get());
                    CHECK(imu_data) << "Invalid  dynamic_cast to 'ImuData' ";
                    call_back(imu_data->Data());
                  }});
}
void OrderedMultiQueue::AddQueue(std::string name, OdomFuction call_back) {
  queues_.emplace(
      name, Queue{{}, [=](std::unique_ptr<Data> data) {
                    DispathcData<OdometryData> *imu_data =
                        dynamic_cast<DispathcData<OdometryData> *>(data.get());
                    CHECK(imu_data) << "Invalid  dynamic_cast to 'OdomFuction' ";
                    call_back(imu_data->Data());
                  }});
}
//
void OrderedMultiQueue::AddData(const std::string &name,
                                std::unique_ptr<Data> data) {
  CHECK(queues_.count(name))<<name;
  {
  std::lock_guard<std::mutex> lock(mutex_);
  queues_[name].queue.push(std::move(data));
  }
  Dispathch();
}
void OrderedMultiQueue::Start() {
  // dispath_thead_ = std::thread([this]() {
  //   while (!kill_thread) {
  //     Dispathch();
  //     std::this_thread::sleep_for(std::chrono::milliseconds(5));
  //   }
  // });
}
void OrderedMultiQueue::Stop() {
  kill_thread = true;
  dispath_thead_.join();
}

//
void OrderedMultiQueue::Dispathch() {
  // while (true) {
    for (const auto &queue : queues_) {
    const Data *next_data = nullptr;
    Queue *next_queue = nullptr;
    std::string next_queue_key;
    for (auto it = queues_.begin(); it != queues_.end();) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (it->second.queue.empty()) {
        for (auto &entry : queues_) {
          if (entry.second.queue.size() > 200) {
            LOG_EVERY_N(WARNING, 60) << "Queue waiting for data: " << it->first;
          }
        }
        return;
      }
      const auto *data = it->second.queue.front().get();
      if (next_queue == nullptr || data->GetTime() < next_data->GetTime()) {
        next_data = data;
        next_queue = &it->second;
        next_queue_key = it->first;
      }
      if (last_dispatched_time_ > next_data->GetTime()) {
        LOG(WARNING) << "Unorder time : '" << it->first << "'";
        next_queue->queue.pop();
        ++it;
        return;
      }
      ++it;
    }
    const common::Time common_start_time = GetStartCommontime();
    int next_queue_size = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      next_queue_size = next_queue->queue.size();
    }
    if (next_data->GetTime() >= common_start_time) {
      last_dispatched_time_ = next_data->GetTime();
      std::unique_ptr<Data> data = nullptr;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        data = std::move(next_queue->queue.front());
        next_queue->queue.pop();
      }
      next_queue->callback(std::move(data));
    } else if (next_queue_size < 2) {
      // CHECK(!next_queue->queue.empty());
      last_dispatched_time_ = next_data->GetTime();
      // next_queue->callback(std::move(next_queue->queue.front()));
      {
        std::lock_guard<std::mutex> lock(mutex_);
        next_queue->queue.pop();
      }
      LOG(INFO) << "Cache 2 early "
                << "'" << next_queue_key << "' "
                << "data to sysytem at "
                << last_dispatched_time_;
    } else {
      std::unique_ptr<Data> next_data_owner = nullptr;
      common::Time next_queue_queue_front_time ;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        next_data_owner = std::move(next_queue->queue.front());
        next_queue->queue.pop();
        next_queue_queue_front_time = next_queue->queue.front()->GetTime();
      }
      // LOG(INFO)<<std::to_string(last_dispatched_time_ ) ;
      // LOG(INFO)<<std::to_string(common_start_time ) ;
      if (next_queue_queue_front_time > common_start_time) {
        last_dispatched_time_ = next_data_owner->GetTime();
        next_queue->callback(std::move(next_data_owner));
      }
      LOG(INFO) << "Drop early " << next_queue_key << " data...";
    }
  }
}

common::Time OrderedMultiQueue::GetStartCommontime() {
  if (common_start_time_.has_value()) return common_start_time_.value();
  common_start_time_ = common::FromUniversal(0);
  for (auto &entry : queues_) {
    common_start_time_  = std::max(common_start_time_.value(),
                                      entry.second.queue.front()->GetTime());
  }
  LOG(INFO) << "All sensor start at time: " << common_start_time_.value();

  return common_start_time_.value();
};

OrderedMultiQueue::~OrderedMultiQueue() {
  std::lock_guard<std::mutex> lock(mutex_);
  Stop();
}
}  // namespace sensor
}  // namespace jarvis