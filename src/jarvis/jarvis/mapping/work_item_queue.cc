#include "jarvis/mapping/work_item_queue.h"

#include "jarvis/common/task.h"

#include <chrono>
#include <thread>

#include "glog/logging.h"
#include "jarvis/common/task.h"
#include "jarvis/common/thread_pool.h"
namespace jarvis {
namespace mapping {

void WorkItemQueue::AddWorkItem(
    const std::function<WorkItem::Result()> &work_item) {
  std::lock_guard<std::mutex> lock(work_queue_mutex_);
  if (work_queue_ == nullptr) {
    work_queue_ = std::make_unique<WorkQueue>();
    auto task = std::make_unique<common::Task>();
    task->SetWorkItem([this]() { DrainWorkQueue(); });
    thread_pool_->Schedule(std::move(task));
  }
  const auto now = std::chrono::steady_clock::now();
  work_queue_->push_back({now, work_item});
}
void WorkItemQueue::DrainWorkQueue() {
  WorkItem::Result process_work_queue = WorkItem::Result::Normal;
  //   LOG(INFO)<<"DrainWorkQueue";
  size_t work_queue_size;
  while (process_work_queue == WorkItem::Result::Normal) {
    std::function<WorkItem::Result()> work_item;
    {
      std::lock_guard<std::mutex> locker(work_queue_mutex_);
      if (work_queue_->empty()) {
        work_queue_.reset();
        return;
      }
      work_item = work_queue_->front().task;
      work_queue_->pop_front();
      work_queue_size = work_queue_->size();
      LOG_EVERY_N(INFO, 10)
          << work_item_name_ << "queue size:" << work_queue_size;
    }
    process_work_queue = work_item();
  }
  if (process_work_queue == WorkItem::Result::kInterruptForImmediateRun) {
    if (interrupt_execution_call_back_) {
      interrupt_execution_call_back_();
    }
  }
  DrainWorkQueue();
}
//
WorkItemQueue::~WorkItemQueue() {
  if (work_queue_ == nullptr) return;
  size_t work_queue_size = 0;
  {
    std::function<WorkItem::Result()> work_item;
    work_queue_size = work_queue_->size();
  }
  //
  LOG(INFO) << work_item_name_ << " work_queue size :" << work_queue_size
            << ", Start Run remain task";
  //
  while (work_queue_size) {
    {
      std::lock_guard<std::mutex> locker(work_queue_mutex_);
      work_queue_size = work_queue_->size();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    LOG(INFO) << "Wait" << work_item_name_ << ",size: " << work_queue_size;
  }
}
}  // namespace mapping
}  // namespace jarvis