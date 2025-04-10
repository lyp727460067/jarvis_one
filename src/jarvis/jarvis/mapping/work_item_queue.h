#ifndef _WORK_ITEM_QUEUE_H
#define _WORK_ITEM_QUEUE_H
#include <memory>
#include <string>
//
namespace jarvis {
namespace mapping {
namespace common {
class ThreadPool;
}
struct WorkItem {
  enum class Result { Normal, kInterruptForImmediateRun };
  std::chrono::steady_clock::time_point time;
  std::function<Result()> task;
};
//
class WorkItemQueue {
 public:
  using WorkQueue = std::deque<WorkItem>;
  //
  WorkItemQueue(const std::string& work_name, common::ThreadPool* thread_pool,
                std::function<void()> interrupt_execution_call_back)
      : work_item_name_(work_name),
        thread_pool_(thread_pool),
        interrupt_execution_call_back_(interrupt_execution_call_back) {}
  void AddWorkItem(const std::function<WorkItem::Result()>& work_item);
  void DrainWorkQueue();
  ~WorkItemQueue();

 private:
  std::string work_item_name_;
  common::ThreadPool* thread_pool_;
  std::function<void()> interrupt_execution_call_back_;
  std::unique_ptr<WorkQueue> work_queue_;
  std::mutex work_queue_mutex_;
};

}  // namespace mapping
}  // namespace jarvis
#endif