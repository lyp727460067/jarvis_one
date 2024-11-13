#include <applications/slam_controller.h>

#include <map>
#include <memory>
#include <set>
#include <strstream>

#include "glog/logging.h"
#include "iostream"

namespace jarvis_pic {
const char* GetExtendInfo();
//
template <typename... CallableArgs>
class Signal {
 public:
  using CallableFunction = std::function<void(CallableArgs...)>;
  using Subscriber = std::shared_ptr<CallableFunction>;

  void Notify(CallableArgs... args) {
    for (size_t i = 0; i < subscribers_.size();) {
      auto sub = subscribers_[i];
      //    if (auto sub = subscribers_[i].lock()) {
      (*sub)(args...);
      i++;
      //    } else {
      //      subscribers_.erase(subscribers_.begin() + i);
      //    }
    }
  }
  Subscriber Subscribe(CallableFunction func) {
    Subscriber sub = std::make_shared<CallableFunction>(std::move(func));
    subscribers_.emplace_back(sub);
    return sub;
  }

 private:
  //    std::vector<std::weak_ptr<CallableFunction>> subscribers_;
  std::vector<std::shared_ptr<CallableFunction>> subscribers_;
};
//

template <typename Entity>
class State {
 public:
  virtual void Enter(Entity* entity) = 0;
  virtual void Execute(Entity* entity) = 0;
  virtual void Exit(Entity* entity) = 0;
  virtual std::string Name() = 0;
  //
  virtual ~State() {}
};
//
class SlamController;
class StateMachine {
 public:
  StateMachine(SlamController* node);
  void Circle();
  void ChageState(const std::string& state_name);
  using StateFactoryFunction =
      std::function<std::unique_ptr<State<SlamController>>()>;
  //
  void Register(const std::string& name, StateFactoryFunction factory);
  //
  std::string GetCurStateName() { return current_state_->Name(); }

 private:
  std::unique_ptr<State<SlamController>> CreateState(
      const std::string& state_name);
  std::unique_ptr<State<SlamController>> current_state_ = nullptr;
  std::unique_ptr<State<SlamController>> next_state_ = nullptr;
  SlamController* node_;
  std::unordered_map<std::string, StateFactoryFunction> factories_;
};
//

}  // namespace jarvis_pic