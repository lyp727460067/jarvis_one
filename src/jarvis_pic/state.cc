#include <state.h>

#include <map>
#include <memory>
#include <set>
#include <strstream>

#include "glog/logging.h"
#include "iostream"

namespace jarvis_pic {

//
//

namespace {

//
constexpr char kIdleState[] = "IdleState";
constexpr char kFileCheckState[] = "FileCheckState";
constexpr char kSlamState[] = "SlamState";
constexpr char kSaveMapState[] = "SaveSlamState";
constexpr char kLocationState[] = "LocationState";
#define RESET "\033[0m"
#define BLACK "\033[30m"   /* Black */
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */
#define WHITE "\033[37m"   /* White */

constexpr char kBikini[] =
    "                                                                 \n"
    " *                                                               \n"
    " *                                                               \n"
    " *           _.._        ,------------.                          \n"
    " *        ,'      `.    ( Hi Bikini VIO)                         \n"
    " *       /  __) __` \\    `-,----------'                          \n"
    " *      (  (`-`(-')  ) _.-'                                      \n"
    " *      /)  \\  = /  (                                            \n"
    " *     /'    |--' .  \\                                           \n"
    " *    (  ,---|  `-.)__`                                          \n"
    " *     )(  `-.,--'   _`-.                                        \n"
    " *    '/,'          (    ',                                      \n"
    " *     (_       ,    `/,-' )                                     \n"
    " *     `.__,  : `-'/  /`--'                                      \n"
    " *       |     `--'  |                                           \n"
    " *       `   `-._   /                                            \n"
    " *        \\        (                                             \n"
    " *        /\\ .      \\.                                           \n"
    " *       / |` \\     ,-\\                                          \n"
    " *      /  \\| .)   /   \\                                         \n"
    " *     ( ,'|\\    ,'     :                                        \n"
    " *     |\\ ,`.`--'/      }                                        \n"
    " *     `,'    \\  |,'    /                                        \n"
    " *    / '-._   `-/      |                                        \n"
    " *    '-.   '-.,'|     ;                                         \n"
    " *   /        _/['---'''']                                       \n"
    " *  :        /  |'-     '                                        \n"
    " *  '           |      /                                         \n"
    " *              |      |                                         \n";
}  // namespace
//

template <typename Entity>
class InitState : public State<Entity> {
 public:
  virtual void Enter(Entity* entity) {}
  virtual void Execute(Entity* entity) {}
  virtual void Exit(Entity* entity) {}
  virtual std::string Name() { return "InitState"; }
  //
  virtual ~InitState() {}
};

template <typename Entity>
class IdleState : public State<Entity> {
 public:
  IdleState(Signal<const std::string&>::CallableFunction call_back) {
    change_state_sigal_.Subscribe(call_back);
  }
  virtual void Enter(Entity* entity) { LOG(INFO) << "Enter " << name_; }
  virtual void Execute(Entity* entity) {
    LOG_EVERY_N(INFO, 1000) << "Executing " << name_;
  }
  virtual void Exit(Entity* entity) { LOG(INFO) << "Exit " << name_; }
  //
  std::string Name() { return name_; }

 private:
  Signal<const std::string&> change_state_sigal_;

  std::string name_ = kIdleState;
};
//
template <typename Entity>
class FileCheckState : public State<Entity> {
 public:
  FileCheckState(const std::string& base_path,
                 Signal<const std::string&,
                        const std::string&>::CallableFunction call_back)
      : base_path_(base_path) {
    change_state_sigal_.Subscribe(call_back);
  }
  void Enter(Entity* entity) {
    //
    LOG(INFO) << "Enter " << name_;
    LOG(INFO) << "Read base path map paths.";
    auto temp = ReadFileFromDir(base_path_);
    sub_map_path_ = std::vector<std::string>(temp.begin(), temp.end());
  }
  void Execute(Entity* entity) {
    //
  }

  void Exit(Entity* entity) { LOG(INFO) << "Exit " << name_; }
  //
  std::string Name() { return name_; }

 private:
  std::string name_ = kFileCheckState;
  const std::string base_path_;
  std::vector<std::string> sub_map_path_;
  Signal<const std::string&, const std::string&> change_state_sigal_;
};
//
//
template <typename Entity>
class SlamState : public State<Entity> {
 public:
  virtual void Enter(Entity* entity) { LOG(INFO) << "Enter " << name_; }
  virtual void Execute(Entity* entity) {
    LOG_EVERY_N(INFO, 1000) << "Executing " << name_;
  }
  virtual void Exit(Entity* entity) { LOG(INFO) << "Exit " << name_; }
  //
  std::string Name() { return name_; }

 private:
  std::string name_ = kSlamState;
};
//
template <typename Entity>
class LocationState : public State<Entity> {
 public:
  virtual void Enter(Entity* entity) { LOG(INFO) << "Enter " << name_; }
  virtual void Execute(Entity* entity) {
    LOG_EVERY_N(INFO, 1000) << "Executing " << name_;
  }
  virtual void Exit(Entity* entity) { LOG(INFO) << "Exit " << name_; }
  //
  std::string Name() { return name_; }

 private:
  std::string name_ = kLocationState;
};
//
template <typename Entity>
class SaveMapState : public State<Entity> {
 public:
  SaveMapState(Signal<const std::string&>::CallableFunction call_back) {
  }

  virtual void Enter(Entity* entity) { LOG(INFO) << "Enter " << name_; }
  virtual void Execute(Entity* entity) {
  }
  virtual void Exit(Entity* entity) { LOG(INFO) << "Exit " << name_; }
  //
  std::string Name() { return name_; }

 private:
  Signal<const std::string&> change_state_sigal_;
  std::string name_ = kSaveMapState;
};
//
StateMachine::StateMachine(SlamController* node)
    : node_(node),
      current_state_(std::make_unique<InitState<SlamController>>()) {}
//
void StateMachine::Circle() {
  if (next_state_) {
    current_state_->Exit(node_);
    current_state_ = std::move(next_state_);
    next_state_ = nullptr;
    current_state_->Enter(node_);
  }
  current_state_->Execute(node_);
}
//
void StateMachine::ChageState(const std::string& state_name) {
  LOG(INFO) << "Set Next State " << state_name;
  CHECK(factories_.count(state_name))
      << "A state named '" << state_name << "' Not registered.";
  next_state_ = factories_[state_name]();
}
//

//
void StateMachine::Register(const std::string& name,
                            StateFactoryFunction factory) {
  CHECK(factories_.count(name) == 0)
      << "A state named '" << name << "' has already been registered.";
  factories_[name] = std::move(factory);
}
//


//
//
SlamController::~SlamController() {
  kill_thread_ = true;
  thread_.join();
}
const char* GetExtendInfo() { return kBikini; }
//

}  // namespace jarvis_pic