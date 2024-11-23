#include "handle_action.h"
#include "logic_dev/communication.h"
#include "boost/filesystem.hpp"
#include "log.h"
#include <algorithm>
#include <unistd.h>

namespace collect_handle_action {

constexpr char kLogPath[] = "/userdata/hj/log/";
constexpr char kMapFiles[] = "/userdata/hj/maps";

void HandleAction::SysActionCallback(const hj_interface::SysAction &msg) {
  HJ_INFO("collect_node receive action:%d", msg.action);
  if (msg.action == hj_interface::SysAction::SYS_ACTION_SHUTDOWN ||
      msg.action == hj_interface::SysAction::SYS_ACTION_SLEEP) {
    shutdown_function_vec_.assign(kShoutdownFunNum, 0);
    hj_interface::CollectBroadcast broadcast;
    broadcast.action = msg.action;
    pub_collect_action_.publish(broadcast);
    sync();
  } else if (msg.from == hj_interface::SysAction::COMM_NODE_MIDDLEWARE &&
      (msg.action == hj_interface::SysAction::SYS_ACTION_RESTORE_FACTORY ||
       msg.action == hj_interface::SysAction::SYS_ACTION_RESET)) {
    function_vec_.assign(kResetFunNum, 0);
    hj_interface::CollectBroadcast broadcast;
    broadcast.action = msg.action;
    pub_collect_action_.publish(broadcast);
  } else {
    hj_interface::SysAction response_action;
    response_action.from = hj_interface::SysAction::COMM_NODE_COLLECT;
    response_action.action = msg.action;
    response_action.res = 0;
    sys_action_response_.publish(response_action);
  }
}

void HandleAction::FuncResponeCallback(const hj_interface::CollectBroadcast &msg) {
  if (msg.action == hj_interface::SysAction::SYS_ACTION_RESTORE_FACTORY ||
      msg.action == hj_interface::SysAction::SYS_ACTION_RESET) {
    if (msg.function < kResetFunNum) {
      function_vec_[msg.function] = 1;
    }
    // function 都处理完成，回复middleware
    bool restDone = std::all_of(function_vec_.begin(), function_vec_.end(), 
          [](uint8_t val){return val == 1;});
    if (restDone) {
      hj_interface::SysAction response_action;
      response_action.from = hj_interface::SysAction::COMM_NODE_COLLECT;
      response_action.action = msg.action;
      response_action.res = 0;
      sys_action_response_.publish(response_action);
    }
  } else if (msg.action == hj_interface::SysAction::SYS_ACTION_SHUTDOWN ||
      msg.action == hj_interface::SysAction::SYS_ACTION_SLEEP) {
    HJ_INFO("shutdown or sleep func:%d", msg.function);
    if (msg.function < kShoutdownFunNum) {
      shutdown_function_vec_[msg.function] = 1;
    }
    // function 都处理完成，回复middleware
    bool restDone = std::all_of(shutdown_function_vec_.begin(), shutdown_function_vec_.end(), 
          [](uint8_t val){return val == 1;});
    if (restDone) {
      HJ_INFO("shutdown or sleep done");
      hj_interface::SysAction response_action;
      response_action.from = hj_interface::SysAction::COMM_NODE_COLLECT;
      response_action.action = msg.action;
      response_action.res = 0;
      sys_action_response_.publish(response_action);
    }
  }
}

void HandleAction::ResetLogHandler(const hj_interface::CollectBroadcast &msg) {
  if (msg.action == hj_interface::SysAction::SYS_ACTION_RESTORE_FACTORY ||
      msg.action == hj_interface::SysAction::SYS_ACTION_RESET) {
    boost::filesystem::path path = boost::filesystem::path(kLogPath);
    boost::filesystem::remove_all(path);
    path = boost::filesystem::path(kMapFiles);
    boost::filesystem::remove_all(path);

    hj_interface::CollectBroadcast response_msg;
    response_msg.action = msg.action;
    response_msg.ack = 0;
    response_msg.function = function_type::kLogFunc;
    pub_func_response_.publish(response_msg);
  }
}

HandleAction::HandleAction() {
  function_vec_.assign(kResetFunNum, 0);
  shutdown_function_vec_.assign(kShoutdownFunNum, 0);
  std::thread async_init_thread([&]() {
    sub_sys_action_ = hj_bf::HJSubscribe(kSysActionReq, 10, &HandleAction::SysActionCallback, this);
    sys_action_response_ = hj_bf::HJAdvertise<hj_interface::SysAction>(kSysActionResp, 10);

    sub_collect_action_ = hj_bf::HJSubscribe("collect_node/notify/func", 10, &HandleAction::ResetLogHandler, this);
    pub_func_response_ = hj_bf::HJAdvertise<hj_interface::CollectBroadcast>("func/response/collect_node", 10);

    sub_func_reponse_ = hj_bf::HJSubscribe("func/response/collect_node", 10, &HandleAction::FuncResponeCallback, this);
    pub_collect_action_ = hj_bf::HJAdvertise<hj_interface::CollectBroadcast>("collect_node/notify/func", 10);
  });
  async_init_thread.detach();
}

HandleAction& HandleAction::GetInstance() {
  static HandleAction instance;
  return instance;
}
}  // namespace collect_handle_action
