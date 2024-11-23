/**
 * @file handle_sys.h
 * @author hao wu (clayderman@yardbot.net)
 * @brief 
 * @version 0.1
 * @date 2024-07-26
 * 
 * @copyright Copyright 2023 HJ Technology Co.Ltd. All rights reserved
 * 
 */
#ifndef SRC_COLLECT_NODE_UTILS_FUNC_INCLUDE_HANDLE_ACTION_H_
#define SRC_COLLECT_NODE_UTILS_FUNC_INCLUDE_HANDLE_ACTION_H_

#include <vector>
#include <string>
#include "function_factory.h"
#include "node_factory.h"
#include "hj_interface/SysAction.h"
#include "hj_interface/CollectBroadcast.h"

namespace collect_handle_action {

enum function_type {
  kIotFunc = 0,
  kLogFunc,
  kPostTunnelFunc,
  kMcuFunc,
  kResetFunNum
};

enum shutdown_function_type {
  kShoutdownIotFunc = 0,
  kShoutdownPostTunnelFunc,
  kShoutdownFunNum
};

class HandleAction {
 public:
  static HandleAction& GetInstance();
  ~HandleAction() = default;
  void SysActionCallback(const hj_interface::SysAction &msg);
  void FuncResponeCallback(const hj_interface::CollectBroadcast &msg);
  void ResetLogHandler(const hj_interface::CollectBroadcast &msg);
 private:
  HandleAction();
  HandleAction(const HandleAction&) = delete;
  HandleAction(HandleAction&&) = delete;

  std::vector<uint8_t> function_vec_;
  std::vector<uint8_t> shutdown_function_vec_;
  hj_bf::HJSubscriber sub_sys_action_;
  hj_bf::HJPublisher sys_action_response_;
  hj_bf::HJSubscriber sub_func_reponse_;
  hj_bf::HJPublisher pub_collect_action_;
  // log handler
  hj_bf::HJSubscriber sub_collect_action_;
  hj_bf::HJPublisher pub_func_response_;
};
}  // namespace collect_handle_action

#endif  // SRC_COLLECT_NODE_UTILS_FUNC_INCLUDE_HANDLE_ACTION_H_
