// @file state_machine.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2024-1-19)
#ifndef INCLUDE_STATE_MACHINE_H//your macro
#define INCLUDE_STATE_MACHINE_H
#include "function_factory.h"
#include "node_factory.h"

namespace middleware_node_ns {//your namespace

class StateMachine : public hj_bf::Function {
 public:
  explicit StateMachine(const rapidjson::Value &json_conf);
  ~StateMachine();

 private:
  // your variables
  // hj_bf::HJPublisher pub_;
  // hj_bf::HJClient client_;
  // hj_bf::HJSubscriber sub_;
  // hj_bf::HJServer service_;
  // hj_bf::HJTimer timer_;
  hj_bf::HJTimer timer_;
  hj_bf::HJSubscriber sub1_;
};
}  // namespace middleware_node_ns

#endif
