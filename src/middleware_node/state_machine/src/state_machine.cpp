// @file state_machine.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2024-1-19)
#include "state_machine.h"

#include "front_end_sml.h"
#include "std_msgs/String.h"
#include "thread"
HJ_REGISTER_FUNCTION(factory) {
  std::cerr << "minos register factory" << FUNCTION_NAME << std::endl;
  factory.registerCreater<middleware_node_ns::StateMachine>(FUNCTION_NAME);
}
namespace middleware_node_ns {

void stateMachineTimer(const hj_bf::HJTimerEvent &) {
  static int i = 0;
  if (i == 0) {
    HJ_TRIGGER_EVENT(e1);
  } else {
    HJ_TRIGGER_EVENT(e2);
  }
  hj_sml::GetSmlInstance().visit_current_states(
      [](auto state) { std::cout << "current state:" << state.c_str() << std::endl; });
  i++;
  /* */
}

void stateMachineTimer2(const hj_bf::HJTimerEvent &) {
  static int i = 0;
  if (i % 2 == 0) {
    HJ_TRIGGER_EVENT(ota);
  } else {
//    HJ_TRIGGER_EVENT(ota_out);
  }
  i++;
  /* */
}

StateMachine::~StateMachine() {
  std::cerr << "minos StateMachine" << std::endl;
  //  timer_.stop();
  // HJ_TRIGGER_EVENT(e3);
}
int loopFunc1() {
  static int num;
  std::cerr << "minos loopFunc1:" << num << std::endl;
  num++;
  while (1) {
    HJ_TRIGGER_EVENT(e2);
  }
}
void otaCallback(const std_msgs::String::ConstPtr &msg) {
  std::cerr << "minos get msg=" <<msg->data <<std::endl;
  if(msg->data == "ota"){
    std::cout << "minos log=" <<HJ_TRIGGER_EVENT(ota) <<std::endl;
//    HJ_TRIGGER_EVENT(ota);
  }else if(msg->data == "ota_out"){
    std::cout << "minos log2222=" <<HJ_TRIGGER_EVENT(ota_out) <<std::endl;
    std::cout << "!!!!!!!!!!"<<std::endl;
//    HJ_TRIGGER_EVENT(ota_out);
  }else {
    std::cout << "minos otaCallback" << std::endl;
  }
}
StateMachine::StateMachine(const rapidjson::Value &json_conf) : hj_bf::Function(json_conf) {
  // read your config
  //  hj_bf::HJTimer timer_;
  // your code
  sub1_ = hj_bf::HJSubscribe("ota_cmd", 10, otaCallback);
//  timer_ = hj_bf::HJCreateTimer("state_machine_timer", 5 * 1000 * 1000, stateMachineTimer);
  std::cerr << "minos just a StateMachine" << std::endl;
  HJ_TRIGGER_EVENT(e1);
  HJ_TRIGGER_EVENT(e2);
  /*
  for (int i = 0; i < 10; i++) {
    auto state1 = std::thread(loopFunc1);  // 开线程
    state1.detach();
  }
  */
//  timer_ = hj_bf::HJCreateTimer("state_machine_timer", 1 * 1000 * 1000, stateMachineTimer2);
  /*
  auto state1 = std::thread(loopFunc1);  // 开线程
  auto state2 = std::thread(loopFunc1);  // 开线程
  state1.detach();
  state2.detach();
  */
}
}  // namespace middleware_node_ns
