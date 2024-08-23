// @file demo_state.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2024-1-19)
#include "front_end_sml.h"
#include "demo_state.h"
#include "log.h"
namespace demo_state {
static int local_state = 0;
HJ_CALLBACK_INSTANCE(DemoStateEntry1) {
  local_state++;
  hj_sml::GetSmlInstance().visit_current_states([](auto state) {
    HJ_INFO("test state:%s",state.c_str());
  });
 // std::cout << "DemoStateEntry1: " << local_state << std::endl;
}
HJ_CALLBACK_INSTANCE(DemoStateEntry2) {
  local_state++;
//  std::cout << "DemoStateEntry2: " << local_state << std::endl;
}
void DemoStateExit1::operator()() {
  local_state++;
//  std::cout << "DemoStateExit1 " << local_state << std::endl;
};
void DemoStateExit2::operator()() {
  local_state++;
//  std::cout << "DemoStateExit2 " << local_state << std::endl;
};
}  // namespace demo_state
