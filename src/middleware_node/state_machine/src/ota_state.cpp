// @file demo_state.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2024-1-19)
#include "ota_state.h"
namespace ota_state {
static int local_state = 0;
HJ_CALLBACK_INSTANCE(OtaStateEntry1) {
  local_state++;
//  std::cout << "OtaStateEntry1: " << local_state << std::endl;
}
HJ_CALLBACK_INSTANCE(OtaStateEntry2) {
  local_state++;
//  std::cout << "OtaStateEntry2: " << local_state << std::endl;
}
void OtaStateExit1::operator()() {
  local_state++;
//  std::cout << "OtaStateExit1 " << local_state << std::endl;
};
void OtaStateExit2::operator()() {
  local_state++;
//  std::cout << "OtaStateExit2 " << local_state << std::endl;
};
}  // namespace demo_state
