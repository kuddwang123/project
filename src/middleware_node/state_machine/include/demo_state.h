// @file demo_state.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2024-1-19)
#ifndef DEMO_STATE_H
#define DEMO_STATE_H
#include "base_sml.h"
namespace demo_state {

HJ_CALLBACK_DEFINE(DemoStateEntry1)
HJ_CALLBACK_DEFINE(DemoStateEntry2)
struct DemoStateExit1 {
  void operator()();
};
struct DemoStateExit2 {
  void operator()();
};
}  // namespace demo_state

#endif
