// @file demo_state.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2024-1-19)
#ifndef OTA_STATE_H
#define OTA_STATE_H
#include "base_sml.h"
namespace ota_state {

HJ_CALLBACK_DEFINE(OtaStateEntry1)
HJ_CALLBACK_DEFINE(OtaStateEntry2)
struct OtaStateExit1 {
  void operator()();
};
struct OtaStateExit2 {
  void operator()();
};
}  // namespace demo_state

#endif
