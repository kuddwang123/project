// @file imu.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-22)
#ifndef SRC_COLLECT_NODE_SELF_CHECK_INCLUDE_SELF_CHECK_H_  // SRC_COLLECT_NODE_SELF_CHECK_INCLUDE_SELF_CHECK_H_
#define SRC_COLLECT_NODE_SELF_CHECK_INCLUDE_SELF_CHECK_H_



#include "hj_interface/HealthCheckCode.h"
#include "function_factory.h"
#include "node_factory.h"
#include "status_code.h"
#include "node_cache.h"

namespace collect_node_self_check {  // your namespace


class SelfCheck : public hj_bf::Function {
 public:
  explicit SelfCheck(const rapidjson::Value &json_conf);
  ~SelfCheck();
 private:
  hj_interface::HealthCheckCode srv_msg_;
};
}  // namespace collect_node_self_check

#endif  // SRC_COLLECT_NODE_SELF_CHECK_INCLUDE_SELF_CHECK_H_
