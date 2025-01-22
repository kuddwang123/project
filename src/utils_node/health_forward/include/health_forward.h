// @file imu.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-22)
#ifndef SRC_UTILS_NODE_HEALTH_FORWARD_INCLUDE_HEALTH_FORWARD_H_  // SRC_UTILS_NODE_HEALTH_FORWARD_INCLUDE_HEALTH_FORWARD_H_
#define SRC_UTILS_NODE_HEALTH_FORWARD_INCLUDE_HEALTH_FORWARD_H_



#include "hj_interface/HealthCheckCode.h"
#include "hj_interface/BigdataUpload.h"
#include "function_factory.h"
#include "node_factory.h"
#include "status_code.h"
#include "node_cache.h"

namespace utils_node_health_forward {  // your namespace


class HealthForward : public hj_bf::Function {
 public:
  explicit HealthForward(const rapidjson::Value &json_conf);
  ~HealthForward();
 private:
  hj_bf::HJPublisher big_data_pub_;
  bool pubBigDataCallback(hj_interface::HealthCheckCodeRequest& req, hj_interface::HealthCheckCodeResponse& res);
};
}  // namespace utils_node_health_forward

#endif  // SRC_UTILS_NODE_HEALTH_FORWARD_INCLUDE_HEALTH_FORWARD_H_
