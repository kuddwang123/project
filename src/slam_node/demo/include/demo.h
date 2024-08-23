// @file demo.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-22)
#ifndef INCLUDE_DEMO_H//your macro
#define INCLUDE_DEMO_H
#include "function_factory.h"
#include "node_factory.h"

namespace slam_node_ns {//your namespace

class Demo : public hj_bf::Function {
 public:
  explicit Demo(const rapidjson::Value &json_conf);
  ~Demo(){};

 private:
  // your variables
  // hj_bf::HJPublisher pub_;
  // hj_bf::HJClient client_;
  // hj_bf::HJSubscriber sub_;
  // hj_bf::HJServer service_;
  // hj_bf::HJTimer timer_;
};
}  // namespace slam_node_ns

#endif
