// @file demo.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-22)
#include "demo.h"

HJ_REGISTER_FUNCTION(factory) {
  std::cerr << "minos register factory" << FUNCTION_NAME << std::endl;
  factory.registerCreater<middleware_node_ns::Demo>(FUNCTION_NAME);
}
namespace middleware_node_ns {

Demo::Demo(const rapidjson::Value &json_conf) : hj_bf::Function(json_conf) {
  // read your config

  // your code
  std::cerr << "minos just a demo" << std::endl;
}
}  // namespace middleware_node_ns
