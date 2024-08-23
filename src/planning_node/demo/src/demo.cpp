// @file demo.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-22)
#include "demo.h"

HJ_REGISTER_FUNCTION(factory) {
  std::cerr << "minos register factory" << FUNCTION_NAME << std::endl;
  factory.registerCreater<planning_node_ns::Demo>(FUNCTION_NAME);
}
namespace planning_node_ns {
void Demo::callback2(const hj_bf::HJTimerEvent &)
{
  static int index = 0;
  while (1) {
    std::cerr << "minos just a demo:" << index<<std::endl;
    sleep(1);
    index++;
  }
}
Demo::Demo(const rapidjson::Value &json_conf) : hj_bf::Function(json_conf) {
  // read your config

  // your code
  std::cerr << "minos just a demo" << std::endl;
  timer2 = hj_bf::HJCreateTimer("timer2", 1 * 1000 * 1000, &Demo::callback2, this);
}
}  // namespace planning_node_ns
