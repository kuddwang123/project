// @file self_check.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-22)
#include "self_check.h"
#include "log.h"
#include "node_cache.h"
#include "logic_dev/communication.h"

HJ_REGISTER_FUNCTION(factory) {
  HJ_INFO("minos register factory, funtion_name:%s", FUNCTION_NAME);
  factory.registerCreater<collect_node_self_check::SelfCheck>(FUNCTION_NAME);
}

namespace collect_node_self_check {

SelfCheck::~SelfCheck() {
  HJ_INFO("~SelfCheck");
}


SelfCheck::SelfCheck(const rapidjson::Value &json_conf) : hj_bf::Function(json_conf) {
  // read your config


  // your code
  srv_msg_.request.code_val = COLLECT_NODE_MAX;
  srv_msg_.request.status = 0;
  hj_bf::HjPushSrv(srv_msg_);

  HJ_INFO("SelfCheck constructor OK, self_check publish OK");
}
}  // namespace collect_node_self_check
