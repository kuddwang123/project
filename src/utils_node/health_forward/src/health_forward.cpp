// @file self_check.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-22)
#include "health_forward.h"
#include <fstream>
#include "log.h"
#include "rapidjson/document.h"

HJ_REGISTER_FUNCTION(factory) {
  HJ_INFO("minos register factory, funtion_name:%s", FUNCTION_NAME);
  factory.registerCreater<utils_node_health_forward::HealthForward>(FUNCTION_NAME);
}

namespace utils_node_health_forward {

static std::string GetVersion() {
  static std::string fw_version = "";
  if (!fw_version.empty()) {
    return fw_version;
  }
  std::ifstream stream1("/etc/version");  // 输入流
  rapidjson::Document document;
  std::string jsonString((std::istreambuf_iterator<char>(stream1)),
                  std::istreambuf_iterator<char>());

  if (!document.Parse(jsonString.data()).HasParseError()) {
    if (document.HasMember("fw_ver") && document["fw_ver"].IsString()) {
      fw_version = document["fw_ver"].GetString();
    }
  }
  return fw_version;
}

HealthForward::~HealthForward() {
  HJ_INFO("~HealthForward");
}

bool HealthForward::pubBigDataCallback(hj_interface::HealthCheckCodeRequest& req,
                               hj_interface::HealthCheckCodeResponse& res) {
  if (req.code_val == status_code::COLLECT_NODE_MAX ||
      req.code_val == status_code::SLAM_INIT_DONE ||
      req.code_val == status_code::PLANING_NODE_MAX ||
      req.code_val == status_code::MCU_SELF_CHECK_DONE) {
    return true;
  }
  uint32_t error_code = req.code_val;
  uint8_t status = req.status;
  double timestamp = ros::Time::now().toSec();
  int64_t timestamp_ms = static_cast<int64_t>(timestamp * 1000);
  std::string fw_version = GetVersion();
  std::string str = R"({"event": "errorEvent", "errorCode":)" +
      std::to_string(error_code)+ R"(, "status": )" + std::to_string(status) +
      R"(, "time": )" +  std::to_string(timestamp_ms) +  R"(, "fwVersion": ")" + fw_version + R"("})";
  hj_interface::BigdataUpload msg;
  msg.payload = str;
  msg.type = hj_interface::BigdataUpload::kBigdataImmediate;
  big_data_pub_.publish(msg);
  HJ_INFO("pubBigDataCallback, str:%s", str.c_str());
  return true;
}


HealthForward::HealthForward(const rapidjson::Value &json_conf) : hj_bf::Function(json_conf) {
  // your code
  big_data_pub_ = hj_bf::HJAdvertise<hj_interface::BigdataUpload>("/big_data_cmd", 10);
  auto func = std::bind(&HealthForward::pubBigDataCallback, this, std::placeholders::_1, std::placeholders::_2);
  hj_bf::registerServerCallback(func);  // 收集报警码的订阅函数

  HJ_INFO("HealthForward constructor OK, self_check publish OK");
}
}  // namespace utils_node_health_forward
