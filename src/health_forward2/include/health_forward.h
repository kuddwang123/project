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
#include "std_msgs/String.h"

namespace utils_node_health_forward {  // your namespace


class HealthForward : public hj_bf::Function {
 public:
  explicit HealthForward(const rapidjson::Value &json_conf);
  ~HealthForward();
 private:
  void TaskNameCallBack(const std_msgs::String::ConstPtr& msg);
  bool pubBigDataCallback(hj_interface::HealthCheckCodeRequest& req, hj_interface::HealthCheckCodeResponse& res);
 private:
  std::mutex mtx_;
  std::string task_name_;
  ros::Publisher big_data_pub_;
  hj_bf::HJSubscriber task_name_sub_;
  ros::NodeHandle nh_;
};
}  // namespace utils_node_health_forward

#endif  // SRC_UTILS_NODE_HEALTH_FORWARD_INCLUDE_HEALTH_FORWARD_H_
