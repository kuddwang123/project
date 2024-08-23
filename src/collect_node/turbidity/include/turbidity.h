/**
 * @file turbidity.h
 * @author hao wu (clayderman@yardbot.net)
 * @brief turbidity sensor collect node
 * @version 0.1
 * @date 2024-06-20
 * 
 * @copyright Copyright 2023 HJ Technology Co.Ltd. All rights reserved
 * 
 */
#ifndef SRC_COLLECT_NODE_TURBIDITY_INCLUDE_TURBIDITY_H_
#define SRC_COLLECT_NODE_TURBIDITY_INCLUDE_TURBIDITY_H_
#include "function_factory.h"
#include "node_factory.h"
#include "uart.h"
#include "status_code.h"
#include "node_cache.h"
#include "std_msgs/Bool.h"
#include "hj_interface/Turbidity.h"
#include "hj_interface/HealthCheckCode.h"

#define DEV_PATH "/dev/ttyWCH1"
#define ERROR_COUNT 10

namespace collect_node_turbidity {  // your namespace

class Turbidimeter : public hj_bf::Function {
 public:
  explicit Turbidimeter(const rapidjson::Value &json_conf);
  ~Turbidimeter();
 private:
  void TurbidimeterTimer(const hj_bf::HJTimerEvent &);
  void RestartCallback(const std_msgs::Bool::ConstPtr& msg);
  bool Start();
 private:
  bool init_status_{false};  // 初始化状态
  bool status_{true};  // 获取数据状态
  int error_count_{0};
  uint32_t frequency_{1000};  // 频率1hz
  hj_bf::SpiUart uart_{DEV_PATH};
  hj_bf::HJPublisher chatter_pub_;
  hj_bf::HJSubscriber restart_sub_;
  hj_bf::HJTimer loop_timer_;
  hj_interface::Turbidity tur_msg_;
  hj_interface::HealthCheckCode srv_msg_;
};
}  // namespace collect_node_turbidity

#endif  // SRC_COLLECT_NODE_TURBIDITY_INCLUDE_TURBIDITY_H_
