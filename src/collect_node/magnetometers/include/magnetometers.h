/**
 * @file magnetometers.h
 * @author hao wu (clayderman@yardbot.net)
 * @brief magnetometer data collection node
 * @version 0.1
 * @date 2024-06-20
 * 
 * @copyright Copyright 2023 HJ Technology Co.Ltd. All rights reserved
 * 
 */
#ifndef SRC_COLLECT_NODE_MAGNETOMETERS_INCLUDE_MAGNETOMETERS_H_
#define SRC_COLLECT_NODE_MAGNETOMETERS_INCLUDE_MAGNETOMETERS_H_
#include "function_factory.h"
#include "node_factory.h"
#include "i2c.h"
#include "node_cache.h"
#include "status_code.h"
#include "std_msgs/Bool.h"
#include "hj_interface/Mag.h"
#include "hj_interface/HealthCheckCode.h"

#define ERROR_COUNT_MAX 10  // 错误计数最大值
#define READY_FLAG 0x80  // 准备好标志位
#define ZY5658_ADDR 0x0C
#define ZY5658_BURST_MODE 0x1E
#define ZY5658_READ_MEASUREMENT  0x4E  // 0100zyxt
#define DEV_PATH "/dev/i2c-2"

namespace collect_node_magnetometer {  // your namespace

class Magnetometer : public hj_bf::Function {
 public:
  explicit Magnetometer(const rapidjson::Value &json_conf);
  ~Magnetometer() { }
 private:
  void LoopFunc(const hj_bf::HJTimerEvent &);
  void RestartCallback(const std_msgs::Bool::ConstPtr& msg);
  bool Start();

 private:
  bool init_status_{false};  // 初始化状态
  bool status_{true};  // 获取数据状态
  int error_count_{0};
  uint32_t frequency_{10};  // 采样频率
  hj_bf::HJPublisher mag_pub_;
  hj_bf::HJSubscriber restart_sub_;
  hj_bf::HJTimer loop_timer_;
  hj_interface::Mag mag_msg_;
  hj_bf::I2C i2c_{DEV_PATH};
  hj_interface::HealthCheckCode srv_msg_;
};
}  // namespace collect_node_magnetometer

#endif  // SRC_COLLECT_NODE_MAGNETOMETERS_INCLUDE_MAGNETOMETERS_H_
