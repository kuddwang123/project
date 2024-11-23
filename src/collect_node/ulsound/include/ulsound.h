/**
 * @file ulsound.h
 * @author hao wu (clayderman@yardbot.net)
 * @brief get ultra sound data from uart
 * @version 0.1
 * @date 2024-06-20
 * 
 * @copyright Copyright 2023 HJ Technology Co.Ltd. All rights reserved
 * 
 */
#ifndef SRC_COLLECT_NODE_ULSOUND_INCLUDE_ULSOUND_H_
#define SRC_COLLECT_NODE_ULSOUND_INCLUDE_ULSOUND_H_
#include <string>
#include <mutex>
#include "function_factory.h"
#include "node_factory.h"
#include "uart.h"
#include "status_code.h"
#include "node_cache.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "hj_interface/TripleUltra.h"
#include "hj_interface/LeftBack.h"
#include "hj_interface/LeftFront.h"
#include "hj_interface/DownRight.h"
#include "hj_interface/HealthCheckCode.h"

#define DEV_PATH_FRONT     "/dev/ttyWCH0"
#define DEV_PATH_SIDE      "/dev/ttyWCH2"
#define SWITCH_PATH_SIDE   "/dev/gpiosw-uls"
#define ERROR_COUNT 10
#define OUTWATER_DISTANCE 65531  // 65531mm,代表单波出水


namespace collect_node_ulsound {  // your namespace

typedef struct serial_data {
  uint8_t databuf[64];  // 发送/接受数据
} ser_Data;

class Ulsound : public hj_bf::Function {
 public:
  explicit Ulsound(const rapidjson::Value &json_conf);
  ~Ulsound();
 private:
  void ReadFront(const hj_bf::HJTimerEvent &);
  void ReadSideFront();
  void ReadSideBack();
  void ReadDown();
  void PushFrontError();
  void PushSideFrontError();
  void PushSideBackError();
  void PushDownError();
  void RestartCallback(const std_msgs::Bool::ConstPtr& msg);
  void TimeDiffCallback(const std_msgs::Float64::ConstPtr& msg);
  ros::Time GetTimeNow();
  bool Start();
 private:
  std::atomic<bool> restart_flag_{false};  // 重启标志
  bool front_init_status_{false};  // 初始化状态
  bool side_init_status_{false};  // 初始化状态
  bool side_front_init_status_{false};  // 初始化状态
  bool side_back_init_status_{false};  // 初始化状态
  bool down_init_status_{false};  // 初始化状态
  bool front_uls_status_{true};       // 三合一超声波传感器状态
  bool side_front_uls_status_{true};   // 侧向超声波传感器状态
  bool side_back_uls_status_{true};   // 侧向超声波开关状态
  bool down_uls_status_{true};   // 侧向超声波开关状态
  int front_error_count_{0};
  int side_front_error_count_{0};
  int side_back_error_count_{0};
  int down_error_count_{0};
  int front_uls_frequency_{70};
  std::atomic<double> time_diff_{0.0};  // system time and RTC time diff
  ser_Data front_data_;
  ser_Data rec_data_;
  std::mutex mutex_;
  std::mutex srv_mutex_;
  hj_interface::HealthCheckCode srv_msg_;
  hj_bf::SpiUart front_uart_{DEV_PATH_FRONT};
  hj_bf::SpiUart side_uart_{DEV_PATH_SIDE};
  hj_bf::SpiUart side_uart_back_{DEV_PATH_SIDE};
  hj_bf::SpiUart side_uart_front_{DEV_PATH_SIDE};
  hj_bf::SpiUart down_uart_{DEV_PATH_SIDE};
  std::string machine_version_;
  hj_bf::HJPublisher triple_ultra_pub_;
  hj_bf::HJSubscriber restart_sub_;
  hj_bf::HJPublisher left_front_pub_;
  hj_bf::HJPublisher left_back_pub_;
  hj_bf::HJPublisher down_right_pub_;
  hj_interface::TripleUltra triple_ultra_msg_;
  hj_interface::LeftFront left_front_msg_;
  hj_interface::LeftBack left_back_msg_;
  hj_interface::DownRight down_right_msg_;
  hj_bf::HJSubscriber sub_time_diff_;
  hj_bf::HJTimer triple_timer_;
};
}  // namespace collect_node_ulsound

#endif  // SRC_COLLECT_NODE_ULSOUND_INCLUDE_ULSOUND_H_
