/**
 * @file pressure.h
 * @author hao wu (clayderman@yardbot.net)
 * @brief get pressure data from pressure sensor
 * @version 0.1
 * @date 2024-06-20
 * 
 * @copyright Copyright 2023 HJ Technology Co.Ltd. All rights reserved
 * 
 */
#ifndef SRC_COLLECT_NODE_WF5803_INCLUDE_PRESSURE_H_
#define SRC_COLLECT_NODE_WF5803_INCLUDE_PRESSURE_H_
#include "function_factory.h"
#include "node_factory.h"
#include "i2c.h"
#include "status_code.h"
#include "node_cache.h"
#include "std_msgs/Bool.h"
#include "hj_interface/Depth.h"
#include "hj_interface/HealthCheckCode.h"

#define DEV_PATH            "/dev/i2c-2"
#define TEMP_CONVERT_FACTOR 256
#define SLAVE_ADDR          0x6c
#define ERROR_COUNT         10
#define SENSOR_STATUS_ADDR  0x02
#define SENSOR_DATA_ADDR    0x06
#define SENSOR_CMD_ADDR     0x30
#define MEANSURE_MODE       0x0a   // 1010, bit3=1:转换开始，转换结束后自动返回.
                                   // bit2~bit0:010表示组合转换（一次温度转换，然后立即进行一次传感器信号转换)
namespace collect_node_pressure {  // your namespace

class PressureSensorWF : public hj_bf::Function {
 public:
  explicit PressureSensorWF(const rapidjson::Value &json_conf);
  ~PressureSensorWF();
 private:
  void PressureTimer(const hj_bf::HJTimerEvent &);
  bool PressureGetData();
  void RestartCallback(const std_msgs::Bool::ConstPtr& msg);
  bool Start();
 private:
  bool init_status_{false};  // 初始化状态
  bool status_{true};
  int error_count_{0};
  int read_error_count_{0};
  uint32_t frequency_{50};  // param for PressureSensorWF
  hj_bf::I2C i2c_{DEV_PATH};
  hj_bf::HJPublisher chatter_pub_;
  hj_bf::HJSubscriber restart_sub_;
  hj_bf::HJTimer loop_timer_;
  hj_interface::Depth pub_msg_;
  hj_interface::HealthCheckCode srv_msg_;
};
}  // namespace collect_node_pressure

#endif  // SRC_COLLECT_NODE_WF5803_INCLUDE_PRESSURE_H_
