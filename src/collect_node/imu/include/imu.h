/**
 * @file imu.h
 * @author hao wu (clayderman@yardbot.net)
 * @brief soc imu data collection node
 * @version 0.1
 * @date 2024-06-20
 * 
 * @copyright Copyright 2023 HJ Technology Co.Ltd. All rights reserved
 * 
 */
#ifndef SRC_COLLECT_NODE_IMU_INCLUDE_IMU_H_
#define SRC_COLLECT_NODE_IMU_INCLUDE_IMU_H_

#include "std_msgs/Bool.h"
#include "hj_interface/SocImu.h"
#include "hj_interface/HealthCheckCode.h"
#include "function_factory.h"
#include "node_factory.h"
#include "status_code.h"
#include "i2c.h"

#define DEV_PATH  "/dev/i2c-1"
#define ERROR_COUNT   10
#define PI_DEFINE     3.141593f
#define DEGREE        180.0f
#define GYRO_ACC_ODR  9.8f
#define READ_DATA_LEN 12
#define SLAVE_ADDR    0x68
#define WHOAMI        0X75
#define DEVICE_ID     0X6f
#define DEVICE_CONFIG 0x11
#define PWR_MGMT0     0x4E
#define GYRO_CONFIG0  0x4F
#define ACCEL_CONFIG0 0x50
#define REG_BANK_SEL  0x76
#define ACCEL_DATA_X1 0x1f

#define DEVICE_CONFIG_RESET_FLAG  0x01
#define REG_BANK_SEL_RESET_VALUE  0x00
#define ACCEL_CONFIG0_RESET_VALUE 0x06
#define GYRO_CONFIG0_RESET_VALUE  0x06
#define PWR_MGMT0_RESET_VALUE     0x1f

#define GYRO_SENSITIVITY_SCALE_FACTOR (16.4f)  // GYRO_CONFIG0初始化值相关，单位：LSB/(º/s)
#define ACCEL_SENSITIVITY_SCALE_FACTOR (2048.0f)  // ACCEL_CONFIG0初始化值相关，单位：LSB/g

namespace collect_node_imu {

class Imu : public hj_bf::Function {
 public:
  explicit Imu(const rapidjson::Value &json_conf);
  ~Imu();
 private:
  void ImuTimer(const hj_bf::HJTimerEvent &);
  bool Iim42652Init();
  void RestartCallback(const std_msgs::Bool::ConstPtr& msg);
  bool Start();
 private:
  bool init_status_{false};  // 初始化状态
  bool status_{true};  // 获取数据状态
  int error_count_{0};
  float frequency_{0.002};  // param for imu
  hj_bf::I2C i2c_{DEV_PATH};
  hj_bf::HJPublisher chatter_pub_;
  hj_bf::HJSubscriber restart_sub_;
  hj_bf::HJTimer loop_timer_;
  hj_interface::SocImu imu_msg_;
  hj_interface::HealthCheckCode srv_msg_;
  hj_bf::HighResolutionTimer high_pef_timer_;
};
}  // namespace collect_node_imu

#endif  // SRC_COLLECT_NODE_IMU_INCLUDE_IMU_H_
