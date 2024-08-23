/**
 * @file pressure.cpp
 * @author hao wu (clayderman@yardbot.net)
 * @brief get pressure data from WF5803 pressure sensor
 * @version 0.1
 * @date 2024-06-20
 * 
 * @copyright Copyright 2023 HJ Technology Co.Ltd. All rights reserved
 * 
 */
#include "pressure.h"
#include "log.h"

HJ_REGISTER_FUNCTION(factory) {
  HJ_INFO("minos register factory,function_name:%s", FUNCTION_NAME);
  factory.registerCreater<collect_node_pressure::PressureSensorWF>(FUNCTION_NAME);
}

namespace collect_node_pressure {

bool PressureSensorWF::PressureGetData() {
  static uint8_t buffer[16];
  int32_t pressure = 0, tmp = 0;
  uint8_t meansure_mode = MEANSURE_MODE;
  float fpressure = 0.0;
  int nominal_pressure = 0;

  i2c_.I2cReadBytes(SLAVE_ADDR, SENSOR_STATUS_ADDR, &buffer[15], 1);
  while (1 != (buffer[15] & 0x01)) {  // status ==1 ready
    usleep(50 * 1000);
    read_error_count_++;

    i2c_.I2cReadBytes(SLAVE_ADDR, SENSOR_STATUS_ADDR, &buffer[15], 1);
    if (read_error_count_ > ERROR_COUNT) {  // 超过10次读取失败，认为出错
      read_error_count_ = 0;
      HJ_WARN("function:%s, I2cReadBytes NOT REDY", __FUNCTION__);
      return false;
    }
  }

  i2c_.I2cReadBytes(SLAVE_ADDR, SENSOR_DATA_ADDR, buffer, 5);

  pressure = buffer[0];
  pressure <<= 8;
  pressure |= buffer[1];
  pressure <<= 8;
  pressure |= buffer[2];

  if (pressure > 8388608) {
    fpressure = (pressure - 16777216) / 8388608.0f;
  } else {
    fpressure = pressure / 8388608.0f;
  }

  fpressure = 125 * fpressure + 17.5;  // unit: kpa
  nominal_pressure = fpressure * 1000;  // unit: pa

  tmp = buffer[4] | static_cast<uint16_t>(buffer[3] << 8);
  tmp = tmp / TEMP_CONVERT_FACTOR;  // Temperature output with an LSB equals to (1/256)℃

  i2c_.I2cWriteBytes(SLAVE_ADDR, SENSOR_CMD_ADDR, &meansure_mode, 1);

  pub_msg_.pressure = nominal_pressure;
  pub_msg_.temp = tmp;
  pub_msg_.timestamp = hj_bf::HJTime::now();
  return true;
}

void PressureSensorWF::PressureTimer(const hj_bf::HJTimerEvent &) {
  bool ret = PressureGetData();
  if (ret) {
    if (!status_) {
      srv_msg_.request.code_val = WF5803_DATA_ERROR;
      srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::NORMAL;
      hj_bf::HjPushSrv(srv_msg_);
      status_ = true;
    }
    error_count_ = 0;
    chatter_pub_.publish(pub_msg_);
  } else {
    if (status_ && error_count_ > ERROR_COUNT) {
      srv_msg_.request.code_val = WF5803_DATA_ERROR;
      srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::ERROR;
      hj_bf::HjPushSrv(srv_msg_);
      status_ = false;
    }
    error_count_++;
  }
}

PressureSensorWF::~PressureSensorWF() {
  HJ_INFO("~PressureSensorWF");
}

void PressureSensorWF::RestartCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data != 0 && !init_status_) {
    HJ_INFO("restart PressureSensorWF");
    Start();
  }
}

bool PressureSensorWF::Start() {
  bool ret = false;
  if (!i2c_.Initialize()) {
    HJ_ERROR("can not open file %s", DEV_PATH);
    init_status_ = false;
    srv_msg_.request.code_val = WF5803_INIT_ERROR;
    srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::FAILED;
    hj_bf::HjPushSrv(srv_msg_);
  } else {
    init_status_ = true;
    uint8_t meansure_mode = MEANSURE_MODE;
    i2c_.I2cWriteBytes(SLAVE_ADDR, SENSOR_CMD_ADDR, &meansure_mode, 1);
    loop_timer_ = hj_bf::HJCreateTimer("PressureTimer", frequency_ * 1000, &PressureSensorWF::PressureTimer, this);
    ret = true;
  }

  return ret;
}

PressureSensorWF::PressureSensorWF(const rapidjson::Value &json_conf) : hj_bf::Function(json_conf) {
  // read your config
  if (json_conf.HasMember("frequency") && json_conf["frequency"].IsInt()) {
    frequency_ = json_conf["frequency"].GetInt();  // unit: ms
  }
  if (json_conf.HasMember("dev") && json_conf["dev"].IsString()) {
    std::string dev_path = json_conf["dev"].GetString();
    if (!dev_path.empty()) {
      i2c_.SetDev(dev_path);
    }
  }

  chatter_pub_ = hj_bf::HJAdvertise<hj_interface::Depth>("depth_chatter", 10);
  restart_sub_ = hj_bf::HJSubscribe("/xxx", 1, &PressureSensorWF::RestartCallback, this);

  // your code
  if (Start()) {
    HJ_INFO("PressureSensorWF init success");
  } else {
    HJ_ERROR("PressureSensorWF init failed");
  }
}
}  // namespace collect_node_pressure
