/**
 * @file magnetometers.cpp
 * @author hao wu (clayderman@yardbot.net)
 * @brief 
 * @version 0.1
 * @date 2024-06-20
 * 
 * @copyright Copyright 2023 HJ Technology Co.Ltd. All rights reserved
 * 
 */
#include "magnetometers.h"
#include "log.h"

HJ_REGISTER_FUNCTION(factory) {
  HJ_INFO("minos register factory, funtion_name:%s", FUNCTION_NAME);
  factory.registerCreater<collect_node_magnetometer::Magnetometer>(FUNCTION_NAME);
}

namespace collect_node_magnetometer {

void Magnetometer::LoopFunc(const hj_bf::HJTimerEvent &) {
  static uint8_t databuf[16];
  bool ret = i2c_.I2cReadBytes(ZY5658_ADDR, ZY5658_READ_MEASUREMENT, databuf, 7);
  if (ret) {  // 数据读取成功
    mag_msg_.custom_time = hj_bf::HJTime::now();
    mag_msg_.mag_x = (databuf[1] << 8) | databuf[2];
    mag_msg_.mag_y = (databuf[3] << 8) | databuf[4];
    mag_msg_.mag_z = (databuf[5] << 8) | databuf[6];

    if (!status_) {
      srv_msg_.request.code_val = MAG_DATA_ERROR;
      srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::NORMAL;
      hj_bf::HjPushSrv(srv_msg_);
    }
    status_ = true;
    error_count_ = 0;
    mag_pub_.publish(mag_msg_);
  } else {
    if (status_ && error_count_ > ERROR_COUNT_MAX) {
      srv_msg_.request.code_val = MAG_DATA_ERROR;
      srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::ERROR;
      hj_bf::HjPushSrv(srv_msg_);
      status_ = false;
      HJ_ERROR("mag read data from device error");
    }
    error_count_++;
  }
}

void Magnetometer::RestartCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data != 0 && !init_status_) {
    HJ_INFO("restart Magnetometer");
    Start();
  }
}

bool Magnetometer::Start() {
  bool ret = false;
  if (!i2c_.Initialize()) {
    HJ_ERROR("can not open file %s", DEV_PATH);
    init_status_ = false;
    srv_msg_.request.code_val = MAG_INIT_ERROR;
    srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::FAILED;
    hj_bf::HjPushSrv(srv_msg_);
  } else {
    uint8_t data = 0;
    int count = 0;
    while (count < ERROR_COUNT_MAX) {  // wait for device ready
      i2c_.I2cReadBytes(ZY5658_ADDR, ZY5658_BURST_MODE, &data, 1);
      if ((data & READY_FLAG) != 0) {
        break;
      }
      usleep(20 * 1000);
      count++;
    }
    if (count >= ERROR_COUNT_MAX) {
      init_status_ = false;
      srv_msg_.request.code_val = MAG_INIT_ERROR;
      srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::FAILED;
      hj_bf::HjPushSrv(srv_msg_);
      HJ_ERROR("mag can not read data from device");
    } else {
      init_status_ = true;
      loop_timer_ = hj_bf::HJCreateTimer("loop_timer", frequency_ * 1000, &Magnetometer::LoopFunc, this);
      ret = true;
    }
  }

  return ret;
}

Magnetometer::Magnetometer(const rapidjson::Value &json_conf) : hj_bf::Function(json_conf) {
  // read your config
  if (json_conf.HasMember("frequency") && json_conf["frequency"].IsInt()) {
    frequency_ = json_conf["frequency"].GetInt();
  }
  if (json_conf.HasMember("dev") && json_conf["dev"].IsString()) {
    std::string dev_path = json_conf["dev"].GetString();
    if (!dev_path.empty()) {
      i2c_.SetDev(dev_path);
    }
  }

  // your code
  mag_pub_ = hj_bf::HJAdvertise<hj_interface::Mag>("mag_chatter", 10);
  restart_sub_ = hj_bf::HJSubscribe("xxx", 10, &Magnetometer::RestartCallback, this);

  if (Start()) {
    HJ_INFO("Magnetometer start success");
  } else {
    HJ_ERROR("Magnetometer start ERROR");
  }
}
}  // namespace collect_node_magnetometer
