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
#include "shm_interface.h"

HJ_REGISTER_FUNCTION(factory) {
  HJ_INFO("minos register factory, funtion_name:%s", FUNCTION_NAME);
  factory.registerCreater<collect_node_magnetometer::Magnetometer>(FUNCTION_NAME);
}

namespace collect_node_magnetometer {

bool Magnetometer::CheckDataReady() {
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
    if (init_status_) {
      srv_msg_.request.code_val = MAG_DATA_ERROR;
      srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::ERROR;
      hj_bf::HjPushSrv(srv_msg_);
      HJ_ERROR("mag CheckDataReady false");
    }
    init_status_ = false;
    return false;
  }
  return true;
}

void Magnetometer::LoopFunc(const hj_bf::HJTimerEvent &) {
  bool res = hj_bf::getVariable("module_restart_flag", module_restart_flag_);
  if (res && module_restart_flag_ == 1) {
    return;
  }
  static bool check_flag = false;
  if (!check_flag && !CheckDataReady()) {
    return;
  } else {
    check_flag = true;
  }
  static uint8_t databuf[16];
  bool ret = i2c_.I2cReadBytes(ZY5658_ADDR, ZY5658_READ_MEASUREMENT, databuf, 7);
  if (ret) {  // 数据读取成功
    mag_msg_.custom_time = GetTimeNow();
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
    if (mag_msg_.mag_x == -1 && mag_msg_.mag_y == -1 && mag_msg_.mag_z == -1) {
      check_flag = false;
      HJ_ERROR("mag read data from device error -1 -1 -1");
      return;
    }
    mag_pub_.publish(mag_msg_);
  } else {
    if (status_ && error_count_ > ERROR_COUNT_MAX) {
      srv_msg_.request.code_val = MAG_DATA_ERROR;
      srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::ERROR;
      hj_bf::HjPushSrv(srv_msg_);
      status_ = false;
      HJ_ERROR("mag read data from device error");
      // check_flag = false;
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
    init_status_ = true;
    loop_timer_ = hj_bf::HJCreateTimer("loop_timer", frequency_ * 1000, &Magnetometer::LoopFunc, this);
    ret = true;
  }

  return ret;
}

void Magnetometer::TimeDiffCallback(const std_msgs::Float64::ConstPtr& msg) {
  time_diff_.store(msg->data);
}

bool Magnetometer::modulePowerManage(bool is_power_on) {
  FILE *file = fopen("/sys/rk8xx/rk8xx_dbg", "w");
  if (file == NULL) {
    perror("Error opening file");
    return false;
  }

  if (is_power_on) {
    HJ_INFO("power on\n");
    // 将数据写入文件
    if (fprintf(file, "w 0xb4 0xff\n") < 0) {
      perror("Error writing to file");
      fclose(file);
      return false;
    }
  } else {
    HJ_INFO("power off\n");
    if (fprintf(file, "w 0xb4 0xf3\n") < 0) {
      perror("Error writing to file");
      fclose(file);
      return false;
    }
  }
  fclose(file);
  return true;
}

ros::Time Magnetometer::GetTimeNow() {
  double time_now = ros::Time::now().toSec();
  double time_diff = time_diff_.load() * 0.001;
  double time_now_diff = time_now - time_diff;
  static int error_count = 0;
  if (time_now_diff < 0) {
    error_count++;
    if (error_count < 10) {
      HJ_ERROR("time_diff_chatter error, time_now:%lf, time_diff:%lf, time_now_diff:%lf", time_now, time_diff, time_now_diff);
    }
    return ros::Time::now();
  } else if (time_now_diff > std::numeric_limits<uint32_t>::max()) {
    error_count++;
    if (error_count < 10) {
      HJ_ERROR("time_diff_chatter error, time_now:%lf, time_diff:%lf, time_now_diff:%lf", time_now, time_diff, time_now_diff);
    }
    return ros::Time::now();
  } else {
    error_count = 0;
    ros::Time now_time = ros::Time().fromSec(time_now_diff);
    return now_time;
  }
}

Magnetometer::Magnetometer(const rapidjson::Value &json_conf) : hj_bf::Function(json_conf) {
  int reboot_flag = 0;
  // system("echo w 0xb4 0xff > /sys/rk8xx/rk8xx_dbg");
  bool ret = modulePowerManage(true);
  if (!ret) {
    HJ_ERROR("power on module error");
  }
  hj_bf::setVariable("module_restart_flag", reboot_flag);  // soc sensor module reboot end
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
  sub_time_diff_ = hj_bf::HJSubscribe("/time_diff_chatter", 1, &Magnetometer::TimeDiffCallback, this);
  if (Start()) {
    HJ_INFO("Magnetometer start success");
  } else {
    HJ_ERROR("Magnetometer start ERROR");
  }
}
}  // namespace collect_node_magnetometer
