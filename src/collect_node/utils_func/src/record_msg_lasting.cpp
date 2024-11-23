/**
 * @file record_msg.cpp
 * @author hao wu (clayderman@yardbot.net)
 * @brief record sensor data to file
 * @version 0.1
 * @date 2024-06-20
 * 
 * @copyright Copyright 2023 HJ Technology Co.Ltd. All rights reserved
 * 
 */
#include <fcntl.h>
#include<ctime>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/statvfs.h>
#include <dirent.h>
#include "record_msg_lasting.h"
#include "log.h"

namespace collect_node_utils_func {

bool RecordMsgLasting::initizlize(const std::string& file, uint8_t size) {
  std::string bat_log_path = file + "/bat.log";
  bat_logger_ = hj_cst_log_add(bat_log_path.data(), INFO_LOG, size*1024*1024, 2);
  if (bat_logger_ == nullptr) {
    HJ_ERROR("Failed to create bat logger.%s,size:%d", bat_log_path.data(), size);
    return false;
  }

  std::string out_water_log_path = file + "/out_water.log";
  out_water_logger_ = hj_cst_log_add(out_water_log_path.data(), INFO_LOG, size*1024*1024, 2);
  if (out_water_logger_ == nullptr) {
    HJ_ERROR("Failed to create out_water logger.");
    return false;
  }

  std::string motor_cur_log_path = file + "/motor_cur.log";
  motor_cur_logger_ = hj_cst_log_add(motor_cur_log_path.data(), INFO_LOG, size*1024*1024, 2);
  if (motor_cur_logger_ == nullptr) {
    HJ_ERROR("Failed to create motor_cur logger.");
    return false;
  }

  std::string tur_log_path = file + "/turbidity.log";
  tur_logger_ = hj_cst_log_add(tur_log_path.data(), INFO_LOG, size*1024*1024, 2);
  if (tur_logger_ == nullptr) {
    HJ_ERROR("Failed to create turbidity logger.");
    return false;
  }

  std::string temp_humidity_log_path = file + "/temp_humidity.log";
  temp_humidity_logger_ = hj_cst_log_add(temp_humidity_log_path.data(), INFO_LOG, size*1024*1024, 2);
  if (temp_humidity_logger_ == nullptr) {
    HJ_ERROR("Failed to create temp_humidity logger.");
    return false;
  }

  std::string pump_motor_speed_log_path = file + "/pump_motor_speed.log";
  pump_motor_speed_logger_ = hj_cst_log_add(pump_motor_speed_log_path.data(), INFO_LOG, size*1024*1024, 2);
  if (pump_motor_speed_logger_ == nullptr) {
    HJ_ERROR("Failed to create pump_motor_speed logger.");
    return false;
  }

  std::string fan_motor_speed_log_path = file + "/fan_motor_speed.log";
  fan_motor_speed_logger_ = hj_cst_log_add(fan_motor_speed_log_path.data(), INFO_LOG, size*1024*1024, 2);
  if (fan_motor_speed_logger_ == nullptr) {
    HJ_ERROR("Failed to create fan_motor_speed logger.");
    return false;
  }

  std::string sen_temp_log_path = file + "/sensor_temp.log";
  sen_temp_logger_ = hj_cst_log_add(sen_temp_log_path.data(), INFO_LOG, size*1024*1024, 2);
  if (sen_temp_logger_ == nullptr) {
    HJ_ERROR("Failed to create sen_temp_log_path logger.");
    return false;
  }

  std::string turn_motor_hall_log_path = file + "/turn_motor_hall.log";
  turn_motor_hall_logger_ = hj_cst_log_add(turn_motor_hall_log_path.data(), INFO_LOG, size*1024*1024, 2);
  if (turn_motor_hall_logger_ == nullptr) {
    HJ_ERROR("Failed to create turn_motor_hall_log_path logger.");
    return false;
  }

  std::string wireless_charging_log_path = file + "/wireless_charging.log";
  wireless_charging_logger_ = hj_cst_log_add(wireless_charging_log_path.data(), INFO_LOG, size*1024*1024, 2);
  if (wireless_charging_logger_ == nullptr) {
    HJ_ERROR("Failed to create wireless_charging_logger_ logger.");
    return false;
  }

#ifdef HJ_T1pro
  std::string dust_plug_log_path = file + "/dust_plug.log";
  dust_plug_logger_ = hj_cst_log_add(dust_plug_log_path.data(), INFO_LOG, size*1024*1024, 2);
  if (dust_plug_logger_ == nullptr) {
    HJ_ERROR("Failed to create dust_plug logger.");
    return false;
  }
#endif

  sub_motor_cur_ = hj_bf::HJSubscribe("motor_cur", 10, &RecordMsgLasting::MotorCurCallback, this);
  sub_tur_ = hj_bf::HJSubscribe("turbidity_data", 10, &RecordMsgLasting::TurbidityCallback, this);
  sub_temp_humidity_ = hj_bf::HJSubscribe("tempHumidity_chatter", 10,
                                            &RecordMsgLasting::TempHumidityCallback, this);
  sub_bat_ = hj_bf::HJSubscribe("bat_chatter", 100, &RecordMsgLasting::BatCallback, this);
  sub_pump_motor_speed_ = hj_bf::HJSubscribe("pumpMotorSpeed_chatter", 10,
                                              &RecordMsgLasting::PumpMotorSpeedCallback, this);
  sub_fan_motor_speed_ = hj_bf::HJSubscribe("fanMotorSpeed_chatter", 10,
                                              &RecordMsgLasting::FanMotorSpeedCallback, this);
  sub_out_water_ = hj_bf::HJSubscribe("outwater_hall", 10, &RecordMsgLasting::OutWaterCallback, this);
  sub_sen_temp_ = hj_bf::HJSubscribe("sensor_temp_chatter", 10, &RecordMsgLasting::SensorsTempCallback, this);
  sub_turn_motor_hall_ = hj_bf::HJSubscribe("/turn_motor_hall_chatter", 10, &RecordMsgLasting::TurnMotorHallCallback, this);
  sub_wireless_charging_ = hj_bf::HJSubscribe("/wireless_charging_chatter", 10, &RecordMsgLasting::WirelessChargingCallback, this);
#ifdef HJ_T1pro
  sub_dust_plug_ = hj_bf::HJSubscribe("t1pro/dust_plug_detection_chatter", 10,
                                      &RecordMsgLasting::DustPlugCallback, this);
#endif

  return true;
}

#ifdef HJ_T1pro
void RecordMsgLasting::DustPlugCallback(const hj_interface::DustPlugDetection::ConstPtr& msg) {
  ros::Time time_now = msg->timestamp;
  std::time_t time_c = time_now.sec;
  std::tm *ptm = std::localtime(&time_c);
  std::ostringstream oss;
  oss << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");
  msg_append(dust_plug_logger_, "%s %u %u\n", oss.str().c_str(), msg->status1, msg->status2);
}
#endif

void RecordMsgLasting::MotorCurCallback(const hj_interface::ElecMotorCur::ConstPtr& msg) {
  ros::Time time_now = msg->custom_time;
  std::time_t time_c = time_now.sec;
  std::tm *ptm = std::localtime(&time_c);
  std::ostringstream oss;
  oss << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");
  msg_append(motor_cur_logger_, "%s %lf %d %d %d %d %d %d %d %d\n", oss.str().c_str(),
          msg->custom_time.toSec(), msg->motor_l, msg->motor_r, msg->pump_l,
          msg->pump_r, msg->turn, msg->dirtybox, msg->flipcover, msg->airbag);
}

void RecordMsgLasting::TurbidityCallback(const hj_interface::Turbidity::ConstPtr& msg) {
  ros::Time time_now = msg->timestamp;
  std::time_t time_c = time_now.sec;
  std::tm *ptm = std::localtime(&time_c);
  std::ostringstream oss;
  oss << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");
  msg_append(tur_logger_, "%s %d\n", oss.str().c_str(), msg->turbidity);
}

void RecordMsgLasting::TempHumidityCallback(const hj_interface::TempHumidity::ConstPtr& msg) {
  ros::Time time_now = ros::Time::now();
  std::time_t time_c = time_now.sec;
  std::tm *ptm = std::localtime(&time_c);
  std::ostringstream oss;
  oss << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");
  msg_append(temp_humidity_logger_, "%s %d %d\n", oss.str().c_str(), msg->humidity, msg->temperature);
}

void RecordMsgLasting::BatCallback(const hj_interface::Bat::ConstPtr& msg) {
  ros::Time time_now = ros::Time::now();
  std::time_t time_c = time_now.sec;
  std::tm *ptm = std::localtime(&time_c);
  std::ostringstream oss;
  oss << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");
  msg_append(bat_logger_, "%s %d %d %d %d %d %d %d %d %d %d\n",
          oss.str().c_str(), msg->power, msg->temp1, msg->temp2, msg->temp3,
          msg->bat_vol, msg->bat_disch_cur, msg->ch_vol, msg->charger_ch_cur,
          msg->bat_cycle_times, msg->bat_health_left);
}

void RecordMsgLasting::PumpMotorSpeedCallback(const hj_interface::PumpMotorSpeed::ConstPtr& msg) {
  ros::Time time_now = msg->timestamp;
  std::time_t time_c = time_now.sec;
  std::tm *ptm = std::localtime(&time_c);
  std::ostringstream oss;
  oss << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");
  msg_append(pump_motor_speed_logger_, "%s %d %d\n", oss.str().c_str(), msg->speed_l, msg->speed_r);
}

void RecordMsgLasting::FanMotorSpeedCallback(const hj_interface::FanMotorSpeed::ConstPtr& msg) {
  ros::Time time_now = msg->timestamp;
  std::time_t time_c = time_now.sec;
  std::tm *ptm = std::localtime(&time_c);
  std::ostringstream oss;
  oss << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");
  msg_append(fan_motor_speed_logger_, "%s %d\n", oss.str().c_str(), msg->speed);
}

void RecordMsgLasting::OutWaterCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
  ros::Time time_now = ros::Time::now();
  std::time_t time_c = time_now.sec;
  std::tm *ptm = std::localtime(&time_c);
  std::ostringstream oss;
  oss << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");
  msg_append(out_water_logger_, "%s %d %d %d\n", oss.str().c_str(), msg->data.at(0), msg->data.at(1), msg->data.at(2));
}

void RecordMsgLasting::SensorsTempCallback(const hj_interface::SensorTemp::ConstPtr& msg) {
  ros::Time time_now = ros::Time::now();
  std::time_t time_c = time_now.sec;
  std::tm *ptm = std::localtime(&time_c);
  std::ostringstream oss;
  oss << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");
  msg_append(sen_temp_logger_, "%s %d %d %d %d %d\n", oss.str().c_str(),
          msg->pump_left, msg->pump_right, msg->left_motor, msg->right_motor, msg->turn);
}

void RecordMsgLasting::TurnMotorHallCallback(const std_msgs::UInt8::ConstPtr& msg) {
  ros::Time time_now = ros::Time::now();
  std::time_t time_c = time_now.sec;
  std::tm *ptm = std::localtime(&time_c);
  std::ostringstream oss;
  oss << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");  // format time string
  msg_append(turn_motor_hall_logger_, "%s %d\n", oss.str().c_str(), msg->data);
}

void RecordMsgLasting::WirelessChargingCallback(const hj_interface::WirelessCharging::ConstPtr& msg) {
  ros::Time time_now = ros::Time::now();
  std::time_t time_c = time_now.sec;
  std::tm *ptm = std::localtime(&time_c);
  std::ostringstream oss;
  oss << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");  // format time string
  msg_append(wireless_charging_logger_, "%s %d %d %d %d %d %d\n", oss.str().c_str(),
          msg->bridge_circuit_vol, msg->charger_ch_vol, msg->bridge_circuit_cur,
          msg->charger_ch_cur, msg->bridge_circuit_temp, msg->coil_temp);
}

RecordMsgLasting::~RecordMsgLasting() {
  if (bat_logger_ != nullptr) {
    hj_cst_log_del(bat_logger_);
  }
  if (out_water_logger_ != nullptr) {
    hj_cst_log_del(out_water_logger_);
  }
  if (motor_cur_logger_ != nullptr) {
    hj_cst_log_del(motor_cur_logger_);
  }
  if (tur_logger_ != nullptr) {
    hj_cst_log_del(tur_logger_);
  }
  if (temp_humidity_logger_ != nullptr) {
    hj_cst_log_del(temp_humidity_logger_);
  }
  if (pump_motor_speed_logger_ != nullptr) {
    hj_cst_log_del(pump_motor_speed_logger_);
  }
  if (fan_motor_speed_logger_ != nullptr) {
    hj_cst_log_del(fan_motor_speed_logger_);
  }
  if (sen_temp_logger_ != nullptr) {
    hj_cst_log_del(sen_temp_logger_);
  }
  if (turn_motor_hall_logger_ != nullptr) {
    hj_cst_log_del(turn_motor_hall_logger_);
  }
  if (wireless_charging_logger_ != nullptr) {
    hj_cst_log_del(wireless_charging_logger_);
  }
#ifdef HJ_T1pro
  if (dust_plug_logger_ != nullptr) {
    hj_cst_log_del(dust_plug_logger_);
  }
#endif
}

void RecordMsgLasting::run(const std::string& dir, uint8_t size) {
  if (access(dir.c_str(), 0) == -1) {  // check folder exist
    int ret = mkdir(dir.c_str(), S_IRWXU);  // create folder
    if (ret == -1) {
      HJ_ERROR("mkdir %s fail\n", dir.c_str());
      return;
    }
  }
  if (!initizlize(dir, size)) {
    HJ_ERROR("Failed to initizlize RecordMsgLasting.");
  }
  HJ_INFO("RecordMsgLasting loaded success\n");
}

void RecordMsgLasting::createInstance(const recordMsgLastingParam& param) {
  uint8_t size = param.size;
  std::string log_prefix = param.record_path;
  
  auto thread = std::thread(&RecordMsgLasting::run, this, log_prefix, size);
  thread.detach();
}
}  // namespace collect_node_utils_func
