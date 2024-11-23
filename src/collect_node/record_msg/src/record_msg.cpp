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
#include "record_msg.h"
#include "log.h"
#include "boost/filesystem.hpp"

HJ_REGISTER_FUNCTION(factory) {
  HJ_INFO("minos register factory, funtion_name:%s", FUNCTION_NAME);
  factory.registerCreater<collect_node_record_msg::RecordMsg>(FUNCTION_NAME);
}

namespace collect_node_record_msg {

bool RecordMsg::initizlize() {
  // check log folder exist
  if (access(log_prefix_.c_str(), 0) == -1) {  // check folder exist
    int ret = mkdir(log_prefix_.c_str(), S_IRWXU);  // create folder
    if (ret == -1) {
      HJ_ERROR("mkdir %s fail\n", log_prefix_.c_str());
      return false;
    }
  } else {
    boost::filesystem::path path1(log_prefix_);
    boost::filesystem::remove_all(path1);
    int ret = mkdir(log_prefix_.c_str(), S_IRWXU);  // create folder
    if (ret == -1) {
      HJ_ERROR("mkdir %s fail\n", log_prefix_.c_str());
      return false;
    }
  }
  // define log file name
  std::string motor = log_prefix_ + "/encoder.log";
  std::string imu = log_prefix_ + "/imu.log";
  std::string triple_ultra = log_prefix_ + "/triple_ultra.log";
  std::string mag = log_prefix_ + "/mag.log";
  std::string nav = log_prefix_ + "/nav.log";
  std::string pressure = log_prefix_ + "/pressure.log";
  std::string soc_imu = log_prefix_ + "/soc_imu.log";
  std::string motor_cur = log_prefix_ + "/motor_cur.log";
  std::string turbidity = log_prefix_ + "/turbidity.log";
  std::string temp_humidity = log_prefix_ + "/temp_humidity.log";
  std::string bat = log_prefix_ + "/bat.log";
  std::string pump_motor_speed = log_prefix_ + "/pump_motor_speed.log";
  std::string fan_motor_speed = log_prefix_ + "/fan_motor_speed.log";
  std::string out_water_check = log_prefix_ + "/out_water_check.log";
#ifdef HJ_T1pro
  std::string left_tof = log_prefix_ + "/left_tof.log";
  std::string down_ray = log_prefix_ + "/down_ray.log";
  std::string dust_plug = log_prefix_ + "/dust_plug.log";
#else
  std::string left_back = log_prefix_ + "/left_back.log";
  std::string left_front = log_prefix_ + "/left_front.log";
  std::string down_right = log_prefix_ + "/down_right.log";
  std::string down_left = log_prefix_ + "/down_left.log";
  std::string down_ray = log_prefix_ + "/down_ray.log";
#endif

  fd_motor_ = ::open(motor.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
  if (fd_motor_ == -1) {
    HJ_ERROR("open Encoder.log fail\n");
    return false;
  }

  fd_imu_ = ::open(imu.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
  if (fd_imu_ == -1) {
    HJ_ERROR("open imu.log fail\n");
    return false;
  }

  fd_triple_ultra_ = ::open(triple_ultra.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
  if (fd_triple_ultra_ == -1) {
    HJ_ERROR("open triple_ultra.log fail\n");
    return false;
  }

#ifdef HJ_T1pro
  fd_tof_ = ::open(left_tof.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
  if (fd_tof_ == -1) {
    HJ_ERROR("open left_tof.log fail\n");
    return false;
  }
  fd_down_ray_ = ::open(down_ray.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
  if (fd_down_ray_ == -1) {
    HJ_ERROR("open down_ray.log fail\n");
    return false;
  }
  fd_dust_plug_ = ::open(dust_plug.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
  if (fd_dust_plug_ == -1) {
    HJ_ERROR("open dust_plug.log fail\n");
    return false;
  }
#else
  fd_left_back_ = ::open(left_back.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
  if (fd_left_back_ == -1) {
    HJ_ERROR("open left_back.log fail\n");
    return false;
  }

  fd_left_front_ = ::open(left_front.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
  if (fd_left_front_ == -1) {
    HJ_ERROR("open left_front.log fail\n");
    return false;
  }
  if (machine_version_ == "P3.5") {
    fd_down_left_ = ::open(down_left.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
    if (fd_down_left_ == -1) {
      HJ_ERROR("open down_left.log fail\n");
      return false;
    }

    fd_down_right_ = ::open(down_right.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
    if (fd_down_right_ == -1) {
      HJ_ERROR("open down_right.log fail\n");
      return false;
    }
  } else {
    fd_down_ray_ = ::open(down_ray.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
    if (fd_down_ray_ == -1) {
      HJ_ERROR("open down_ray.log fail\n");
      return false;
    }
  }
#endif

  fd_mag_ = ::open(mag.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
  if (fd_mag_ == -1) {
    HJ_ERROR("open mag.log fail\n");
    return false;
  }

  fd_nav_ = ::open(nav.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
  if (fd_nav_ == -1) {
    HJ_ERROR("open nav.log fail\n");
    return false;
  }

  fd_pressure_ = ::open(pressure.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
  if (fd_pressure_ == -1) {
    HJ_ERROR("open pressure.log fail\n");
    return false;
  }

  fd_soc_imu_ = ::open(soc_imu.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
  if (fd_soc_imu_ == -1) {
    HJ_ERROR("open soc_imu.log fail\n");
    return false;
  }

  fd_motor_cur_ = ::open(motor_cur.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
  if (fd_motor_cur_ == -1) {
    HJ_ERROR("open motor_cur.log fail\n");
    return false;
  }

  fd_tur_ = ::open(turbidity.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
  if (fd_tur_ == -1) {
    HJ_ERROR("open turbidity.log fail\n");
    return false;
  }
  fd_temp_humidity_ = ::open(temp_humidity.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
  if (fd_temp_humidity_ == -1) {
    HJ_ERROR("open temp_humidity.log fail\n");
    return false;
  }

  fd_bat_ = ::open(bat.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
  if (fd_bat_ == -1) {
    HJ_ERROR("open bat.log fail\n");
    return false;
  }

  fd_bat_ = ::open(bat.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
  if (fd_bat_ == -1) {
    HJ_ERROR("open bat.log fail\n");
    return false;
  }

  fd_pump_motor_speed_ = ::open(pump_motor_speed.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
  if (fd_pump_motor_speed_ == -1) {
    HJ_ERROR("open pump_motor_speed.log fail\n");
    return false;
  }

  fd_fan_motor_speed_ = ::open(fan_motor_speed.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
  if (fd_fan_motor_speed_ == -1) {
    HJ_ERROR("open fan_motor_speed.log fail\n");
    return false;
  }

  fd_out_water_ = ::open(out_water_check.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
  if (fd_out_water_ == -1) {
    HJ_ERROR("open out_water_check.log fail\n");
    return false;
  }
  std::string motor_time = log_prefix_ + "/motor_time.log";
  std::string imu_time = log_prefix_ + "/imu_time.log";
  fd_motor_time_ = ::open(motor_time.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
  fd_imu_time_ = ::open(imu_time.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
  // std::array<std
  if (record_all_topics_) {
    imu_time_sub_ = hj_bf::HJSubscribe("imu_time_chatter", 100, &RecordMsg::WriteImu, this);  // 50HZ
    motor_tmie_sub_ = hj_bf::HJSubscribe("motor_time_chatter", 100, &RecordMsg::WriteMotor, this);  // 50HZ
    sub_motor_ = hj_bf::HJSubscribe("motor_chatter", 100, &RecordMsg::MotorChatterCallback, this);  // 50HZ
    sub_imu_ = hj_bf::HJSubscribe("imu_chatter", 100, &RecordMsg::ImuChatterCallback, this);  // 100HZ
    sub_mag_ = hj_bf::HJSubscribe("mag_chatter", 100, &RecordMsg::MagChatterCallback, this);  // 100HZ
    sub_motor_set_ = hj_bf::HJSubscribe("nav_motor", 100, &RecordMsg::MotorSetChatterCallback, this);
    sub_pressure_ = hj_bf::HJSubscribe("depth_chatter", 100, &RecordMsg::PressureChatterCallback, this);  // 20HZ
    sub_soc_imu_ = hj_bf::HJSubscribe("soc_imu_chatter", 100, &RecordMsg::SocImuChatterCallback, this);  // 500HZ
    sub_motor_cur_ = hj_bf::HJSubscribe("motor_cur", 100, &RecordMsg::MotorCurCallback, this);  // 10HZ
    sub_tur_ = hj_bf::HJSubscribe("turbidity_data", 100, &RecordMsg::TurbidityCallback, this);  // 1HZ
    sub_temp_humidity_ = hj_bf::HJSubscribe("tempHumidity_chatter", 100,
      &RecordMsg::TempHumidityCallback, this);  // 1HZ
    sub_bat_ = hj_bf::HJSubscribe("bat_chatter", 100, &RecordMsg::BatCallback, this);  // 1HZ
    sub_pump_motor_speed_ = hj_bf::HJSubscribe("pumpMotorSpeed_chatter", 100, &RecordMsg::PumpMotorSpeedCallback, this);
    sub_fan_motor_speed_ = hj_bf::HJSubscribe("fanMotorSpeed_chatter", 100, &RecordMsg::FanMotorSpeedCallback, this);
    sub_out_water_ = hj_bf::HJSubscribe("outwater_detect", 100, &RecordMsg::OutWaterCallback, this);
    sub_triple_ultra_ = hj_bf::HJSubscribe("triple_ultra", 100, &RecordMsg::TripleUltraCallback, this);
#ifdef HJ_T1pro
    sub_tof_ = hj_bf::HJSubscribe("t1pro/left_tof", 100, &RecordMsg::TofChatterCallback, this);
    sub_down_ray_ = hj_bf::HJSubscribe("t1pro/down_ray", 100, &RecordMsg::FallCallback, this);  // 10HZ
    sub_dust_plug_ = hj_bf::HJSubscribe("t1pro/dust_plug_detection_chatter", 100, &RecordMsg::DustPlugCallback, this);
#else
    sub_left_back_ = hj_bf::HJSubscribe("x9/left_back", 100, &RecordMsg::LeftBackCallback, this);
    sub_left_front_ = hj_bf::HJSubscribe("x9/left_front", 100, &RecordMsg::LeftFrontCallback, this);
    if (machine_version_ == "P3.5") {
      sub_down_left_ = hj_bf::HJSubscribe("x9/down_left", 100, &RecordMsg::DownLeftCallback, this);
      sub_down_right_ = hj_bf::HJSubscribe("x9/down_right", 100, &RecordMsg::DownRightCallback, this);
    } else {
      sub_down_ray_ = hj_bf::HJSubscribe("x9/down_ray", 100, &RecordMsg::FallCallback, this);  // 10HZ
    }
#endif
  } else {
    if (topics_.find("motor_chatter") != topics_.end()) {
      sub_motor_ = hj_bf::HJSubscribe("motor_chatter", 100, &RecordMsg::MotorChatterCallback, this);  // 50HZ
    }
    if (topics_.find("imu_chatter") != topics_.end()) {
      sub_imu_ = hj_bf::HJSubscribe("imu_chatter", 100, &RecordMsg::ImuChatterCallback, this);  // 100HZ
    }
    if (topics_.find("mag_chatter") != topics_.end()) {
      sub_mag_ = hj_bf::HJSubscribe("mag_chatter", 100, &RecordMsg::MagChatterCallback, this);  // 100HZ
    }
    if (topics_.find("nav_motor") != topics_.end()) {
      sub_motor_set_ = hj_bf::HJSubscribe("nav_motor", 100, &RecordMsg::MotorSetChatterCallback, this);
    }
    if (topics_.find("depth_chatter") != topics_.end()) {
      sub_pressure_ = hj_bf::HJSubscribe("depth_chatter", 100, &RecordMsg::PressureChatterCallback, this);  // 20HZ
    }
    if (topics_.find("soc_imu_chatter") != topics_.end()) {
      sub_soc_imu_ = hj_bf::HJSubscribe("soc_imu_chatter", 100, &RecordMsg::SocImuChatterCallback, this);  // 500HZ
    }
    if (topics_.find("motor_cur") != topics_.end()) {
      sub_motor_cur_ = hj_bf::HJSubscribe("/motor_cur", 100, &RecordMsg::MotorCurCallback, this);  // 10HZ
    }
    if (topics_.find("turbidity_data") != topics_.end()) {
      sub_tur_ = hj_bf::HJSubscribe("turbidity_data", 100, &RecordMsg::TurbidityCallback, this);  // 1HZ
    }
    if (topics_.find("tempHumidity_chatter") != topics_.end()) {
      sub_temp_humidity_ = hj_bf::HJSubscribe("tempHumidity_chatter", 100,
        &RecordMsg::TempHumidityCallback, this);  // 1HZ
    }
    if (topics_.find("bat_chatter") != topics_.end()) {
      sub_bat_ = hj_bf::HJSubscribe("bat_chatter", 100, &RecordMsg::BatCallback, this);  // 1HZ
    }
    if (topics_.find("pumpMotorSpeed_chatter") != topics_.end()) {
      sub_pump_motor_speed_ = hj_bf::HJSubscribe("pumpMotorSpeed_chatter", 100,
        &RecordMsg::PumpMotorSpeedCallback, this);
    }
    if (topics_.find("fanMotorSpeed_chatter") != topics_.end()) {
      sub_fan_motor_speed_ = hj_bf::HJSubscribe("fanMotorSpeed_chatter", 100, &RecordMsg::FanMotorSpeedCallback, this);
    }
    if (topics_.find("outwater_detect") != topics_.end()) {
      sub_out_water_ = hj_bf::HJSubscribe("outwater_detect", 100, &RecordMsg::OutWaterCallback, this);
    }
    if (topics_.find("triple_ultra") != topics_.end()) {
      sub_triple_ultra_ = hj_bf::HJSubscribe("triple_ultra", 100, &RecordMsg::TripleUltraCallback, this);
    }
#ifdef HJ_T1pro
    if (topics_.find("t1pro/left_tof") != topics_.end()) {
      sub_tof_ = hj_bf::HJSubscribe("t1pro/left_tof", 100, &RecordMsg::TofChatterCallback, this);
    }
    if (topics_.find("t1pro/down_ray") != topics_.end()) {
      sub_down_ray_ = hj_bf::HJSubscribe("t1pro/down_ray", 100, &RecordMsg::FallCallback, this);  // 10HZ
    }
    if (topics_.find("t1pro/dust_plug_detection_chatter") != topics_.end()) {
      sub_dust_plug_ = hj_bf::HJSubscribe("t1pro/dust_plug_detection_chatter", 100, &RecordMsg::DustPlugCallback, this);
    }
#else
    if (topics_.find("x9/left_back") != topics_.end()) {
      sub_left_back_ = hj_bf::HJSubscribe("x9/left_back", 100, &RecordMsg::LeftBackCallback, this);
    }
    if (topics_.find("x9/left_front") != topics_.end()) {
      sub_left_front_ = hj_bf::HJSubscribe("x9/left_front", 100, &RecordMsg::LeftFrontCallback, this);
    }
    if (machine_version_ == "P3.5") {
      if (topics_.find("x9/down_left") != topics_.end()) {
        sub_down_left_ = hj_bf::HJSubscribe("x9/down_left", 100, &RecordMsg::DownLeftCallback, this);
      }
      if (topics_.find("x9/down_right") != topics_.end()) {
        sub_down_right_ = hj_bf::HJSubscribe("x9/down_right", 100, &RecordMsg::DownRightCallback, this);
      }
    } else {
      if (topics_.find("x9/down_ray") != topics_.end()) {
        sub_down_ray_ = hj_bf::HJSubscribe("x9/down_ray", 100, &RecordMsg::FallCallback, this);  // 10HZ
      }
    }
#endif
  }

  return true;
}

void RecordMsg::WriteMotor(const hj_interface::Atime::ConstPtr &msg) {
  {
    std::lock_guard<std::mutex> msg_lock(mutex_space_);
    if (!space_enabled_) {
      return;
    }
  }
  char str[256] = {0};
  snprintf(str, sizeof(str), "%lf %lf %lu\n",
          msg->timestamp_current.toSec(), msg->timestamp_origin.toSec(), msg->index);
  write(fd_motor_time_, str, strlen(str));
}

void RecordMsg::WriteImu(const hj_interface::Atime::ConstPtr &msg) {
  {
    std::lock_guard<std::mutex> msg_lock(mutex_space_);
    if (!space_enabled_) {
      return;
    }
  }
  char str[256] = {0};
  snprintf(str, sizeof(str), "%lf %lf %lu\n",
          msg->timestamp_current.toSec(), msg->timestamp_origin.toSec(), msg->index);
  write(fd_imu_time_, str, strlen(str));
}

void RecordMsg::MotorChatterCallback(const hj_interface::Encoder::ConstPtr &msg) {
  {
    std::lock_guard<std::mutex> msg_lock(mutex_space_);
    if (!space_enabled_) {
      return;
    }
  }
  char str[256] = {0};
  snprintf(str, sizeof(str), "%lf %lu %f %f\n",
          msg->custom_time.toSec(), msg->index, msg->left_msg, msg->right_msg);
  write(fd_motor_, str, strlen(str));
}

void RecordMsg::TripleUltraCallback(const hj_interface::TripleUltra::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> msg_lock(mutex_space_);
    if (!space_enabled_) {
      return;
    }
  }
  char str[256] = {0};
  snprintf(str, sizeof(str), "%lf %u %u %u %u\n",
          msg->timestamp.toSec(), msg->front_l,
          msg->front_m, msg->front_r, msg->status);
  write(fd_triple_ultra_, str, strlen(str));
}

void RecordMsg::ImuChatterCallback(const hj_interface::Imu::ConstPtr &msg) {
  {
    std::lock_guard<std::mutex> msg_lock(mutex_space_);
    if (!space_enabled_) {
      return;
    }
  }
  char str[256] = {0};
  snprintf(str, sizeof(str), "%lf %ld %d %d %d %d %d %d %d %d %d\n",
          msg->custom_time.toSec(), msg->index,
          msg->roll, msg->pitch, msg->yaw, msg->gyro_x, msg->gyro_y,
          msg->gyro_z, msg->accel_x, msg->accel_y, msg->accel_z);
  write(fd_imu_, str, strlen(str));
}

void RecordMsg::SocImuChatterCallback(const hj_interface::SocImu::ConstPtr &msg) {
  {
    std::lock_guard<std::mutex> msg_lock(mutex_space_);
    if (!space_enabled_) {
      return;
    }
  }
  char str[256] = {0};
  snprintf(str, sizeof(str), "%lf %f %f %f %f %f %f\n",
          msg->timestamp.toSec(), msg->accel_x, msg->accel_y,
          msg->accel_z, msg->gyro_x, msg->gyro_y, msg->gyro_z);
  write(fd_soc_imu_, str, strlen(str));
}

#ifdef HJ_T1pro
void RecordMsg::TofChatterCallback(const hj_interface::LeftTof::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> msg_lock(mutex_space_);
    if (!space_enabled_) {
      return;
    }
  }

  char str[128] = {0};
  snprintf(str, sizeof(str), "%lf %u %u\n",
          msg->timestamp.toSec(), msg->dist_front, msg->dist_back);
  write(fd_tof_, str, strlen(str));
}

void RecordMsg::FallCallback(const hj_interface::DownRay::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> msg_lock(mutex_space_);
    if (!space_enabled_) {
      return;
    }
  }
  char str[128] = {0};
  snprintf(str, sizeof(str), "%lf %u %u\n",
          msg->timestamp.toSec(), msg->left_down, msg->right_down);
  write(fd_down_ray_, str, strlen(str));
}
void RecordMsg::DustPlugCallback(const hj_interface::DustPlugDetection::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> msg_lock(mutex_space_);
    if (!space_enabled_) {
      return;
    }
  }
  char str[128] = {0};
  snprintf(str, sizeof(str), "%lf %u %u\n",
          msg->timestamp.toSec(), msg->status1, msg->status2);
  write(fd_dust_plug_, str, strlen(str));
}
#else
void RecordMsg::LeftBackCallback(const hj_interface::LeftBack::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> msg_lock(mutex_space_);
    if (!space_enabled_) {
      return;
    }
  }
  char str[128] = {0};
  snprintf(str, sizeof(str), "%lf %u %u\n",
          msg->timestamp.toSec(), msg->dist, msg->status);
  write(fd_left_back_, str, strlen(str));
}

void RecordMsg::LeftFrontCallback(const hj_interface::LeftFront::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> msg_lock(mutex_space_);
    if (!space_enabled_) {
      return;
    }
  }
  char str[128] = {0};
  snprintf(str, sizeof(str), "%lf %u %u\n",
          msg->timestamp.toSec(), msg->dist, msg->status);
  write(fd_left_front_, str, strlen(str));
}

void RecordMsg::DownRightCallback(const hj_interface::DownRight::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> msg_lock(mutex_space_);
    if (!space_enabled_) {
      return;
    }
  }
  char str[128] = {0};
  snprintf(str, sizeof(str), "%lf %u %u\n",
          msg->timestamp.toSec(), msg->dist, msg->status);
  write(fd_down_right_, str, strlen(str));
}

void RecordMsg::DownLeftCallback(const hj_interface::DownLeft::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> msg_lock(mutex_space_);
    if (!space_enabled_) {
      return;
    }
  }
  char str[128] = {0};
  snprintf(str, sizeof(str), "%lf %u\n",
          msg->timestamp.toSec(), msg->ray_value);
  write(fd_down_left_, str, strlen(str));
}

void RecordMsg::FallCallback(const hj_interface::DownRay::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> msg_lock(mutex_space_);
    if (!space_enabled_) {
      return;
    }
  }
  char str[128] = {0};
  snprintf(str, sizeof(str), "%lf %u %u\n",
          msg->timestamp.toSec(), msg->left_down, msg->right_down);
  write(fd_down_ray_, str, strlen(str));
}
#endif

void RecordMsg::MagChatterCallback(const hj_interface::Mag::ConstPtr &msg) {
  {
    std::lock_guard<std::mutex> msg_lock(mutex_space_);
    if (!space_enabled_) {
      return;
    }
  }
  char str[128] = {0};
  snprintf(str, sizeof(str), "%lf %d %d %d\n",
          msg->custom_time.toSec(), msg->mag_x, msg->mag_y, msg->mag_z);
  write(fd_mag_, str, strlen(str));
}

void RecordMsg::MotorSetChatterCallback(const hj_interface::Nav::ConstPtr &msg) {
  {
    std::lock_guard<std::mutex> msg_lock(mutex_space_);
    if (!space_enabled_) {
      return;
    }
  }
  char str[256] = {0};
  snprintf(str, sizeof(str), "%lf %f %f\n",
          msg->custom_time.toSec(), msg->left_msg,
          msg->right_msg);
  write(fd_nav_, str, strlen(str));
}

void RecordMsg::PressureChatterCallback(const hj_interface::Depth::ConstPtr &msg) {
  {
    std::lock_guard<std::mutex> msg_lock(mutex_space_);
    if (!space_enabled_) {
      return;
    }
  }
  char str[256] = {0};
  snprintf(str, sizeof(str), "%lf %d %d\n",
          msg->timestamp.toSec(), msg->pressure, msg->temp);
  write(fd_pressure_, str, strlen(str));
}

void RecordMsg::MotorCurCallback(const hj_interface::ElecMotorCur::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> msg_lock(mutex_space_);
    if (!space_enabled_) {
      return;
    }
  }
  ros::Time time_now = msg->custom_time;
  std::time_t time_c = time_now.sec;
  std::tm *ptm = std::localtime(&time_c);
  std::ostringstream oss;
  oss << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");

  char str[256] = {0};
  snprintf(str, sizeof(str), "%s %d %d %d %d %d %d %d %d\n", oss.str().c_str(),
          msg->motor_l, msg->motor_r, msg->pump_l, msg->pump_r, msg->turn, msg->dirtybox,
          msg->flipcover, msg->airbag);
  write(fd_motor_cur_, str, strlen(str));
}

void RecordMsg::TurbidityCallback(const hj_interface::Turbidity::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> msg_lock(mutex_space_);
    if (!space_enabled_) {
      return;
    }
  }
  char str[256] = {0};
  snprintf(str, sizeof(str), "%lf %d\n", msg->timestamp.toSec(), msg->turbidity);
  write(fd_tur_, str, strlen(str));
}

void RecordMsg::TempHumidityCallback(const hj_interface::TempHumidity::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> msg_lock(mutex_space_);
    if (!space_enabled_) {
      return;
    }
  }
  ros::Time time_now = ros::Time::now();
  std::time_t time_c = time_now.sec;
  std::tm *ptm = std::localtime(&time_c);
  std::ostringstream oss;
  oss << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");

  char str[128] = {0};
  snprintf(str, sizeof(str), "%s %d %d\n", oss.str().c_str(), msg->humidity, msg->temperature);
  write(fd_temp_humidity_, str, strlen(str));
}

void RecordMsg::BatCallback(const hj_interface::Bat::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> msg_lock(mutex_space_);
    if (!space_enabled_) {
      return;
    }
  }
  ros::Time time_now = ros::Time::now();
  std::time_t time_c = time_now.sec;
  std::tm *ptm = std::localtime(&time_c);
  std::ostringstream oss;
  oss << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");

  char str[128] = {0};
  snprintf(str, sizeof(str), "%s %d %d %d %d %d %d %d %d %d %d\n",
          oss.str().c_str(), msg->power, msg->temp1, msg->temp2, msg->temp3,
          msg->bat_vol, msg->bat_disch_cur, msg->ch_vol, msg->charger_ch_cur,
          msg->bat_cycle_times, msg->bat_health_left);
  write(fd_bat_, str, strlen(str));
}

void RecordMsg::PumpMotorSpeedCallback(const hj_interface::PumpMotorSpeed::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> msg_lock(mutex_space_);
    if (!space_enabled_) {
      return;
    }
  }
  ros::Time time_now = msg->timestamp;
  std::time_t time_c = time_now.sec;
  std::tm *ptm = std::localtime(&time_c);
  std::ostringstream oss;
  oss << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");

  char str[128] = {0};
  snprintf(str, sizeof(str), "%s %d %d\n", oss.str().c_str(), msg->speed_l, msg->speed_r);
  write(fd_pump_motor_speed_, str, strlen(str));
}

void RecordMsg::FanMotorSpeedCallback(const hj_interface::FanMotorSpeed::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> msg_lock(mutex_space_);
    if (!space_enabled_) {
      return;
    }
  }
  ros::Time time_now = msg->timestamp;
  std::time_t time_c = time_now.sec;
  std::tm *ptm = std::localtime(&time_c);
  std::ostringstream oss;
  oss << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");

  char str[128] = {0};
  snprintf(str, sizeof(str), "%s %d\n", oss.str().c_str(), msg->speed);
  write(fd_fan_motor_speed_, str, strlen(str));
}

void RecordMsg::OutWaterCallback(const std_msgs::UInt8::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> msg_lock(mutex_space_);
    if (!space_enabled_) {
      return;
    }
  }
  ros::Time time_now = ros::Time::now();
  std::time_t time_c = time_now.sec;
  std::tm *ptm = std::localtime(&time_c);
  std::ostringstream oss;
  oss << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");

  char str[128] = {0};
  snprintf(str, sizeof(str), "%s %d\n", oss.str().c_str(), msg->data);
  write(fd_out_water_, str, strlen(str));
}

RecordMsg::~RecordMsg() {
  CloseFiles();
}

void RecordMsg::CloseFiles() {
#ifdef HJ_T1pro
  ::close(fd_tof_);
  ::close(fd_dust_plug_);
#else
  ::close(fd_left_back_);
  ::close(fd_left_front_);
  ::close(fd_down_left_);
  ::close(fd_down_right_);
#endif
  ::close(fd_down_ray_);
  ::close(fd_motor_);
  ::close(fd_imu_);
  ::close(fd_mag_);
  ::close(fd_nav_);
  ::close(fd_pressure_);
  ::close(fd_soc_imu_);
  ::close(fd_motor_cur_);
  ::close(fd_tur_);
  ::close(fd_temp_humidity_);
  ::close(fd_bat_);
}


void RecordMsg::ScheduledCheckDisk() {
  while (true) {
    int32_t dir_size = static_cast<int32_t>(GetDirectorySize(log_prefix_.data()));
    if (dir_size >= space_limit_) {
      std::lock_guard<std::mutex> msg_lock(mutex_space_);
      space_enabled_ = false;
    } else {
      std::lock_guard<std::mutex> msg_lock(mutex_space_);
      space_enabled_ = true;
    }
    std::this_thread::sleep_for(std::chrono::seconds(10));
  }
}

float RecordMsg::GetDirectorySize(const char *dir) {
  struct dirent *entry = nullptr;
  struct stat statbuf;
  float totalSize = 0.0;
  DIR *dp = opendir(dir);
  if (dp == nullptr) {
    HJ_ERROR("Cannot open dir: %s\n", dir);   // 目录不存在
    return 0.0;
  }

  // 先加上自身目录的大小
  lstat(dir, &statbuf);
  totalSize += statbuf.st_size;

  while ((entry = readdir(dp)) != NULL) {
    char subdir[256];
    int res = snprintf(subdir, sizeof(subdir), "%s/%s", dir, entry->d_name);
    if (res < 0 || res >= 256) {
      HJ_ERROR("Failed to construct subdir path.");
      continue;
    }
    lstat(subdir, &statbuf);

    if (S_ISDIR(statbuf.st_mode)) {
      if (strcmp(".", entry->d_name) == 0 ||
        strcmp("..", entry->d_name) == 0) {
        continue;
      }

      float subDirSize = GetDirectorySize(subdir);
      totalSize += subDirSize;
    } else {
      totalSize += statbuf.st_size;
    }
  }

  closedir(dp);
  return totalSize / 1024 / 1024;
}

bool RecordMsg::CheckDisk() {
  uint64_t value_1M = 1048576ull;
  struct statvfs fiData{};
  if ((statvfs(log_prefix_.data(), &fiData)) < 0) {
    HJ_WARN("Failed to check filesystem stats.");
    return true;
  }
  uint64_t free_space = 0;
  free_space = static_cast<uint64_t>(fiData.f_bsize) * static_cast<uint64_t>(fiData.f_bavail);
  if (free_space < space_limit_ * value_1M)  {   // 30MB
    HJ_ERROR("Less than %s of space free on disk with '%s'.  Disabling recording.", "30MB", log_prefix_.data());
    std::lock_guard<std::mutex> msg_lock(mutex_);
    writing_enabled_ = false;
    return false;
  } else if (free_space < 3 * space_limit_ * value_1M) {
    HJ_WARN("Less than  %d of space free on disk with '%s'.", 3 * space_limit_, log_prefix_.data());
  } else {
    std::lock_guard<std::mutex> msg_lock(mutex_);
    writing_enabled_ = true;
  }
  return true;
}

RecordMsg::RecordMsg(const rapidjson::Value &json_conf) : hj_bf::Function(json_conf) {
  if (json_conf.HasMember("space_limit") && json_conf["space_limit"].IsInt()) {
    space_limit_ = json_conf["space_limit"].GetInt();
  }
  if (json_conf.HasMember("machine_version") && json_conf["machine_version"].IsString()) {
    machine_version_ = json_conf["machine_version"].GetString();
  }
  if (json_conf.HasMember("topics") && json_conf["topics"].IsArray()) {
    for (auto &topic : json_conf["topics"].GetArray()) {
      if (topic.IsString()) {
        if (strcmp(topic.GetString(), "all") == 0) {
          record_all_topics_ = true;
          break;
        }
        topics_.emplace(std::make_pair(topic.GetString(), true));
      }
    }
  }

  if (json_conf.HasMember("enable") && json_conf["enable"].IsInt()) {
    int record_enable = json_conf["enable"].GetInt();
    if (record_enable == 1) {
      if (json_conf.HasMember("dir") && json_conf["dir"].IsString()) {
        log_prefix_ = json_conf["dir"].GetString();
      } else {
        log_prefix_ = "/userdata/hj/log/sensor_data";
      }
      if (initizlize()) {
        auto state = std::thread(&RecordMsg::ScheduledCheckDisk, this);  // 开线程
        state.detach();
      } else {
        HJ_ERROR("Failed to initizlize RecordMsg.");
        CloseFiles();
      }
    }
  }
  HJ_INFO("RecordMsg loaded success\n");
}
}  // namespace collect_node_record_msg
