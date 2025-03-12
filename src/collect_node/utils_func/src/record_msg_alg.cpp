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
#include "record_msg_alg.h"
#include "log.h"
#include "hjlog.h"
#include "boost/filesystem.hpp"
#include "hj_interface/FileUpload.h"

namespace collect_node_utils_func {

void RecordMsg::SubTopic() {
  imu_time_sub_ = hj_bf::HJSubscribe("imu_time_chatter", 100, &RecordMsg::WriteImu, this);  // 50HZ
  motor_tmie_sub_ = hj_bf::HJSubscribe("motor_time_chatter", 100, &RecordMsg::WriteMotor, this);  // 50HZ
  sub_motor_ = hj_bf::HJSubscribe("motor_chatter", 100, &RecordMsg::MotorChatterCallback, this);  // 50HZ
  sub_imu_ = hj_bf::HJSubscribe("imu_chatter", 100, &RecordMsg::ImuChatterCallback, this);  // 100HZ
  sub_mag_ = hj_bf::HJSubscribe("mag_chatter", 100, &RecordMsg::MagChatterCallback, this);  // 100HZ
  sub_motor_set_ = hj_bf::HJSubscribe("nav_motor", 100, &RecordMsg::MotorSetChatterCallback, this);
  sub_pressure_ = hj_bf::HJSubscribe("depth_chatter", 100, &RecordMsg::PressureChatterCallback, this);  // 20HZ
  sub_triple_ultra_ = hj_bf::HJSubscribe("triple_ultra", 100, &RecordMsg::TripleUltraCallback, this);
  sub_left_back_ = hj_bf::HJSubscribe("x9/left_back", 100, &RecordMsg::LeftBackCallback, this);
  sub_left_front_ = hj_bf::HJSubscribe("x9/left_front", 100, &RecordMsg::LeftFrontCallback, this);
#ifdef HJ_T1pro
  sub_tof_ = hj_bf::HJSubscribe("t1pro/left_tof", 100, &RecordMsg::TofChatterCallback, this);
  sub_down_ray_ = hj_bf::HJSubscribe("t1pro/down_ray", 100, &RecordMsg::FallCallback, this);  // 10HZ
#else
  sub_down_left_ = hj_bf::HJSubscribe("x9/down_left", 100, &RecordMsg::DownLeftCallback, this);
  sub_down_right_ = hj_bf::HJSubscribe("x9/down_right", 100, &RecordMsg::DownRightCallback, this);
#endif
  sub_motor_cur_ = hj_bf::HJSubscribe("motor_cur", 10, &RecordMsg::MotorCurCallback, this);
  sub_wireless_charging_ = hj_bf::HJSubscribe("/wireless_charging_chatter", 10, &RecordMsg::WirelessChargingCallback, this);
  // upload_sub_ = hj_bf::HJSubscribe("/upload_msg", 1, &RecordMsg::UploadCallback, this);
  // pub_upload_ = hj_bf::HJAdvertise<hj_interface::FileUpload>("/upload/file", 10);
}

bool RecordMsg::OpenLogFile(const std::string& log_prefix) {
  // define log file name
  std::string motor = log_prefix + "/encoder.log";
  std::string imu = log_prefix + "/imu.log";
  std::string triple_ultra = log_prefix + "/triple_ultra.log";
  std::string mag = log_prefix + "/mag.log";
  std::string nav = log_prefix + "/nav.log";
  std::string pressure = log_prefix + "/pressure.log";
  std::string left_back = log_prefix + "/left_back.log";
  std::string left_front = log_prefix + "/left_front.log";
#ifdef HJ_T1pro
  std::string left_tof = log_prefix + "/left_tof.log";
  std::string down_ray = log_prefix + "/down_ray.log";
#else
  std::string down_right = log_prefix + "/down_right.log";
  std::string down_left = log_prefix + "/down_left.log";
#endif
  std::string motor_time = log_prefix + "/motor_time.log";
  std::string imu_time = log_prefix + "/imu_time.log";

  stream_motor_ = std::ofstream(motor.data(), std::ios::out | std::ios::trunc);
  if (!stream_motor_.is_open()) {
    HJ_ERROR("open Encoder.log fail\n");
    return false;
  }

  stream_imu_ = std::ofstream(imu.data(), std::ios::out | std::ios::trunc);
  if (!stream_imu_.is_open()) {
    HJ_ERROR("open imu.log fail\n");
    return false;
  }

  stream_triple_ultra_ = std::ofstream(triple_ultra.data(), std::ios::out | std::ios::trunc);
  if (!stream_triple_ultra_.is_open()) {
    HJ_ERROR("open triple_ultra.log fail\n");
    return false;
  }

  stream_left_back_ = std::ofstream(left_back.data(), std::ios::out | std::ios::trunc);
  if (!stream_left_back_.is_open()) {
    HJ_ERROR("open left_back.log fail\n");
    return false; 
  }

  stream_left_front_ = std::ofstream(left_front.data(), std::ios::out | std::ios::trunc);
  if (!stream_left_front_.is_open()) {
    HJ_ERROR("open left_front.log fail\n");
    return false;
  }

#ifdef HJ_T1pro
  stream_tof_ = std::ofstream(left_tof.data(), std::ios::out | std::ios::trunc);
  if (!stream_tof_.is_open()) {
    HJ_ERROR("open left_tof.log fail\n");
    return false;
  }

  stream_down_ray_ = std::ofstream(down_ray.data(), std::ios::out | std::ios::trunc);
  if (!stream_down_ray_.is_open()) {
    HJ_ERROR("open down_ray.log fail\n");
    return false;
  }
#else
  stream_down_left_ = std::ofstream(down_left.data(), std::ios::out | std::ios::trunc);
  if (!stream_down_left_.is_open()) {
    HJ_ERROR("open down_left.log fail\n");
    return false;
  }

  stream_down_right_ = std::ofstream(down_right.data(), std::ios::out | std::ios::trunc);
  if (!stream_down_right_.is_open()) {
    HJ_ERROR("open down_right.log fail\n");
    return false;
  }
#endif

  stream_mag_ = std::ofstream(mag.data(), std::ios::out | std::ios::trunc);
  if (!stream_mag_.is_open()) {
    HJ_ERROR("open mag.log fail\n");
    return false;
  }

  stream_nav_ = std::ofstream(nav.data(), std::ios::out | std::ios::trunc);
  if (!stream_nav_.is_open()) {
    HJ_ERROR("open nav.log fail\n");
    return false;
  }

  stream_pressure_ = std::ofstream(pressure.data(), std::ios::out | std::ios::trunc);
  if (!stream_pressure_.is_open()) {
    HJ_ERROR("open pressure.log fail\n");
    return false;
  }

  stream_motor_time_ = std::ofstream(motor_time.data(), std::ios::out | std::ios::trunc);
  if (!stream_motor_time_.is_open()) {
    HJ_ERROR("open motor_time.log fail\n");
    return false;
  }

  stream_imu_time_ = std::ofstream(imu_time.data(), std::ios::out | std::ios::trunc);
  if (!stream_imu_time_.is_open()) {
    HJ_ERROR("open imu_time.log fail\n");
    return false;
  }

  return true;
}

void RecordMsg::MotorCurCallback(const hj_interface::ElecMotorCur::ConstPtr& msg) {
  ros::Time time_now = msg->custom_time;
  std::time_t time_c = time_now.sec;
  std::tm *ptm = std::localtime(&time_c);
  std::ostringstream oss;
  oss << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");
#ifdef HJ_RELEASE_VER
  static int motor_cur_count = 0;
  motor_cur_count++;
  if (motor_cur_count == 30) {
    msg_append(motor_cur_logger_, "%s %d %d %d %d %d %d %d %d\n", oss.str().c_str(),
          msg->motor_l, msg->motor_r, msg->pump_l,
          msg->pump_r, msg->turn, msg->dirtybox, msg->flipcover, msg->airbag);
    motor_cur_count = 0;
  }
#else
  msg_append(motor_cur_logger_, "%s %d %d %d %d %d %d %d %d\n", oss.str().c_str(),
          msg->motor_l, msg->motor_r, msg->pump_l,
          msg->pump_r, msg->turn, msg->dirtybox, msg->flipcover, msg->airbag);
#endif
}

void RecordMsg::WirelessChargingCallback(const hj_interface::WirelessCharging::ConstPtr& msg) {
  ros::Time time_now = ros::Time::now();
  std::time_t time_c = time_now.sec;
  std::tm *ptm = std::localtime(&time_c);
  std::ostringstream oss;
  oss << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");  // format time string
#ifdef HJ_RELEASE_VER
  static int wireless_charging_count = 0;
  wireless_charging_count++;
  if (wireless_charging_count == 3) {
    msg_append(wireless_charging_logger_, "%s %d %d %d %d %d %d %d %d %d %d\n", oss.str().c_str(),
          msg->bridge_circuit_vol, msg->charger_ch_vol, msg->bridge_circuit_cur,
          msg->charger_ch_cur, msg->bridge_circuit_temp, msg->coil_temp,
          msg->charging_cradle_vol, msg->charging_cradle_cur, msg->charging_cradle_temp, msg->charging_cradle_status);
    wireless_charging_count = 0;
  }
#else
  msg_append(wireless_charging_logger_, "%s %d %d %d %d %d %d %d %d %d %d\n", oss.str().c_str(),
          msg->bridge_circuit_vol, msg->charger_ch_vol, msg->bridge_circuit_cur,
          msg->charger_ch_cur, msg->bridge_circuit_temp, msg->coil_temp,
          msg->charging_cradle_vol, msg->charging_cradle_cur, msg->charging_cradle_temp, msg->charging_cradle_status);
#endif
}

void RecordMsg::WriteMotor(const hj_interface::Atime::ConstPtr &msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[256] = {0};
  snprintf(str, sizeof(str), "%lf %lf %lu\n",
          msg->timestamp_current.toSec(), msg->timestamp_origin.toSec(), msg->index);
  stream_motor_time_ << str;
  stream_motor_time_.flush();
}

void RecordMsg::WriteImu(const hj_interface::Atime::ConstPtr &msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[256] = {0};
  snprintf(str, sizeof(str), "%lf %lf %lu %u\n",
          msg->timestamp_current.toSec(), msg->timestamp_origin.toSec(), msg->index, msg->flag);
  stream_imu_time_ << str;
  stream_imu_time_.flush();
}

void RecordMsg::MotorChatterCallback(const hj_interface::Encoder::ConstPtr &msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[256] = {0};
  snprintf(str, sizeof(str), "%lf %lu %f %f\n",
          msg->custom_time.toSec(), msg->index, msg->left_msg, msg->right_msg);
  stream_motor_ << str;
  stream_motor_.flush();
}

void RecordMsg::TripleUltraCallback(const hj_interface::TripleUltra::ConstPtr& msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[256] = {0};
  snprintf(str, sizeof(str), "%lf %u %u %u %u\n",
          msg->timestamp.toSec(), msg->front_l,
          msg->front_m, msg->front_r, msg->status);
  stream_triple_ultra_ << str;
  stream_triple_ultra_.flush();
}

void RecordMsg::ImuChatterCallback(const hj_interface::Imu::ConstPtr &msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[256] = {0};
  snprintf(str, sizeof(str), "%lf %ld %d %d %d %d %d %d %d %d %d %d\n",
          msg->custom_time.toSec(), msg->index,
          msg->roll, msg->pitch, msg->yaw, msg->gyro_x, msg->gyro_y,
          msg->gyro_z, msg->accel_x, msg->accel_y, msg->accel_z, msg->flag);
  stream_imu_ << str;
  stream_imu_.flush();
}

void RecordMsg::LeftBackCallback(const hj_interface::LeftBack::ConstPtr& msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[128] = {0};
  snprintf(str, sizeof(str), "%lf %u %u\n",
          msg->timestamp.toSec(), msg->dist, msg->status);
  stream_left_back_ << str;
  stream_left_back_.flush();
}

void RecordMsg::LeftFrontCallback(const hj_interface::LeftFront::ConstPtr& msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[128] = {0};
  snprintf(str, sizeof(str), "%lf %u %u\n",
          msg->timestamp.toSec(), msg->dist, msg->status);
  stream_left_front_ << str;
  stream_left_front_.flush();
}

#ifdef HJ_T1pro
void RecordMsg::TofChatterCallback(const hj_interface::LeftTof::ConstPtr& msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }

  char str[128] = {0};
  snprintf(str, sizeof(str), "%lf %u %u\n",
          msg->timestamp.toSec(), msg->dist_front, msg->dist_back);
  stream_tof_ << str;
  stream_tof_.flush();
}

void RecordMsg::FallCallback(const hj_interface::DownRay::ConstPtr& msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[128] = {0};
  snprintf(str, sizeof(str), "%lf %u %u\n",
          msg->timestamp.toSec(), msg->left_down, msg->right_down);
  stream_down_ray_ << str;
  stream_down_ray_.flush();
}
#else

void RecordMsg::DownRightCallback(const hj_interface::DownRight::ConstPtr& msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[128] = {0};
  snprintf(str, sizeof(str), "%lf %u %u\n",
          msg->timestamp.toSec(), msg->dist, msg->status);
  stream_down_right_ << str;
  stream_down_right_.flush();
}

void RecordMsg::DownLeftCallback(const hj_interface::DownLeft::ConstPtr& msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[128] = {0};
  snprintf(str, sizeof(str), "%lf %u\n",
          msg->timestamp.toSec(), msg->ray_value);
  stream_down_left_ << str;
  stream_down_left_.flush();
}
#endif

void RecordMsg::MagChatterCallback(const hj_interface::Mag::ConstPtr &msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[128] = {0};
  snprintf(str, sizeof(str), "%lf %d %d %d\n",
          msg->custom_time.toSec(), msg->mag_x, msg->mag_y, msg->mag_z);
  stream_mag_ << str;
  stream_mag_.flush();
}

void RecordMsg::MotorSetChatterCallback(const hj_interface::Nav::ConstPtr &msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[256] = {0};
  snprintf(str, sizeof(str), "%lf %f %f\n",
          msg->custom_time.toSec(), msg->left_msg,
          msg->right_msg);
  stream_nav_ << str;
  stream_nav_.flush();
}

void RecordMsg::PressureChatterCallback(const hj_interface::Depth::ConstPtr &msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[256] = {0};
  snprintf(str, sizeof(str), "%lf %d %d\n",
          msg->timestamp.toSec(), msg->pressure, msg->temp);
  stream_pressure_ << str;
  stream_pressure_.flush();
}

RecordMsg::~RecordMsg() {
  CloseFiles();
  if (motor_cur_logger_ != nullptr) {
    hj_cst_log_del(motor_cur_logger_);
  }
  if (wireless_charging_logger_ != nullptr) {
    hj_cst_log_del(wireless_charging_logger_);
  }
}

void RecordMsg::CloseFiles() {
#ifdef HJ_T1pro
  stream_tof_.close();
  stream_down_ray_.close();
#else
  stream_down_left_.close();
  stream_down_right_.close();
#endif
  stream_triple_ultra_.close();
  stream_left_front_.close();
  stream_left_back_.close();
  stream_motor_.close();
  stream_imu_.close();
  stream_mag_.close();
  stream_nav_.close();
  stream_pressure_.close();
  stream_motor_time_.close();
  stream_imu_time_.close();
}


void RecordMsg::ScheduledCheckDisk() {
  std::string log_path = "";
  while (true) {
    int32_t dir_size = static_cast<int32_t>(GetDirectorySize(log_prefix_.data()) / 1024 / 1024);
    if (dir_size >= space_limit_) {
      space_enabled_.store(false);
      // CloseFiles();
      // HJ_WARN("Disk space is full, stop recording.");
    } else {
      space_enabled_.store(true);
    }
    std::this_thread::sleep_for(std::chrono::seconds(30));
  }
}

uint64_t RecordMsg::GetDirectorySize(const char *dir) {
  struct dirent *entry = nullptr;
  struct stat statbuf;
  uint64_t totalSize = 0;
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

      uint64_t subDirSize = GetDirectorySize(subdir);
      totalSize += subDirSize;
    } else {
      totalSize += statbuf.st_size;
    }
  }

  closedir(dp);
  return totalSize;
}

void RecordMsg::StartRecording(const std::string& path) {
  if (access(path.c_str(), 0) == -1) {  // check folder exist
    int ret = mkdir(path.c_str(), S_IRWXU);  // create folder
    if (ret == -1) {
      HJ_ERROR("mkdir %s fail\n", path.c_str());
      return;
    }
  } else {
    boost::filesystem::path path1(path);
    boost::filesystem::remove_all(path1);
    int ret = mkdir(path.c_str(), S_IRWXU);  // create folder
    if (ret == -1) {
      HJ_ERROR("mkdir %s fail\n", path.c_str());
      return;
    }
  }
  HJ_INFO("Start recording log to %s\n", path.c_str());

  // 打开文件
  if (OpenLogFile(path)) {
    writing_enabled_.store(true);
  } else {
    HJ_ERROR("Open log file fail\n");
  }
  HJ_INFO("Start recording log success\n");
}

void RecordMsg::FinishTask() {
  // 关闭文件
  writing_enabled_.store(false);
  CloseFiles();
}

void RecordMsg::DealTaskCallback(const hj_interface::SensorDataRecord::ConstPtr& msg) {
  // 这里是处理任务的函数
  // 11: 水面任务  13: 池壁任务 14：水线任务 15：池底任务 35: 结束当前任务录制  其他：未知小任务
  writing_enabled_.store(false);
  switch (msg->action_cmd) {
    case 11: {
      // 开始水面任务录制
      StartRecording(log_prefix_ + "/data/water_surface");
      HJ_INFO("Start recording water surface task.");
      break;
    }
    case 13: {
      // 开始池壁任务录制
      StartRecording(log_prefix_ + "/data/pool_wall");
      HJ_INFO("Start recording pool wall task.");
      break;
    }
    case 14: {
      // 开始水线任务录制
      StartRecording(log_prefix_ + "/data/water_line");
      HJ_INFO("Start recording water line task.");
      break;
    }
    case 15: {
      // 开始池底任务录制
      StartRecording(log_prefix_ + "/data/pool_bottom");
      HJ_INFO("Start recording pool bottom task.");
      break;
    }
    case 35:
    case 36: {
      // 结束当前任务录制
      FinishTask();
      HJ_INFO("Stop recording current task.");
      break;
    }
    default:
      // 开始其他任务录制
      StartRecording(log_prefix_ + "/data/other_task");
      HJ_INFO("Start recording other_task.");
      break;
  }
}

void RecordMsg::UploadCallback(const std_msgs::Bool::ConstPtr& msg) {
  // 上传文件
  // std::string log_path = "";
  // {
  //   std::lock_guard<std::mutex> lock(mtx_);
  //   log_path = cur_log_path_;
  // }
  // std::string zip_file_name = log_path + "/data.zip";
  // bool ret = hj_bf::CreateZipFileByDir(zip_file_name, log_path);
  // if (ret) {
  //   hj_interface::FileUpload file_upload_msg;
  //   file_upload_msg.type = hj_interface::FileUpload::DEVICELOG;
  //   file_upload_msg.filePath = zip_file_name;
  //   file_upload_msg.deleteOnSuccess = 1;
  //   pub_upload_.publish(file_upload_msg);
  // }
}

void RecordMsg::Init(uint8_t task_enable) {
  if (access(log_prefix_.c_str(), 0) == -1) {  // check folder exist
    int ret = mkdir(log_prefix_.c_str(), S_IRWXU);  // create folder
    if (ret == -1) {
      HJ_ERROR("mkdir %s fail\n", log_prefix_.c_str());
      return;
    }
  }
  std::string data_path = log_prefix_ + "/data";
  if (access(data_path.c_str(), 0) == -1) {  // check folder exist
    int ret = mkdir(data_path.c_str(), S_IRWXU);  // create folder
    if (ret == -1) {
      HJ_ERROR("mkdir %s fail\n", data_path.c_str());
      return;
    }
  } else {
#ifndef HJ_RELEASE_VER
    boost::filesystem::path path(data_path);
    boost::filesystem::remove_all(path);
    int ret = mkdir(data_path.c_str(), S_IRWXU);  // create folder
    if (ret == -1) {
      HJ_ERROR("mkdir %s fail\n", data_path.c_str());
      return;
    }
#endif
  }

  if (task_enable == 0) {
    StartRecording(data_path);
  } else {
    // task sub
    task_sub_ = hj_bf::HJSubscribe("sensor_data_record", 10, &RecordMsg::DealTaskCallback, this);
  }

  std::string motor_cur_log_path = log_prefix_ + "/motor_cur.log";
#ifdef HJ_RELEASE_VER
  int size = 2*1024*1024;
#else
  int size = 10*1024*1024;
#endif
  motor_cur_logger_ = hj_cst_log_add(motor_cur_log_path.data(), INFO_LOG, size, 3);
  if (motor_cur_logger_ == nullptr) {
    HJ_ERROR("Failed to create motor_cur logger.");
    return;
  }

  std::string wireless_charging_log_path = log_prefix_ + "/wireless_charging.log";
  wireless_charging_logger_ = hj_cst_log_add(wireless_charging_log_path.data(), INFO_LOG, size, 3);
  if (wireless_charging_logger_ == nullptr) {
    HJ_ERROR("Failed to create wireless_charging_logger_ logger.");
    return;
  }

  SubTopic();
}

void RecordMsg::createInstance(const recordMsgParam& param) {
  space_limit_ = param.space_limit;
  log_prefix_ = param.record_path;
  uint8_t task_enable = param.task_enable;
  
  auto thread_init = std::thread(&RecordMsg::Init, this, task_enable);  // 开线程
  thread_init.detach();

  auto state = std::thread(&RecordMsg::ScheduledCheckDisk, this);  // 开线程
  state.detach();

  HJ_INFO("RecordMsg loaded success\n");
}
}  // namespace collect_node_utils_func
