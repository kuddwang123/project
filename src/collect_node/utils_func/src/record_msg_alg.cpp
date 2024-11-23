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
  // task sub
  task_sub_ = hj_bf::HJSubscribe("sensor_data_record", 10, &RecordMsg::DealTaskCallback, this);
  upload_sub_ = hj_bf::HJSubscribe("/upload_msg", 1, &RecordMsg::UploadCallback, this);
  pub_upload_ = hj_bf::HJAdvertise<hj_interface::FileUpload>("/upload/file", 10);
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
#else
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

  fd_motor_time_ = ::open(motor_time.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);
  fd_imu_time_ = ::open(imu_time.data(), O_RDWR | O_CREAT | O_TRUNC, 0666);

  return true;
}

void RecordMsg::WriteMotor(const hj_interface::Atime::ConstPtr &msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[256] = {0};
  snprintf(str, sizeof(str), "%lf %lf %lu\n",
          msg->timestamp_current.toSec(), msg->timestamp_origin.toSec(), msg->index);
  if (write(fd_motor_time_, str, strlen(str)) < 0) {
    HJ_ERROR("WriteMotor Failed to write to file.");
  }
}

void RecordMsg::WriteImu(const hj_interface::Atime::ConstPtr &msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[256] = {0};
  snprintf(str, sizeof(str), "%lf %lf %lu %u\n",
          msg->timestamp_current.toSec(), msg->timestamp_origin.toSec(), msg->index, msg->flag);
  if (write(fd_imu_time_, str, strlen(str)) < 0) {
    HJ_ERROR("WriteImu Failed to write to file.");
  }
}

void RecordMsg::MotorChatterCallback(const hj_interface::Encoder::ConstPtr &msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[256] = {0};
  snprintf(str, sizeof(str), "%lf %lu %f %f\n",
          msg->custom_time.toSec(), msg->index, msg->left_msg, msg->right_msg);
  if (write(fd_motor_, str, strlen(str)) < 0) {
    HJ_ERROR("MotorChatterCallback Failed to write to file.");
  }
}

void RecordMsg::TripleUltraCallback(const hj_interface::TripleUltra::ConstPtr& msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[256] = {0};
  snprintf(str, sizeof(str), "%lf %u %u %u %u\n",
          msg->timestamp.toSec(), msg->front_l,
          msg->front_m, msg->front_r, msg->status);
  if (write(fd_triple_ultra_, str, strlen(str)) < 0) {
    HJ_ERROR("TripleUltraCallback Failed to write to file.");
  }
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
  if (write(fd_imu_, str, strlen(str)) < 0) {
    HJ_ERROR("ImuChatterCallback Failed to write to file.");
  }
}

void RecordMsg::LeftBackCallback(const hj_interface::LeftBack::ConstPtr& msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[128] = {0};
  snprintf(str, sizeof(str), "%lf %u %u\n",
          msg->timestamp.toSec(), msg->dist, msg->status);
  if (write(fd_left_back_, str, strlen(str)) < 0) {
    HJ_ERROR("LeftBackCallback Failed to write to file.");
  }
}

void RecordMsg::LeftFrontCallback(const hj_interface::LeftFront::ConstPtr& msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[128] = {0};
  snprintf(str, sizeof(str), "%lf %u %u\n",
          msg->timestamp.toSec(), msg->dist, msg->status);
  if (write(fd_left_front_, str, strlen(str)) < 0) {
    HJ_ERROR("LeftFrontCallback Failed to write to file.");
  }
}

#ifdef HJ_T1pro
void RecordMsg::TofChatterCallback(const hj_interface::LeftTof::ConstPtr& msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }

  char str[128] = {0};
  snprintf(str, sizeof(str), "%lf %u %u\n",
          msg->timestamp.toSec(), msg->dist_front, msg->dist_back);
  if (write(fd_tof_, str, strlen(str)) < 0) {
    HJ_ERROR("TofChatterCallback Failed to write to file.");
  }
}

void RecordMsg::FallCallback(const hj_interface::DownRay::ConstPtr& msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[128] = {0};
  snprintf(str, sizeof(str), "%lf %u %u\n",
          msg->timestamp.toSec(), msg->left_down, msg->right_down);
  if (write(fd_down_ray_, str, strlen(str)) < 0) {
    HJ_ERROR("FallCallback Failed to write to file.");
  }
}
#else

void RecordMsg::DownRightCallback(const hj_interface::DownRight::ConstPtr& msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[128] = {0};
  snprintf(str, sizeof(str), "%lf %u %u\n",
          msg->timestamp.toSec(), msg->dist, msg->status);
  if (write(fd_down_right_, str, strlen(str)) < 0) {
    HJ_ERROR("DownRightCallback Failed to write to file.");
  }
}

void RecordMsg::DownLeftCallback(const hj_interface::DownLeft::ConstPtr& msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[128] = {0};
  snprintf(str, sizeof(str), "%lf %u\n",
          msg->timestamp.toSec(), msg->ray_value);
  if (write(fd_down_left_, str, strlen(str)) < 0) {
    HJ_ERROR("DownLeftCallback Failed to write to file.");
  }
}
#endif

void RecordMsg::MagChatterCallback(const hj_interface::Mag::ConstPtr &msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[128] = {0};
  snprintf(str, sizeof(str), "%lf %d %d %d\n",
          msg->custom_time.toSec(), msg->mag_x, msg->mag_y, msg->mag_z);
  if (write(fd_mag_, str, strlen(str)) < 0) {
    HJ_ERROR("MagChatterCallback Failed to write to file.");
  }
}

void RecordMsg::MotorSetChatterCallback(const hj_interface::Nav::ConstPtr &msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[256] = {0};
  snprintf(str, sizeof(str), "%lf %f %f\n",
          msg->custom_time.toSec(), msg->left_msg,
          msg->right_msg);
  if (write(fd_nav_, str, strlen(str)) < 0) {
    HJ_ERROR("MotorSetChatterCallback Failed to write to file.");
  }
}

void RecordMsg::PressureChatterCallback(const hj_interface::Depth::ConstPtr &msg) {
  if (!writing_enabled_.load() || !space_enabled_.load()) {
    return;
  }
  char str[256] = {0};
  snprintf(str, sizeof(str), "%lf %d %d\n",
          msg->timestamp.toSec(), msg->pressure, msg->temp);
  if (write(fd_pressure_, str, strlen(str)) < 0) {
    HJ_ERROR("PressureChatterCallback Failed to write to file.");
  }
}

RecordMsg::~RecordMsg() {
  CloseFiles();
}

void RecordMsg::CloseFiles() {
#ifdef HJ_T1pro
  ::close(fd_tof_);
  ::close(fd_down_ray_);
#else
  ::close(fd_down_left_);
  ::close(fd_down_right_);
#endif
  ::close(fd_left_back_);
  ::close(fd_left_front_);
  ::close(fd_motor_);
  ::close(fd_imu_);
  ::close(fd_mag_);
  ::close(fd_nav_);
  ::close(fd_pressure_);
  ::close(fd_motor_time_);
  ::close(fd_imu_time_);
}


void RecordMsg::ScheduledCheckDisk() {
  std::string log_path = "";
  while (true) {
    {
      std::lock_guard<std::mutex> lock(mtx_);
      log_path = cur_log_path_;
    }
    int32_t dir_size = static_cast<int32_t>(GetDirectorySize(log_path.data()) / 1024 / 1024);
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
  switch (msg->action_cmd) {
    case 11: {
      // 开始水面任务录制
      StartRecording(log_prefix_ + "/water_surface");
      {
        std::lock_guard<std::mutex> lock(mtx_);
        cur_log_path_ = log_prefix_ + "/water_surface";
      }
      HJ_INFO("Start recording water surface task.");
      break;
    }
    case 13: {
      // 开始池壁任务录制
      StartRecording(log_prefix_ + "/pool_wall");
      {
        std::lock_guard<std::mutex> lock(mtx_);
        cur_log_path_ = log_prefix_ + "/pool_wall";
      }
      HJ_INFO("Start recording pool wall task.");
      break;
    }
    case 14: {
      // 开始水线任务录制
      StartRecording(log_prefix_ + "/water_line");
      {
        std::lock_guard<std::mutex> lock(mtx_);
        cur_log_path_ = log_prefix_ + "/water_line";
      }
      HJ_INFO("Start recording water line task.");
      break;
    }
    case 15: {
      // 开始池底任务录制
      StartRecording(log_prefix_ + "/pool_bottom");
      {
        std::lock_guard<std::mutex> lock(mtx_);
        cur_log_path_ = log_prefix_ + "/pool_bottom";
      }
      HJ_INFO("Start recording pool bottom task.");
      break;
    }
    case 35: {
      // 结束当前任务录制
      FinishTask();
      HJ_INFO("Stop recording current task.");
      break;
    }
    default:
      // 开始其他任务录制
      StartRecording(log_prefix_ + "/other_task");
      {
        std::lock_guard<std::mutex> lock(mtx_);
        cur_log_path_ = log_prefix_ + "/other_task";
      }
      HJ_INFO("Start recording other_task.");
      break;
  }
}

void RecordMsg::UploadCallback(const std_msgs::Bool::ConstPtr& msg) {
  // 上传文件
  std::string log_path = "";
  {
    std::lock_guard<std::mutex> lock(mtx_);
    log_path = cur_log_path_;
  }
  std::string zip_file_name = log_path + "/data.zip";
  bool ret = hj_bf::CreateZipFileByDir(zip_file_name, log_path);
  if (ret) {
    hj_interface::FileUpload file_upload_msg;
    file_upload_msg.type = hj_interface::FileUpload::DEVICELOG;
    file_upload_msg.filePath = zip_file_name;
    file_upload_msg.deleteOnSuccess = 1;
    pub_upload_.publish(file_upload_msg);
  }
}

void RecordMsg::Init(uint8_t task_enable) {
  if (access(log_prefix_.c_str(), 0) == -1) {  // check folder exist
    int ret = mkdir(log_prefix_.c_str(), S_IRWXU);  // create folder
    if (ret == -1) {
      HJ_ERROR("mkdir %s fail\n", log_prefix_.c_str());
      return;
    }
  } else {
    boost::filesystem::path path1(log_prefix_);
    boost::filesystem::remove_all(path1);
    int ret = mkdir(log_prefix_.c_str(), S_IRWXU);  // create folder
    if (ret == -1) {
      HJ_ERROR("mkdir %s fail\n", log_prefix_.c_str());
      return;
    }
  }

  if (task_enable == 0) {
    if (OpenLogFile(log_prefix_)) {
      cur_log_path_ = log_prefix_;
      writing_enabled_.store(true);
    } else {
      cur_log_path_ = log_prefix_;
      HJ_ERROR("param.task_enable false,Open log file fail\n");
    }
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
