/**
 * @file ulsound.cpp
 * @author hao wu (clayderman@yardbot.net)
 * @brief get ulsound data
 * @version 0.1
 * @date 2024-06-20
 * 
 * @copyright Copyright 2023 HJ Technology Co.Ltd. All rights reserved
 * 
 */

#include "ulsound.h"
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/prctl.h>
#include <thread>
#include <queue>
#include <deque>
#include "log.h"
#include <unistd.h>
#include "shm_interface.h"


HJ_REGISTER_FUNCTION(factory) {
  HJ_INFO("minos register factory,function_name:%s", FUNCTION_NAME);
  factory.registerCreater<collect_node_ulsound::Ulsound>(FUNCTION_NAME);
}

namespace collect_node_ulsound {

void Ulsound::PushFrontError() {
  if (!front_uls_status_ && front_error_count_ == 0) {
    srv_msg_.request.code_val = ULSOUND_FRONT_USS_DATA_ERROR;
    srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::NORMAL;
    hj_bf::HjPushSrv(srv_msg_);
    front_uls_status_ = true;
  } else if (front_uls_status_ && front_error_count_ > ERROR_COUNT) {
    srv_msg_.request.code_val = ULSOUND_FRONT_USS_DATA_ERROR;
    srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::ERROR;
    hj_bf::HjPushSrv(srv_msg_);
    front_uls_status_ = false;
  }
}

void Ulsound::PushSideFrontError() {
  if (!side_front_uls_status_ && side_front_error_count_ ==0) {
    srv_msg_.request.code_val = ULSOUND_SIDE_MID_DATA_ERROR;
    srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::NORMAL;
    hj_bf::HjPushSrv(srv_msg_);
    side_front_uls_status_ = true;
  } else if (side_front_uls_status_ && side_front_error_count_ > ERROR_COUNT) {
    srv_msg_.request.code_val = ULSOUND_SIDE_MID_DATA_ERROR;
    srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::ERROR;
    hj_bf::HjPushSrv(srv_msg_);
    side_front_uls_status_ = false;
  }
}

void Ulsound::PushSideBackError() {
  if (!side_back_uls_status_ && side_back_error_count_ == 0) {
    srv_msg_.request.code_val = ULSOUND_SIDE_BACK_DATA_ERROR;
    srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::NORMAL;
    hj_bf::HjPushSrv(srv_msg_);
    side_back_uls_status_ = true;
  } else if (side_back_uls_status_ && side_back_error_count_ > ERROR_COUNT) {
    srv_msg_.request.code_val = ULSOUND_SIDE_BACK_DATA_ERROR;
    srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::ERROR;
    hj_bf::HjPushSrv(srv_msg_);
    side_back_uls_status_ = false;
  }
}

void Ulsound::PushDownError() {
  if (!down_uls_status_ && down_error_count_ == 0) {
    srv_msg_.request.code_val = ULSOUND_DOWN_LEFT_DATA_ERROR;
    srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::NORMAL;
    hj_bf::HjPushSrv(srv_msg_);
    down_uls_status_ = true;
  } else if (down_uls_status_ && down_error_count_ > ERROR_COUNT) {
    srv_msg_.request.code_val = ULSOUND_DOWN_LEFT_DATA_ERROR;
    srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::ERROR;
    hj_bf::HjPushSrv(srv_msg_);
    down_uls_status_ = false;
  }
}

void Ulsound::ReadFront(const hj_bf::HJTimerEvent &) {  // 串口发送线程函数
  uint8_t senddata = 0x10;
  int ret = 0;
  u_int8_t read_size = 64;   // 读取的字节数, 64个字节:模组断电上电，有概率造成一帧数据被截断
  static std::queue<uint8_t> data_queue;
  int module_restart_flag = 0;
  bool res = hj_bf::getVariable("module_restart_flag", module_restart_flag);
  if (res && module_restart_flag != 0) {
    usleep(300000);  // 等待300ms,等待模块重启完成
    return;
  }
  // read front
  ret = write(front_uart_.GetFd(), &senddata, 1);
  if (ret > 0) {
    ret = read(front_uart_.GetFd(), &(front_data_.databuf[0]), read_size);
    if (ret > 0 && (ret % 8) == 0) {
      if (front_data_.databuf[7] ==
          ((front_data_.databuf[0] + front_data_.databuf[1] + front_data_.databuf[2] + front_data_.databuf[3] +
            front_data_.databuf[4] + front_data_.databuf[5] + front_data_.databuf[6]) & 0xff)) {
        triple_ultra_msg_.front_l = front_data_.databuf[5] * 256 + front_data_.databuf[6];
        triple_ultra_msg_.front_m = front_data_.databuf[3] * 256 + front_data_.databuf[4];
        triple_ultra_msg_.front_r = front_data_.databuf[1] * 256 + front_data_.databuf[2];
        triple_ultra_msg_.status = 0;
        triple_ultra_msg_.timestamp = GetTimeNow();
        front_error_count_ = 0;
        triple_ultra_pub_.publish(triple_ultra_msg_);
        PushFrontError();
      } else {
        triple_ultra_msg_.timestamp = GetTimeNow();
        triple_ultra_msg_.status = 1;
        triple_ultra_pub_.publish(triple_ultra_msg_);
        HJ_ERROR("front sum error, status:%x", triple_ultra_msg_.status);
        PushFrontError();
        front_error_count_++;
      }
      while (!data_queue.empty()) {  // 正常数据，清空缓存的异常数据
        data_queue.pop();
      }
    } else if (ret == 0) {
      triple_ultra_msg_.timestamp = GetTimeNow();
      triple_ultra_msg_.status = 1;
      triple_ultra_msg_.front_l = 65535;
      triple_ultra_msg_.front_m = 65535;
      triple_ultra_msg_.front_r = 65535;
      triple_ultra_pub_.publish(triple_ultra_msg_);
      PushFrontError();
      front_error_count_++;
    } else {  // 未读取到8个字节，缓存拼接
      for (int i = 0; i < ret; i++) {
        data_queue.push(front_data_.databuf[i]);
      }
      // 帧头数据固定为0xff，如果读取不到0xff则丢弃
      while (!data_queue.empty() && data_queue.front() != 0xff) {
        data_queue.pop();
      }
      if (data_queue.size() >= read_size) {
        for (int i = 0; i < read_size; i++) {
          front_data_.databuf[i] = data_queue.front();
          data_queue.pop();
        }
        if (front_data_.databuf[7] ==
            ((front_data_.databuf[0] + front_data_.databuf[1] + front_data_.databuf[2] + front_data_.databuf[3] +
              front_data_.databuf[4] + front_data_.databuf[5] + front_data_.databuf[6]) & 0xff)) {
          triple_ultra_msg_.front_l = front_data_.databuf[5] * 256 + front_data_.databuf[6];
          triple_ultra_msg_.front_m = front_data_.databuf[3] * 256 + front_data_.databuf[4];
          triple_ultra_msg_.front_r = front_data_.databuf[1] * 256 + front_data_.databuf[2];
          triple_ultra_msg_.status = 0;
          triple_ultra_msg_.timestamp = GetTimeNow();
          front_error_count_ = 0;
          triple_ultra_pub_.publish(triple_ultra_msg_);
          PushFrontError();
          HJ_INFO("splicing front OK");
        } else {
          triple_ultra_msg_.timestamp = GetTimeNow();
          triple_ultra_msg_.status = 1;
          triple_ultra_pub_.publish(triple_ultra_msg_);
          if (front_error_count_ < ERROR_COUNT) {
            HJ_ERROR("splicing front check error, status:%x", triple_ultra_msg_.status);
          }
          PushFrontError();
          front_error_count_++;
        }
      }
      front_error_count_++;
      PushFrontError();
      if (front_error_count_ < ERROR_COUNT) {
        HJ_INFO("recv triple error: %d,  status:%x, databuf [%d,%d,%d,%d,%d,%d,%d,%d]", ret, triple_ultra_msg_.status,
              front_data_.databuf[0], front_data_.databuf[1], front_data_.databuf[2], front_data_.databuf[3],
              front_data_.databuf[4], front_data_.databuf[5], front_data_.databuf[6], front_data_.databuf[7]);
      }
    }
  } else {
    triple_ultra_msg_.timestamp = GetTimeNow();
    triple_ultra_msg_.status = 1;
    triple_ultra_pub_.publish(triple_ultra_msg_);
    if (front_error_count_ < ERROR_COUNT) {
      HJ_ERROR("send front error!,status:%x", triple_ultra_msg_.status);
    }
    PushFrontError();
    front_error_count_++;
  }
}

void Ulsound::ReadSideFront() {
  prctl(PR_SET_NAME, "ulsound_readSideFront");
  uint8_t senddata = 0x10;
  int ret = 0;
  int module_restart_flag = 0;
  while (hj_bf::ok()) {
    bool res = hj_bf::getVariable("module_restart_flag", module_restart_flag);
    if (res && module_restart_flag != 0) {
      usleep(300000);  // 等待300ms,等待模块重启完成
      continue;
    }
    ret = write(side_uart_front_.GetFd(),  &senddata, 1);
    if (ret > 0) {
      ret = read(side_uart_front_.GetFd(), &(rec_data_.databuf[20]), 8);
      if (ret > 0 && (ret % 4 == 0)) {
        if (rec_data_.databuf[23] ==
            ((rec_data_.databuf[20] + rec_data_.databuf[21] + rec_data_.databuf[22]) & 0xff)) {
          left_front_msg_.dist = rec_data_.databuf[21]*256 + rec_data_.databuf[22];
          if (left_front_msg_.dist == OUTWATER_DISTANCE) {
            left_front_msg_.status = 2;
          } else {
            left_front_msg_.status = 0;
          }

          left_front_msg_.timestamp = GetTimeNow();
          left_front_pub_.publish(left_front_msg_);
          side_front_error_count_ = 0;
          PushSideFrontError();
        } else {
          left_front_msg_.timestamp = GetTimeNow();
          left_front_msg_.status = 1;
          left_front_pub_.publish(left_front_msg_);
          HJ_ERROR("left uss J171 sum error!,status=%x", left_front_msg_.status);
          PushSideFrontError();
          side_front_error_count_++;
        }
      } else {
        left_front_msg_.timestamp = GetTimeNow();
        left_front_msg_.status = 1;
        left_front_pub_.publish(left_front_msg_);
        if (side_front_error_count_ < ERROR_COUNT) {
          HJ_ERROR("recv left uss J171 error: %d, status:%x,[%d,%d,%d,%d]", ret, left_front_msg_.status,
                rec_data_.databuf[20], rec_data_.databuf[21], rec_data_.databuf[22], rec_data_.databuf[23]);
        }
        PushSideFrontError();
        side_front_error_count_++;
      }
    } else {
      left_front_msg_.timestamp = GetTimeNow();
      left_front_msg_.status = 1;
      left_front_pub_.publish(left_front_msg_);
      if (side_front_error_count_ < ERROR_COUNT) {
        HJ_ERROR("send left uss J171 error!, status=%x", left_front_msg_.status);
      }
      PushSideFrontError();
      side_front_error_count_++;
    }
  }
}

void Ulsound::ReadSideBack() {
  prctl(PR_SET_NAME, "ulsound_readSideBack");
  uint8_t senddata = 0x10;
  int ret = 0;
  int module_restart_flag = 0;
  while (hj_bf::ok()) {
    bool res = hj_bf::getVariable("module_restart_flag", module_restart_flag);
    if (res && module_restart_flag != 0) {
      usleep(300000);  // 等待300ms,等待模块重启完成
      continue;
    }
    ret = write(side_uart_back_.GetFd(), &senddata, 1);
    if (ret > 0) {
      ret = read(side_uart_back_.GetFd(), &(rec_data_.databuf[10]), 8);
      if (ret > 0 && (ret % 4) ==0) {
        if (rec_data_.databuf[13] ==
            ((rec_data_.databuf[10] + rec_data_.databuf[11] + rec_data_.databuf[12]) & 0xff)) {
          left_back_msg_.dist = rec_data_.databuf[11] * 256 + rec_data_.databuf[12];
          if (left_back_msg_.dist == OUTWATER_DISTANCE) {
            left_back_msg_.status = 2;
          } else {
            left_back_msg_.status = 0;
          }

          left_back_msg_.timestamp = GetTimeNow();
          left_back_pub_.publish(left_back_msg_);
          side_back_error_count_ = 0;
          PushSideBackError();
        } else {
          left_back_msg_.timestamp = GetTimeNow();
          left_back_msg_.status = 1;
          left_back_pub_.publish(left_back_msg_);
          if (side_back_error_count_ < ERROR_COUNT) {
            HJ_ERROR("left uss J165 sum error,%d, [%d,%d,%d], status=%x",
                  rec_data_.databuf[13], rec_data_.databuf[10],
                  rec_data_.databuf[11], rec_data_.databuf[12], left_back_msg_.status);
          }
          PushSideBackError();
          side_back_error_count_++;
        }
      } else {
        left_back_msg_.timestamp = GetTimeNow();
        left_back_msg_.status = 1;
        left_back_pub_.publish(left_back_msg_);
        if (side_back_error_count_ < ERROR_COUNT) {
          HJ_ERROR("recv left uss J165 error %d, status=%x, [%d,%d,%d,%d]", ret, left_back_msg_.status,
                rec_data_.databuf[10], rec_data_.databuf[11], rec_data_.databuf[12], rec_data_.databuf[13]);
        }
        PushSideBackError();
        side_back_error_count_++;
      }
    } else {
      left_back_msg_.timestamp = GetTimeNow();
      left_back_msg_.status = 1;
      left_back_pub_.publish(left_back_msg_);
      if (side_back_error_count_ < ERROR_COUNT) {
        HJ_ERROR("send left uss J165 error!,status=%x", left_back_msg_.status);
      }
      PushSideBackError();
      side_back_error_count_++;
    }
  }
}

void Ulsound::ReadDown() {
  prctl(PR_SET_NAME, "ulsound_ReadDown");
  uint8_t senddata = 0x10;
  u_int8_t read_size = 4;
  int ret = 0;
  uint8_t data_buffer[8] = {0};
  std::deque<uint8_t> data_deque;
  int module_restart_flag = 0;
  while (hj_bf::ok()) {
    bool res = hj_bf::getVariable("module_restart_flag", module_restart_flag);
    if (res && module_restart_flag != 0) {
      usleep(300000);  // 等待300ms,等待模块重启完成
      continue;
    }
    ret = write(down_uart_.GetFd(), &senddata, 1);
    if (ret > 0) {
      ret = read(down_uart_.GetFd(), &(data_buffer[0]), 8);
      if (ret > 0 && (ret % 4) ==0) {
        if (data_buffer[3] ==
            ((data_buffer[0] + data_buffer[1] + data_buffer[2]) & 0xff)) {
          down_right_msg_.dist = data_buffer[1] * 256 + data_buffer[2];
          if (down_right_msg_.dist == OUTWATER_DISTANCE) {
            down_right_msg_.status = 2;
          } else {
            down_right_msg_.status = 0;
          }
          down_right_msg_.timestamp = GetTimeNow();
          down_right_pub_.publish(down_right_msg_);
          down_error_count_ = 0;
          PushDownError();
        } else {
          down_right_msg_.timestamp = GetTimeNow();
          down_right_msg_.status = 1;
          down_right_pub_.publish(down_right_msg_);
          HJ_ERROR("down uss sum error");
          PushDownError();
          down_error_count_++;
        }
        data_deque.clear();
      } else if(ret == 0) {
        down_right_msg_.timestamp = GetTimeNow();
        down_right_msg_.status = 1;
        down_right_msg_.dist = 65535;
        down_right_pub_.publish(down_right_msg_);
        PushDownError();
        down_error_count_++;
      } else {
        for (int i = 0; i < ret; i++) {
          data_deque. emplace_back(data_buffer[i]);
        }
        // 帧头数据固定为0xff，如果读取不到0xff则丢弃
        while (!data_deque.empty() && data_deque.front() != 0xff) {
          data_deque.pop_front();
        }
        if (data_deque.size() >= read_size) {
          for (int i = 0; i < read_size; i++) {
            data_buffer[i] = data_deque.front();
            data_deque.pop_front();
          }
          if (data_buffer[3] ==
            ((data_buffer[0] + data_buffer[1] + data_buffer[2]) & 0xff)) {
            down_right_msg_.dist = data_buffer[1] * 256 + data_buffer[2];
            down_right_msg_.timestamp = GetTimeNow();
            down_right_msg_.status = 0;
            down_right_pub_.publish(down_right_msg_);
            down_error_count_ = 0;
            PushDownError();
            HJ_INFO("splicing front OK");
          } else {
            down_right_msg_.timestamp = GetTimeNow();
            down_right_msg_.status = 1;
            down_right_pub_.publish(down_right_msg_);
            HJ_ERROR("splicing down check error, status:%x", down_right_msg_.status);
            PushDownError();
            down_error_count_++;
          }
        }
        down_error_count_++;
        PushDownError();
        if (down_error_count_ < ERROR_COUNT) {
          HJ_INFO("recv down_right_msg_ error: %d,  status:%x, databuf [%d,%d,%d,%d]", ret, down_right_msg_.status,
                data_buffer[0], data_buffer[1], data_buffer[2], data_buffer[3]);
        }
      }
    } else {
      down_right_msg_.timestamp = GetTimeNow();
      down_right_msg_.status = 1;
      down_right_pub_.publish(down_right_msg_);
      if (down_error_count_ < ERROR_COUNT) {
        HJ_ERROR("send down uss error!,status=%x", down_right_msg_.status);
      }
      PushDownError();
      down_error_count_++;
    }
  }
}

Ulsound::~Ulsound() {}

void Ulsound::RestartCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data != 0) {
    HJ_INFO("restart Ulsound");
    Start();
  }
}

bool Ulsound::Start() {
  bool ret = false;

  // init 三合一超声
  if (!front_init_status_) {
    ret = front_uart_.Initialize(115200, 0, 8, 1, 'N');
#if DHJ_X9
    ret &= front_uart_.LibttyRs485Set(false);
#endif
    if (!ret) {
      HJ_ERROR("front uls ret init = %d", ret);
      srv_msg_.request.code_val = ULSOUND_FRONT_USS_INIT_ERROR;
      srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::FAILED;
      hj_bf::HjPushSrv(srv_msg_);
      front_init_status_ = false;
    } else {
      front_init_status_ = true;
      triple_timer_ = hj_bf::HJCreateTimer("triple_ultra_timer",
                front_uls_frequency_ * 1000, &Ulsound::ReadFront, this);
      HJ_INFO("front_init OK");
    }
  } else {
    HJ_INFO("front_init_status_ already OK");
  }

  if (machine_version_ == "T1pro_v2") {
    HJ_INFO("T1pro v2 machine_version_ don't have side uls");
  } else if (machine_version_ == "P4" || machine_version_ == "T1pro_v3") {
    if (!side_front_init_status_) {
      ret = side_uart_front_.Initialize(115200, 0, 8, 1, 'N');
      ret &= side_uart_front_.LibttyRs485Set(false);
      if (!ret) {
        HJ_ERROR("side front uls ret init = %d", ret);
        srv_msg_.request.code_val = ULSOUND_SIDE_MID_INIT_ERROR;
        srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::FAILED;
        hj_bf::HjPushSrv(srv_msg_);
        side_front_init_status_ = false;
      } else {
        side_front_init_status_ = true;
        auto state = std::thread(&Ulsound::ReadSideFront, this);  // 开线程
        state.detach();
        HJ_INFO("front_side ULS OK");
      }
    } else {
      HJ_INFO("side_front_init_status_ already OK");
    }

    if (!side_back_init_status_) {
      ret = side_uart_back_.Initialize(115200, 0, 8, 1, 'N');
      ret &= side_uart_back_.LibttyRs485Set(false);
      if (!ret) {
        HJ_ERROR("side back uls ret init = %d", ret);
        srv_msg_.request.code_val = ULSOUND_SIDE_BACK_INIT_ERROR;
        srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::FAILED;
        hj_bf::HjPushSrv(srv_msg_);
        side_back_init_status_ = false;
      } else {
        side_back_init_status_ = true;
        auto state = std::thread(&Ulsound::ReadSideBack, this);  // 开线程
        state.detach();
        HJ_INFO("back_side ULS OK");
      }
    } else {
      HJ_INFO("side_back_init_status_ already OK");
    }
    if (machine_version_ == "P4") {
      if (!down_init_status_) {
        ret = down_uart_.Initialize(115200, 0, 8, 1, 'N');
        ret &= down_uart_.LibttyRs485Set(false);
        if (!ret) {
          HJ_ERROR("down uls ret init = %d", ret);
          srv_msg_.request.code_val = ULSOUND_DOWN_INIT_ERROR;
          srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::FAILED;
          hj_bf::HjPushSrv(srv_msg_);
          down_init_status_ = false;
        } else {
          down_init_status_ = true;
          auto state = std::thread(&Ulsound::ReadDown, this);  // 开线程
          state.detach();
          HJ_INFO("down_side ULS OK");
        }
      } else {
        HJ_INFO("down_init_status_ already OK");
      }
    }
  }

  return ret;
}

void Ulsound::TimeDiffCallback(const std_msgs::Float64::ConstPtr& msg) {
  time_diff_.store(msg->data);
  HJ_INFO("time_diff_now:%f", msg->data);
}

ros::Time Ulsound::GetTimeNow() {
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

Ulsound::Ulsound(const rapidjson::Value &json_conf) : hj_bf::Function(json_conf) {
  // read your config
  if (json_conf.HasMember("front_dev") && json_conf["front_dev"].IsString()) {
    std::string dev_path = json_conf["front_dev"].GetString();
    if (!dev_path.empty()) {
      front_uart_.SetDev(dev_path);
    }
  }
  if (json_conf.HasMember("side_dev") && json_conf["side_dev"].IsString()) {
    std::string dev_path = json_conf["side_dev"].GetString();
    if (!dev_path.empty()) {
      side_uart_.SetDev(dev_path);
    }
  }
  if (json_conf.HasMember("side_dev_back") && json_conf["side_dev_back"].IsString()) {
    std::string dev_path = json_conf["side_dev_back"].GetString();
    if (!dev_path.empty()) {
      side_uart_back_.SetDev(dev_path);
    }
  }
  if (json_conf.HasMember("side_dev_front") && json_conf["side_dev_front"].IsString()) {
    std::string dev_path = json_conf["side_dev_front"].GetString();
    if (!dev_path.empty()) {
      side_uart_front_.SetDev(dev_path);
    }
  }
  if (json_conf.HasMember("down_dev") && json_conf["down_dev"].IsString()) {
    std::string dev_path = json_conf["down_dev"].GetString();
    if (!dev_path.empty()) {
      down_uart_.SetDev(dev_path);
    }
  }
  if (json_conf.HasMember("machine_version") && json_conf["machine_version"].IsString()) {
    std::string machine_version = json_conf["machine_version"].GetString();
    if (!machine_version.empty()) {
      machine_version_ = machine_version;
    }
  }
  if (json_conf.HasMember("front_uls_frequency") && json_conf["front_uls_frequency"].IsInt()) {
    front_uls_frequency_ = json_conf["front_uls_frequency"].GetInt();
  }

  triple_ultra_pub_ = hj_bf::HJAdvertise<hj_interface::TripleUltra>("triple_ultra", 10);
  left_front_pub_ = hj_bf::HJAdvertise<hj_interface::LeftFront>("x9/left_front", 10);
  left_back_pub_ = hj_bf::HJAdvertise<hj_interface::LeftBack>("x9/left_back", 10);
  down_right_pub_ = hj_bf::HJAdvertise<hj_interface::DownRight>("x9/down_right", 10);
  restart_sub_ = hj_bf::HJSubscribe("/xxx", 1, &Ulsound::RestartCallback, this);
  sub_time_diff_ = hj_bf::HJSubscribe("/time_diff_chatter", 1, &Ulsound::TimeDiffCallback, this);

  triple_ultra_msg_.front_l = 65535;
  triple_ultra_msg_.front_m = 65535;
  triple_ultra_msg_.front_r = 65535;
  left_back_msg_.dist = 65535;
  left_front_msg_.dist = 65535;
  down_right_msg_.dist = 65535;

  Start();
  HJ_INFO("Ulsound init success");
}
}  // namespace collect_node_ulsound
