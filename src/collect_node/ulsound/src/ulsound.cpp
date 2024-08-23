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
#include "log.h"


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

void Ulsound::ReadFront() {  // 串口发送线程函数
  prctl(PR_SET_NAME, "ulsound_readFront");
  uint8_t senddata = 0x10;
  int ret = 0;
  u_int8_t read_size = 8;   // 读取的字节数
  std::queue<uint8_t> data_queue;
  while (true) {
    // read front
    ret = write(front_uart_.GetFd(), &senddata, 1);
    if (ret > 0) {
      ret = read(front_uart_.GetFd(), &(front_data_.databuf[0]), read_size);
      if (ret > 0 && (ret % read_size) == 0) {
        if (front_data_.databuf[7] ==
            ((front_data_.databuf[0] + front_data_.databuf[1] + front_data_.databuf[2] + front_data_.databuf[3] +
              front_data_.databuf[4] + front_data_.databuf[5] + front_data_.databuf[6]) & 0xff)) {
          triple_ultra_msg_.front_l = front_data_.databuf[1] * 256 + front_data_.databuf[2];
          triple_ultra_msg_.front_m = front_data_.databuf[3] * 256 + front_data_.databuf[4];
          triple_ultra_msg_.front_r = front_data_.databuf[5] * 256 + front_data_.databuf[6];
          triple_ultra_msg_.status = 0;
          triple_ultra_msg_.timestamp = hj_bf::HJTime::now();
          front_error_count_ = 0;
          triple_ultra_pub_.publish(triple_ultra_msg_);
          PushFrontError();
        } else {
          triple_ultra_msg_.timestamp = hj_bf::HJTime::now();
          triple_ultra_msg_.status = 1;
          triple_ultra_pub_.publish(triple_ultra_msg_);
          HJ_ERROR("front sum error, status:%x", triple_ultra_msg_.status);
          PushFrontError();
          front_error_count_++;
        }
        while (!data_queue.empty()) {  // 正常数据，清空缓存的异常数据
          data_queue.pop();
        }
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
            triple_ultra_msg_.front_l = front_data_.databuf[1] * 256 + front_data_.databuf[2];
            triple_ultra_msg_.front_m = front_data_.databuf[3] * 256 + front_data_.databuf[4];
            triple_ultra_msg_.front_r = front_data_.databuf[5] * 256 + front_data_.databuf[6];
            triple_ultra_msg_.status = 0;
            triple_ultra_msg_.timestamp = hj_bf::HJTime::now();
            front_error_count_ = 0;
            triple_ultra_pub_.publish(triple_ultra_msg_);
            PushFrontError();
            HJ_INFO("splicing front OK");
          } else {
            triple_ultra_msg_.timestamp = hj_bf::HJTime::now();
            triple_ultra_msg_.status = 1;
            triple_ultra_pub_.publish(triple_ultra_msg_);
            HJ_ERROR("splicing front check error, status:%x", triple_ultra_msg_.status);
            PushFrontError();
            front_error_count_++;
          }
        }
        front_error_count_++;
        PushFrontError();
        HJ_INFO("recv triple error: %d,  status:%x, databuf [%d,%d,%d,%d,%d,%d,%d,%d]", ret, triple_ultra_msg_.status,
                front_data_.databuf[0], front_data_.databuf[1], front_data_.databuf[2], front_data_.databuf[3],
                front_data_.databuf[4], front_data_.databuf[5], front_data_.databuf[6], front_data_.databuf[7]);
      }
    } else {
      triple_ultra_msg_.timestamp = hj_bf::HJTime::now();
      triple_ultra_msg_.status = 1;
      triple_ultra_pub_.publish(triple_ultra_msg_);
      HJ_ERROR("send front error!,status:%x", triple_ultra_msg_.status);
      PushFrontError();
      front_error_count_++;
    }
  }
}

void Ulsound::ReadSide() {  // 串口发送线程函数
  prctl(PR_SET_NAME, "ulsound_readSide");
  uint8_t sw_to_b = 1, sw_to_f = 0;
  uint8_t senddata = 0x10;
  int ret = 0;
  while (true) {
    usleep(1*1000);
    // system(SWITCH_TO_J165);
    ret = write(side_switch_fd_, &sw_to_b, sizeof(char));
    if (ret < 0) {
      HJ_ERROR("switch sw_to_b error: %d\n", ret);
    }
    usleep(1*1000);
    // read left uss J165
    ret = write(side_uart_.GetFd(), &senddata, 1);
    if (ret > 0) {
      ret = read(side_uart_.GetFd(), &(rec_data_.databuf[10]), 8);
      if (ret > 0 && (ret % 4) ==0) {
        if (rec_data_.databuf[13] ==
            ((rec_data_.databuf[10] + rec_data_.databuf[11] + rec_data_.databuf[12]) & 0xff)) {
          left_back_msg_.dist = rec_data_.databuf[11] * 256 + rec_data_.databuf[12];
          left_back_msg_.timestamp = hj_bf::HJTime::now();
          left_back_msg_.status = 0;
          left_back_pub_.publish(left_back_msg_);
          side_back_error_count_ = 0;
          PushSideBackError();
        } else {
          left_back_msg_.timestamp = hj_bf::HJTime::now();
          left_back_msg_.status = 1;
          left_back_pub_.publish(left_back_msg_);
          HJ_ERROR("left uss J165 sum error,%d, [%d,%d,%d], status=%x",
                  rec_data_.databuf[13], rec_data_.databuf[10],
                  rec_data_.databuf[11], rec_data_.databuf[12], left_back_msg_.status);
          PushSideBackError();
          side_back_error_count_++;
        }
      } else {
        left_back_msg_.timestamp = hj_bf::HJTime::now();
        left_back_msg_.status = 1;
        left_back_pub_.publish(left_back_msg_);
        HJ_ERROR("recv left uss J165 error %d, status=%x, [%d,%d,%d,%d]", ret, left_back_msg_.status,
                rec_data_.databuf[10], rec_data_.databuf[11], rec_data_.databuf[12], rec_data_.databuf[13]);
        PushSideBackError();
        side_back_error_count_++;
      }
    } else {
      left_back_msg_.timestamp = hj_bf::HJTime::now();
      left_back_msg_.status = 1;
      left_back_pub_.publish(left_back_msg_);
      HJ_ERROR("send left uss J165 error!,status=%x", left_back_msg_.status);
      PushSideBackError();
      side_back_error_count_++;
    }

    // 1:J171, 0:J165
    /* 收发完以后切换一定要加延迟 */
    usleep(1*1000);
    ret = write(side_switch_fd_, &sw_to_f, sizeof(char));
    if (ret < 0) {
      HJ_ERROR("switch sw_to_f error: %d\n", ret);
    }
    usleep(1*1000);

    // read left uss J171
    ret = write(side_uart_.GetFd(),  &senddata, 1);
    if (ret > 0) {
        ret = read(side_uart_.GetFd(), &(rec_data_.databuf[20]), 8);
        if (ret > 0 && (ret % 4 == 0)) {
          if (rec_data_.databuf[23] ==
              ((rec_data_.databuf[20] + rec_data_.databuf[21] + rec_data_.databuf[22]) & 0xff)) {
            left_front_msg_.dist = rec_data_.databuf[21]*256 + rec_data_.databuf[22];
            left_front_msg_.status = 0;
            left_front_msg_.timestamp = hj_bf::HJTime::now();
            left_front_pub_.publish(left_front_msg_);
            side_front_error_count_ = 0;
            PushSideFrontError();
          } else {
            left_front_msg_.timestamp = hj_bf::HJTime::now();
            left_front_msg_.status = 1;
            left_front_pub_.publish(left_front_msg_);
            HJ_ERROR("left uss J171 sum error!,status=%x", left_front_msg_.status);
            PushSideFrontError();
            side_front_error_count_++;
          }
        } else {
          left_front_msg_.timestamp = hj_bf::HJTime::now();
          left_front_msg_.status = 1;
          left_front_pub_.publish(left_front_msg_);
          HJ_ERROR("recv left uss J171 error: %d, status:%x, [%d,%d,%d,%d]", ret, left_front_msg_.status,
                  rec_data_.databuf[20], rec_data_.databuf[21], rec_data_.databuf[22], rec_data_.databuf[23]);
          PushSideFrontError();
          side_front_error_count_++;
        }
    } else {
      left_front_msg_.timestamp = hj_bf::HJTime::now();
      left_front_msg_.status = 1;
      left_front_pub_.publish(left_front_msg_);
      HJ_ERROR("send left uss J171 error!, status=%x", left_front_msg_.status);
      PushSideFrontError();
      side_front_error_count_++;
    }
  }
}

void Ulsound::ReadSideFront() {
  prctl(PR_SET_NAME, "ulsound_readSideFront");
  uint8_t senddata = 0x10;
  int ret = 0;
  while (true) {
    ret = write(side_uart_front_.GetFd(),  &senddata, 1);
    if (ret > 0) {
      ret = read(side_uart_front_.GetFd(), &(rec_data_.databuf[20]), 8);
      if (ret > 0 && (ret % 4 == 0)) {
        if (rec_data_.databuf[23] ==
            ((rec_data_.databuf[20] + rec_data_.databuf[21] + rec_data_.databuf[22]) & 0xff)) {
          left_front_msg_.dist = rec_data_.databuf[21]*256 + rec_data_.databuf[22];
          left_front_msg_.status = 0;
          left_front_msg_.timestamp = hj_bf::HJTime::now();
          left_front_pub_.publish(left_front_msg_);
          side_front_error_count_ = 0;
          PushSideFrontError();
        } else {
          left_front_msg_.timestamp = hj_bf::HJTime::now();
          left_front_msg_.status = 1;
          left_front_pub_.publish(left_front_msg_);
          HJ_ERROR("left uss J171 sum error!,status=%x", left_front_msg_.status);
          PushSideFrontError();
          side_front_error_count_++;
        }
      } else {
        left_front_msg_.timestamp = hj_bf::HJTime::now();
        left_front_msg_.status = 1;
        left_front_pub_.publish(left_front_msg_);
        HJ_ERROR("recv left uss J171 error: %d, status:%x,[%d,%d,%d,%d]", ret, left_front_msg_.status,
                rec_data_.databuf[20], rec_data_.databuf[21], rec_data_.databuf[22], rec_data_.databuf[23]);
        PushSideFrontError();
        side_front_error_count_++;
      }
    } else {
      left_front_msg_.timestamp = hj_bf::HJTime::now();
      left_front_msg_.status = 1;
      left_front_pub_.publish(left_front_msg_);
      HJ_ERROR("send left uss J171 error!, status=%x", left_front_msg_.status);
      PushSideFrontError();
      side_front_error_count_++;
    }
  }
}

void Ulsound::ReadSideBack() {
  prctl(PR_SET_NAME, "ulsound_readSideBack");
  uint8_t senddata = 0x10;
  int ret = 0;
  while (true) {
    ret = write(side_uart_back_.GetFd(), &senddata, 1);
    if (ret > 0) {
      ret = read(side_uart_back_.GetFd(), &(rec_data_.databuf[10]), 8);
      if (ret > 0 && (ret % 4) ==0) {
        if (rec_data_.databuf[13] ==
            ((rec_data_.databuf[10] + rec_data_.databuf[11] + rec_data_.databuf[12]) & 0xff)) {
          left_back_msg_.dist = rec_data_.databuf[11] * 256 + rec_data_.databuf[12];
          left_back_msg_.timestamp = hj_bf::HJTime::now();
          left_back_msg_.status = 0;
          left_back_pub_.publish(left_back_msg_);
          side_back_error_count_ = 0;
          PushSideBackError();
        } else {
          left_back_msg_.timestamp = hj_bf::HJTime::now();
          left_back_msg_.status = 1;
          left_back_pub_.publish(left_back_msg_);
          HJ_ERROR("left uss J165 sum error,%d, [%d,%d,%d], status=%x",
                  rec_data_.databuf[13], rec_data_.databuf[10],
                  rec_data_.databuf[11], rec_data_.databuf[12], left_back_msg_.status);
          PushSideBackError();
          side_back_error_count_++;
        }
      } else {
        left_back_msg_.timestamp = hj_bf::HJTime::now();
        left_back_msg_.status = 1;
        left_back_pub_.publish(left_back_msg_);
        HJ_ERROR("recv left uss J165 error %d, status=%x, [%d,%d,%d,%d]", ret, left_back_msg_.status,
                rec_data_.databuf[10], rec_data_.databuf[11], rec_data_.databuf[12], rec_data_.databuf[13]);
        PushSideBackError();
        side_back_error_count_++;
      }
    } else {
      left_back_msg_.timestamp = hj_bf::HJTime::now();
      left_back_msg_.status = 1;
      left_back_pub_.publish(left_back_msg_);
      HJ_ERROR("send left uss J165 error!,status=%x", left_back_msg_.status);
      PushSideBackError();
      side_back_error_count_++;
    }
  }
}

void Ulsound::ReadDown() {
  prctl(PR_SET_NAME, "ulsound_ReadDown");
  uint8_t senddata = 0x10;
  int ret = 0;
  uint8_t data_buffer[8] = {0};
  while (true) {
    ret = write(down_uart_.GetFd(), &senddata, 1);
    if (ret > 0) {
      ret = read(down_uart_.GetFd(), &(data_buffer[0]), 8);
      if (ret > 0 && (ret % 4) ==0) {
        if (data_buffer[3] ==
            ((data_buffer[0] + data_buffer[1] + data_buffer[2]) & 0xff)) {
          down_right_msg_.dist = data_buffer[1] * 256 + data_buffer[2];
          down_right_msg_.timestamp = hj_bf::HJTime::now();
          down_right_msg_.status = 0;
          down_left_pub_.publish(down_right_msg_);
          down_error_count_ = 0;
          PushDownError();
        } else {
          down_right_msg_.timestamp = hj_bf::HJTime::now();
          down_right_msg_.status = 1;
          down_left_pub_.publish(down_right_msg_);
          HJ_ERROR("down uss sum error");
          PushDownError();
          down_error_count_++;
        }
      } else {
        down_right_msg_.timestamp = hj_bf::HJTime::now();
        down_right_msg_.status = 1;
        down_left_pub_.publish(down_right_msg_);
        HJ_ERROR("recv down uss error %d, status=%x, [%d,%d,%d,%d]", ret, down_right_msg_.status,
                data_buffer[0], data_buffer[1], data_buffer[2], data_buffer[3]);
        PushDownError();
        down_error_count_++;
      }
    } else {
      down_right_msg_.timestamp = hj_bf::HJTime::now();
      down_right_msg_.status = 1;
      down_left_pub_.publish(down_right_msg_);
      HJ_ERROR("send down uss error!,status=%x", down_right_msg_.status);
      PushDownError();
      down_error_count_++;
    }
  }
}

Ulsound::~Ulsound() {
  close(side_switch_fd_);
}

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
      auto state = std::thread(&Ulsound::ReadFront, this);  // 开线程
      state.detach();
      HJ_INFO("front_init OK");
    }
  } else {
    HJ_INFO("front_init_status_ already OK");
  }

  if (machine_version_ == "T1pro") {
    HJ_INFO("T1pro machine_version_ don't have side uls");
  } else if (machine_version_ == "P3.5") {  // 侧边超声在3.5版本上电路变化，需要单独处理
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
  } else {  // 侧边超声在3.5版本之前，侧边超声共用一个串口
    if (!side_init_status_) {
      if (side_switch_fd_ < 0) {
        side_switch_fd_ = open(SWITCH_PATH_SIDE, O_RDWR);
        if (side_switch_fd_ < 0) {
          HJ_ERROR("side_switch_fd_ open  failed\n");
        }
      }

      ret = side_uart_.Initialize(115200, 0, 8, 1, 'N');
      ret &= side_uart_.LibttyRs485Set(false);
      if (!ret) {
        HJ_ERROR("side uls ret init = %d", ret);
        srv_msg_.request.code_val = ULSOUND_SIDE_MID_INIT_ERROR;
        srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::FAILED;
        hj_bf::HjPushSrv(srv_msg_);
        side_init_status_ = false;
      } else {
        side_init_status_ = true;
        auto state = std::thread(&Ulsound::ReadSide, this);  // 开线程
        state.detach();
        HJ_INFO("p3 front_side ULS OK");
      }
    } else {
      HJ_INFO("side_init_status_ already OK");
    }
  }

  return ret;
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
  if (json_conf.HasMember("machine_version") && json_conf["machine_version"].IsString()) {
    std::string machine_version = json_conf["machine_version"].GetString();
    if (!machine_version.empty()) {
      machine_version_ = machine_version;
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

  triple_ultra_pub_ = hj_bf::HJAdvertise<hj_interface::TripleUltra>("triple_ultra", 10);
  left_front_pub_ = hj_bf::HJAdvertise<hj_interface::LeftFront>("x9/left_front", 10);
  left_back_pub_ = hj_bf::HJAdvertise<hj_interface::LeftBack>("x9/left_back", 10);
  down_left_pub_ = hj_bf::HJAdvertise<hj_interface::DownRight>("x9/down_right", 10);
  restart_sub_ = hj_bf::HJSubscribe("/xxx", 1, &Ulsound::RestartCallback, this);

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
