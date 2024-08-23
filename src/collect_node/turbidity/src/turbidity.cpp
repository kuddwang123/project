// @file wf5803.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-22)
#include "turbidity.h"
#include "log.h"

HJ_REGISTER_FUNCTION(factory) {
  HJ_INFO("minos register factory, function_name:%s", FUNCTION_NAME);
  factory.registerCreater<collect_node_turbidity::Turbidimeter>(FUNCTION_NAME);
}

namespace collect_node_turbidity {

void Turbidimeter::TurbidimeterTimer(const hj_bf::HJTimerEvent &) {
  static uint8_t s_send_tur_data[5] = {0x18, 0x05, 0x00, 0x01, 0x0d};  // 主机读脏污数据协议，0x01为写指令
  static uint8_t s_read_buf[8];  // 从机发送数据帧 {0x18，0x05，0x00，脏污值，0x0d}
                                 // 0x18为帧头固定值，0x05为数据长度，0x00为写应答指令，脏污值为0~255， 0x0d为帧尾固定值
  uint8_t turbidity = 0;

  int ret = write(uart_.GetFd(), s_send_tur_data, 5);
  if (ret > 0) {
    ret = read(uart_.GetFd(), s_read_buf, 5);
    if (0 == (ret % 5) && 0x18 == s_read_buf[0] && 0x0d == s_read_buf[4]) {
      turbidity = s_read_buf[3];
      tur_msg_.timestamp = ros::Time::now();
      tur_msg_.turbidity = turbidity;
      if (!status_) {
        srv_msg_.request.code_val = TUR_DATA_ERROR;
        srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::NORMAL;
        hj_bf::HjPushSrv(srv_msg_);
        status_ = true;
      }
      error_count_ = 0;
      chatter_pub_.publish(tur_msg_);
    } else {
      if (status_ && error_count_ > ERROR_COUNT) {
        srv_msg_.request.code_val = TUR_DATA_ERROR;
        srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::ERROR;
        hj_bf::HjPushSrv(srv_msg_);
        status_ = false;
        HJ_ERROR("turbidimeter_timer read frame format error.ret=%d", ret);
      }
      error_count_++;
    }
  } else {
    if (status_ && error_count_ > ERROR_COUNT) {
      srv_msg_.request.code_val = TUR_DATA_ERROR;
      srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::ERROR;
      hj_bf::HjPushSrv(srv_msg_);
      status_ = false;
      HJ_ERROR("write to  fd_turbid_ error.fd=%d", uart_.GetFd());
    }
    error_count_++;
  }
}

Turbidimeter::~Turbidimeter() {
  HJ_INFO("~Turbidimeter");
}

void Turbidimeter::RestartCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data != 0 && !init_status_) {
    HJ_INFO("restart Turbidimeter");
    Start();
  }
}

bool Turbidimeter::Start() {
  bool ret = uart_.Initialize(9600, 0, 8, 1, 'N');
  ret &=uart_.LibttyRs485Set(false);
  if (!ret) {
    HJ_ERROR("turbidity ret init = %d", ret);
    init_status_ = false;
    srv_msg_.request.code_val = TUR_INIT_ERROR;
    srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::FAILED;
    hj_bf::HjPushSrv(srv_msg_);
  } else {
    init_status_ = true;
    loop_timer_ = hj_bf::HJCreateTimer("TurbidimeterTimer",
                frequency_ * 1000, &Turbidimeter::TurbidimeterTimer, this);  // 30HZ
  }
  return ret;
}

Turbidimeter::Turbidimeter(const rapidjson::Value &json_conf) : hj_bf::Function(json_conf) {
  // read your config
  if (json_conf.HasMember("frequency") && json_conf["frequency"].IsInt()) {
    frequency_ = json_conf["frequency"].GetInt();
  }
  if (json_conf.HasMember("dev") && json_conf["dev"].IsString()) {
    std::string dev_path = json_conf["dev"].GetString();
    if (!dev_path.empty()) {
      uart_.SetDev(dev_path);
    }
  }

  // your  code
  chatter_pub_ = hj_bf::HJAdvertise<hj_interface::Turbidity>("turbidity_data", 10);
  restart_sub_ = hj_bf::HJSubscribe("/xxx", 1, &Turbidimeter::RestartCallback, this);

  if (Start()) {
    HJ_INFO("turbidity init success");
  } else {
    HJ_ERROR("turbidity init failed");
  }
}
}  // namespace collect_node_turbidity
