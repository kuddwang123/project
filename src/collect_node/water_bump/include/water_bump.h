// @file demo.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-22)
#ifndef INCLUDE_DEMO_H  // your macro
#define INCLUDE_DEMO_H
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>

#include "function_factory.h"
#include "hj_interface/Kbd.h"
#include "log.h"
#include "node_factory.h"

#define RET_ERR -1
#define RET_OK 0
#define FALSE 0
#define TRUE 1

#define FRAMEHEAD 0xAA
#define FRAMETAIL 0x55
#define FRAMECTRL 0xA5

#define MAX_PACKAGE_SIZE 64
#define LEN_QUE_RECV 1024

#define PUMP_SWITCH 0x10           // pump_status
#define PUMP_FLOWRATE 0x11         // pump_speed
#define LEFT_SPOUT_VEER 0x12       // turn_motor_l
#define RIGHT_SPOUT_VEER 0x13      // turn_motor_r
#define SLEF_CLEAN_SWITCH 0x14     // fan_status
#define SLEF_CLEAN_ROTATION 0x15   // not use
#define PUMP_DUTY_CYCLE 0x16       // pump_speed
#define UPPER_SUCTION_SWITCH 0x17  // flip_cover_angle
#define DIVE_MOTOR 0x18            // airbag_status
#define INFLACTION_TIME 0x19       // airbag_time
#define WHEEL 0x20                 // left_msg right_msg

namespace collect_node_ns {  // your namespace
typedef struct {
  unsigned char *buffer;
  int head;
  int tail;
  int size;
} Queue;

class WaterBump : public hj_bf::Function {
 public:
  explicit WaterBump(const rapidjson::Value &json_conf);
  ~WaterBump();
  void kbdCtrlChatterCallback(const hj_interface::Kbd::ConstPtr &msg);

 private:
  void WritePackage(uint8_t *buf, uint8_t len);
  // your variables
  // hj_bf::HJPublisher pub_;
  // hj_bf::HJClient client_;
  // hj_bf::HJSubscriber sub_;
  // hj_bf::HJServer service_;
  // hj_bf::HJTimer timer_;
  hj_bf::HJSubscriber sub_kbd_ctrl_;
  int fd_;
  bool driver_ok_{false};
  Queue *queue_;
};
}  // namespace collect_node_ns

#endif
