// @file heartbeat.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-22)
#ifndef INCLUDE_HEARTBEAT_H
#define INCLUDE_HEARTBEAT_H
#include "function_factory.h"
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

namespace collect_node_ns {

typedef struct {
  unsigned char* buffer;
  int head;
  int tail;
  int size;
} Queue;
class HeartBeat : public hj_bf::Function {
 public:
  explicit HeartBeat(const rapidjson::Value& json_conf);
  ~HeartBeat();

 private:
  hj_bf::HJTimer loop_timer_;
};
}  // namespace collect_node_ns

#endif
