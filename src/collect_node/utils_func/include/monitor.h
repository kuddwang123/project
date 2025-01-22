// @file turbidity.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-22)
#ifndef SRC_COLLECT_NODE_UTILS_FUNC_INCLUDE_MONITOR_H_
#define SRC_COLLECT_NODE_UTILS_FUNC_INCLUDE_MONITOR_H_
#include <string>
#include "function_factory.h"
#include "node_factory.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/String.h"
#include <mutex>
#include <condition_variable>

#define VMRSS_LINE 22

#define USER_TIME 13
#define KERNEL_TIME 14
#define ALL_USER_TIME 15
#define ALL_DEAD_TIME 16


namespace collect_node_utils_func {  // your namespace
typedef enum {
  TASK_BUILD_MAP_START = 1,
  TASK_BUILD_MAP_FINISH = 2,
  TASK_RELOCATE_START = 3,
  TASK_RELOCATE_FINISH = 4,
  TASK_CLEAN_START = 5,
  TASK_CLEAN_FINISH = 6
} TaskType;

class Monitor {
 public:
  Monitor() = default;
  ~Monitor();
  void Init();
 private:
  void Stringsplit(std::string str, const char split, std::vector<std::string>& rst);
  int64_t GetCpuTotalOccupy();
  int64_t GetCpuProcOccupy(int pid);
  uint32_t GetCpuUsageRatio(int pid);
  int32_t GetMemoryUsage(int pid);
  int32_t GetAllMemory();
  pid_t GetProcessPidByName(const std::string& proc_name);
  // void TaskCallBack(const std_msgs::UInt8::ConstPtr& msg);
  void TaskNameCallBack(const std_msgs::String::ConstPtr& msg);
  void CalCpuMemeory();
  void UploadCoredump();
  void pubBootType(bool is_core_dump);
 private:
  std::mutex mtx_;
  // std::atomic<uint8_t> task_{0};
  std::string task_name_;
  int32_t collect_node_pid_{-1};
  int32_t slam_node_pid_{-1};
  int32_t planning_node_pid_{-1};
  int32_t middleware_node_pid_{-1};
  int32_t utils_node_pid_{-1};
  // hj_bf::HJSubscriber task_sub_;
  hj_bf::HJSubscriber task_name_sub_;
  // std::condition_variable cv_;
  std::string fw_version_{"1.0.0"};
  hj_bf::HJPublisher bootType_pub_;
};
}  // namespace collect_node_utils_func

#endif  // SRC_COLLECT_NODE_UTILS_FUNC_INCLUDE_MONITOR_H_
