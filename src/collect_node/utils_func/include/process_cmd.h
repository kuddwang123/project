// @file process_cmd.h
// @brief
//
// Copyright 2024 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2024-12-17)
#ifndef INCLUDE_PROCESS_CMD_H_  // NOLINT
#define INCLUDE_PROCESS_CMD_H_  // NOLINT

#include <hj_interface/AppData.h>
#include "hj_interface/AppOnlineType.h"
#include "std_msgs/UInt8.h"
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "node_factory.h"

namespace process_cmd {

class ProcessCmd {
 public:
  static ProcessCmd& GetInstance();
  ~ProcessCmd();

 private:
  ProcessCmd();
  ProcessCmd(const ProcessCmd&) = delete;
  ProcessCmd(ProcessCmd&&) = delete;
  ProcessCmd& operator=(const ProcessCmd&) = delete;
  ProcessCmd& operator=(ProcessCmd&&) = delete;
  void CmdProcesser();
  void GetCmdCallback(const hj_interface::AppData::ConstPtr& msg);
  void UpdateLogLevelCallback(const std_msgs::UInt8::ConstPtr& msg);
  bool SwitchLog(const std::string& payload);
  bool ParseMsg(const std::string& key, const std::string& payload);
  void IotOnlineCallback(const hj_interface::AppOnlineType::ConstPtr& msg);
  void CheckVersion();
  void CreateOldVersionConfigure(const std::string& build_class, const std::string& fw_version);
  void CreateSwitchLog(const std::string& log_version);
  void SwitchLogTimerCallback(const hj_bf::HJSteadyTimerEvent&);
  std::deque<hj_interface::AppData> deque_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cond_;
  hj_bf::HJSubscriber cmd_sub_;
  hj_bf::HJSubscriber online_status_sub_;
  hj_bf::HJSubscriber ota_trigger_sub_;
  hj_bf::HJPublisher update_log_version_pub_;
  hj_bf::HJPublisher rsp_to_cloud_pub_;
  hj_bf::HJSteadyTimer switch_log_timer_;
  bool exit_flag_{false};
};
}  // namespace process_cmd
#endif  //  INCLUDE_PROCESS_CMD_H_  // NOLINT
