// @file process_cmd.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2024-7-24)
#include "process_cmd.h"

#include <hj_interface/AppData.h>
#include <hj_interface/AppMsg.h>
#include <hj_interface/FileUpload.h>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <roscpp/SetLoggerLevel.h>
#include <sys/prctl.h>

#include <boost/filesystem.hpp>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>

#include "hj_zip.h"
#include "log.h"
#include "log_factory.h"
#include "node_factory.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
namespace process_cmd {
constexpr int kProcessTimeoutMinutes = 10;
constexpr char kCmdProcessTopicName[] = "/cmd_process";
constexpr char kCmdProcessAppOnlineTopicName[] = "/AppOnline";
constexpr char kCmdProcessReportAppTopicName[] = "/ReportApp";
constexpr char kCmdProcessRESAppTopicName[] = "/RespToApp";
constexpr char kCmdProcessUpdateLogVersionKey[] = "LogInfoReport";
constexpr char kSwitchLogOpen[] = "debug";
constexpr char kSwitchLogClose[] = "release";
constexpr char kRosLevelName[] = "/set_logger_level";
constexpr char kOtaTrigerTopicName[] = "otaSucTrigerLog";
constexpr char kVersionPath[] = "/etc/version";
constexpr char kOldVersionPath[] = "/userdata/hj/config/old_version_back.json";
ProcessCmd::ProcessCmd() {
  cmd_sub_ = hj_bf::HJSubscribe(process_cmd::kCmdProcessTopicName, 10, &ProcessCmd::GetCmdCallback, this);
  online_status_sub_ = hj_bf::HJSubscribe(kCmdProcessAppOnlineTopicName, 10, &ProcessCmd::IotOnlineCallback, this);
  ota_trigger_sub_ = hj_bf::HJSubscribe(process_cmd::kOtaTrigerTopicName, 10, &ProcessCmd::UpdateLogLevelCallback, this);
  update_log_version_pub_ = hj_bf::HJAdvertise<hj_interface::AppMsg>(kCmdProcessReportAppTopicName, 10);
  rsp_to_cloud_pub_ = hj_bf::HJAdvertise<hj_interface::AppMsg>(kCmdProcessRESAppTopicName, 10);
  switch_log_timer_ = hj_bf::HJCreateSteadyTimer("switchLog",  5* 1000 * 1000, &ProcessCmd::SwitchLogTimerCallback, this, true);
  std::thread process_thread(&ProcessCmd::CmdProcesser, this);
  process_thread.detach();
}

void ProcessCmd::SwitchLogTimerCallback(const hj_bf::HJSteadyTimerEvent&) {
  HJ_ERROR("SwitchLogTimerCallback ");
  CheckVersion();
}
void ProcessCmd::CreateSwitchLog(const std::string& log_version) {
  std::string payload;
  if (log_version == "release") {
    payload = R"({"logVersion": "release"})";
    HJ_ERROR("payload release :%s", payload.c_str());
  } else if (log_version == "debug" || log_version == "local" || log_version == "daily") {
    payload = R"({"logVersion": "debug"})";
    HJ_ERROR("payload debug :%s", payload.c_str());
  } else {
    HJ_ERROR("CreateSwitchLog log_version:%s", log_version.c_str());
    return;
  }
  SwitchLog(payload);
}
void ProcessCmd::CreateOldVersionConfigure(const std::string& build_class, const std::string& fw_version) {
  rapidjson::Document document_out;
  document_out.SetObject();
  rapidjson::Value keyValue(build_class.c_str(), document_out.GetAllocator());
  document_out.AddMember("build_class", keyValue, document_out.GetAllocator());

  rapidjson::Value keyValue2(fw_version.c_str(), document_out.GetAllocator());
  document_out.AddMember("fw_ver", keyValue2, document_out.GetAllocator());

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document_out.Accept(writer);
  std::ofstream outfile;
  outfile.open(kOldVersionPath, std::ios_base::out);
  if (!outfile.is_open()) {
    HJ_ERROR("open or create file failed:%s", kVersionPath);
    return;
  }
  outfile << buffer.GetString() << std::endl;
  outfile.close();
}
void ProcessCmd::CheckVersion() {
  std::string build_class;
  std::string fw_version;
  std::ifstream stream_version(kVersionPath);
  if (!stream_version.is_open()) {
    HJ_ERROR("open file failed:%s", kVersionPath);
    return;
  }
  rapidjson::Document document;
  std::string jsonString((std::istreambuf_iterator<char>(stream_version)), std::istreambuf_iterator<char>());

  if (!document.Parse(jsonString.data()).HasParseError()) {
    if (document.HasMember("build_class") && document["build_class"].IsString()) {
      build_class = document["build_class"].GetString();
    }
    if (document.HasMember("fw_ver") && document["fw_ver"].IsString()) {
      fw_version = document["fw_ver"].GetString();
    }
  } else {
    HJ_ERROR("parse file failed:%s", kVersionPath);
    return;
  }
  if (build_class.empty() || fw_version.empty()) {
    HJ_ERROR("empty build_class:%s, fw_version:%s", build_class.c_str(), fw_version.c_str());
    return;
  }
  stream_version.close();

  std::ifstream stream_version_old(kOldVersionPath);
  if (!stream_version_old.is_open()) {
    HJ_ERROR("open file failed:%s", kOldVersionPath);
    CreateOldVersionConfigure(build_class, fw_version);
    CreateSwitchLog(build_class);
    return;
  }

  std::string jsonString_old((std::istreambuf_iterator<char>(stream_version_old)), std::istreambuf_iterator<char>());
  std::string build_class_old;
  std::string fw_version_old;
  if (!document.Parse(jsonString_old.data()).HasParseError()) {
    if (document.HasMember("build_class") && document["build_class"].IsString()) {
      build_class_old = document["build_class"].GetString();
    }
    if (document.HasMember("fw_ver") && document["fw_ver"].IsString()) {
      fw_version_old = document["fw_ver"].GetString();
    }
  } else {
    HJ_ERROR("parse file failed:%s", kOldVersionPath);
    stream_version_old.close();
    CreateOldVersionConfigure(build_class, fw_version);
    CreateSwitchLog(build_class);
    return;
  }
  if (build_class_old.empty() || fw_version_old.empty()) {
    HJ_ERROR("old configure empty build_class:%s, fw_version:%s", build_class_old.c_str(), fw_version_old.c_str());
    CreateOldVersionConfigure(build_class, fw_version);
    CreateSwitchLog(build_class);
    return;
  }

  if (fw_version_old == fw_version) {
    HJ_ERROR("build_class_old == fw_version");
    return;
  }
  CreateOldVersionConfigure(build_class, fw_version);
  CreateSwitchLog(build_class);
}
void ProcessCmd::IotOnlineCallback(const hj_interface::AppOnlineType::ConstPtr& msg) {
  static bool update_log_version_flag = false;
  if (false == update_log_version_flag) {
    HJ_INFO("ProcessCmd IotOnlineCallback pub type:%d", msg->type);
    if (hj_interface::AppOnlineType::BT_IOT == msg->type || hj_interface::AppOnlineType::IOT == msg->type) {

      hj_interface::AppData temp_data;
      temp_data.key = kCmdProcessUpdateLogVersionKey;
      std::shared_ptr<hj_bf::NodeConfig> node_config = std::make_shared<hj_bf::NodeConfig>();
      if (hj_bf::readRemoteConfigure(node_config, false)) {
        rapidjson::Document document;
        document.SetObject();
        rapidjson::Value keyValue(node_config->log_level.c_str(), document.GetAllocator());
        document.AddMember("logVersion", keyValue, document.GetAllocator());
        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        document.Accept(writer);
        temp_data.payload = buffer.GetString();
      } else {
        HJ_ERROR("readRemoteConfigure FAIL, will not update logVersion");
#ifdef HJ_RELEASE_VER
        std::string payload = R"({"logVersion": "release"})";
#else
        std::string payload = R"({"logVersion": "debug"})";
#endif
        SwitchLog(payload);//MINOS 重复发了
        // return;
      }
      hj_interface::AppMsg temp_msg;
      temp_msg.appdata.push_back(temp_data);
      temp_msg.to = hj_interface::AppMsg::SHADOW;
      update_log_version_pub_.publish(temp_msg);
    }
  }
}

void ProcessCmd::CmdProcesser() {
  HJ_INFO("IN CmdProcesser");
  prctl(PR_SET_NAME, "CmdProcesser");
  uint64_t log_id = 0;
  std::vector<int> types;
  std::unique_lock<std::mutex> weakup_lk(queue_mutex_);
  std::deque<hj_interface::AppData> temp_deque;
  while (true) {
    queue_cond_.wait_for(weakup_lk, std::chrono::minutes(kProcessTimeoutMinutes));
    HJ_INFO("trigger CmdProcesser");
    if (exit_flag_ == true) {
      return;
    }
    while (!deque_.empty()) {
      temp_deque.swap(deque_);
      weakup_lk.unlock();
      while (!temp_deque.empty()) {
        hj_interface::AppData temp = temp_deque.front();
        temp_deque.pop_front();
        types.clear();
        if (ParseMsg(temp.key, temp.payload)) {
        } else {
          HJ_ERROR("ParseMsg error");
        }
      }
      weakup_lk.lock();
    }
  }
}
void ProcessCmd::UpdateLogLevelCallback(const std_msgs::UInt8::ConstPtr& msg) {
  HJ_ERROR("UpdateLogLevelCallback");
#ifdef HJ_RELEASE_VER
  std::string payload = R"({"logVersion": "release"})";
#else
  std::string payload = R"({"logVersion": "debug"})";
#endif
  SwitchLog(payload);
}

void ProcessCmd::GetCmdCallback(const hj_interface::AppData::ConstPtr& msg) {
  HJ_INFO("GetCmdCallback");
  std::unique_lock<std::mutex> lk(queue_mutex_);
  deque_.emplace_back(*msg);
  queue_cond_.notify_one();
}
bool ProcessCmd::SwitchLog(const std::string& payload) {
  bool ret = true;
  std::string switch_log;
  std::vector<std::string> node_vec;
  node_vec.emplace_back("/collect_node");
  node_vec.emplace_back("/middleware_node");
  node_vec.emplace_back("/slam_node");
  node_vec.emplace_back("/planning_node");
  node_vec.emplace_back("/utils_node");
  HJ_ERROR("SwitchLog payload :%s",payload.c_str());
  rapidjson::Document document;
  document.Parse(payload.c_str());
  if (!document.HasParseError() && document.IsObject()) {
    if (document.HasMember("logVersion") && document["logVersion"].IsString()) {
      switch_log = document["logVersion"].GetString();
      hj_bf::HJClient client;
      roscpp::SetLoggerLevel srv;
      srv.request.logger = "ros";
      client = hj_bf::HJCreateClient<roscpp::SetLoggerLevel>("/collect_node/set_logger_level");
      if (switch_log == kSwitchLogOpen) {
        srv.request.level = "INFO";
      } else if (switch_log == kSwitchLogClose) {
        srv.request.level = "ERROR";
      }
      for (int i = 0; i < node_vec.size(); i++) {
        std::string service_name = node_vec[i] + kRosLevelName;
        client = hj_bf::HJCreateClient<roscpp::SetLoggerLevel>(service_name);
        if (client.exists()) {
          if (client.call(srv)) {
            HJ_INFO("set log level success: %s", service_name.c_str());
          } else {
            HJ_ERROR("set log level fail: %s", service_name.c_str());
            ret = false;
          }
        } else {
          HJ_ERROR("node server is not exist: %s", service_name.c_str());
        }
      }
      if (true == ret) {
        if (hj_bf::setLogLevelConfigure(switch_log)) {
          hj_interface::AppData temp_data;
          temp_data.key = kCmdProcessUpdateLogVersionKey;
          rapidjson::Document document;
          document.SetObject();
          rapidjson::Value keyValue(switch_log.c_str(), document.GetAllocator());
          document.AddMember("logVersion", keyValue, document.GetAllocator());
          rapidjson::StringBuffer buffer;
          rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
          document.Accept(writer);
          temp_data.payload = buffer.GetString();

          hj_interface::AppMsg temp_msg;
          temp_msg.appdata.push_back(temp_data);
          temp_msg.to = hj_interface::AppMsg::SHADOW;
          update_log_version_pub_.publish(temp_msg);

        } else {
          HJ_ERROR("setLogLevelConfigure fail");
        }
      } else {
        HJ_ERROR("client call fail, no need save log level");
      }
      // if (client.call(srv)) {
      //   HJ_INFO("set log level success");
      //   return true;
      // } else {
      //   HJ_ERROR("set log level fail");
      //   return false;
      // }
    } else {
      HJ_ERROR("cant analysis SwitchLog payload :%s", payload.c_str());
      return false;
    }
  } else {
    HJ_ERROR("cant analysis SwitchLog payload :%s", payload.c_str());
    return false;
  }
  return ret;
}
bool ProcessCmd::ParseMsg(const std::string& key, const std::string& payload) {
  HJ_INFO("key:%s, payload:%s", key.c_str(), payload.c_str());
  bool ret = true;
  if (key == "switchLog") {
    ret = SwitchLog(payload);
    hj_interface::AppMsg resp;
    hj_interface::AppData appdata;
    resp.from = 0x06;
    appdata.res = ret ? 0 : -1;
    appdata.key = key;
    appdata.payload = payload;
    resp.appdata.emplace_back(appdata);
    rsp_to_cloud_pub_.publish(resp);
    return ret;
  }
  return false;
}

ProcessCmd& ProcessCmd::GetInstance() {
  static ProcessCmd process_cmd_instance;
  return process_cmd_instance;
}
ProcessCmd::~ProcessCmd() {
  std::unique_lock<std::mutex> lk(queue_mutex_);
  exit_flag_ = true;
  queue_cond_.notify_one();
}
}  // namespace process_cmd
