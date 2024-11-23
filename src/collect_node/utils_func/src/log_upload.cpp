// @file log_upload.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2024-7-24)
#include "log_upload.h"

#include <hj_interface/AppData.h>
#include <hj_interface/FileUpload.h>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <boost/filesystem.hpp>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>

#include "hj_zip.h"
#include "log.h"
#include "node_factory.h"
#include "std_msgs/String.h"
namespace log_upload {

constexpr int kProcessTimeoutMinutes = 10;
constexpr char kLogUploadTopicName[] = "/getLogRecord";
constexpr char kLogUploadTempPath[] = "/tmp";
constexpr char kLogUploadCoreDumpDir[] = "/userdata/hj/log/core_dump";
constexpr char kLogUploadLogDir[] = "/userdata/hj/log/logging";
constexpr char kLogUploadSensorDataDir[] = "/userdata/hj/log/sensor_data_alg";
constexpr char kLogUploadErrorLog[] = "log_err";
constexpr char kLogUploadFileTopicName[] = "/upload/file";
constexpr char kLogUploadDefaultSN[] = "000";
constexpr char kLogUploadConfigFile[] = "/tmp/devInfo.json";

void PrintListName(const std::vector<std::string>& list) {
  HJ_INFO("minos pack file list:");
  for (auto path_name : list) {
    HJ_INFO("%s", path_name.c_str());
  }
}

bool GetSnFromConfig(std::string& sn) {
  std::ifstream in(kLogUploadConfigFile, std::ios::in | std::ios::binary);
  if (!in.is_open()) {
    HJ_ERROR(" cant open,name  :%s", kLogUploadConfigFile);
    return false;
  } else {
    rapidjson::Document document;
    rapidjson::IStreamWrapper isw(in);
    document.ParseStream(isw);

    if (document.HasParseError()) {
      HJ_ERROR("cant analysis  file name:%s", kLogUploadConfigFile);
      return false;
    }
    if (document.IsObject()) {
      if (document.HasMember("sn") && document["sn"].IsString()) {
        sn = document["sn"].GetString();
      }
    }
  }
  return true;
}

void GetFiles(boost::filesystem::path& path, std::vector<std::string>& files) {
  if (boost::filesystem::exists(path)) {
    for (boost::filesystem::directory_iterator it(path); it != boost::filesystem::directory_iterator(); ++it) {
      //  for (const auto& entry : boost::filesystem::directory_iterator(directory_path2)) {
      if (boost::filesystem::is_regular_file(it->path())) {
        files.emplace_back(it->path().string());
      }
    }
  }
}

void GetFilesWithPrefix(boost::filesystem::path& path, std::vector<std::string>& files, const std::string prefix) {
  if (boost::filesystem::exists(path)) {
    for (boost::filesystem::directory_iterator it(path); it != boost::filesystem::directory_iterator(); ++it) {
      if (boost::filesystem::is_regular_file(it->path()) && (it->path().filename().string().find(prefix) == 0)) {
        files.emplace_back(it->path().string());
      }
    }
  }
}
bool LogUpload::PackMuc(const std::string& timestamp_str, std::string& pack_name) {
  pack_name = kLogUploadTempPath;
  pack_name += "/" + timestamp_str + "-Aiper" + LogUpload::GetInstance().GetSn() + "-mcu.zip";

  std::vector<std::string> all_file = {};
  boost::filesystem::path directory_path = "/userdata/hj/log/mcukey";
  GetFiles(directory_path, all_file);
  PrintListName(all_file);
  return hj_bf::CreateZipFileByFiles(pack_name, all_file);
}
bool LogUpload::PackSoc(const std::string& timestamp_str, std::string& pack_name) {
  pack_name = kLogUploadTempPath;
  pack_name += "/" + timestamp_str + "-Aiper" + LogUpload::GetInstance().GetSn() + "-soc.zip";

  boost::filesystem::path directory_path = "/tmp/log";
  std::vector<std::string> all_file = {};
  GetFiles(directory_path, all_file);

  directory_path = "/userdata/log";
  GetFiles(directory_path, all_file);

  all_file.emplace_back("/etc/version");
  PrintListName(all_file);
  return hj_bf::CreateZipFileByFiles(pack_name, all_file);
}

bool LogUpload::PackMiddleware(const std::string& timestamp_str, std::string& pack_name) {
  pack_name = kLogUploadTempPath;
  pack_name += "/" + timestamp_str + "-Aiper" + LogUpload::GetInstance().GetSn() + "-middleware.zip";
  std::vector<std::string> all_file = {};
  std::string prefix = "collect_node_";
  boost::filesystem::path directory_path = log_dir_;
  GetFilesWithPrefix(directory_path, all_file, prefix);
  directory_path = core_dump_dir_ + "/dump_collect_node";
  GetFiles(directory_path, all_file);
  GetFilesWithPrefix(directory_path, all_file, error_dir_);
  PrintListName(all_file);
  return hj_bf::CreateZipFileByFiles(pack_name, all_file);
}

bool LogUpload::PackSlam(const std::string& timestamp_str, std::string& pack_name) {
  pack_name = kLogUploadTempPath;
  pack_name += "/" + timestamp_str + "-Aiper" + LogUpload::GetInstance().GetSn() + "-slam.zip";
  std::vector<std::string> all_file = {};
  std::string prefix = "slam_node_";
  boost::filesystem::path directory_path = log_dir_;
  GetFilesWithPrefix(directory_path, all_file, prefix);
  directory_path = core_dump_dir_ + "/dump_slam_node";
  GetFiles(directory_path, all_file);
  GetFilesWithPrefix(directory_path, all_file, error_dir_);
  PrintListName(all_file);
  return hj_bf::CreateZipFileByFiles(pack_name, all_file);
}

bool LogUpload::PackPlanning(const std::string& timestamp_str, std::string& pack_name) {
  pack_name = kLogUploadTempPath;
  pack_name += "/" + timestamp_str + "-Aiper" + LogUpload::GetInstance().GetSn() + "-planning.zip";
  std::vector<std::string> all_file = {};
  std::string prefix = "planning_node_";
  boost::filesystem::path directory_path = log_dir_;
  GetFilesWithPrefix(directory_path, all_file, prefix);
  directory_path = core_dump_dir_ + "/dump_planning_node";
  GetFiles(directory_path, all_file);
  GetFilesWithPrefix(directory_path, all_file, error_dir_);
  PrintListName(all_file);
  return hj_bf::CreateZipFileByFiles(pack_name, all_file);
}

bool LogUpload::PackApp(const std::string& timestamp_str, std::string& pack_name) {
  pack_name = kLogUploadTempPath;
  pack_name += "/" + timestamp_str + "-Aiper" + LogUpload::GetInstance().GetSn() + "-app.zip";
  std::vector<std::string> all_file = {};
  std::string prefix = "middleware_node_";
  boost::filesystem::path directory_path = log_dir_;
  GetFilesWithPrefix(directory_path, all_file, prefix);
  directory_path = core_dump_dir_ + "/dump_middleware_node";
  GetFiles(directory_path, all_file);
  GetFilesWithPrefix(directory_path, all_file, error_dir_);
  PrintListName(all_file);
  return hj_bf::CreateZipFileByFiles(pack_name, all_file);
}

bool LogUpload::PackUtils(const std::string& timestamp_str, std::string& pack_name) {
  pack_name = kLogUploadTempPath;
  pack_name += "/" + timestamp_str + "-Aiper" + LogUpload::GetInstance().GetSn() + "-utils.zip";
  std::vector<std::string> all_file = {};
  std::string prefix = "utils_node_";
  boost::filesystem::path directory_path = log_dir_;
  GetFilesWithPrefix(directory_path, all_file, prefix);
  directory_path = core_dump_dir_ + "/dump_utils_node";
  GetFiles(directory_path, all_file);
  GetFilesWithPrefix(directory_path, all_file, error_dir_);
  PrintListName(all_file);
  return hj_bf::CreateZipFileByFiles(pack_name, all_file);
}

bool LogUpload::PackSensorData(const std::string& timestamp_str, std::string& pack_name) {
  pack_name = kLogUploadTempPath;
  pack_name += "/" + timestamp_str + "-Aiper" + LogUpload::GetInstance().GetSn() + "-sensor_data.zip";
  return hj_bf::CreateZipFileByDir(pack_name, sensor_data_);
}

bool LogUpload::PackAll(const std::string& timestamp_str, std::string& pack_name) {
  pack_name = kLogUploadTempPath;
  pack_name += "/" + timestamp_str + "-Aiper" + LogUpload::GetInstance().GetSn() + "-all.zip";
  std::vector<std::string> all_file = {};

  boost::filesystem::path directory_path = log_dir_;
  GetFiles(directory_path, all_file);
  directory_path = "/tmp/log";
  GetFiles(directory_path, all_file);
  directory_path = "/userdata/log";
  GetFiles(directory_path, all_file);
  directory_path = "/userdata/hj/log/mcukey";
  GetFiles(directory_path, all_file);
  all_file.emplace_back("/etc/version");
  directory_path = core_dump_dir_;
  GetFiles(directory_path, all_file);
  PrintListName(all_file);
  return hj_bf::CreateZipFileByFiles(pack_name, all_file);
}

bool LogUpload::Pack(enum LogUploadTypes type, std::string& pack_name) {
  double timestamp = hj_bf::HJTime::now().toSec();
  HJ_INFO("IN Pack type:%d", type);
  std::string timestamp_str = std::to_string(timestamp);
  switch (type) {
    case kLogUploadSoc: {
      return PackSoc(timestamp_str, pack_name);
    }
    case kLogUploadMcu: {
      return PackMuc(timestamp_str, pack_name);
    }
    case kLogUploadMiddleware: {
      HJ_INFO("IN Pack kLogUploadMiddleware");
      return PackMiddleware(timestamp_str, pack_name);
    }
    case kLogUploadSlam: {
      return PackSlam(timestamp_str, pack_name);
    }
    case kLogUploadPlanning: {
      return PackPlanning(timestamp_str, pack_name);
    }
    case kLogUploadApp: {
      return PackApp(timestamp_str, pack_name);
    }
    case kLogUploadUtils: {
      HJ_INFO("IN Pack kLogUploadUtils1");
      return PackUtils(timestamp_str, pack_name);
      //      return PackApp(timestamp_str, pack_name);
    }
    case kLogUploadSensorData: {
      HJ_INFO("IN Pack Sensor_data");
      return PackSensorData(timestamp_str, pack_name);
      //      return PackApp(timestamp_str, pack_name);
    }
    case kLogUploadAll: {
      return PackAll(timestamp_str, pack_name);
    }
    default: {
      HJ_INFO("IN Pack default");
      return false;
    }
  }
}

LogUpload::LogUpload() {
  sn_ = kLogUploadDefaultSN;
  core_dump_dir_ = kLogUploadCoreDumpDir;
  log_dir_ = kLogUploadLogDir;
  error_dir_ = kLogUploadErrorLog;
  sensor_data_ = kLogUploadSensorDataDir;
  upload_file_cmd_sub_ = hj_bf::HJSubscribe(log_upload::kLogUploadTopicName, 10, &LogUpload::GetCmdCallback, this);
  std::thread process_thread(&LogUpload::UploadProcesser, this);
  process_thread.detach();
}
std::string& LogUpload::GetSn() { return sn_; }

void LogUpload::UploadProcesser() {
  HJ_INFO("IN UploadProcesser");
  if (!GetSnFromConfig(sn_)) {
    HJ_ERROR("GET sn fail");
  }
  uint64_t log_id = 0;
  std::vector<int> types;
  std::unique_lock<std::mutex> weakup_lk(queue_cond_mutex_);
  std::deque<hj_interface::AppData> temp_deque;
  while (true) {
    //    std::cv_status::timeout == queue_cond_.wait_for(weakup_lk, std::chrono::minutes(kProcessTimeoutMinutes));
    queue_cond_.wait_for(weakup_lk, std::chrono::minutes(kProcessTimeoutMinutes));
    HJ_INFO("trigger queue_cond_");
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
        if (ParseMsg(temp.payload, log_id, types)) {
          for (size_t i = 0; i < types.size(); i++) {
            TriggerS3UploadFile(types[i], log_id);
          }
        } else {
          HJ_ERROR("ParseMsg error");
        }
      }
      weakup_lk.lock();
    }
  }
}

void LogUpload::GetCmdCallback(const hj_interface::AppData::ConstPtr& msg) {
  HJ_INFO("GetCmdCallback");
  std::unique_lock<std::mutex> lk(queue_mutex_);
  deque_.emplace_back(*msg);
  queue_cond_.notify_one();
}

bool LogUpload::ParseMsg(const std::string& payload, uint64_t& id, std::vector<int>& types_out) {
  HJ_INFO("payload:%s", payload.c_str());
  rapidjson::Document document;
  document.Parse(payload.c_str());
  if (!document.HasParseError() && document.IsObject()) {
    if (document.HasMember("controlLogId") && document["controlLogId"].IsUint64()) {
      id = document["controlLogId"].GetUint64();
    } else {
      HJ_ERROR("cant analysis uploadfile cmd key%s, is int %d, has:%d", "controlLogId",
               document["controlLogId"].IsUint64(), document.HasMember("controlLogId"));
    }
    if (document.HasMember("type") && document["type"].IsArray()) {
      const rapidjson::Value& types = document["type"];
      for (rapidjson::SizeType i = 0; i < types.Size(); ++i) {
        const rapidjson::Value& type = types[i];
        types_out.emplace_back(type.GetUint64());
      }
    } else if (document.HasMember("type") && document["type"].IsUint64()) {
      types_out.emplace_back(document["type"].GetUint64());
      HJ_INFO("type just one");
    } else {
      HJ_ERROR("NO  type ");
      return false;
    }
  } else {
    HJ_ERROR("cant analysis uploadfile cmd string to json");
    return false;
  }
  return true;
}
bool LogUpload::TriggerS3UploadFile(int type, uint64_t log_id) {
  static hj_bf::HJPublisher log_upload_pub_s3 =
      hj_bf::HJAdvertise<hj_interface::FileUpload>(kLogUploadFileTopicName, 10);
  std::string pack_name;
  if (Pack(static_cast<enum LogUploadTypes>(type), pack_name)) {
    hj_interface::FileUpload temp_path;
    temp_path.filePath = pack_name;
    temp_path.deleteOnSuccess = 1;
    temp_path.logId = log_id;
    temp_path.type = hj_interface::FileUpload::DEVICELOG;
    HJ_INFO("Will pub s3, %ld", log_id);
    log_upload_pub_s3.publish(temp_path);
    return true;
  } else {
    HJ_ERROR("Pack fail log_id:%ld", log_id);
    return false;
  }
}

LogUpload& LogUpload::GetInstance() {
  static LogUpload log_upload_instance;
  return log_upload_instance;
  //  return std::make_shared<LogUpload>();
}
LogUpload::~LogUpload() {
  std::unique_lock<std::mutex> lk(queue_mutex_);
  exit_flag_ = true;
  queue_cond_.notify_one();
}
}  // namespace log_upload
