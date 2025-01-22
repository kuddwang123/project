// @file log_upload.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2024-7-24)
#ifndef INCLUDE_LOG_UPLOAD_H_  // NOLINT
#define INCLUDE_LOG_UPLOAD_H_  // NOLINT

#include <hj_interface/AppData.h>

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "node_factory.h"

namespace log_upload {
enum LogUploadTypes {
  kLogUploadAll = 1,
  kLogUploadSoc = 2,
  kLogUploadMcu = 3,
  kLogUploadMiddleware = 4,
  kLogUploadSlam = 5,
  kLogUploadPlanning = 6,
  kLogUploadApp = 7,
  kLogUploadUtils = 8,
  kLogUploadSensorData = 9,
  kLogUploadReserve = 10
};

class LogUpload {
 public:
  static LogUpload& GetInstance();
  ~LogUpload();
  std::string& GetSn();

 private:
  LogUpload();
  LogUpload(const LogUpload&) = delete;
  LogUpload(LogUpload&&) = delete;
  LogUpload& operator=(const LogUpload&) = delete;
  LogUpload& operator=(LogUpload&&) = delete;
  void UploadProcesser();
  void GetCmdCallback(const hj_interface::AppData::ConstPtr& msg);
  bool ParseMsg(const std::string& payload, uint64_t& id, std::vector<int>& types_out);
  bool TriggerS3UploadFile(int type, uint64_t log_id);
  bool PackMuc(const std::string& timestamp_str, std::string& pack_name);
  bool PackSoc(const std::string& timestamp_str, std::string& pack_name);
  bool PackMiddleware(const std::string& timestamp_str, std::string& pack_name);
  bool PackSlam(const std::string& timestamp_str, std::string& pack_name);
  bool PackPlanning(const std::string& timestamp_str, std::string& pack_name);
  bool PackApp(const std::string& timestamp_str, std::string& pack_name);
  bool PackUtils(const std::string& timestamp_str, std::string& pack_name);
  bool PackSensorData(const std::string& timestamp_str, std::string& pack_name);
  bool PackAll(const std::string& timestamp_str, std::string& pack_name);
  bool Pack(enum LogUploadTypes type, std::string& pack_name);
  std::deque<hj_interface::AppData> deque_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cond_;
  hj_bf::HJSubscriber upload_file_cmd_sub_;
  bool exit_flag_{false};
  std::string sn_;
  std::string core_dump_dir_;
  std::string log_dir_;
  std::string log_other_;
  std::string error_dir_;
  std::string sensor_data_;
  std::string password_;
};
}  // namespace log_upload
#endif  //  INCLUDE_LOG_UPLOAD_H_  // NOLINT
