/**
 * @file monitor.cpp
 * @author hao wu (clayderman@yardbot.net)
 * @brief 
 * @version 0.1
 * @date 2024-07-05
 * 
 * @copyright Copyright 2023 HJ Technology Co.Ltd. All rights reserved
 * 
 */
#include "monitor.h"
#include <sys/stat.h>
#include <sys/sysinfo.h>
#include <sys/time.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>
#include <cstring>
#include <cmath>
#include <unordered_map>
#include "log.h"
#include "big_data.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "status_code.h"
#include "node_cache.h"
#include "shm_interface.h"
#include "boost/filesystem.hpp"
#include "hj_interface/HealthCheckCode.h"
#include "hj_interface/BootType.h"

namespace collect_node_utils_func {
constexpr char kNodeMonitorJson[] = "/userdata/hj/log/node_monitor.json";
constexpr char kTaskName[] = "/userdata/hj/log/.task_name";

std::unordered_map<std::string, std::string> kNodeInfoMap = {
  {"collect_node", "collectNode"},
  {"middleware_node", "middlewareNode"},
  {"planning_node", "planningNode"},
  {"slam_node", "slamNode"},
  {"utils_node", "utilsNode"}
};
std::unordered_map<std::string, uint32_t> kNodeMapCodeValue = {
  {"collect_node", COLLECT_NODE_COREDUMP_ERROR},
  {"middleware_node", MIDDLEWARE_NODE_COREDUMP_ERROR},
  {"planning_node", PLANING_NODE_COREDUMP_ERROR},
  {"slam_node", SLAM_NODE_COREDUMP_ERROR},
  {"utils_node", UTILS_NODE_COREDUMP_ERROR}
};

void Monitor::Stringsplit(std::string str, const char split, std::vector<std::string>& rst) {
  std::istringstream iss(str);  // 输入流
  std::string token;  // 接收缓冲区
  while (getline(iss, token, split)) {  // 以split为分隔符
    rst.emplace_back(token);
  }
}

int64_t Monitor::GetCpuTotalOccupy() {
  // get total cpu use time
  // different mode cpu occupy time
  int64_t user_time = 0;
  int64_t nice_time = 0;
  int64_t system_time = 0;
  int64_t idle_time = 0;

  FILE* fd = nullptr;
  char buff[1024] = {0};

  fd = fopen("/proc/stat", "r");
  if (nullptr == fd) {
    return 0;
  }

  if (fgets(buff, sizeof(buff), fd) == nullptr) {
    HJ_ERROR("fgets failed");
  }
  char name[64] = {0};
  sscanf(buff, "%s %ld %ld %ld %ld", name, &user_time, &nice_time, &system_time, &idle_time);
  fclose(fd);

  return (user_time + nice_time + system_time + idle_time);
}

int64_t Monitor::GetCpuProcOccupy(int pid) {
  // get specific pid cpu use time

  FILE* fd = nullptr;
  char line_buff[1024] = {0};
  std::string file_name = "/proc/" + std::to_string(pid) + "/stat";

  fd = fopen(file_name.data(), "r");
  if (nullptr == fd) {
    return 0;
  }

  if (fgets(line_buff, sizeof(line_buff), fd) == nullptr) {
    HJ_ERROR("fgets failed");
  }
  fclose(fd);

  std::vector<std::string> items;
  Stringsplit(line_buff, ' ', items);
  int64_t utime = std::stoll(items[13]);
  int64_t stime = std::stoll(items[14]);
  // int64_t cutime = std::stoll(items[15]);
  // int64_t cstime = std::stoll(items[16]);

  return (utime + stime);
}

uint32_t Monitor::GetCpuUsageRatio(int pid) {
  int64_t totalcputime1 = 0, totalcputime2 = 0;
  int64_t procputime1 = 0, procputime2 = 0;

  totalcputime1 = GetCpuTotalOccupy();
  procputime1 = GetCpuProcOccupy(pid);

  // FIXME: the 200ms is a magic number, works well
  usleep(200000);

  totalcputime2 = GetCpuTotalOccupy();
  procputime2 = GetCpuProcOccupy(pid);

  float pcpu = 0.0;
  if (0 != totalcputime2 - totalcputime1) {
    pcpu = (procputime2 - procputime1) / static_cast<float>(totalcputime2 - totalcputime1);
  } else {
    HJ_INFO("totalcputime2 - totalcputime1 is zero");
  }

  int cpu_num = get_nprocs();
  pcpu *= cpu_num;  // should multiply cpu num in multiple cpu machine
  return static_cast<uint32_t>(pcpu * 100);  // convert to percenage
}

// get specific process physical memeory occupation size by pid (MB)
int32_t Monitor::GetMemoryUsage(int pid) {
  char file_name[64] = {0};
  FILE* fd = nullptr;
  char line_buff[512] = {0};
  snprintf(file_name, sizeof(file_name), "/proc/%d/status", pid);

  fd = fopen(file_name, "r");
  if (nullptr == fd) {
    return 0;
  }

  char name[64];
  int vmrss = 0;
  for (int i = 0; i < VMRSS_LINE - 1; i++) {
    if (fgets(line_buff, sizeof(line_buff), fd) == nullptr) {
      HJ_ERROR("fgets failed");
    }
  }

  if (fgets(line_buff, sizeof(line_buff), fd) != nullptr) {  // get vmrss line
    sscanf(line_buff, "%s %d", name, &vmrss);
  }
  fclose(fd);

  return vmrss;
}

int32_t Monitor::GetAllMemory() {
  const char* file_name = "/proc/meminfo";
  FILE* fd = nullptr;
  char line_buff[512] = {0};

  fd = fopen(file_name, "r");
  if (nullptr == fd) {
    return 0;
  }

  char name[64];
  int vmrss = 0;
  if (fgets(line_buff, sizeof(line_buff), fd) != nullptr) {
    sscanf(line_buff, "%s %d", name, &vmrss);
  }
  fclose(fd);

  return vmrss;
}

int32_t Monitor::GetProcessPidByName(const std::string& proc_name) {
  FILE *fp = nullptr;
  char buf[50] = {'\0'};
  int32_t pid = -1;
  std::string cmd = "pidof " + proc_name;

  if ((fp = popen(cmd.data(), "r")) != nullptr) {
    if (fgets(buf, 50, fp) != nullptr) {
      pid = atoi(buf);
    }
    pclose(fp);
  } else {
    HJ_ERROR("popen failed");
  }

  return pid;
}

Monitor::~Monitor() {
}

// void Monitor::TaskCallBack(const std_msgs::UInt8::ConstPtr& msg) {
//   // task_.store(msg->data);
//   HJ_INFO("task_ is %d", msg->data);
// }

void Monitor::TaskNameCallBack(const std_msgs::String::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (msg->data == "finish") {
    task_name_ = "";
  } else {
    task_name_ = msg->data;
  }
  std::ofstream file_writer(kTaskName);
  if (file_writer) {
    file_writer << task_name_;
    file_writer.close();
  }
  
  HJ_INFO("task name: %s", task_name_.data());
}

void Monitor::CalCpuMemeory() {
  int32_t total_memery = GetAllMemory();
  // uint8_t old_task = 0;
  uint64_t collect_node_cpu = 0, collect_node_memory = 0;
  uint64_t slam_node_cpu = 0, slam_node_memory = 0;
  uint64_t planning_node_cpu = 0, planning_node_memory = 0;
  uint64_t middleware_node_cpu = 0, middleware_node_memory = 0;
  uint64_t utils_node_cpu = 0, utils_node_memory = 0;
  int count = 0;
  bool is_start = false;  // task start flag
  std::string task_name = "";
  std::string associated_task = "";

  while (true) {
    // old_task = task_.load();
    {
      std::lock_guard<std::mutex> lock(mtx_);
      task_name = task_name_;  // associatedTaskId
      if (!task_name.empty()) {
        associated_task = task_name_;
      }
    }
    if (task_name.empty() && is_start && count > 0) {
      ros::Time timestamp = ros::Time::now();
      double time_now = timestamp.toSec();
      int64_t timestamp_ms = static_cast<int64_t>(time_now * 1000);
      std::string collect_node_cpu_avg = std::to_string(1.0 * collect_node_cpu / count);
      std::string collect_node_mem_avg = std::to_string(100.0 * collect_node_memory / count / total_memery);
      std::string str = R"({"event": "nodeOccupancyEvent", "associatedTaskId": ")" + associated_task + R"(", "node": "collectNode", "cpuOcc": )" +
                      collect_node_cpu_avg.substr(0, collect_node_cpu_avg.find('.') + 2) +
                      R"(, "memOcc": )" + collect_node_mem_avg.substr(0, collect_node_mem_avg.find('.') + 2) +
                      R"(, "time": )" +  std::to_string(timestamp_ms) +  R"(, "fwVersion": ")" + fw_version_ + R"("})";
      big_data::InsertBigdata(str);
      HJ_INFO("big_data: %s\n", str.c_str());
      std::string slam_node_cpu_avg = std::to_string(1.0 * slam_node_cpu / count);
      std::string slam_node_mem_avg = std::to_string(100.0 * slam_node_memory / count / total_memery);
      str = R"({"event": "nodeOccupancyEvent", "associatedTaskId": ")" + associated_task + R"(", "node": "slamNode", "cpuOcc": )" +
            slam_node_cpu_avg.substr(0, slam_node_cpu_avg.find('.') + 2) +
            R"(, "memOcc": )" + slam_node_mem_avg.substr(0, slam_node_mem_avg.find('.') + 2) +
            R"(, "time": )" +  std::to_string(timestamp_ms) +  R"(, "fwVersion": ")" + fw_version_ + R"("})";
      big_data::InsertBigdata(str);
      HJ_INFO("big_data: %s\n", str.c_str());
      std::string planning_node_cpu_avg = std::to_string(1.0 * planning_node_cpu / count);
      std::string planning_node_mem_avg = std::to_string(100.0 * planning_node_memory / count / total_memery);
      str = R"({"event": "nodeOccupancyEvent", "associatedTaskId": ")" + associated_task + R"(", "node": "planningNode", "cpuOcc": )" +
            planning_node_cpu_avg.substr(0, planning_node_cpu_avg.find('.') + 2) +
            R"(, "memOcc": )" + planning_node_mem_avg.substr(0, planning_node_mem_avg.find('.') + 2) +
            R"(, "time": )" +  std::to_string(timestamp_ms) +  R"(, "fwVersion": ")" + fw_version_ + R"("})";
      big_data::InsertBigdata(str);
      HJ_INFO("big_data: %s\n", str.c_str());
      std::string middleware_node_cpu_avg = std::to_string(1.0 * middleware_node_cpu / count);
      std::string middleware_node_mem_avg = std::to_string(100.0 * middleware_node_memory / count / total_memery);
      str = R"({"event": "nodeOccupancyEvent", "associatedTaskId": ")" + associated_task + R"(", "node": "middlewareNode", "cpuOcc": )" +
            middleware_node_cpu_avg.substr(0, middleware_node_cpu_avg.find('.') + 2) +
            R"(, "memOcc": )" + middleware_node_mem_avg.substr(0, middleware_node_mem_avg.find('.') + 2) +
            R"(, "time": )" +  std::to_string(timestamp_ms) +  R"(, "fwVersion": ")" + fw_version_ + R"("})";
      big_data::InsertBigdata(str);
      HJ_INFO("big_data: %s\n", str.c_str());
      std::string utils_node_cpu_avg = std::to_string(1.0 * utils_node_cpu / count);
      std::string utils_node_mem_avg = std::to_string(100.0 * utils_node_memory / count / total_memery);
      str = R"({"event": "nodeOccupancyEvent", "associatedTaskId": ")" + associated_task + R"(", "node": "utilsNode", "cpuOcc": )" +
            utils_node_cpu_avg.substr(0, utils_node_cpu_avg.find('.') + 2) +
            R"(, "memOcc": )" + utils_node_mem_avg.substr(0, utils_node_mem_avg.find('.') + 2) +
            R"(, "time": )" +  std::to_string(timestamp_ms) +  R"(, "fwVersion": ")" + fw_version_ + R"("})";
      big_data::InsertBigdata(str);
      HJ_INFO("big_data: %s\n", str.c_str());

      count = 0;
      is_start = false;
      collect_node_cpu = 0;
      collect_node_memory = 0;
      slam_node_cpu = 0;
      slam_node_memory = 0;
      planning_node_cpu = 0;
      planning_node_memory = 0;
      middleware_node_cpu = 0;
      middleware_node_memory = 0;
      utils_node_cpu = 0;
      utils_node_memory = 0;
    } else if (!task_name.empty()) {
      collect_node_pid_ = GetProcessPidByName("collect_node");
      slam_node_pid_ = GetProcessPidByName("slam_node");
      planning_node_pid_ = GetProcessPidByName("planning_node");
      middleware_node_pid_ = GetProcessPidByName("middleware_node");
      utils_node_pid_ = GetProcessPidByName("utils_node");
      is_start = true;
      collect_node_cpu += GetCpuUsageRatio(collect_node_pid_);
      collect_node_memory += GetMemoryUsage(collect_node_pid_);
      slam_node_cpu += GetCpuUsageRatio(slam_node_pid_);
      slam_node_memory += GetMemoryUsage(slam_node_pid_);
      planning_node_cpu += GetCpuUsageRatio(planning_node_pid_);
      planning_node_memory += GetMemoryUsage(planning_node_pid_);
      middleware_node_cpu += GetCpuUsageRatio(middleware_node_pid_);
      middleware_node_memory += GetMemoryUsage(middleware_node_pid_);
      utils_node_cpu += GetCpuUsageRatio(utils_node_pid_);
      utils_node_memory += GetMemoryUsage(utils_node_pid_);
      count++;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

void Monitor::pubBootType(bool is_core_dump) {
  hj_interface::BootType bootmsg;
  if (is_core_dump) {
    bootmsg.type = 6;  // 6: coredump reboot
    bootType_pub_.publish(bootmsg);
    HJ_INFO("pubBootType: coredump reboot");
  } else {
    uint8_t boot_type = 255;
    uint8_t ango_flag = 255;
    uint8_t ango_data = 255;
    bool ret = hj_bf::getVariable("bootType", boot_type);
    while (!ret) {
      ret = hj_bf::getVariable("bootType", boot_type);
      if (!ret) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
    }
    ret = hj_bf::getVariable("angoflag", ango_flag);
    while (!ret) {
      ret = hj_bf::getVariable("angoflag", ango_flag);
      if (!ret) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
    }
    ret = hj_bf::getVariable("angodata", ango_data);
    while (!ret) {
      ret = hj_bf::getVariable("angodata", ango_data);
      if (!ret) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
    }
    bootmsg.type = boot_type;
    if (ango_flag != 255 && ango_data != 255) {
      rapidjson::Document document;
      document.SetObject();

      rapidjson::Value angoReq(rapidjson::kObjectType);
      angoReq.AddMember("type", ango_flag, document.GetAllocator());
      angoReq.AddMember("data", ango_data, document.GetAllocator());
      document.AddMember("anGeReq", angoReq, document.GetAllocator());
      rapidjson::StringBuffer buffer;
      rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
      document.Accept(writer);
      bootmsg.ango = std::string(buffer.GetString());
    }

    bootType_pub_.publish(bootmsg);
    HJ_INFO("pubBootType: %d, msg:%s", boot_type, bootmsg.ango.data());
  }
}

void Monitor::UploadCoredump() {
  boost::filesystem::path json_path(kNodeMonitorJson);
  if (!boost::filesystem::exists(json_path)) {
    pubBootType(false);
    return;
  }
  std::ifstream stream1(kNodeMonitorJson);  // 输入流
  if (!stream1.is_open()) {
    HJ_ERROR("open file failed %s", kNodeMonitorJson);
    pubBootType(false);
    return;
  }
  std::vector<std::string> dump_node_list;
  {
    // lock_guard<file_lock> guard(lock);
    for (std::string line; std::getline(stream1, line);) {
      HJ_INFO("line: %s", line.c_str());
      if (!line.empty()) {
        dump_node_list.emplace_back(line);
      }
    }
    std::ofstream file_writer(kNodeMonitorJson, std::ios_base::out);
  }

  for (auto& line : dump_node_list) {
    rapidjson::Document document;
    if (!document.Parse(line.data()).HasParseError()) {
      std::string node_name = "";
      uint64_t time_ms = 0;
      if (document.HasMember("node") && document["node"].IsString()) {
        node_name = document["node"].GetString();
      }
      if (document.HasMember("timestamp") && document["timestamp"].IsDouble()) {
        double timestamp_ms = document["timestamp"].GetDouble();
        time_ms = static_cast<uint64_t>(timestamp_ms * 1000);
      }
      std::string task_name = "";
      std::ifstream file_read(kTaskName);
      if (file_read) {
        std::ostringstream ss;
        ss << file_read.rdbuf();  // 读取整个文件内容到字符串流
        task_name = ss.str();
        file_read.close();
      }
      std::string str = R"({"event": "nodeCorruptionEvent", "associatedTaskId": ")" + task_name + R"(", "node": ")" + kNodeInfoMap[node_name] +
            R"(", "time": )" +  std::to_string(time_ms) +  R"(, "fwVersion": ")" + fw_version_ + R"("})";
      HJ_INFO("UploadCoredump big_data: %s\n", str.c_str());
      big_data::InsertBigdata(str);
      std::ofstream file_writer(kTaskName, std::ios_base::out);
    }
  }
  if (dump_node_list.empty()) {
    pubBootType(false);
  } else {
    pubBootType(true);
  }
}

void Monitor::Init() {
  std::ifstream stream1("/etc/version");  // 输入流
  if (!stream1.is_open()) {
    HJ_ERROR("open file failed");
  }
  rapidjson::Document document;
  std::string jsonString((std::istreambuf_iterator<char>(stream1)),
                  std::istreambuf_iterator<char>());

  if (!document.Parse(jsonString.data()).HasParseError()) {
    if (document.HasMember("fw_ver") && document["fw_ver"].IsString()) {
      fw_version_ = document["fw_ver"].GetString();
    }
  }
  // task_sub_ = hj_bf::HJSubscribe("/middleware_task", 1, &Monitor::TaskCallBack, this);
  task_name_sub_ = hj_bf::HJSubscribe("/middleware_task_name", 1, &Monitor::TaskNameCallBack, this);
  bootType_pub_ = hj_bf::HJAdvertise<hj_interface::BootType>("/boot_type", 1, true);
  auto state = std::thread(&Monitor::CalCpuMemeory, this);
  state.detach();
  auto coredump_upload_thread = std::thread(&Monitor::UploadCoredump, this);
  coredump_upload_thread.detach();
}
}  // namespace collect_node_utils_func
