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
#include <string>
#include <thread>
#include <chrono>
#include <cstring>
#include <cmath>
#include <unordered_map>
#include "log.h"
#include "big_data.h"
#include "rapidjson/document.h"
#include "status_code.h"
#include "hj_interface/HealthCheckCode.h"
#include "node_cache.h"
#include "boost/filesystem.hpp"


namespace collect_node_utils_func {
constexpr char kNodeMonitorJson[] = "/userdata/hj/log/node_monitor.json";
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
  }

  pclose(fp);
  return pid;
}



Monitor::~Monitor() {
}

void Monitor::TaskCallBack(const std_msgs::UInt8::ConstPtr& msg) {
  task_.store(msg->data);
  HJ_INFO("task_ is %d", msg->data);
}

void Monitor::CalCpuMemeory() {
  int32_t total_memery = GetAllMemory();
  uint8_t old_task = 0;
  uint64_t collect_node_cpu = 0, collect_node_memory = 0;
  uint64_t slam_node_cpu = 0, slam_node_memory = 0;
  uint64_t planning_node_cpu = 0, planning_node_memory = 0;
  uint64_t middleware_node_cpu = 0, middleware_node_memory = 0;
  uint64_t utils_node_cpu = 0, utils_node_memory = 0;
  int count = 0;
  bool is_start = false;  // task start flag
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

  while (true) {
    old_task = task_.load();

    if ((old_task == TASK_BUILD_MAP_FINISH || old_task == TASK_RELOCATE_FINISH ||
        old_task == TASK_CLEAN_FINISH) && is_start && count > 0) {
      ros::Time timestamp = ros::Time::now();
      double time_now = timestamp.toSec();
      int64_t timestamp_ms = static_cast<int64_t>(time_now * 1000);
      std::string collect_node_cpu_avg = std::to_string(1.0 * collect_node_cpu / count);
      std::string collect_node_mem_avg = std::to_string(100.0 * collect_node_memory / count / total_memery);
      std::string str = R"({"event": "nodeOccupancyEvent", "node": "collectNode", "cpuOcc": )" +
                      collect_node_cpu_avg.substr(0, collect_node_cpu_avg.find('.') + 2) +
                      R"(, "memOcc": )" + collect_node_mem_avg.substr(0, collect_node_mem_avg.find('.') + 2) +
                      R"(, "time": )" +  std::to_string(timestamp_ms) +  R"(, "fwVersion": ")" + fw_version_ + R"("})";
      big_data::InsertBigdata(str);
      // HJ_INFO("big_data: %s\n", str.c_str());
      std::string slam_node_cpu_avg = std::to_string(1.0 * slam_node_cpu / count);
      std::string slam_node_mem_avg = std::to_string(100.0 * slam_node_memory / count / total_memery);
      str = R"({"event": "nodeOccupancyEvent", "node": "slamNode", "cpuOcc": )" +
            slam_node_cpu_avg.substr(0, slam_node_cpu_avg.find('.') + 2) +
            R"(, "memOcc": )" + slam_node_mem_avg.substr(0, slam_node_mem_avg.find('.') + 2) +
            R"(, "time": )" +  std::to_string(timestamp_ms) +  R"(, "fwVersion": ")" + fw_version_ + R"("})";
      big_data::InsertBigdata(str);
      // HJ_INFO("big_data: %s\n", str.c_str());
      std::string planning_node_cpu_avg = std::to_string(1.0 * planning_node_cpu / count);
      std::string planning_node_mem_avg = std::to_string(100.0 * planning_node_memory / count / total_memery);
      str = R"({"event": "nodeOccupancyEvent", "node": "planningNode", "cpuOcc": )" +
            planning_node_cpu_avg.substr(0, planning_node_cpu_avg.find('.') + 2) +
            R"(, "memOcc": )" + planning_node_mem_avg.substr(0, planning_node_mem_avg.find('.') + 2) +
            R"(, "time": )" +  std::to_string(timestamp_ms) +  R"(, "fwVersion": ")" + fw_version_ + R"("})";
      big_data::InsertBigdata(str);
      // HJ_INFO("big_data: %s\n", str.c_str());
      std::string middleware_node_cpu_avg = std::to_string(1.0 * middleware_node_cpu / count);
      std::string middleware_node_mem_avg = std::to_string(100.0 * middleware_node_memory / count / total_memery);
      str = R"({"event": "nodeOccupancyEvent", "node": "middlewareNode", "cpuOcc": )" +
            middleware_node_cpu_avg.substr(0, middleware_node_cpu_avg.find('.') + 2) +
            R"(, "memOcc": )" + middleware_node_mem_avg.substr(0, middleware_node_mem_avg.find('.') + 2) +
            R"(, "time": )" +  std::to_string(timestamp_ms) +  R"(, "fwVersion": ")" + fw_version_ + R"("})";
      big_data::InsertBigdata(str);
      // HJ_INFO("big_data: %s\n", str.c_str());
      std::string utils_node_cpu_avg = std::to_string(1.0 * utils_node_cpu / count);
      std::string utils_node_mem_avg = std::to_string(100.0 * utils_node_memory / count / total_memery);
      str = R"({"event": "nodeOccupancyEvent", "node": "utilsNode", "cpuOcc": )" +
            utils_node_cpu_avg.substr(0, utils_node_cpu_avg.find('.') + 2) +
            R"(, "memOcc": )" + utils_node_mem_avg.substr(0, utils_node_mem_avg.find('.') + 2) +
            R"(, "time": )" +  std::to_string(timestamp_ms) +  R"(, "fwVersion": ")" + fw_version_ + R"("})";
      big_data::InsertBigdata(str);
      // HJ_INFO("big_data: %s\n", str.c_str());

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
    } else if (old_task == TASK_BUILD_MAP_START ||
          old_task == TASK_RELOCATE_START ||old_task == TASK_CLEAN_START) {
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

void Monitor::UploadCoredump(const hj_bf::HJTimerEvent &event) {
  boost::filesystem::path json_path(kNodeMonitorJson);
  if (!boost::filesystem::exists(json_path)) {
    coredump_upload_timer_.stop();
    return;
  }
  std::ifstream stream1(kNodeMonitorJson);  // 输入流
  if (!stream1.is_open()) {
    HJ_ERROR("open file failed %s", kNodeMonitorJson);
    coredump_upload_timer_.stop();
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
      if (document.HasMember("timestamp") && document["timestamp"].IsFloat()) {
        float timestamp_ms = document["timestamp"].GetFloat();
        time_ms = static_cast<uint64_t>(timestamp_ms * 1000);
      }
      std::string str = R"({"event": "nodeCorruptionEvent", "node": )" + kNodeInfoMap[node_name] +
            R"(, "time": )" +  std::to_string(time_ms) +  R"(, "fwVersion": ")" + fw_version_ + R"("})";
      // HJ_INFO("UploadCoredump big_data: %s\n", str.c_str());
      big_data::InsertBigdata(str);
    }
  }
  coredump_upload_timer_.stop();
}

void Monitor::Init() {
  task_sub_ = hj_bf::HJSubscribe("/middleware_task", 1, &Monitor::TaskCallBack, this);
  auto state = std::thread(&Monitor::CalCpuMemeory, this);
  state.detach();
  coredump_upload_timer_ = hj_bf::HJCreateTimer("coredump_upload_timer",
                1000 * 1000, &Monitor::UploadCoredump, this);  // 30HZ
}
}  // namespace collect_node_utils_func
