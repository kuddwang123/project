// @file collect_node_manager.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-15)
#include <sstream>
#include <thread>

#include "big_data.h"
#include "def.h"
#include "function_factory.h"
#include "log_factory.h"
#include "node_cache.h"
#include "log.h"
#include "node_factory.h"
#include "shm.h"
#include <fstream>
int main(int argc, char** argv) {
#if 1  // for debug, 优化开机时间
  std::fstream file("/proc/uptime", std::ios::in);
  if (file.is_open()) {
    std::string line;
    std::getline(file, line);
    std::cout << "collect_node Current uptime:" << line.c_str() << std::endl;
    file.close();
  }
#endif

  std::string temp_config_path;
  char* switch_cstr = NULL;
  switch_cstr = getenv(hj_bf::g_node_config_file_env_set);
  if (switch_cstr) {
    temp_config_path = switch_cstr;
  }

  std::shared_ptr<hj_bf::NodeConfig> node_config = std::make_shared<hj_bf::NodeConfig>();
  node_config->node_name = NODE_NAME;
  node_config->log_config_path = LOG_CONFIG_PATH;
  node_config->log_level = LOG_LEVEL;
  std::cerr << "step011 " << std::endl;
  std::cout << "step011 " << std::endl;
  std::cout << "LOG_LEVEL: " << LOG_LEVEL << std::endl;
  // std::vector<unsigned char> crypt_val(CRYPT_VAL, CRYPT_VAL + strlen(CRYPT_VAL));
  std::string crypt_val = CRYPT_VAL;
  // std::cout << "CRYPT_VAL: " << crypt_val << std::endl;
  node_config->crypt_val.assign(crypt_val.begin(), crypt_val.end());
  std::cerr << "step012 " << std::endl;
  std::cout << "step012 " << std::endl;
  hj_bf::Shm::createInstance();
#ifdef HJ_AMD64
  if (temp_config_path.empty()) {

    hj_bf::readConfigure(CONFIG_PATH, node_config);
  } else {
    std::cerr << "step013 " << std::endl;
    std::cout << "step013 " << std::endl;
    temp_config_path =
        temp_config_path + "/" + NODE_NAME + "/" + NODE_NAME + "/config" + "/" + PROJECT_NUMBER + "/amd64/config.json";
    hj_bf::readConfigure(temp_config_path, node_config);
    std::cerr << "step014 " << std::endl;
    std::cout << "step014 " << std::endl;
    hj_bf::readRemoteConfigure(node_config, true, node_config->remote_config_path);
  }
#endif
#ifdef HJ_AARCH64
  temp_config_path = temp_config_path + "/" + NODE_NAME + "/config.json";
  hj_bf::readConfigure(temp_config_path, node_config);
  // std::string remote_config_name = NODE_NAME + "_remote_config.json";
  hj_bf::readRemoteConfigure(node_config, true, node_config->remote_config_path);
#endif
  std::cerr << "step01 " << std::endl;
  std::cout << "step01 " << std::endl;
  uint32_t temp_ops = 0;
  if (node_config->all_log_close == true) {
    std::cout << "close all log: " << node_config->node_name << std::endl;
    temp_ops = ros::init_options::NoRosout;
  }
  hj_bf::nodeInit(argc, argv, node_config->node_name, node_config, temp_ops);
  if (node_config->node_log_close == false) {
    hj_bf::logInit(node_config->log_config_path, node_config->crypt_val);
    hj_bf::logLevelSetRos(node_config->log_level);
  } else {
    std::cout << "close log, node name:" << node_config->node_name << std::endl;
  }
  std::cerr << "step0 " << std::endl;
  std::cout << "step0 " << std::endl;

  
  std::cerr << "step1 " << std::endl;
  hj_bf::HealthCheckInit();
  std::cerr << "step2 " << std::endl;
  hj_bf::nodeInstance();
  hj_bf::nodeStart();
#if 1  // for debug, 优化开机时间
  std::fstream file1("/proc/uptime", std::ios::in);
  if (file1.is_open()) {
    std::string line;
    std::getline(file1, line);
    std::cout << "collect_node end Current uptime:" << line.c_str() << std::endl;
    file1.close();
  }
#endif
  hj_bf::nodeSpin();
  //  ros::waitForShutdown();
  return 0;
}
