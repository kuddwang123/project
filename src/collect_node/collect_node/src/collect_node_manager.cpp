// @file collect_node_manager.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-15)
#include <sstream>
#include <thread>

#include "def.h"
#include "function_factory.h"
#include "node_factory.h"
#include "log_factory.h"
#include "node_cache.h"
#include "log.h"
#include "shm.h"
#include "big_data.h"
int main(int argc, char** argv) {
  char* switch_cstr = NULL;
  uint32_t temp_ops = 0;
  switch_cstr = getenv(hj_bf::g_all_log_close);
  if (switch_cstr) {
    if (strcmp(switch_cstr, "close") == 0) {
      std::cout << "close all log: " << ros::this_node::getName() << std::endl;
      temp_ops = ros::init_options::NoRosout;
    }
  }

  hj_bf::nodeInit(argc, argv, "collect_node", 4, temp_ops);
  hj_bf::logInit(LOG_CONFIG_PATH, CRYPT_VAL);
  hj_bf::Shm::createInstance();
  big_data::Init();
  hj_bf::HealthCheckInit();
  std::string temp_lib_path;
  std::string temp_config_path;
  hj_bf::HJGetParam(g_lib_path_param_name, temp_lib_path);
  hj_bf::HJGetParam(g_config_path_param_name, temp_config_path);
#ifdef HJ_AMD64
  if (temp_lib_path.empty() || temp_config_path.empty()) {
    hj_bf::getConfigure(CONFIG_PATH, SO_PATH);
  } else {
    temp_lib_path = temp_lib_path + "/" + NODE_NAME;
    temp_config_path =
        temp_config_path + "/" + NODE_NAME + "/" + NODE_NAME + "/config" + "/" + PROJECT_NUMBER + "/amd64/config.json";
    std::cerr << "minos temp_lib_path:" << temp_lib_path << std::endl;
    std::cerr << "minos temp_config_path:" << temp_config_path << std::endl;
    hj_bf::getConfigure(temp_config_path, temp_lib_path);
  }
#endif
#ifdef HJ_AARCH64
  temp_config_path = temp_config_path + "/" + NODE_NAME + "/config.json";
  temp_lib_path = temp_lib_path + "/" + NODE_NAME;
  std::cerr << "minos temp_lib_path:" << temp_lib_path << std::endl;
  std::cerr << "minos temp_config_path:" << temp_config_path << std::endl;
  hj_bf::getConfigure(temp_config_path, temp_lib_path);
#endif
  hj_bf::registerSignal();
  hj_bf::nodeStart();
  hj_bf::nodeSpin();
//  ros::waitForShutdown();
  return 0;
}
