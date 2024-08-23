

// @file log_factory.cpp
// @brief
//
// Copyright 2024 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-3-7)
#include "log_factory.h"

#include <dirent.h>
#include <log4cxx/helpers/properties.h>
#include <log4cxx/rollingfileappender.h>

#include <algorithm>

#include "log4cxx/appenderskeleton.h"
#include "log4cxx/logger.h"
#include "log4cxx/propertyconfigurator.h"
#include "log4cxx/spi/loggingevent.h"
#include "node_factory.h"
namespace hj_bf {
constexpr char g_log_config_path_param_name[] = "HJ_LOG_CONFIG_PATH";
constexpr char g_log_close[] = "HJ_LOG_CLOSE_";
constexpr char g_pass_word[] = "221100";
void deleteFilesWithPrefix(const std::string& directory, const std::string& prefix) {
  DIR* dir = opendir(directory.c_str());
  if (dir == nullptr) {
    std::cerr << "Failed to open directory: " << directory << std::endl;
    return;
  }
  struct dirent* entry = nullptr;
  while ((entry = readdir(dir)) != nullptr) {
    std::string filename = entry->d_name;
    // 检查文件是否是普通文件并且以指定的前缀开头
    if (entry->d_type == DT_REG && filename.rfind(prefix, 0) == 0) {
      std::string filePath = directory;
      filePath.append("/").append(filename);
      if (remove(filePath.c_str()) == 0) {
        std::cerr << "Deleted file: " << filePath << std::endl;
      } else {
        std::cerr << "Failed to delete file: " << filePath << std::endl;
      }
    }
  }
  closedir(dir);
}

bool logInit(const std::string& config_path,int crypt_flag) {
  std::string temp_log_close = g_log_close;
  std::string temp_node_name = ros::this_node::getName();
  temp_node_name.erase(std::remove(temp_node_name.begin(), temp_node_name.end(), '/'), temp_node_name.end());
  temp_log_close += temp_node_name;
  char* switch_cstr = NULL;
  switch_cstr = getenv(temp_log_close.c_str());
  if (switch_cstr != NULL) {
    if (strcmp(switch_cstr, "close") == 0) {
      std::cout << "close log of module: " << ros::this_node::getName() << std::endl;
      return true;
    }
  }
  std::string temp_config_path = config_path;
  bool ret = false;
  char* config_file_cstr = NULL;
  config_file_cstr = getenv(g_log_config_path_param_name);
  if (NULL != config_file_cstr) {
    temp_config_path = config_file_cstr;
  }
  log4cxx::PropertyConfigurator::configure(temp_config_path.c_str());
  log4cxx::LoggerPtr rootLogger = log4cxx::Logger::getRootLogger();
  log4cxx::RollingFileAppenderPtr appender = rootLogger->getAppender("file");
  if (nullptr != appender) {
    // std::string open = OPEN_CRYPT;
    // if (open == "true") {
    //   appender->setEncryption(true);
    //   std::cout << "wangqing true:" << std::endl;
    // } else {
    //   appender->setEncryption(false);
    //   std::cout << "wangqing false:" << std::endl;
    // }
    if (crypt_flag == 0) {
      appender->setEncryption(false);
      std::cout << "wangqing false:" << std::endl;
    } else {
      appender->setEncryption(true);
      std::cout << "wangqing true:" << std::endl;
    }

    std::vector<unsigned char> source(g_pass_word, g_pass_word + strlen(g_pass_word));
    appender->setPassWord(source);
    std::string temp_file_path = appender->getFile();
    size_t lastSlashPos = temp_file_path.find_last_of('/');

    if (lastSlashPos != std::string::npos) {
      std::string filename = temp_file_path.substr(lastSlashPos + 1);
      //      std::cout << "Filename: " << filename << std::endl;
      std::string path = temp_file_path.substr(0, lastSlashPos);
      //      std::cout << "Path: " << path << std::endl;
      std::string new_log_file_name = path + ros::this_node::getName() + "_" + filename;
      appender->setFile(new_log_file_name.c_str());
      // deleteFilesWithPrefix(path, ros::this_node::getName().substr(1));
      log4cxx::helpers::Pool p;
      appender->activateOptions(p);
      ret = true;
      //      std::cout << "wangqing:" << new_log_file_name << std::endl;
    }
  }else {
    std::cerr << "err appender == null:" << std::endl;
  }
  return ret;
}
}  // namespace hj_bf
