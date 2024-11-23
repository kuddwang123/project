

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
// constexpr char g_log_config_path_param_name[] = "HJ_LOG_CONFIG_PATH";
constexpr char g_log_close[] = "HJ_LOG_CLOSE_";
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

bool logInit(const std::string& config_path, const std::vector<unsigned char>& pass_word) {
  bool ret = false;
  log4cxx::PropertyConfigurator::configure(config_path.c_str());
  log4cxx::LoggerPtr rootLogger = log4cxx::Logger::getRootLogger();
  log4cxx::RollingFileAppenderPtr appender = rootLogger->getAppender("file");
  std::vector<unsigned char> temp_pass_word = pass_word;
  if (nullptr != appender) {
    if (temp_pass_word.size() == 6) {
      appender->setEncryption(true);
      std::cout << "wangqing open:" << std::endl;
      appender->setPassWord(temp_pass_word);
    } else {
      appender->setEncryption(false);
      std::cout << "wangqing close:" << std::endl;
    }
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
    }
    
  } else {
    std::cerr << "err appender == null:" << std::endl;
  }
  return ret;
}
}  // namespace hj_bf
