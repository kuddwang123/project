

// @file log_factory.cpp
// @brief
//
// Copyright 2024 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-3-7)
#include "log_factory.h"

#include <dirent.h>
#include <log4cxx/helpers/properties.h>
#include <log4cxx/rollingfileappender.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <ros/console.h>

#include <algorithm>
#include <boost/filesystem.hpp>
#include <fstream>

#include "hj_utils.h"
#include "log4cxx/appenderskeleton.h"
#include "log4cxx/logger.h"
#include "log4cxx/propertyconfigurator.h"
#include "log4cxx/spi/loggingevent.h"
#include "node_factory.h"
#include "rapidjson/document.h"
#include "rapidjson/error/en.h"
#include "rapidjson/filereadstream.h"
#include "shm_interface.h"
namespace hj_bf {

// constexpr char g_log_config_path_param_name[] = "HJ_LOG_CONFIG_PATH";
constexpr char g_log_close[] = "HJ_LOG_CLOSE_";
constexpr char g_remote_config_file_lock_name[] = "remote log config lock";
static std::string g_remote_config_path = "";
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
bool logLevelSetRos(const std::string& level_str) {
  ros::console::levels::Level level;
  if (level_str == "debug") {
    level = ros::console::levels::Info;
  } else if (level_str == "release") {
    level = ros::console::levels::Error;
  } else {
    return false;
  }

  bool ret = ::ros::console::set_logger_level("ros", level);
  return ret;
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
bool readConfig(rapidjson::Document* document) {
  std::string file_name = g_remote_config_path + "/" + g_remote_config_name;
  hj_bf::MinosLock my_lock(g_remote_config_file_lock_name);
  int state = access(file_name.c_str(), R_OK | W_OK);
  if (state == -1) {
    std::cerr << "file not exist :" << file_name << std::endl;
    boost::filesystem::path dir_path(g_remote_config_path);
    if (!boost::filesystem::exists(dir_path)) {
      if (boost::filesystem::create_directories(dir_path)) {
        std::cout << "Directories created successfully :" << g_remote_config_path << std::endl;
      } else {
        std::cerr << "Directories already exist." << g_remote_config_path << std::endl;
      }
    }

    std::ofstream outfile;
    outfile.open(file_name, std::ios_base::out);
    if (!outfile.is_open()) {
      return false;
    }
    outfile.close();
  }

  std::ifstream config_file;
  config_file.open(file_name, std::ios::in | std::ios::binary);
  if (!config_file.is_open()) {
    std::cerr << "config file not exist!, name :" << file_name << std::endl;
    return false;
  }
  std::string data((std::istreambuf_iterator<char>(config_file)), (std::istreambuf_iterator<char>()));

  document->Parse<rapidjson::kParseCommentsFlag | rapidjson::kParseTrailingCommasFlag>(data.c_str());
  if (!document->HasParseError() && document->IsObject()) {
  } else {
    std::cerr << "cant analysis " << file_name << " string to json" << std::endl;
    std::remove(file_name.c_str());
    return false;
  }
  return true;
}
static bool writeConfig(const rapidjson::Document& document) {
  std::string file_name = g_remote_config_path + "/" + g_remote_config_name;
  hj_bf::MinosLock my_lock(g_remote_config_file_lock_name);
  int state = access(file_name.c_str(), R_OK | W_OK);
  if (state == -1) {
    boost::filesystem::path dir_path(g_remote_config_path);
    if (!boost::filesystem::exists(dir_path)) {
      if (boost::filesystem::create_directories(dir_path)) {
        std::cout << "Directories created successfully :" << g_remote_config_path << std::endl;
      } else {
        std::cerr << "Directories already exist." << g_remote_config_path << std::endl;
      }
    }
  }
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  std::ofstream outfile;
  outfile.open(file_name, std::ios_base::out);
  if (!outfile.is_open()) {
    return false;
  }
  outfile << buffer.GetString() << std::endl;
  outfile.close();
  return true;
}

bool readRemoteConfigure(std::shared_ptr<struct NodeConfig> out_config, bool create, const std::string& remote_path) {
  if (remote_path != "") {
    g_remote_config_path = remote_path;  // out_config->remote_config_path;
  }
  rapidjson::Document document;
  if (readConfig(&document)) {
    std::cout << "readConfig success" << std::endl;
    if (document.HasMember("log_level") && document["log_level"].IsString()) {
      out_config->log_level = document["log_level"].GetString();
      std::cerr << "rreadRemoteConfigure log_level:" << out_config->log_level << std::endl;
    }
  } else if (create == true) {
    std::cerr << "readConfig fail and will create it" << std::endl;
    document.SetObject();
    rapidjson::Value keyValue(out_config->log_level.c_str(), document.GetAllocator());
    document.AddMember("log_level", keyValue, document.GetAllocator());
    writeConfig(document);
  } else {
    std::cerr << "readConfig fail" << std::endl;
    return false;
  }
  return true;
}

bool setLogLevelConfigure(const std::string& level) {
  rapidjson::Document document;
  if (readConfig(&document)) {
    if (document.HasMember("log_level") && document["log_level"].IsString()) {
      document["log_level"].SetString(level.c_str(), document.GetAllocator());
      writeConfig(document);
    } else {
      std::cerr << "readConfig and have not key log_level " << std::endl;
      return false;
    }
  } else {
    std::cerr << "readConfig fail and will create it" << std::endl;
    document.SetObject();
    rapidjson::Value keyValue(level.c_str(), document.GetAllocator());
    document.AddMember("log_level", keyValue, document.GetAllocator());
    writeConfig(document);
  }
  return true;
}
}  // namespace hj_bf
