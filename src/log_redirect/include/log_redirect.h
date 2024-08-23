// @file demo.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-22)
#ifndef INCLUDE_DEMO_H//your macro
#define INCLUDE_DEMO_H
#include "function_factory.h"
#include "node_factory.h"
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/prctl.h>

#define DEFAULT_ERR_FILE_SIZE (1000 * 1000)//1mB
#define DEFAULT_ERR_FILE_PATH "/tmp/logging"
#define DEFAULT_COUT_FILE_SIZE (1000 * 1000)//1mB
#define DEFAULT_COUT_FILE_PATH "/tmp/logging"

namespace log_redirect_ns {//your namespace
class OperateDir {
 public:
  OperateDir();
  int CopyDir(const std::string& srcDirPath, const std::string& desDirPath);
  int DeleteDir(const std::string& srcDirPath);
  int CopyFileExt(const std::string& srcFile, const std::string& desFile);
  bool IsFile(std::string filename) const;
  bool IsDIR(std::string filefodler) const;

 private:
  bool _MakeDir(const std::string& pathName);
  bool _GetCopyList(std::string src, std::string dest,
                    std::vector<std::string>& fileNameList);
  int _CopyList(const std::string& srcDirPath, const std::string& desDirPath,
                const std::vector<std::string>& fileNameList);
  int _DeleteList(const std::string& srcDirPath,
                  const std::vector<std::string>& fileNameList);
  bool _GetDeleteList(std::string pathName,
                      std::vector<std::string>& fileNameList);
};

class LogRedirect : public hj_bf::Function {
 public:
  explicit LogRedirect(const rapidjson::Value& json_conf);
  ~LogRedirect(){};

 private:
  // your variables
  // hj_bf::HJPublisher pub_;
  // hj_bf::HJClient client_;
  // hj_bf::HJSubscriber sub_;
  // hj_bf::HJServer service_;
  // hj_bf::HJTimer timer_;
};
}  // namespace log_redirect_ns

#endif
