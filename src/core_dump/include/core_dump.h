// @file demo.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-22)
#ifndef INCLUDE_DEMO_H//your macro
#define INCLUDE_DEMO_H
#include "function_factory.h"
#include "node_factory.h"
#include "client/linux/handler/exception_handler.h"
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define CORE_DUMP_FILE_PATH "/userdata/hj/log/core_dump"
#define CORE_DUMP_FILE_NUM_MAX 10
namespace core_dump_ns {//your namespace
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

class CoreDump : public hj_bf::Function {
 public:
  explicit CoreDump(const rapidjson::Value &json_conf);
  ~CoreDump(){};

 private:
  // your variables
  // hj_bf::HJPublisher pub_;
  // hj_bf::HJClient client_;
  // hj_bf::HJSubscriber sub_;
  // hj_bf::HJServer service_;
  // hj_bf::HJTimer timer_;
};
}  // namespace core_dump_ns

#endif
