// @file demo.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-22)
#include "core_dump.h"

#include "fstream"
#include <string>
#include <boost/filesystem.hpp>
#include "log.h"
HJ_REGISTER_FUNCTION(factory) {
  factory.registerCreater<core_dump_ns::CoreDump>(FUNCTION_NAME);
}
namespace core_dump_ns {

OperateDir::OperateDir() {}

bool OperateDir::IsFile(std::string filename) const {
  struct stat buffer;
  return (stat(filename.c_str(), &buffer) == 0 && S_ISREG(buffer.st_mode));
}

bool OperateDir::IsDIR(std::string filefodler) const {
  struct stat buffer;
  return (stat(filefodler.c_str(), &buffer) == 0 && S_ISDIR(buffer.st_mode));
}

int OperateDir::CopyFileExt(const std::string &srcFile, const std::string &desFile) {
  std::ifstream in;
  in.open(srcFile);
  if (!in) {
    HJ_WARN("open src file failed:%s", srcFile.c_str());
    return -1;
  }

  std::ofstream out;
  out.open(desFile);
  if (!out) {
    HJ_WARN("create new file failed:%s", desFile.c_str());
    in.close();
    return -1;
  }

  out << in.rdbuf();

  out.close();
  in.close();
  return 0;
}
int OperateDir::DeleteDir(const std::string &srcDirPath) {
  std::vector<std::string> fileNameList;
  if (!IsDIR(srcDirPath)) {
    HJ_WARN("the name not a dir:%s", srcDirPath.c_str());
    return 0;
  }
  if (!_GetDeleteList(srcDirPath, fileNameList)) {
    return -1;
  }
  if (0 != _DeleteList(srcDirPath, fileNameList)) {
    return -1;
  }

  if (0 != remove(srcDirPath.c_str())) {
    HJ_WARN("remove a dir err :%s", srcDirPath.c_str());
    return -1;
  }
  return 0;
}
bool OperateDir::_GetDeleteList(std::string pathName, std::vector<std::string> &fileNameList) {
  DIR *dir;
  struct dirent *ptr;

  if ((dir = opendir(pathName.c_str())) == NULL) {
    HJ_WARN("%s not found ", pathName.c_str());
    return false;
  }
  while ((ptr = readdir(dir)) != NULL) {
    if ((!strcmp(ptr->d_name, ".")) || (!strcmp(ptr->d_name, "..")))  // current / parent
    {
      continue;
    }

    else if (ptr->d_type == 8)  // file
    {
      HJ_INFO("deletes file :%s", ptr->d_name);
      fileNameList.push_back(ptr->d_name);
    } else if (ptr->d_type == 10) {  // link file
      continue;
    } else if (ptr->d_type == 4) {  // dir
      std::string newSrcFilePath = pathName + "/" + ptr->d_name;
      HJ_INFO("deletes subdir src:%s", newSrcFilePath.c_str());
      if (0 != DeleteDir(newSrcFilePath)) {
        HJ_WARN("fail to delete dir: %s", newSrcFilePath.c_str());
        return false;
      }
    }
  }
  closedir(dir);
  return true;
}

int OperateDir::CopyDir(const std::string &srcDirPath,
                        const std::string &desDirPath) {  // just dir
  if (!IsDIR(srcDirPath)) {
    HJ_WARN("the name not a dir:%s", srcDirPath.c_str());
    return 0;
  }
  if (!_MakeDir(desDirPath)) {
    return -1;
  }
  std::vector<std::string> fileNameList;
  if (!_GetCopyList(srcDirPath, desDirPath, fileNameList)) {
    return -1;
  }
  if (0 != _CopyList(srcDirPath, desDirPath, fileNameList)) {
    return -1;
  }
  return 0;
}

bool OperateDir::_MakeDir(const std::string &pathName) {
  int state = access(pathName.c_str(), R_OK | W_OK);
  if (state == 0) {
    if (IsDIR(pathName)) {
      if (0 != DeleteDir(pathName.c_str())) {
        HJ_WARN("rm path error:%s", pathName.c_str());
        return false;
      }
    }
    if (IsFile(pathName)) {  //??
      if (0 != remove(pathName.c_str())) {
        HJ_WARN("remove file err:%s", pathName.c_str());
        return false;
      }
    }
  }
  if (::mkdir(pathName.c_str(), S_IRWXU | S_IRGRP | S_IXGRP) < 0) {
    HJ_WARN("create path error:%s", pathName.c_str());
    return false;
  }
  return true;
}

bool OperateDir::_GetCopyList(std::string src, std::string dest, std::vector<std::string> &fileNameList) {
  DIR *dir;
  struct dirent *ptr;

  if ((dir = opendir(src.c_str())) == NULL) {
    HJ_WARN("%s not found", src.c_str());
    return false;
  }
  std::string nowSrcFilePath;
  while ((ptr = readdir(dir)) != NULL) {
    if ((!strcmp(ptr->d_name, ".")) || (!strcmp(ptr->d_name, "..")))  // current / parent
    {
      continue;
    }

    else if (ptr->d_type == 8)  // file
    {
      HJ_INFO("push file:%s", ptr->d_name);
      fileNameList.push_back(ptr->d_name);
    } else if (ptr->d_type == 10) {  // link file
      continue;
    } else if (ptr->d_type == 4) {  // dir
      std::string newSrcFilePath = src + "/" + ptr->d_name;
      std::string newDesFilePath = dest + "/" + ptr->d_name;
      HJ_INFO("subdir src:%s, subdir dest:%s", newSrcFilePath.c_str(), newDesFilePath.c_str());
      if (0 != CopyDir(newSrcFilePath, newDesFilePath)) {
        HJ_WARN("fail to process:%s", ptr->d_name);
        return false;
      }
    }
  }
  closedir(dir);
  return true;
}

int OperateDir::_DeleteList(const std::string &srcDirPath, const std::vector<std::string> &fileNameList) {
  for (uint32_t i = 0; i < fileNameList.size(); i++) {
    std::string nowSrcFilePath;
    nowSrcFilePath = srcDirPath + "/" + fileNameList.at(i);
    if (0 != remove(nowSrcFilePath.c_str())) {
      HJ_WARN("remove file err:%s", nowSrcFilePath.c_str());
      return -1;
    }
  }
  return 0;
}

int OperateDir::_CopyList(const std::string &srcDirPath, const std::string &desDirPath,
                          const std::vector<std::string> &fileNameList) {
  for (uint32_t i = 0; i < fileNameList.size(); i++) {
    std::string nowSrcFilePath, nowDesFilePath;
    nowSrcFilePath = srcDirPath + "/" + fileNameList.at(i);
    nowDesFilePath = desDirPath + "/" + fileNameList.at(i);
    std::ifstream in;
    in.open(nowSrcFilePath);
    if (!in) {
      HJ_WARN("open src file failed:%s", nowSrcFilePath.c_str());
      return -1;
    }

    std::ofstream out;
    out.open(nowDesFilePath);
    if (!out) {
      HJ_WARN("create new file failed:%s", nowDesFilePath.c_str());
      in.close();
      return -1;
    }

    out << in.rdbuf();

    out.close();
    in.close();
  }
  return 0;
}
bool dumpCallback(const google_breakpad::MinidumpDescriptor &descriptor, void *context, bool succeeded) {
  std::cerr << "Dump path: " << descriptor.path() << std::endl;
  return succeeded;
}

// static google_breakpad::MinidumpDescriptor descriptor(CORE_DUMP_FILE_PATH);
// static google_breakpad::ExceptionHandler eh(descriptor, NULL, dumpCallback, NULL, true, -1);
void crash() {
  volatile int *a = (int *)(NULL);
  *a = 1;
}
CoreDump::CoreDump(const rapidjson::Value &json_conf) : hj_bf::Function(json_conf) {
  std::string dump_file_name = CORE_DUMP_FILE_PATH;
  // read your config
  if (json_conf.HasMember("dump_path") && json_conf["dump_path"].IsString()) {
    dump_file_name = json_conf["dump_path"].GetString();
    HJ_IMPORTANT("minos get dump_path:%s", dump_file_name.c_str());
  }
  static google_breakpad::MinidumpDescriptor descriptor(dump_file_name.c_str());
  static google_breakpad::ExceptionHandler eh(descriptor, NULL, dumpCallback, NULL, true, -1);
  // your code
  if (boost::filesystem::exists(dump_file_name)) {
    HJ_IMPORTANT("minos Path exists:%s",dump_file_name.c_str());
    std::size_t file_count = 0;

    for (const auto& entry : boost::filesystem::directory_iterator(dump_file_name)) {
        if (boost::filesystem::is_regular_file(entry.status())) {
            ++file_count;
        }
    }
    if(file_count > CORE_DUMP_FILE_NUM_MAX){
      HJ_IMPORTANT("minos too many CoreDump file");
      std::string old_file = "";
      std::time_t old_time = 0;
      for (const auto& entry : boost::filesystem::directory_iterator(dump_file_name)) {
          std::time_t last_modified_time = boost::filesystem::last_write_time(entry.path().string());
          if (old_time == 0) {
            old_time = last_modified_time;
            old_file = entry.path().string();
          } else {
            if (old_time > last_modified_time) {
              old_time = last_modified_time;
              old_file = entry.path().string();
            }
          }
      }
      boost::filesystem::remove(old_file);
      HJ_IMPORTANT("minos old CoreDump file%s",old_file.c_str());
    }

//    boost::filesystem::remove_all(dump_file_name);
  } else {
    boost::filesystem::create_directories(dump_file_name);
  }
  hj_bf::registerSignal();
  HJ_IMPORTANT("minos just a CoreDump");
}

}  // namespace core_dump_ns
