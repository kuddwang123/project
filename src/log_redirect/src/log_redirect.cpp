// @file demo.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-22)
#include "log_redirect.h"

#include <thread>

#include "fstream"
#include "log.h"
HJ_REGISTER_FUNCTION(factory) {
  HJ_IMPORTANT("minos register factory:%s", FUNCTION_NAME);
  factory.registerCreater<log_redirect_ns::LogRedirect>(FUNCTION_NAME);
}
namespace log_redirect_ns {

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

static void WriteLogErrTread(size_t max_file_size, const std::string &file_dir_path) {
  prctl(PR_SET_NAME, "log_err_file");
  // 重定向stderr
  auto se_fn = file_dir_path + "/log_err";
  int log_fd = -1;
  int filesIndex = 0;
  std::string localfileName = se_fn + std::to_string(filesIndex);
  HJ_IMPORTANT("WriteLogErrTread, file name:%s", localfileName.c_str());
  while (1) {
    if (log_fd < 0) {
      log_fd = open(localfileName.c_str(), O_RDWR | O_CREAT | O_APPEND, 0644);
      if (log_fd != -1) {
        dup2(log_fd, STDERR_FILENO);
      } else {
        sleep(1);
      }
    } else {
      struct stat info;
      stat(localfileName.c_str(), &info);
      int size = info.st_size;
      if (size > max_file_size) {
        while (1) {
          filesIndex++;
          if (filesIndex >= 2) {
            filesIndex = 0;
          }
          localfileName = se_fn + std::to_string(filesIndex);
          int ret = close(log_fd);
          if (!ret) {
            log_fd = -1;
            OperateDir dir;
            int state = access(localfileName.c_str(), R_OK | W_OK);
            if (state == 0) {
              if (dir.IsFile(localfileName)) {
                struct stat info;
                stat(localfileName.c_str(), &info);
                int size = info.st_size;
                if (size > max_file_size) {
                  remove(localfileName.c_str());
                }
              }
            }

            break;
          }
        }
      } else {
        sleep(1);
      }
    }
  }
  if (log_fd != -1) {
    close(log_fd);
  }
  return;
}
static void WriteLogCoutTread(size_t max_file_size, const std::string &file_dir_path) {
  prctl(PR_SET_NAME, "log_cout_file");

  auto se_fn = file_dir_path + "/log_cout";
  int log_fd = -1;
  HJ_INFO("WriteLogCoutTread, file name:%s", se_fn.c_str());
  while (1) {
    if (log_fd < 0) {
      log_fd = open(se_fn.c_str(), O_RDWR | O_CREAT | O_APPEND, 0644);
      if (log_fd != -1) {
        dup2(log_fd, STDOUT_FILENO);
      } else {
        sleep(1);
      }
    } else {
      struct stat info;
      stat(se_fn.c_str(), &info);
      int size = info.st_size;
      if (size > max_file_size) {
        while (1) {
          int ret = close(log_fd);
          if (!ret) {
            log_fd = -1;
            break;
          }
        }
      } else {
        sleep(1);
      }
    }
  }
  if (log_fd != -1) {
    close(log_fd);
  }
  return;
}
LogRedirect::LogRedirect(const rapidjson::Value &json_conf) : hj_bf::Function(json_conf) {
  int err_log_file_size = DEFAULT_ERR_FILE_SIZE;
  std::string err_log_file_path = DEFAULT_ERR_FILE_PATH;
  int cout_log_file_size = DEFAULT_COUT_FILE_SIZE;
  std::string cout_log_file_path = DEFAULT_COUT_FILE_PATH;
  // read your config
  if (json_conf.HasMember("err_log_file_size") && json_conf["err_log_file_size"].IsInt()) {
    err_log_file_size = json_conf["err_log_file_size"].GetInt();
    HJ_IMPORTANT("minos get err_log_file_size:%d", err_log_file_size);
  }
  if (json_conf.HasMember("err_log_file_path") && json_conf["err_log_file_path"].IsString()) {
    err_log_file_path = json_conf["err_log_file_path"].GetString();
    HJ_IMPORTANT("minos get err_log_file_path:%s", err_log_file_path.c_str());
  }
  if (json_conf.HasMember("cout_log_file_size") && json_conf["cout_log_file_size"].IsInt()) {
    cout_log_file_size = json_conf["cout_log_file_size"].GetInt();
    HJ_IMPORTANT("minos get cout_log_file_size:%d", cout_log_file_size);
  }
  if (json_conf.HasMember("cout_log_file_path") && json_conf["cout_log_file_path"].IsString()) {
    cout_log_file_path = json_conf["cout_log_file_path"].GetString();
    HJ_IMPORTANT("minos get cout_log_file_path:%s", cout_log_file_path.c_str());
  }
  OperateDir dir;
  int state = access(DEFAULT_ERR_FILE_PATH, R_OK | W_OK);
  if (state == 0) {
    if (dir.IsFile(DEFAULT_ERR_FILE_PATH)) {  //??
      if (0 != remove(DEFAULT_ERR_FILE_PATH)) {
        HJ_ERROR("remove file err:%s", DEFAULT_ERR_FILE_PATH);
      }
      mkdir(DEFAULT_ERR_FILE_PATH, 0777);
    }
  } else {
    mkdir(DEFAULT_ERR_FILE_PATH, 0777);
  }
  state = access(err_log_file_path.c_str(), R_OK | W_OK);
  if (state == 0) {
    if (dir.IsFile(err_log_file_path.c_str())) {  //??
      if (0 != remove(err_log_file_path.c_str())) {
        HJ_ERROR("remove file err:%s", err_log_file_path.c_str());
      }
      mkdir(err_log_file_path.c_str(), 0777);
    }
  } else {
    mkdir(err_log_file_path.c_str(), 0777);
  }
  // your code
  std::thread t_err(WriteLogErrTread, 10 * 1024 * 1024, err_log_file_path);
  t_err.detach();
  //  std::thread t_cout(WriteLogCoutTread, cout_log_file_size, cout_log_file_path);
  //  t_cout.detach();
  HJ_IMPORTANT("minos just a LogRedirect");
}

}  // namespace log_redirect_ns
