#pragma once
#include <string>
#include <curl/curl.h>
#include <curl/easy.h>
#include <assert.h>
#include <fstream>
#include "Common.h"

namespace aiper_ota {
namespace utils {

std::string getFileMd5(const char* filepath);
bool isFileExist(const char* filepath);
bool removeDir(const char* dir);
bool emptyDir(const char* dir);
bool removeFile(const char* file);
bool touchFile(const char* file);
bool renameFile(const char* oldf, const char* newf);
uintmax_t getAvailDiskSpace(const char* path);

class DevInfo
{
  public:
    DevInfo(const char* filepath);
    ~DevInfo();
    bool openSucc();
    std::string model() const {return model_;}
    std::string sn() const {return sn_;}
    std::string version() const {return version_;}
    std::string bleName() const {return bleName_;}
    std::string waddr() const  {return waddr_;}
    std::string mode() const {return mode_;}

  private:
    void parse();

  private:
    std::ifstream ifs_;
    std::string model_;
    std::string sn_;
    std::string version_;
    std::string bleName_;
    std::string waddr_;
    std::string mode_;
};


}

}//namespace aiper_ota