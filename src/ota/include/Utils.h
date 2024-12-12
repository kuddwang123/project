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

class Curl
{
public:
	Curl();
	~Curl();
 
    //设置连接URL
	bool setUrl(const std::string& url);
    //设置执行超时（秒）
	bool setTimeout(int nSecond);
    //设置连接超时（秒）
	bool setConnectTimeout(int nSecond);
    //设置目标文件
    bool setDestFile(const std::string& file);
    //设置最大下载速率
    bool setMaxDlSpeed(long speed);
    //获取下载结果
    CURLcode getResult();
    //开始下载
    bool startDownLoad();
    //停止下载
    bool stopDownLoad();
    //获取错误
    std::string getError();
    //注册进度上报方法
    void setDlProgressCb(const dlProgressFunc& cb) {progressRptCb_ = cb;}

private:
	static size_t writeCallback(void* ptr, size_t nSize, size_t nMemByte, FILE* stream);
    static int progressCallbackStatic(void *clientp, curl_off_t dltotal, curl_off_t dlnow, curl_off_t ultotal, curl_off_t ulnow);
    int progressCallback(curl_off_t dltotal, curl_off_t dlnow, curl_off_t ultotal, curl_off_t ulnow);

private:
	CURL *curl_;
	std::string url_;
    std::string file_;
    FILE* fp_;
    long dllen_;
	CURLcode curlCode_;
    bool run_;
    curl_off_t dlnow_;
    dlProgressFunc progressRptCb_;
};


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