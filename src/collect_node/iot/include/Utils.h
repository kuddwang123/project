#pragma once

#include <string>
#include <curl/curl.h>
#include <curl/easy.h>
#include "rapidjson/document.h"
#include <fstream>
#include <mutex>
#include <condition_variable>
namespace collect_node_iot {

namespace utils {
std::string documentToString(const rapidjson::Document& doc);
std::string documentToString(const rapidjson::Value& doc);
bool hexStringToUnsignedCharArray(const std::string& hexstring, unsigned char* byteArray, size_t len);
bool asciiStringToUnsignedCharArray(const std::string& asciistring, unsigned char* byteArray, size_t len);
std::string aesCbc128Encode(const std::string& plaintext, unsigned char* key, unsigned char* iv);
bool aesCbc128Decode(const unsigned char* ciphertext, ssize_t ciphertext_len, unsigned char* key, unsigned char* iv, std::string& out);
std::string getIpAddrString();
std::string xor_encrypt(const std::string& in);
void replaceSpaceWithUnderline(std::string& str);
std::string randomString(uint8_t length);
std::string removeChar(const std::string& str, char ch);

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

class AwsCertParser
{
  public:
    AwsCertParser(const rapidjson::Value& doc);
    AwsCertParser(const char* filepath);
    bool parseOk();
    std::string cert() const {return cert_;}
    std::string prikey() const {return prikey_;}
    std::string thingname() const {return thingname_;}
    std::string endpoint() const {return endpoint_;}
  
  private:
    void parseDocument(const rapidjson::Value& doc);

  private:
    std::string cert_;
    std::string prikey_;
    std::string thingname_;
    std::string endpoint_;
};

class CountDownLatch
{
  public:
    CountDownLatch(const CountDownLatch&) = delete;
    void operator=(const CountDownLatch&) = delete;
    CountDownLatch(int cnt);

    int getCountDown();
    void countDown();
    bool await(bool wait);
    void resetCount(int cnt);

  private:
    int count_;
    std::mutex mtx_;
    std::condition_variable cond_;
};

enum CurlFlag
{
	kNone = 0,
	kDownload,
	kPost,
	kGet,
};
class Curl
{
public:
	Curl();
	~Curl();
    
	/******************************************************************************
	* libcurl C++ interface
	******************************************************************************/

    //设置连接URL
	bool setUrl(const std::string& url);
    //设置执行超时（秒）
	bool setTimeout(int nSecond);
    //设置连接超时（秒）
	bool setConnectTimeout(int nSecond);
    //添加自定义头
	bool addHeader(std::string key, std::string value);
	//清理HTTP列表头
    void clearHeaderList();
	//获取错误详细信息
    std::string getError() const;
    //Post 字符串数据
	bool post(std::string data);
	//Get 请求
    bool get(std::string url);
    //获取Post/Get请求返回数据
	const std::string& getResponse() const;
    //获取Post/Get请求返回数据
	const char*	getResponseStr() const;
    //获取错误码
    int getErrorCode() const {return curlCode_;}
    //根据code获取错误详细原因
    static std::string getErrorByCode(int code);
    
private:
	static size_t writeCallback(void* pBuffer, size_t nSize, size_t nMemByte, void* pParam);
 
private:
	CURL *curl_;
	std::string url_;
	CURLcode curlCode_;
	std::string response_;
	CurlFlag flag_;
	curl_slist *curlList_;
};

}

}