#include <curl/curl.h>
#include <curl/easy.h>
#include <string>

namespace collect_node_posttunnel {
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
    //Put 上传文件
    bool put(std::string filename);
    //Put 上传文件
    bool put(std::string filename, uint64_t size);
    //获取请求返回数据
	const std::string& getResponse() const;
    //获取请求返回数据
	const char*	getResponseStr() const;
    //获取response code
    int64_t getResponseCode();

private:
	static size_t writeCallback(void* pBuffer, size_t nSize, size_t nMemByte, void* pParam);

private:
	CURL *curl_;
	std::string url_;
	CURLcode curlCode_;
    std::string response_;
	curl_slist *curlList_;
    int64_t responseCode_;
};

} //namespace collect_node_posttunnel