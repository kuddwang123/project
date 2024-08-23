#include <boost/filesystem.hpp>
#include "Curl.h"
#include <cstdio>
#include "log.h"

namespace collect_node_posttunnel {
Curl::Curl(void)
	: curl_(NULL)
	,curlCode_(CURLE_OK)
	,curlList_(NULL)
{
	curl_ = curl_easy_init();
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, writeCallback);
	curl_easy_setopt(curl_, CURLOPT_WRITEDATA, this);
    curl_easy_setopt(curl_, CURLOPT_VERBOSE, 1L);
    curl_easy_setopt(curl_, CURLOPT_SSL_VERIFYPEER, 0L);
    curl_easy_setopt(curl_, CURLOPT_SSL_VERIFYHOST, 0L);
}
 
 
Curl::~Curl(void)
{
	clearHeaderList();
	curl_easy_cleanup(curl_);
}
 
bool Curl::setUrl(const std::string& url)
{
    if ( url == url_ ) 
        return true;
    url_ = url;
    curlCode_ = curl_easy_setopt(curl_, CURLOPT_URL, url_.data());
    return CURLE_OK == curlCode_;
}
 
bool Curl::setTimeout(int nSecond)
{
    if (nSecond < 0)
        return false;
    curlCode_ = curl_easy_setopt(curl_, CURLOPT_TIMEOUT, nSecond);
    return CURLE_OK == curlCode_;
}
 
bool Curl::setConnectTimeout(int nSecond)
{
    if (nSecond < 0)
        return false;
    curlCode_ = curl_easy_setopt(curl_, CURLOPT_CONNECTTIMEOUT, nSecond);
    return CURLE_OK == curlCode_;
}
 
bool Curl::addHeader(std::string key, std::string value)
{
    assert(key.size() > 0 && value.size() > 0);
    std::string strHeader(key);
    strHeader.append(": ");
    strHeader.append(value);
    curlList_ = curl_slist_append(curlList_, strHeader.c_str());
    curlCode_ = curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, curlList_);
    return CURLE_OK == curlCode_;
}
 
void Curl::clearHeaderList()
{
    if (curlList_)
    {
        curl_slist_free_all(curlList_);
        curlList_ = NULL;
    }
}
 
std::string Curl::getError() const
{
    return std::string(curl_easy_strerror(curlCode_));
}
 
bool Curl::post(std::string data)
{
    if (url_.empty())
        return false;
    curl_easy_setopt(curl_, CURLOPT_POST, 1);
    curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, data.c_str());
    curl_easy_setopt(curl_, CURLOPT_POSTFIELDSIZE, data.size());
    //curl_easy_setopt(curl_, CURLOPT_URL, url_.c_str());
    
    curlCode_ = curl_easy_perform(curl_);
    HJ_INFO("curl post ret code:%d\n", curlCode_);
    return CURLE_OK == curlCode_;
}

const std::string& Curl::getResponse() const
{
	return response_;
}
 
const char* Curl::getResponseStr() const
{
	return response_.c_str();
}

int64_t Curl::getResponseCode()
{
    int64_t  http_code = 0;
    curl_easy_getinfo(curl_, CURLINFO_RESPONSE_CODE, &http_code);
    return http_code;
}

bool Curl::put(std::string filename)
{
    boost::filesystem::path filepath(filename);
    if (!boost::filesystem::exists(filepath)) {
        HJ_ERROR("file [%s] is not exist, put fail!\n", filename.data());
        return false;
    }
   
    FILE *file = fopen(filename.data(), "rb");
    if (file == NULL) {
        HJ_ERROR("fopen file [%s] error, put fail!\n", filename.data());
        return false;
    }

    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    curl_easy_setopt(curl_, CURLOPT_UPLOAD, 1);
    curl_easy_setopt(curl_, CURLOPT_CUSTOMREQUEST, "PUT");
    curl_easy_setopt(curl_, CURLOPT_READDATA, file);
    curl_easy_setopt(curl_, CURLOPT_INFILESIZE_LARGE, (curl_off_t)file_size);
    
    curlCode_ = curl_easy_perform(curl_);
    HJ_INFO("curl put ret code:%d\n", curlCode_);

    fclose(file);
    return CURLE_OK == curlCode_;
}

bool Curl::put(std::string filename, uint64_t size)
{
    boost::filesystem::path filepath(filename);
    if (!boost::filesystem::exists(filepath)) {
        HJ_ERROR("file [%s] is not exist, put fail!\n", filename.data());
        return false;
    }
   
    FILE *file = fopen(filename.data(), "rb");
    if (file == NULL) {
        HJ_ERROR("fopen file [%s] error, put fail!\n", filename.data());
        return false;
    }

    curl_easy_setopt(curl_, CURLOPT_UPLOAD, 1);
    curl_easy_setopt(curl_, CURLOPT_CUSTOMREQUEST, "PUT");
    curl_easy_setopt(curl_, CURLOPT_READDATA, file);
    curl_easy_setopt(curl_, CURLOPT_INFILESIZE_LARGE, (curl_off_t)size);
    
    curlCode_ = curl_easy_perform(curl_);
    HJ_INFO("curl put ret code:%d\n", curlCode_);

    fclose(file);
    return CURLE_OK == curlCode_;
}

size_t Curl::writeCallback(void* pBuffer, size_t nSize, size_t nMemByte, void* pParam)
{
	Curl* pThis = (Curl*)pParam;
	int written = 0;
	
    pThis->response_.append((const char*)pBuffer, nSize*nMemByte);
    written = nSize*nMemByte;

	return written;
}

} //namespace collect_node_posttunnel