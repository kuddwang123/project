#include "Utils.h"
#include "nlohmann/json.hpp"
#include <boost/filesystem.hpp>
#include <fstream>
#include "Common.h"
#include "hjlog.h"

namespace aiper_ota {
namespace utils {

std::string getFileMd5(const char* filepath)
{
    char buffer[64] = {0};
    FILE* fp = NULL;
    std::string cmdStr = "md5sum ";
    cmdStr.append(filepath);

    fp = popen(cmdStr.c_str(), "r");
    
    if (!fp) {
        //HJ_ERROR("popen [%s] failed\n", cmdStr.c_str());
        return "";
    }

    fgets(buffer, sizeof(buffer), fp);

    fclose(fp);

    std::string result = buffer;
    std::string md5 = result.substr(0, 32);
    
    return md5;
}

bool isFileExist(const char* filepath)
{
    boost::filesystem::path boostfile(filepath);
    return boost::filesystem::exists(boostfile);
}

bool removeDir(const char* dirpath)
{
    boost::filesystem::path dir(dirpath);
    try {
        if (boost::filesystem::exists(dir)) {
            boost::filesystem::remove_all(dir);
        }
    } catch (const boost::filesystem::filesystem_error& e) {
        HJ_CST_TIME_DEBUG(ota_logger, "Error deleting directory [%s]:[%s]\n",
            dirpath, e.what());
        return false;
    }

    HJ_CST_TIME_DEBUG(ota_logger, "Remove directory [%s] done\n", dirpath);
    return true;
}

bool emptyDir(const char* dirpath)
{
    boost::filesystem::path dir(dirpath);
    try {
        if (boost::filesystem::exists(dir) && boost::filesystem::is_directory(dir)) {
            for (auto& entry : boost::filesystem::directory_iterator(dir)) {  
                boost::filesystem::remove_all(entry);
            }
        }
    } catch (const boost::filesystem::filesystem_error& e) {
        HJ_CST_TIME_DEBUG(ota_logger, "Error deleting directory [%s]:[%s]\n",
            dirpath, e.what());
        return false;
    }

    HJ_CST_TIME_DEBUG(ota_logger, "Clear directory [%s] done\n", dirpath);
    return true;
}

bool removeFile(const char* filepath)
{
    boost::filesystem::path file(filepath);
    try {
        if (boost::filesystem::exists(file)) {
            if (!boost::filesystem::remove(file)) {
                return false;
            }
        } 
    } catch (const boost::filesystem::filesystem_error& e) {
        HJ_CST_TIME_DEBUG(ota_logger, "Error deleting file [%s]:[%s]\n",
            filepath, e.what());
        return false;
    }

    HJ_CST_TIME_DEBUG(ota_logger, "Remove file [%s] done\n", filepath);
    return true;
}

bool touchFile(const char* file)
{
    std::ofstream ofs(file, std::ios::out | std::ios::trunc);
    if (ofs) {
        ofs.close();
        return true;
    } else {
        return false;
    }
}

bool renameFile(const char* oldf, const char* newf)
{
    boost::filesystem::path old_file(oldf);
    boost::filesystem::path new_file(newf);

    try {
        if (!boost::filesystem::exists(old_file)) {
            HJ_CST_TIME_ERROR(ota_logger, "Error: Source file does not exist\n");
            return false;
        }

        // 检查目标文件夹是否存在，不存在则创建
        if (!boost::filesystem::exists(new_file.parent_path())) {
            boost::filesystem::create_directories(new_file.parent_path());
        }

        boost::filesystem::copy_file(old_file, new_file, boost::filesystem::copy_option::overwrite_if_exists);
        boost::filesystem::remove(old_file);
    } catch (const boost::filesystem::filesystem_error& e) {
        HJ_CST_TIME_ERROR(ota_logger, "%s\n", e.what());
        return false;
    }

    return true;

}

uintmax_t getAvailDiskSpace(const char* path)
{
    boost::filesystem::space_info si = boost::filesystem::space(path);
    return si.available;
}

Curl::Curl()
	: curl_(NULL),
    fp_(NULL),
    dllen_(0),
	curlCode_(CURLE_OK),
    dlnow_(-1)
{
	curl_ = curl_easy_init();
    assert(curl_);
	curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, writeCallback);
    curl_easy_setopt(curl_, CURLOPT_XFERINFOFUNCTION, progressCallbackStatic);
    curl_easy_setopt(curl_, CURLOPT_NOPROGRESS, 0L);
    curl_easy_setopt(curl_, CURLOPT_XFERINFODATA, this);
    //curl_easy_setopt(curl_, CURLOPT_HTTP_VERSION, CURL_HTTP_VERSION_1_1);
	//curl_easy_setopt(curl_, CURLOPT_WRITEDATA, this);
}
 
 
Curl::~Curl(void)
{
    if (curl_) {
	    curl_easy_cleanup(curl_);
    }
    if (fp_) {
        fclose(fp_);
    }
}
 
bool Curl::setUrl(const std::string& url)
{
    if ( url == url_ ) 
        return true;
    url_ = url;
    curlCode_ = curl_easy_setopt(curl_, CURLOPT_URL, url_.c_str());
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

bool Curl::setDestFile(const std::string& file)
{
    if (fp_) {
        ::fclose(fp_);
        fp_ = NULL;
    }

    dllen_ = 0;

    if (NULL == (fp_ = fopen(file.data(), "rb"))) {
        fp_ = fopen(file.data(), "wb");
    } else {
        fclose(fp_);
        fp_ == fopen(file.data(), "ab");
        if (!fp_) {
            return false;
        }
        //fseek(fp_, 0L, SEEK_END);
        dllen_ = ftell(fp_);
        if (dllen_ < 0) {
            return false;
        }
        fprintf(stdout, "get dllen:%ld\n", dllen_);
    }
    file_ = file;

    if (CURLE_OK != (curlCode_ = curl_easy_setopt(curl_, CURLOPT_WRITEDATA, fp_))) {
        return false;
    }

    if (CURLE_OK != (curlCode_ = curl_easy_setopt(curl_, CURLOPT_RESUME_FROM, dllen_))) {
        return false;
    }

    return true;
}

bool Curl::startDownLoad()
{
    run_ = true;
    dlnow_ = 0;
    curlCode_ = curl_easy_perform(curl_);
    fflush(fp_);
    run_ = false;
    return CURLE_OK == curlCode_;
}

bool Curl::stopDownLoad()
{
    run_ = false;
    return true;
}

CURLcode Curl::getResult()
{
    return curlCode_;
}

bool Curl::setMaxDlSpeed(long speed)
{
    curlCode_ = curl_easy_setopt(curl_, CURLOPT_MAX_RECV_SPEED_LARGE, (curl_off_t)speed);
    return CURLE_OK == curlCode_;
}

std::string Curl::getError()
{
    return std::string(curl_easy_strerror(curlCode_));
}
 
size_t Curl::writeCallback(void* ptr, size_t nSize, size_t nMemByte, FILE* stream)
{
	ssize_t written = fwrite(ptr, nSize, nMemByte, stream);
    //fprintf(stdout, "write size:%ld\n", written);
    //fflush(stream);
    return nSize*nMemByte;
}

int Curl::progressCallbackStatic(void *clientp, curl_off_t dltotal, curl_off_t dlnow, curl_off_t ultotal, curl_off_t ulnow) 
{
    Curl* self = static_cast<Curl*>(clientp); // 转换为实例对象
    return self->progressCallback(dltotal, dlnow, ultotal, ulnow);
}

int Curl::progressCallback(curl_off_t dltotal, curl_off_t dlnow, curl_off_t ultotal, curl_off_t ulnow)
{
    static int cnt = 0;
    if (dltotal > 0) {
        int progress = (int)((double)(dllen_+dlnow) / (dllen_+dltotal) * 100.0);
        //fprintf(stdout, "Download progress: %d\n", progress);
        if (progressRptCb_) {
            progressRptCb_(progress);
        }
        //fprintf(stdout, "%ld, %ld , Download progress: %.2f%%\n",dltotal,dlnow,  progress);
        //fflush(stdout);
    }

    if (dlnow_ == dlnow && dlnow != 0) {
        ++cnt;
        //fprintf(stderr, "down load equal: %ld, %ld\n", dlnow_, dlnow);
        if (cnt >= 50) {
           HJ_CST_TIME_ERROR(ota_logger, "download weak, abort\n");
           cnt = 0;
           return -1; 
        }
    } else {
        cnt = 0;
    }
    dlnow_ = dlnow;
    
    if (run_) {
        return 0; // 返回0表示继续下载，返回非零值会中止传输
    } else {
        return -1;
    }
}

DevInfo::DevInfo(const char* filepath):
    ifs_(filepath)
{
    parse();
}

DevInfo::~DevInfo()
{
    if (this->openSucc())
        ifs_.close();
}

bool DevInfo::openSucc()
{
    return ifs_.is_open();
}

void DevInfo::parse()
{
    if (!ifs_.is_open())
        return;
    
    std::string jsonString((std::istreambuf_iterator<char>(ifs_)),
                    std::istreambuf_iterator<char>());

    try {
        nlohmann::json dev = nlohmann::json::parse(jsonString);
        model_ = dev["model"];
        sn_ = dev["sn"];
        version_ = dev["version"];
        bleName_ = dev["bleName"];
        waddr_ = dev["waddr"];
        mode_ = dev["mode"];
    } catch (nlohmann::json::exception& e) {
        HJ_CST_TIME_DEBUG(ota_logger, "parse json fail:%s\n", jsonString.c_str());
        ifs_.close();
    }
}

}
}//namespace aiper_ota