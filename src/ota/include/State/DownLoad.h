#pragma once 

#include "BaseState.h"
#include "Utils.h"
//#include <atomic>
#include <string>
#include <atomic>
#include <condition_variable>
#include <thread>

namespace aiper_ota {
class CurlImpl {
public:
    CurlImpl();
    ~CurlImpl();
    bool start(const std::string& url, const std::string& md5, const std::string& dlfile, int timeout);
    bool isRun() const {return isRun_.load();}
    CURLcode getCurlCode() {return curlUtil_.getResult();}
    void setDlRptcb(const otaStatusReportFunc& cb) {dlProgressRptFunc_ = cb;}
    void setMode(uint8_t mode) {mode_ = mode;}
    void stop();

private:
    std::atomic<bool> isRun_;
    uint8_t mode_;
    utils::Curl curlUtil_;
    otaStatusReportFunc dlProgressRptFunc_;
    int progress_;

private:
    void dlProgress(int progress);
};


typedef std::shared_ptr<CurlImpl> CurlImplPtr;
class Download: public BaseState {
public:
    Download(ros::NodeHandle n, const otaStatusReportFunc& otaRptFunc);
    bool dowork(const boost::any& para) override;
    work_state getStatus() override;
    void stop();

private:
    int timeout_;
    std::string urlOld_;
    std::string md5ExpOld_;
    std::string fileOld_;
    std::string nextVer_;
    uint8_t mode_; //0手动，1自动
    CurlImplPtr curlImpPtr_;
    //const char* autoDlMark_ = "/userdata/autoOtaDl.mark";
    const char* otaBinFile_ = "/userdata/ota/ota.bin";
    const char* otaTmpDir_ = "/userdata/otatmp/";

private:
    //void stopCurDownload();
};

typedef std::shared_ptr<Download> DownloadPtr;

}//namespace aiper_ota