#include "hjlog.h"
#include "State/DownLoad.h"
#include <hj_interface/AppMsg.h>
#include <boost/filesystem.hpp>
#include "Utils.h"

namespace aiper_ota {

CurlImpl::CurlImpl():
    isRun_(false),
    progress_(0),
    dlrst_(BP_OK)
{   
    //curlUtil_.setMaxDlSpeed(1024 * 500);
    curlUtil_.setDlProgressCb(boost::bind(&CurlImpl::dlProgress, this, boost::placeholders::_1));
    curlUtil_.setConnectTimeout(15);
}

CurlImpl::~CurlImpl()
{
    HJ_CST_TIME_DEBUG(ota_logger, "curl impl deconstruct!\n");
}

void CurlImpl::dlProgress(int progress)
{
    if (mode_ == 1) {
        return;
    }

    if (progress - progress_ >= 1) {
        if (dlProgressRptFunc_) {
            dlProgressRptFunc_(1, progress/2, "", 0);
        }
    }
    progress_ = progress;
}

void CurlImpl::stop()
{
    if (isRun_.load()) {
        curlUtil_.stopDownLoad();
    }
}

bool CurlImpl::start(const std::string& url, const std::string& md5, const std::string& dlfile, int timeout)
{
    if (timeout > 0) {
        if (!curlUtil_.setTimeout(timeout)) {
            HJ_CST_TIME_ERROR(ota_logger, "set timeout fail\n");
            return false;
        }
    }

    if (!curlUtil_.setUrl(url)) {
        HJ_CST_TIME_ERROR(ota_logger, "set url [%s] fail\n", url.c_str());
        return false;
    }

    if (!curlUtil_.setDestFile(dlfile)) {
        HJ_CST_TIME_ERROR(ota_logger, "set dest file [%s] fail\n", dlfile.c_str());
        return false;
    }

    dlrst_ = BP_OK;
    isRun_.store(true);
    bool dlrst = curlUtil_.startDownLoad();
    progress_ = 0;
    auto md5dl = utils::getFileMd5(dlfile.data());

    isRun_.store(false);

    if (!dlrst) {
        dlrst_ = BP_DLFAIL_NETWORK_FAIL;
        HJ_CST_TIME_ERROR(ota_logger, "download fail\n");
        return false;
    }

    if (md5 != utils::getFileMd5(dlfile.data())) {
        utils::removeFile(dlfile.data());
        HJ_CST_TIME_ERROR(ota_logger, "download md5 not match\n");
        dlrst_ = BP_DLFAIL_MD5_MISMATCH;
        return false;
    }

    return true;
/*
    if (dlrst && (md5 != utils::getFileMd5(dlfile.data()))) {
        utils::removeFile(dlfile.data());
        HJ_CST_TIME_ERROR(ota_logger, "download success but md5 not match\n");
    }

    return dlrst && (md5 == utils::getFileMd5(dlfile.data()));
*/
}

#if 0
void CurlImpl::stop()
{
    if (isRun_.load()) {
        return;
    }

    curlUtil_.stopDownLoad();
    if (dlthread_.joinable()) {
        dlthread_.join();
        isRun_.store(false);
        HJ_CST_TIME_DEBUG(ota_logger, "dl thread join\n");
    }

    HJ_CST_TIME_DEBUG(ota_logger, "dl thread stop!\n");
    return; 
}
#endif 

Download::Download(ros::NodeHandle n, const otaStatusReportFunc& otaRptFunc):
    BaseState(n, otaRptFunc),
    curlImpPtr_(std::make_shared<CurlImpl>())
{
    curlImpPtr_->setDlRptcb(otaRptFunc_);
    boost::filesystem::path dir(otaTmpDir_);
    if (!boost::filesystem::is_directory(dir)) {
        if (!boost::filesystem::create_directories(dir)) {
            HJ_CST_TIME_ERROR(ota_logger, "create dir fail\n");
        }
    }
}

void Download::stop()
{
    curlImpPtr_->stop();
    return;
}

bool Download::dowork(const boost::any& para)
{
    DlPara param = boost::any_cast<DlPara>(para);

    mode_ = param.mode_;
    md5ExpOld_ = param.md5_;
    timeout_ = param.timeout_;

    curlImpPtr_->setMode(mode_);

    if (mode_ == 0) {
        otaRptFunc_(1, 0, "", 0);
    }

    std::string dlfile = std::string(otaTmpDir_) + param.md5_ + "_tmp.bin";
    if (utils::isFileExist(dlfile.data())) {
        HJ_CST_TIME_DEBUG(ota_logger, "file [%s] exists\n", dlfile.c_str());
    } else {
        bool ret = utils::emptyDir(otaTmpDir_);
        assert(ret);
    }

    bool result = curlImpPtr_->start(param.url_, param.md5_, dlfile, timeout_);

    if (result) {
        bool ret = utils::renameFile(dlfile.data(), otaBinFile_);
        assert(ret);
        HJ_CST_TIME_DEBUG(ota_logger, "rename bin file success!\n");
        if (mode_ == 0) {
            otaRptFunc_(1, 50, "", 0);
        } 
    } else {
        HJ_CST_TIME_DEBUG(ota_logger, "download fail : %d\n", curlImpPtr_->getCurlCode());
        /*
        if (curlImpPtr_->getCurlCode() != CURLE_OPERATION_TIMEDOUT &&
            curlImpPtr_->getCurlCode() != CURLE_ABORTED_BY_CALLBACK) {
            assert(utils::removeFile(dlfile.data()));
        }*/
    }
    
    HJ_CST_TIME_DEBUG(ota_logger, "download work end\n");
    return result;
}

int Download::getDlResult()
{
    return curlImpPtr_->getDlResult();
}

work_state Download::getStatus()
{
    if (!curlImpPtr_) {
        return READY;
    } else {
        return curlImpPtr_->isRun() ? WORKING : END;
    }
}

}//namespce aiper_ota