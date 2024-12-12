#include "hjlog.h"
#include "State/DownLoad.h"
#include <hj_interface/AppMsg.h>
#include <boost/filesystem.hpp>
#include "Utils.h"

namespace aiper_ota {

CurlImpl::CurlImpl():
    isRun_(false),
    progress_(0)
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
    //fprintf(stdout, "dl progress:%d...\n", progress);
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
        assert(curlUtil_.setTimeout(timeout));
    }

    assert(curlUtil_.setUrl(url));
    assert(curlUtil_.setDestFile(dlfile));


    isRun_.store(true);
    bool dlrst = curlUtil_.startDownLoad();
    progress_ = 0;
    auto md5dl = utils::getFileMd5(dlfile.data());
    //fprintf(stdout, "md5=%s, md5dl=%s, dl result:%d\n",md5.c_str(), md5dl.c_str(), dlrst);
    /*
    dlthread_ = std::thread th([&]() {
        dlrst = curlUtil_.startDownLoad();
        //cond_.notify_all();
    });
    
    //cond_.wait(lc);
    if (dlthread_.joinable()) {
        dlthread_.join();
        HJ_CST_TIME_DEBUG(ota_logger, "dl thread joined\n");
    }
    */
    

    isRun_.store(false);

    if (dlrst && (md5 != utils::getFileMd5(dlfile.data()))) {
        utils::removeFile(dlfile.data());
        HJ_CST_TIME_ERROR(ota_logger, "download success but md5 not match\n");
    }

    return dlrst && (md5 == utils::getFileMd5(dlfile.data()));
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
        assert(boost::filesystem::create_directories(dir));
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
/*
    if (!md5ExpOld_.empty()) {
        if (param.md5_ != md5ExpOld_) {
            HJ_CST_TIME_DEBUG(ota_logger, "download trigger different: [%s] [%s]\n",
                param.md5_.c_str(), md5ExpOld_.c_str());
            return false;
        }
    }
*/
    mode_ = param.mode_;
    md5ExpOld_ = param.md5_;
    timeout_ = param.timeout_;

    curlImpPtr_->setMode(mode_);

    if (mode_ == 1) {
        //utils::touchFile(autoDlMark_);
    } else if (mode_ == 0) {
        otaRptFunc_(1, 0, "", 0);
    }

    std::string dlfile = std::string(otaTmpDir_) + param.md5_ + "_tmp.bin";
    if (utils::isFileExist(dlfile.data())) {
        HJ_CST_TIME_DEBUG(ota_logger, "file [%s] exists\n", dlfile.c_str());
    } else {
        assert(utils::emptyDir(otaTmpDir_));
    }

    bool result = curlImpPtr_->start(param.url_, param.md5_, dlfile, timeout_);
    
    if (mode_ == 1) {
        //utils::removeFile(autoDlMark_);
    }

    if (result) {
        assert(utils::renameFile(dlfile.data(), otaBinFile_));
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

work_state Download::getStatus()
{
    if (!curlImpPtr_) {
        return READY;
    } else {
        return curlImpPtr_->isRun() ? WORKING : END;
    }
}

}//namespce aiper_ota