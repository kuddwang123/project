#include "State/UnPack.h"
#include "hjlog.h"
#include <fstream>
#include <nlohmann/json.hpp>

namespace aiper_ota {
const char* Unpack::cutResultFile_ = "/userdata/ota/OtaCutResult.json";

Unpack::Unpack(ros::NodeHandle n, const otaStatusReportFunc& otaRptFunc, int timeout):
    BaseState(n, otaRptFunc),
    timeout_(timeout)
{
    unPackPub_ = n_.advertise<std_msgs::String>("/system/dlResult", 1);
    unPackSub_ = n_.subscribe("/system/recvBinLocation", 1, &Unpack::upbackCallBack, this);
    progressTmr_ = n_.createTimer(ros::Duration(2.0), &Unpack::progressTmrCb, this, false, false);
}

bool Unpack::dowork(const boost::any& para)
{
    std::string nextver = boost::any_cast<std::string>(para);
    otaRptFunc_(2, 50, "", 0);
    progressCnt_ = 0;
    status_ = WORKING;
    HJ_CST_TIME_DEBUG(ota_logger, "pub unpack!\n");
    std_msgs::String msg;
    unPackPub_.publish(msg);
    progressTmr_.start();

    std::unique_lock<std::mutex> lc(mtx_);
    auto waitrst = cond_.wait_for(lc, std::chrono::seconds(timeout_));
    if (waitrst == std::cv_status::timeout) {
        HJ_CST_TIME_DEBUG(ota_logger, "unpack timeout!\n");
        status_ = END;
        progressTmr_.stop();
        return false;
    }

    HJ_CST_TIME_DEBUG(ota_logger, "receive unpack msg: %s\n", unPackMsg_.c_str());
    std::ofstream file(cutResultFile_, std::ios::out | std::ios::trunc);
    if (!file.is_open()) {
        HJ_CST_TIME_ERROR(ota_logger, "open cut file fail\n");
        status_ = END;
        progressTmr_.stop();
        return false;
    }    

    file << unPackMsg_;
    file.close();
    
    try {
        nlohmann::json js = nlohmann::json::parse(unPackMsg_);
        int code = js["code"];
        std::string version = std::string("V")+js["fw_ver"].get<std::string>();
        if (code != 0) {
            progressTmr_.stop();
            return false;
        }

        if (version != nextver) {
            HJ_CST_TIME_ERROR(ota_logger, "version not match: %s %s\n", nextver.c_str(), version.c_str());
            progressTmr_.stop();
            return false;
        }
    } catch (const std::exception& e) {
        HJ_CST_TIME_ERROR(ota_logger, "json parse fail\n");
        progressTmr_.stop();
        return false;
    }

    progressTmr_.stop();
    status_ = END;
    otaRptFunc_(2, 70, "", 0);
    
    return true;
}

void Unpack::progressTmrCb(const ros::TimerEvent&)
{
    ++progressCnt_;
    if (50+progressCnt_ > 70) {
        progressTmr_.stop();
        return;
    }
    otaRptFunc_(2, 50+progressCnt_, "", 0);
}

void Unpack::upbackCallBack(const std_msgs::String::ConstPtr& msg)
{
    if (status_ != WORKING) {
        return;
    }
    std::unique_lock<std::mutex> lc(mtx_);
    unPackMsg_ = msg->data;
    cond_.notify_one();
}

} //namespace aiper_ota