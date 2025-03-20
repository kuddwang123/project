#include "State/UnPack.h"
#include "hjlog.h"
#include <fstream>
#include <nlohmann/json.hpp>

namespace aiper_ota {

Unpack::Unpack(ros::NodeHandle n, const otaStatusReportFunc& otaRptFunc, int timeout):
    BaseState(n, otaRptFunc),
    timeout_(timeout)
{
    unPackPub_ = n_.advertise<std_msgs::String>("/system/dlResult", 1);
    unPackSub_ = n_.subscribe("/system/recvBinLocation", 1, &Unpack::upbackCallBack, this);
}

bool Unpack::dowork(const boost::any&)
{
    status_ = WORKING;
    std_msgs::String msg;
    unPackPub_.publish(msg);

    std::unique_lock<std::mutex> lc(mtx_);
    auto waitrst = cond_.wait_for(lc, std::chrono::seconds(timeout_));
    if (waitrst == std::cv_status::timeout) {
        fprintf(stderr, "unpack timeout!\n");
        status_ = END;
        return false;
    }

    try {
        nlohmann::json js = nlohmann::json::parse(unPackMsg_);
        int code = js["code"];
        if (code != 0) {
            return false;
        }
    } catch (const std::exception& e) {
        fprintf(stderr, "cut file json parse fail\n");
        return false;
    }
    
    return true;
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