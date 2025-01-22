#pragma once
#include "BaseState.h"
#include <condition_variable>
#include <mutex>
#include <std_msgs/String.h>
#include "Utils.h"

namespace aiper_ota {

class Unpack: public BaseState {

public:
    Unpack(ros::NodeHandle n, const otaStatusReportFunc& otaRptFunc, int timeout);
    bool dowork(const boost::any& para) override;
    std::string getCutMsg() const {return unPackMsg_;}
    static const char* cutResultFile_;
    std::string getUnPackFailMsg() const {return unPackFailMsg_;}

private:
    int timeout_;
    ros::Publisher unPackPub_;
    ros::Subscriber unPackSub_;
    ros::Timer progressTmr_;
    std::condition_variable cond_;
    std::mutex mtx_;
    std::string unPackMsg_;
    std::string unPackFailMsg_;
    int progressCnt_;

private:
    void upbackCallBack(const std_msgs::String::ConstPtr& msg);
    void progressTmrCb(const ros::TimerEvent&);
};

typedef std::shared_ptr<Unpack> UnPackPtr;

} //namespace aiper_ota