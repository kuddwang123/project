#pragma once
#include "State/UnPack.h"
#include "State/Upgrade.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/String.h"
#include "nlohmann/json.hpp"
#include <condition_variable>
#include <deque>
#include <atomic>
#include <thread>

namespace aiper_ota {
class Schedule {
public:
    Schedule(ros::NodeHandle n);
    ~Schedule();
    void construct();
    void run();

private:
    ros::NodeHandle n_;
    ros::Subscriber otaEnterSub_;
    ros::Publisher otaEnterPub_;
    ros::Publisher otaQuitPub_;
    ros::Publisher resetMcuPub_;

    UnPackPtr unpackPtr_;
    UpgradePtr upGradePtr_;

    std::atomic<uint8_t> otaPermit_;
    std::atomic<bool> runflag_;
    std::thread workThread_;
    std::condition_variable cond_;
    std::mutex mtx_;

private:
    void otaStatusRptCb(int state, int progress, std::string msg, int error);
    void otaResultRptCb(int state, int ledstate, int basestate, int socstate, std::string msg);
    void otaEnterCallBack(const std_msgs::UInt8::ConstPtr& msg);
    void doOtaThread();
    bool doOta();
    void mcuReset();
};
}//namespace aiper_ota